// SPDX-License-Identifier: Apache-2.0
/// \file
/// esp32-a2dp-xq: ESP32 as a Bluetooth A2DP source (sends audio to headphones and speakers)
/// with SBC-XQ.
///
///  - SBC-XQ: Dual Channel, bitpool 38, 452 kbps at 44.1 kHz, when the headphones accept it;
///    otherwise Joint Stereo with bitpool up to 53 (328 kbps), the best standard SBC setting.
///    Requires ESP-IDF 6.0 or newer (esp_a2d_source_set_pref_mcc). On older ESP-IDF, including
///    the Arduino core 3.x, the library runs with the stack's standard SBC configuration.
///  - Per-device memory: the result of XQ on each pair of headphones (works / failed twice) and
///    the user's choice are kept in a small text file, plus a built-in list of models known to
///    break with Dual Channel.
///  - PCM FIFO (~300 ms, PSRAM when available) with TPDF dither from 24/32-bit to 16-bit.
///  - AVRCP: buttons on the headphones (play, pause, next, volume...) and absolute volume in
///    both directions.
///  - Discovery, pairing (Secure Simple Pairing "just works", legacy PIN 0000), reconnect to the
///    last device.
///
/// Classic ESP32 only (ESP32-S3/C3/C6 have no Bluetooth Classic). The application writes
/// stereo PCM at 44.1 kHz:
///
///   a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
///   cfg.device_name = "My speaker source";
///   a2dp_xq_init(&cfg);
///   a2dp_xq_connect("aa:bb:cc:dd:ee:ff");   // or a2dp_xq_scan(true) and pick a device
///   a2dp_xq_start();
///   for (;;) a2dp_xq_write_s16(pcm, frames, portMAX_DELAY);
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define A2DP_XQ_VERSION_MAJOR 0
#define A2DP_XQ_VERSION_MINOR 1
#define A2DP_XQ_VERSION_PATCH 0
#define A2DP_XQ_VERSION_STRING "0.1.0"

/// The Bluedroid SBC encoder takes 16-bit stereo PCM at this rate.
#define A2DP_XQ_SAMPLE_RATE 44100u

typedef enum {
    A2DP_XQ_STATE_OFF = 0,     ///< not initialized
    A2DP_XQ_STATE_IDLE,        ///< no headphones connected
    A2DP_XQ_STATE_SCANNING,    ///< discovery running
    A2DP_XQ_STATE_CONNECTING,
    A2DP_XQ_STATE_CONNECTED,
} a2dp_xq_state_t;

typedef struct {
    char name[64];
    char addr[18];   ///< "aa:bb:cc:dd:ee:ff"
    int8_t rssi;     ///< dBm, 0 if unknown
    bool paired;
    bool connected;
} a2dp_xq_device_t;

typedef struct {
    a2dp_xq_state_t state;
    char device_name[64];   ///< connected (or connecting) device
    char device_addr[18];
    char codec[16];         ///< "SBC-XQ" or "SBC", "" when not connected
    char mode[24];          ///< "Dual Channel", "Joint Stereo"
    uint8_t bitpool;
    uint16_t bitrate_kbps;
    uint32_t sample_rate;
} a2dp_xq_status_t;

typedef enum {
    A2DP_XQ_REMOTE_PLAY_PAUSE = 0,
    A2DP_XQ_REMOTE_PLAY,
    A2DP_XQ_REMOTE_PAUSE,
    A2DP_XQ_REMOTE_STOP,
    A2DP_XQ_REMOTE_NEXT,
    A2DP_XQ_REMOTE_PREV,
    A2DP_XQ_REMOTE_VOL_UP,
    A2DP_XQ_REMOTE_VOL_DOWN,
    A2DP_XQ_REMOTE_VOLUME_ABS,  ///< value = 0..127 set on the headphones (their own volume changed)
    A2DP_XQ_REMOTE_LINK_DOWN,   ///< audio is started but no headphones are connected and no
                                /// reconnect is pending (switched off, out of range): pause
} a2dp_xq_remote_t;

typedef enum {
    A2DP_XQ_EVENT_STATE = 0,       ///< state changed: event.state
    A2DP_XQ_EVENT_DEVICE_FOUND,    ///< discovery found or named a device: event.device
    A2DP_XQ_EVENT_PAIRING_FAILED,  ///< event.device.addr
    A2DP_XQ_EVENT_CODEC,           ///< the stream configuration is known: a2dp_xq_get_status()
} a2dp_xq_event_type_t;

typedef struct {
    a2dp_xq_event_type_t type;
    a2dp_xq_state_t state;
    a2dp_xq_device_t device;
} a2dp_xq_event_t;

/// Both callbacks run in the library's worker task, outside its locks: they may call back into
/// the library. Keep them short.
typedef void (*a2dp_xq_event_cb_t)(const a2dp_xq_event_t *event, void *user);
typedef void (*a2dp_xq_remote_cb_t)(a2dp_xq_remote_t cmd, int value, void *user);

typedef struct {
    const char *device_name;       ///< our name as the headphones see it
    bool allow_sbc_xq;             ///< try Dual Channel bitpool 38 first (ESP-IDF >= 6.0)
    bool auto_reconnect;           ///< connect to the last device after init and after link loss
    const char *last_device;       ///< "aa:bb:..." to reconnect to, NULL = most recently used
    const char *state_dir;         ///< per-device memory is kept here (SD card), NULL = RAM only
    a2dp_xq_event_cb_t event_cb;
    void *event_user;
    a2dp_xq_remote_cb_t remote_cb;  ///< AVRCP passthrough and volume from the headphones
    void *remote_user;
    int8_t worker_core;            ///< core of the worker task (-1 = any)
} a2dp_xq_config_t;

#define A2DP_XQ_CONFIG_DEFAULT()                                                                             \
    {                                                                                                        \
        .device_name = "ESP32", .allow_sbc_xq = true, .auto_reconnect = true, .last_device = NULL,           \
        .state_dir = NULL, .event_cb = NULL, .event_user = NULL, .remote_cb = NULL, .remote_user = NULL,     \
        .worker_core = 0,                                                                                    \
    }

/// True when this build can negotiate SBC-XQ (ESP-IDF >= 6.0 with the internal SBC encoder).
bool a2dp_xq_supported(void);

// ------------------------------------------------------------------- lifecycle ----
/// Start the controller (BR/EDR only, the BLE memory is released), Bluedroid, A2DP source and
/// AVRCP. Takes about 100 KB of internal RAM. ESP_ERR_NOT_SUPPORTED when Bluetooth Classic or
/// A2DP is disabled in sdkconfig.
esp_err_t a2dp_xq_init(const a2dp_xq_config_t *cfg);
/// Disconnect and release everything (controller disabled, memory returned).
void a2dp_xq_deinit(void);
bool a2dp_xq_is_initialized(void);

// ----------------------------------------------------------------------- audio ----
/// Open the audio path: the stream to the headphones starts as soon as they are connected.
esp_err_t a2dp_xq_start(void);
/// Close the audio path (queued audio is dropped) and suspend the stream.
void a2dp_xq_stop(void);
/// Pause keeps the queued audio; after 5 s of pause the stream is suspended so the headphones can
/// sleep their decoder.
void a2dp_xq_pause(bool paused);

/// Queue interleaved stereo frames at A2DP_XQ_SAMPLE_RATE. Blocks up to timeout_ms while the
/// FIFO is full (it drains at the real-time rate while the stream runs). Returns the frames
/// accepted (may be fewer than requested) or -1 when the audio path is not started.
int32_t a2dp_xq_write_s16(const int16_t *pcm, uint32_t frames, uint32_t timeout_ms);
/// Same from left-justified 32-bit samples (24-bit audio in the top bits): reduced to 16 bits
/// with TPDF dither, so quiet passages keep their resolution.
int32_t a2dp_xq_write_q31(const int32_t *pcm, uint32_t frames, uint32_t timeout_ms);
/// Drop what is queued (seek, skip).
void a2dp_xq_flush(void);
/// Frames queued and not yet sent.
uint32_t a2dp_xq_buffered_frames(void);

// ---------------------------------------------------------------------- volume ----
/// Output volume in dB (<= 0). Headphones with AVRCP absolute volume get it as
/// SetAbsoluteVolume and play at their own gain; for other headphones the library attenuates in
/// its 16-bit conversion. Their own volume changes arrive as A2DP_XQ_REMOTE_VOLUME_ABS.
void a2dp_xq_set_volume_db(float db);
/// AVRCP absolute volume (0..127) <-> dB: 127 = 0 dB, 1..127 span 60 dB in equal steps, 0 = mute.
float a2dp_xq_volume_to_db(int v127);
uint8_t a2dp_xq_volume_from_db(float db);

// ------------------------------------------------------------- devices and link ----
void a2dp_xq_get_status(a2dp_xq_status_t *out);
/// "SBC-XQ 452 kbps", "SBC 328 kbps", "" when not connected.
void a2dp_xq_describe(char *out, size_t out_size);
/// Inquiry for about 10 s; A2DP_XQ_EVENT_DEVICE_FOUND per device (audio devices only).
void a2dp_xq_scan(bool start);
/// Bonded devices and the ones found by the last scan: connected first, then paired, then by
/// signal strength. Returns the number written.
uint32_t a2dp_xq_devices(a2dp_xq_device_t *out, uint32_t max);
esp_err_t a2dp_xq_connect(const char *addr);
void a2dp_xq_disconnect(void);
/// Remove the bond and the stored results of a device.
void a2dp_xq_forget(const char *addr);
/// Allow or forbid SBC-XQ for one device (kept in state_dir); takes effect on the next
/// connection. Allowing again also clears an earlier failure verdict.
esp_err_t a2dp_xq_set_xq_allowed(const char *addr, bool allowed);

typedef struct {
    uint32_t underruns;          ///< times the encoder found the FIFO empty while audio was expected
    uint32_t underrun_frames;    ///< silence frames inserted by those underruns
    uint32_t fifo_frames;        ///< frames queued now
    uint32_t fifo_limit_frames;  ///< FIFO size used (~300 ms)
    bool streaming;              ///< A2DP media stream started
    bool abs_volume;             ///< headphones support AVRCP absolute volume
    uint8_t remote_volume;       ///< last absolute volume reported by the headphones, 0..127
    uint16_t sink_delay_ms;      ///< delay reported by the headphones (A2DP delay reporting), 0 if none
} a2dp_xq_stats_t;
void a2dp_xq_get_stats(a2dp_xq_stats_t *out);

#ifdef __cplusplus
}
#endif
