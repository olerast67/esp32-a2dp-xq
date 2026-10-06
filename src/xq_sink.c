// SPDX-License-Identifier: Apache-2.0
// Audio path: the application writes stereo PCM at 44.1 kHz, the library reduces 32-bit input
// to 16 bits with TPDF dither into a FIFO (~300 ms, PSRAM when available), and the A2DP data
// callback of the Bluedroid internal SBC encoder pulls from that FIFO, filling silence when it
// runs dry.
//
// Volume: a2dp_xq.c sends it to headphones with AVRCP absolute volume and sets the gain here to
// 0 dB, or sets the gain here to the volume for headphones without it. The gain is applied
// before the dither, in the same pass.
#ifdef ESP_PLATFORM

#include <stdatomic.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "xq_internal.h"
#include "xq_util.h"
#include "xq_volume.h"

static const char *TAG = "a2dp_xq";

#define S16_CHUNK_FRAMES 256  // s16 -> Q31 staging for attenuated 16-bit writes

// Internal RAM: atomic read-modify-write does not work on PSRAM addresses.
static xq_fifo_t s_fifo;
static bool s_fifo_ready;
static SemaphoreHandle_t s_space;  // given by the consumer after it frees space
static atomic_bool s_open;
static atomic_bool s_paused;
static _Atomic uint32_t s_last_write_ms;
static _Atomic int32_t s_gain_q30 = XQ_FIFO_GAIN_UNITY;
static atomic_bool s_discard;
static uint32_t s_dither = 0x1234567u;  // producer task only

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

// ------------------------------------------------------------------ lifecycle ----

int xq_sink_init(void) {
    if (s_fifo_ready) return XQ_OK;
    int rc = xq_fifo_init(&s_fifo, XQ_FIFO_CAPACITY_FRAMES, XQ_FIFO_LIMIT_FRAMES, XQ_FIFO_PREBUFFER_FRAMES);
    if (rc) {
        ESP_LOGE(TAG, "FIFO allocation failed (%d)", rc);
        return rc;
    }
    s_space = xSemaphoreCreateBinary();
    if (!s_space) {
        xq_fifo_free(&s_fifo);
        return XQ_ENOMEM;
    }
    atomic_store(&s_open, false);
    atomic_store(&s_paused, false);
    atomic_store(&s_discard, false);
    atomic_store(&s_gain_q30, XQ_FIFO_GAIN_UNITY);
    s_fifo_ready = true;
    return XQ_OK;
}

void xq_sink_deinit(void) {
    if (!s_fifo_ready) return;
    atomic_store(&s_open, false);
    s_fifo_ready = false;
    xq_fifo_free(&s_fifo);
    if (s_space) {
        vSemaphoreDelete(s_space);
        s_space = NULL;
    }
}

bool xq_sink_is_open(void) { return atomic_load(&s_open); }
bool xq_sink_is_paused(void) { return atomic_load(&s_paused); }
void xq_sink_set_gain_db(float db) { atomic_store(&s_gain_q30, xq_volume_gain_q30(db)); }
void xq_sink_discard(void) { atomic_store(&s_discard, true); }

void xq_sink_get_stats(a2dp_xq_stats_t *out) {
    if (!out || !s_fifo_ready) return;
    out->underruns = atomic_load_explicit(&s_fifo.underruns, memory_order_relaxed);
    out->underrun_frames = atomic_load_explicit(&s_fifo.underrun_frames, memory_order_relaxed);
    out->fifo_frames = xq_fifo_level(&s_fifo);
    out->fifo_limit_frames = s_fifo.limit;
}

// ------------------------------------------------------------ A2DP callback ----

int32_t xq_sink_data_cb(uint8_t *buf, int32_t len) {
    // len < 0 is the stack's "flush" hint when the stream stops or suspends. The FIFO holds
    // the application's audio, which must survive a pause (suspend) untouched, so nothing is
    // dropped here; the application flushes when it seeks or skips.
    if (!buf || len <= 0) return 0;
    uint32_t frames = (uint32_t)len / 4u;
    if (!s_fifo_ready || !atomic_load(&s_open) || atomic_load(&s_paused)) {
        memset(buf, 0, (size_t)len);
        return len;
    }
    if (atomic_exchange(&s_discard, false)) xq_fifo_discard(&s_fifo);
    bool idle = (uint32_t)(now_ms() - atomic_load(&s_last_write_ms)) > XQ_PRODUCER_IDLE_MS;
    xq_fifo_read(&s_fifo, (int16_t *)(void *)buf, frames, idle);
    if ((uint32_t)len > frames * 4u) memset(buf + frames * 4u, 0, (size_t)len - frames * 4u);
    xSemaphoreGive(s_space);
    // Always report the full length: a short read makes the encoder log an underflow and
    // keep a partial residue; silence here keeps the SBC frame timing regular.
    return len;
}

// --------------------------------------------------------------------- public ----

esp_err_t a2dp_xq_start(void) {
    if (!s_fifo_ready || !a2dp_xq_is_initialized()) return ESP_ERR_INVALID_STATE;
    if (atomic_load(&s_open)) return ESP_OK;
    xq_fifo_flush(&s_fifo);
    s_dither = 0x1234567u;
    atomic_store(&s_last_write_ms, now_ms());
    atomic_store(&s_open, true);
    xq_core_sink_open(true);
    return ESP_OK;
}

void a2dp_xq_stop(void) {
    if (!atomic_exchange(&s_open, false)) return;
    atomic_store(&s_paused, false);
    if (s_fifo_ready) xq_fifo_flush(&s_fifo);
    xq_core_sink_open(false);
}

void a2dp_xq_pause(bool paused) {
    if (atomic_exchange(&s_paused, paused) == paused) return;
    xq_core_sink_paused(paused);
}

void a2dp_xq_flush(void) {
    if (s_fifo_ready) xq_fifo_flush(&s_fifo);
}

uint32_t a2dp_xq_buffered_frames(void) { return s_fifo_ready ? xq_fifo_level(&s_fifo) : 0; }

// One write call: `put` stores up to n frames from frame `done` on and returns how many.
typedef uint32_t (*put_fn_t)(const void *pcm, uint32_t done, uint32_t n);

static int32_t write_frames(const void *pcm, uint32_t frames, uint32_t timeout_ms, put_fn_t put) {
    if (!s_fifo_ready || !atomic_load(&s_open)) return -1;
    if (!frames) return 0;
    if (!pcm) return -1;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint32_t done = 0;
    for (;;) {
        uint32_t n = put(pcm, done, frames - done);
        done += n;
        if (done >= frames) break;
        if (n) continue;  // a partial store (staging chunk): there may be more room
        int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) break;
        // FIFO full (normal while streaming, or the stream is not running yet): wait for the
        // consumer to free space, in slices so the deadline is honoured.
        uint32_t wait_ms = (uint32_t)(left_us / 1000);
        if (wait_ms > 20) wait_ms = 20;
        xSemaphoreTake(s_space, pdMS_TO_TICKS(wait_ms) + 1);
    }
    if (done) atomic_store(&s_last_write_ms, now_ms());
    return (int32_t)done;
}

static uint32_t put_q31(const void *pcm, uint32_t done, uint32_t n) {
    const int32_t *p = (const int32_t *)pcm + (size_t)done * 2u;
    return xq_fifo_write_q31_gain(&s_fifo, p, n, &s_dither, atomic_load(&s_gain_q30));
}

static uint32_t put_s16(const void *pcm, uint32_t done, uint32_t n) {
    const int16_t *p = (const int16_t *)pcm + (size_t)done * 2u;
    int32_t gain = atomic_load(&s_gain_q30);
    if (gain >= XQ_FIFO_GAIN_UNITY) return xq_fifo_write_s16(&s_fifo, p, n);  // bit-exact copy
    // Attenuated: through Q31 so the gain gets its dither.
    int32_t tmp[S16_CHUNK_FRAMES * 2];
    if (n > S16_CHUNK_FRAMES) n = S16_CHUNK_FRAMES;
    for (uint32_t i = 0; i < n * 2u; i++) tmp[i] = (int32_t)((uint32_t)(uint16_t)p[i] << 16);
    return xq_fifo_write_q31_gain(&s_fifo, tmp, n, &s_dither, gain);
}

int32_t a2dp_xq_write_q31(const int32_t *pcm, uint32_t frames, uint32_t timeout_ms) {
    return write_frames(pcm, frames, timeout_ms, put_q31);
}

int32_t a2dp_xq_write_s16(const int16_t *pcm, uint32_t frames, uint32_t timeout_ms) {
    return write_frames(pcm, frames, timeout_ms, put_s16);
}

#endif  // ESP_PLATFORM
