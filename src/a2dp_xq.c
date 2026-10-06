// SPDX-License-Identifier: Apache-2.0
// Bluetooth A2DP source: controller in BR/EDR-only mode, Bluedroid, GAP (inquiry with name
// resolution, Secure Simple Pairing "just works", bonded devices), A2DP source with the
// Bluedroid internal SBC encoder, SBC-XQ negotiation with per-device memory, AVRCP controller
// (absolute volume) and target (buttons on the headphones).
//
// SBC-XQ: once the headphones report their SBC capabilities, the library asks for Dual
// Channel, 16 blocks, 8 subbands, loudness, bitpool 38..38 with esp_a2d_source_set_pref_mcc().
// The Bluedroid encoder then climbs its target bitrate until bitpool 38 fits (452 kbps). A link
// lost soon after starting XQ, or a refused stream start, counts against XQ for that device;
// two strikes and the device uses Joint Stereo from then on. Needs ESP-IDF >= 6.0; older
// versions use the stack's default Joint Stereo configuration.
//
// Volume: the application volume (a2dp_xq_set_volume_db) goes to headphones with AVRCP
// absolute volume as SetAbsoluteVolume and the PCM path plays at 0 dB; their own volume
// changes come back as A2DP_XQ_REMOTE_VOLUME_ABS. Without absolute volume (or until the
// headphones acknowledged it) the PCM path attenuates in its 16-bit conversion. The two never
// attenuate on top of each other.
//
// Link loss: while audio is started and not paused but no headphones are connected and no
// reconnect is pending, A2DP_XQ_REMOTE_LINK_DOWN asks the application to pause.
//
// Threads: Bluedroid calls our callbacks in its own tasks. The callbacks copy what they need
// into a message and post it to one worker task, which owns the state machine; the public
// functions post commands to the same queue. Getters read the state under s_bt.lock.
#include "sdkconfig.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "a2dp_xq.h"
#include "xq_devices.h"
#include "xq_internal.h"
#include "xq_volume.h"
#include "xq_util.h"
#include "xq_sbc.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_BLUEDROID_ENABLED) && defined(CONFIG_BT_CLASSIC_ENABLED) && \
    defined(CONFIG_BT_A2DP_ENABLE)
#define XQ_BT_SUPPORTED 1
#else
#define XQ_BT_SUPPORTED 0
#endif

#if XQ_BT_SUPPORTED && defined(CONFIG_BT_AVRCP_ENABLED)
#define XQ_AVRCP 1
#else
#define XQ_AVRCP 0
#endif

#include "esp_idf_version.h"
// ESP-IDF 6.0: sink capability reports and the preferred codec configuration (SBC-XQ).
#define XQ_HAVE_PREF_MCC (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0))
// ESP-IDF 6.1: the report of every sink codec (ignored, it carries a pointer).
#define XQ_HAVE_ALL_CAPS_EVT (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 1, 0))
// ESP-IDF 5.5: connection handles, esp_a2d_cie_sbc_t fields and the audio MTU.
#define XQ_HAVE_CONN_HDL (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0))

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if XQ_BT_SUPPORTED
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#if XQ_AVRCP
#include "esp_avrc_api.h"
#endif
#endif

static const char *TAG = "a2dp_xq";

// Arduino releases the Bluetooth controller memory before setup() unless a library says it
// needs Bluetooth Classic: core 3.3 and newer through this header's constructor, older 3.x
// cores through a strong btInUse().
#if defined(ARDUINO) && XQ_BT_SUPPORTED
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#else
bool btInUse(void) { return true; }
#endif
#endif

#if XQ_BT_SUPPORTED

_Static_assert(XQ_SBC_CIE_SF_44K == ESP_A2D_SBC_CIE_SF_44K, "CIE sampling frequency bits");
_Static_assert(XQ_SBC_CIE_CH_DUAL == ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL, "CIE channel mode bits");
_Static_assert(XQ_SBC_CIE_CH_JOINT == ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO, "CIE channel mode bits");
_Static_assert(XQ_SBC_CIE_BLOCKS_16 == ESP_A2D_SBC_CIE_BLOCK_LEN_16, "CIE block length bits");
_Static_assert(XQ_SBC_CIE_SUBBANDS_8 == ESP_A2D_SBC_CIE_NUM_SUBBANDS_8, "CIE subband bits");
_Static_assert(XQ_SBC_CIE_ALLOC_LOUDNESS == ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS, "CIE allocation bits");

// ------------------------------------------------------------------ tuning ----
#define WORKER_STACK_BYTES 4096
#define WORKER_PRIORITY 8
#define WORKER_TICK_MS 200
#define QUEUE_LEN 32
#define FOUND_MAX 16
#define INQUIRY_LEN 8                 // x 1.28 s = 10.24 s
#define CONNECT_TIMEOUT_MS 20000      // a page attempt that never reports back
#define AUTO_RECONNECT_ATTEMPTS 6
#define USER_CONNECT_ATTEMPTS 2
#define CAPS_WAIT_MS 2000             // wait for the sink capabilities before deciding on XQ
#define PREF_MCC_TIMEOUT_MS 4000
#define XQ_OK_AFTER_MS 30000          // XQ stream alive this long = XQ works with this device
#define XQ_FAIL_WINDOW_MS 20000       // link loss this soon after starting XQ counts against XQ
#define PAUSE_SUSPEND_MS 5000         // suspend the A2DP stream after this much pause
#define START_RETRY_MS 1000
#define REMOTE_SUSPEND_RETRY_MS 2000
#define START_FAILS_MAX 3
#define MEDIA_ACK_TIMEOUT_MS 5000     // a media command without acknowledgement
#define STACK_OP_TIMEOUT_MS 3000
#define DISCONNECT_WAIT_MS 3000
#define TL_COUNT 16                   // AVRCP transaction labels 0..15
#define ABS_VOLUME_ACK_MS 2000        // SetAbsoluteVolume without an answer: send again
#define ABS_VOLUME_TRIES 3            // then keep the sink's own attenuation
#define LINK_DOWN_PAUSE_MS 1000       // open and playing without a link this long: pause

static const uint32_t k_reconnect_delay_ms[] = {3000, 5000, 10000, 20000, 30000, 60000};

// Event group bits: completions signalled directly from the Bluedroid callbacks.
#define EVB_A2D_INIT BIT0
#define EVB_A2D_DEINIT BIT1
#define EVB_CT_INIT BIT2
#define EVB_CT_DEINIT BIT3
#define EVB_TG_INIT BIT4
#define EVB_TG_DEINIT BIT5
#define EVB_DISCONNECTED BIT6
#define EVB_WORKER_DONE BIT7

// ---------------------------------------------------------------- messages ----
typedef enum {
    MSG_A2D = 1,
    MSG_CT,
    MSG_TG,
    MSG_DISC_RES,
    MSG_DISC_STATE,
    MSG_REMOTE_NAME,
    MSG_AUTH,
    MSG_CMD_CONNECT,
    MSG_CMD_DISCONNECT,
    MSG_CMD_SCAN,
    MSG_CMD_VOLUME,
    MSG_CMD_FORGET,
    MSG_CMD_SINK_OPEN,
    MSG_CMD_SINK_PAUSED,
    MSG_CMD_QUIT,
} msg_type_t;

typedef struct {
    uint8_t addr[6];
    int8_t rssi;
    bool has_rssi;
    bool has_cod;
    uint32_t cod;
    char name[XQ_NAME_MAX];
} disc_info_t;

typedef struct {
    uint8_t type;   // msg_type_t
    uint16_t event;
    union {
        esp_a2d_cb_param_t a2d;
#if XQ_AVRCP
        esp_avrc_ct_cb_param_t ct;
        esp_avrc_tg_cb_param_t tg;
#endif
        disc_info_t disc;
        struct {
            bool started;
        } disc_state;
        struct {
            uint8_t addr[6];
            bool ok;
            char name[XQ_NAME_MAX];
        } named;  // remote name, authentication
        uint8_t addr[6];
        bool flag;
        uint8_t volume;
    } u;
} bt_msg_t;

// ------------------------------------------------------------------- state ----
typedef enum { MEDIA_IDLE = 0, MEDIA_CHECKING, MEDIA_STARTING, MEDIA_STARTED, MEDIA_SUSPENDING } media_state_t;
typedef enum { XQ_UNDECIDED = 0, XQ_PENDING, XQ_ACTIVE, XQ_OFF } xq_phase_t;

typedef struct {
    uint8_t addr[6];
    char name[XQ_NAME_MAX];
    int8_t rssi;
    bool name_requested;
} found_t;

typedef struct {
    a2dp_xq_event_t events[6];
    uint32_t event_count;
    struct {
        a2dp_xq_remote_t cmd;
        int value;
    } remote[4];
    uint32_t remote_count;
} outbox_t;

static struct {
    bool initialized;
    // stack stages brought up (for a clean teardown after partial failures)
    bool ctrl_inited, ctrl_enabled, bd_inited, bd_enabled, ct_inited, tg_inited, a2d_inited;
    bool sink_ready;

    // configuration (strings copied)
    bool allow_xq;
    bool auto_reconnect;
    char device_name[48];
    char state_dir[XQ_PATH_MAX];
    a2dp_xq_remote_cb_t remote_cb;
    void *remote_user;
    a2dp_xq_event_cb_t event_cb;
    void *event_user;
    int8_t worker_core;

    SemaphoreHandle_t lock;
    QueueHandle_t queue;
    bool queue_caps;
    TaskHandle_t task;
    bool task_caps;
    EventGroupHandle_t events;

    a2dp_xq_state_t state;
    bool scanning;
    bool connecting;
    bool connected;
    uint8_t peer[6];
    char peer_name[XQ_NAME_MAX];
#if XQ_HAVE_CONN_HDL
    esp_a2d_conn_hdl_t conn_hdl;
#endif
    int64_t connected_at_us;

    media_state_t media;
    int64_t media_since_us;  // when `media` last changed (stuck-state watchdog)
    int64_t start_retry_at_us;
    uint32_t start_fails;
    bool sink_open;
    bool sink_paused;
    int64_t paused_at_us;

    bool have_caps;
    xq_sbc_cie_t caps;
    bool have_cfg;
    xq_sbc_cie_t cfg;
    xq_phase_t xq;
    int64_t xq_deadline_us;
    int64_t stream_started_us;
    bool xq_verdict_done;
    uint16_t sink_delay_ms;

    bool ct_connected;
    bool abs_volume;           // the headphones support absolute volume (notification caps)
    uint8_t remote_volume;
    bool abs_active;           // they apply our volume: the sink plays at 0 dB
    bool abs_pending;          // SetAbsoluteVolume sent, no answer yet
    bool abs_sent_valid;
    uint8_t abs_sent;          // last volume sent to or reported by the headphones
    uint8_t abs_tries;
    int64_t abs_sent_us;

    bool have_prev_peer;       // headphones of the last connection (the FIFO audio was made for them)
    uint8_t prev_peer[6];
    int64_t stall_since_us;    // sink open and playing without a link since, 0 = not stalled
    bool stall_reported;
    uint8_t tl;
    bool tg_play_status_registered;

    found_t found[FOUND_MAX];
    uint32_t found_count;
    bool name_query_active;

    // connection target (user or automatic reconnect)
    bool want_connect;
    uint8_t target[6];
    uint32_t attempts;
    uint32_t max_attempts;
    int64_t next_attempt_us;
    bool pending_connect;  // connect `target` once discovery stopped / the old link is down

    xq_devices_t store;
    outbox_t out;
} s_bt;

static int64_t now_us(void) { return esp_timer_get_time(); }

// Player volume as last handed to the sink (audio task writes, worker reads), in 1/100 dB.
#define WANT_NONE INT32_MIN
static _Atomic int32_t s_want_cdb = WANT_NONE;
static atomic_bool s_abs_active;  // mirror of s_bt.abs_active for the audio task

static int32_t cdb_from_db(float db) {
    if (!(db > -120.0f)) return -12000;  // mute, -inf
    if (db > 0.0f) db = 0.0f;
    return (int32_t)lroundf(db * 100.0f);
}

static void set_media(media_state_t m) {
    s_bt.media = m;
    s_bt.media_since_us = esp_timer_get_time();
}
static bool addr_eq(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }

// ------------------------------------------------------------ outgoing notes ----
// Events and remote commands are collected while the lock is held and delivered after it
// is released, so a subscriber may call back into this module.

static a2dp_xq_event_t *emit_event(a2dp_xq_event_type_t type) {
    outbox_t *o = &s_bt.out;
    if (!s_bt.event_cb || o->event_count >= sizeof o->events / sizeof o->events[0]) return NULL;
    a2dp_xq_event_t *ev = &o->events[o->event_count++];
    memset(ev, 0, sizeof *ev);
    ev->type = type;
    ev->state = s_bt.state;
    return ev;
}

__attribute__((unused)) static void emit_remote(a2dp_xq_remote_t cmd, int value) {
    outbox_t *o = &s_bt.out;
    if (!s_bt.remote_cb || o->remote_count >= sizeof o->remote / sizeof o->remote[0]) return;
    o->remote[o->remote_count].cmd = cmd;
    o->remote[o->remote_count].value = value;
    o->remote_count++;
}

static void flush_outbox(void) {
    outbox_t o = s_bt.out;  // worker-only data: copy then clear
    s_bt.out.event_count = 0;
    s_bt.out.remote_count = 0;
    for (uint32_t i = 0; i < o.event_count; i++) {
        if (s_bt.event_cb) s_bt.event_cb(&o.events[i], s_bt.event_user);
    }
    for (uint32_t i = 0; i < o.remote_count; i++) {
        if (s_bt.remote_cb) s_bt.remote_cb(o.remote[i].cmd, o.remote[i].value, s_bt.remote_user);
    }
}

static void update_state(void) {
    a2dp_xq_state_t st = s_bt.connected ? A2DP_XQ_STATE_CONNECTED
                    : s_bt.connecting ? A2DP_XQ_STATE_CONNECTING
                    : s_bt.scanning ? A2DP_XQ_STATE_SCANNING
                                    : A2DP_XQ_STATE_IDLE;
    if (st != s_bt.state) {
        s_bt.state = st;
        emit_event(A2DP_XQ_EVENT_STATE);
    }
}

static void save_store(void) {
    if (s_bt.state_dir[0]) xq_devices_save(&s_bt.store, s_bt.state_dir);
}

static found_t *found_find(const uint8_t *addr) {
    for (uint32_t i = 0; i < s_bt.found_count; i++) {
        if (addr_eq(s_bt.found[i].addr, addr)) return &s_bt.found[i];
    }
    return NULL;
}

static const char *known_name(const uint8_t *addr) {
    const xq_device_rec_t *r = xq_devices_find(&s_bt.store, addr);
    if (r && r->name[0]) return r->name;
    const found_t *f = found_find(addr);
    if (f && f->name[0]) return f->name;
    return NULL;
}

__attribute__((unused)) static uint8_t next_tl(void) {
    s_bt.tl = (uint8_t)((s_bt.tl + 1u) % TL_COUNT);
    return s_bt.tl;
}

// ----------------------------------------------------------- queue plumbing ----

static bool post_msg(const bt_msg_t *m, TickType_t wait) {
    QueueHandle_t q = s_bt.queue;
    if (!q) return false;
    if (xQueueSend(q, m, wait) != pdTRUE) {
        ESP_LOGW(TAG, "event queue full, message %u dropped", (unsigned)m->type);
        return false;
    }
    return true;
}

static void post_cmd(msg_type_t type, const uint8_t *addr, bool flag, uint8_t volume) {
    if (!s_bt.initialized) return;
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = (uint8_t)type;
    if (addr) memcpy(m.u.addr, addr, 6);
    if (type == MSG_CMD_SCAN || type == MSG_CMD_SINK_OPEN || type == MSG_CMD_SINK_PAUSED) m.u.flag = flag;
    if (type == MSG_CMD_VOLUME) m.u.volume = volume;
    post_msg(&m, pdMS_TO_TICKS(100));
}

// --------------------------------------------------------- stack callbacks ----
// These run in Bluedroid tasks: copy and post, nothing else (except the pairing replies,
// which the stack expects from the callback).

static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    EventGroupHandle_t eg = s_bt.events;
    if (event == ESP_A2D_PROF_STATE_EVT) {
        if (eg) {
            xEventGroupSetBits(eg, param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS ? EVB_A2D_INIT
                                                                                            : EVB_A2D_DEINIT);
        }
        return;
    }
#if XQ_HAVE_ALL_CAPS_EVT
    if (event == ESP_A2D_REPORT_SNK_ALL_CODEC_CAPS_EVT) return;  // pointer payload; the selected
                                                                 // sink's caps come separately
#endif
    if (event == ESP_A2D_CONNECTION_STATE_EVT && param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED &&
        eg) {
        xEventGroupSetBits(eg, EVB_DISCONNECTED);
    }
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_A2D;
    m.event = (uint16_t)event;
    m.u.a2d = *param;
    post_msg(&m, 0);
}

static void copy_bytes_name(char *dst, const uint8_t *src, size_t len) {
    if (len >= XQ_NAME_MAX) len = XQ_NAME_MAX - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static void parse_disc_prop(disc_info_t *d, const esp_bt_gap_dev_prop_t *pr) {
    if (!pr->val || pr->len <= 0) return;
    switch (pr->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            if (pr->len >= (int)sizeof(uint32_t)) {
                memcpy(&d->cod, pr->val, sizeof(uint32_t));
                d->has_cod = true;
            }
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            d->rssi = *(const int8_t *)pr->val;
            d->has_rssi = true;
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            if (!d->name[0]) copy_bytes_name(d->name, (const uint8_t *)pr->val, (size_t)pr->len);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            if (!d->name[0]) {
                uint8_t *eir = (uint8_t *)pr->val;
                uint8_t n = 0;
                uint8_t *name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &n);
                if (!name) name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &n);
                if (name && n) copy_bytes_name(d->name, name, n);
            }
            break;
        default: break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *p) {
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            m.type = MSG_DISC_RES;
            memcpy(m.u.disc.addr, p->disc_res.bda, 6);
            for (int i = 0; i < p->disc_res.num_prop; i++) parse_disc_prop(&m.u.disc, &p->disc_res.prop[i]);
            break;
        }
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            m.type = MSG_DISC_STATE;
            m.u.disc_state.started = p->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED;
            break;
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            m.type = MSG_REMOTE_NAME;
            memcpy(m.u.named.addr, p->read_rmt_name.bda, 6);
            m.u.named.ok = p->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS;
            copy_bytes_name(m.u.named.name, p->read_rmt_name.rmt_name, strnlen((const char *)p->read_rmt_name.rmt_name,
                                                                                ESP_BT_GAP_MAX_BDNAME_LEN));
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            m.type = MSG_AUTH;
            memcpy(m.u.named.addr, p->auth_cmpl.bda, 6);
            m.u.named.ok = p->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS;
            copy_bytes_name(m.u.named.name, p->auth_cmpl.device_name,
                            strnlen((const char *)p->auth_cmpl.device_name, ESP_BT_GAP_MAX_BDNAME_LEN));
            break;
        // SSP is on at run time (esp_bluedroid_config_t.ssp_en, IDF 6.x has no Kconfig switch
        // for it any more): without these replies every pairing would time out.
        case ESP_BT_GAP_CFM_REQ_EVT:
            // We declare NoInputNoOutput, so SSP runs "just works": accept.
            esp_bt_gap_ssp_confirm_reply(p->cfm_req.bda, true);
            return;
        case ESP_BT_GAP_KEY_REQ_EVT:
            // Passkey entry is impossible without a keyboard on our side.
            esp_bt_gap_ssp_passkey_reply(p->key_req.bda, false, 0);
            return;
        case ESP_BT_GAP_PIN_REQ_EVT: {
            // Legacy pairing (pre-2.1 headphones): the near-universal fixed PIN 0000.
            esp_bt_pin_code_t pin;
            memset(pin, '0', sizeof pin);
            esp_bt_gap_pin_reply(p->pin_req.bda, true, p->pin_req.min_16_digit ? 16 : 4, pin);
            return;
        }
        default: return;
    }
    post_msg(&m, 0);
}

#if XQ_AVRCP
static void ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    EventGroupHandle_t eg = s_bt.events;
    if (event == ESP_AVRC_CT_PROF_STATE_EVT) {
        if (!eg) return;
        esp_avrc_init_state_t st = param->avrc_ct_init_stat.state;
        if (st == ESP_AVRC_INIT_SUCCESS || st == ESP_AVRC_INIT_ALREADY) xEventGroupSetBits(eg, EVB_CT_INIT);
        if (st == ESP_AVRC_DEINIT_SUCCESS || st == ESP_AVRC_DEINIT_ALREADY) xEventGroupSetBits(eg, EVB_CT_DEINIT);
        return;
    }
    switch (event) {
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
        case ESP_AVRC_CT_REMOTE_FEATURES_EVT: break;
        default: return;  // metadata/cover art carry pointers and are not used
    }
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_CT;
    m.event = (uint16_t)event;
    m.u.ct = *param;
    post_msg(&m, 0);
}

static void tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    EventGroupHandle_t eg = s_bt.events;
    if (event == ESP_AVRC_TG_PROF_STATE_EVT) {
        if (!eg) return;
        esp_avrc_init_state_t st = param->avrc_tg_init_stat.state;
        if (st == ESP_AVRC_INIT_SUCCESS || st == ESP_AVRC_INIT_ALREADY) xEventGroupSetBits(eg, EVB_TG_INIT);
        if (st == ESP_AVRC_DEINIT_SUCCESS || st == ESP_AVRC_DEINIT_ALREADY) xEventGroupSetBits(eg, EVB_TG_DEINIT);
        return;
    }
    switch (event) {
        case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
        case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: break;
        default: return;  // player application settings carry pointers and are not supported
    }
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_TG;
    m.event = (uint16_t)event;
    m.u.tg = *param;
    post_msg(&m, 0);
}
#endif  // XQ_AVRCP

// ----------------------------------------------------------- media stream ----

static bool want_stream(void) {
    return s_bt.connected && s_bt.sink_open && !s_bt.sink_paused && (s_bt.xq == XQ_ACTIVE || s_bt.xq == XQ_OFF);
}

static void maybe_start_stream(void) {
    if (!want_stream() || s_bt.media != MEDIA_IDLE || now_us() < s_bt.start_retry_at_us) return;
    if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) == ESP_OK) {
        set_media(MEDIA_CHECKING);
    } else {
        s_bt.start_retry_at_us = now_us() + (int64_t)START_RETRY_MS * 1000;
    }
}

static void suspend_stream(void) {
    if (s_bt.media != MEDIA_STARTED && s_bt.media != MEDIA_STARTING) return;
    if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND) == ESP_OK) set_media(MEDIA_SUSPENDING);
}

#if XQ_AVRCP
static void tg_send_play_status(esp_avrc_rn_rsp_t rsp) {
    esp_avrc_rn_param_t p;
    memset(&p, 0, sizeof p);
    p.playback = (s_bt.sink_open && !s_bt.sink_paused) ? ESP_AVRC_PLAYBACK_PLAYING : ESP_AVRC_PLAYBACK_PAUSED;
    esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE, rsp, &p);
}
#endif

static void play_status_changed(void) {
#if XQ_AVRCP
    if (s_bt.tg_play_status_registered) {
        tg_send_play_status(ESP_AVRC_RN_RSP_CHANGED);
        s_bt.tg_play_status_registered = false;  // the controller registers again
    }
#endif
}

// ----------------------------------------------------------------- SBC-XQ ----

#if XQ_HAVE_PREF_MCC
static void cie_to_esp(const xq_sbc_cie_t *c, esp_a2d_mcc_t *mcc) {
    memset(mcc, 0, sizeof *mcc);
    mcc->type = ESP_A2D_MCT_SBC;
    mcc->cie.sbc_info.samp_freq = c->samp_freq & 0x0F;
    mcc->cie.sbc_info.ch_mode = c->ch_mode & 0x0F;
    mcc->cie.sbc_info.block_len = c->block_len & 0x0F;
    mcc->cie.sbc_info.num_subbands = c->num_subbands & 0x03;
    mcc->cie.sbc_info.alloc_mthd = c->alloc_mthd & 0x03;
    mcc->cie.sbc_info.min_bitpool = c->min_bitpool;
    mcc->cie.sbc_info.max_bitpool = c->max_bitpool;
}
#endif


// The stream configuration as the stack reports it: separate fields since ESP-IDF 5.5, the
// 4 bytes of the codec information element before.
#if XQ_HAVE_CONN_HDL
static void cie_from_esp(const esp_a2d_cie_sbc_t *e, xq_sbc_cie_t *c) {
    c->samp_freq = e->samp_freq;
    c->ch_mode = e->ch_mode;
    c->block_len = e->block_len;
    c->num_subbands = e->num_subbands;
    c->alloc_mthd = e->alloc_mthd;
    c->min_bitpool = e->min_bitpool;
    c->max_bitpool = e->max_bitpool;
}
#define CIE_FROM_MCC(mcc, out) cie_from_esp(&(mcc).cie.sbc_info, (out))
#else
#define CIE_FROM_MCC(mcc, out) xq_sbc_cie_from_bytes((mcc).cie.sbc, (out))
#endif

// The configuration the stack uses when no preference is set: Joint Stereo 16/8 loudness,
// bitpool range intersected with the sink (bta_av_co.c, btc_av_sbc_default_config).
static void default_cfg(xq_sbc_cie_t *c) {
    xq_sbc_joint_config(c);
    if (s_bt.have_caps) {
        if (s_bt.caps.min_bitpool > c->min_bitpool) c->min_bitpool = s_bt.caps.min_bitpool;
        if (s_bt.caps.max_bitpool < c->max_bitpool) c->max_bitpool = s_bt.caps.max_bitpool;
    }
}

static void xq_off(void) {
    s_bt.xq = XQ_OFF;
    if (!s_bt.have_cfg) {
        default_cfg(&s_bt.cfg);
        s_bt.have_cfg = true;
    }
    emit_event(A2DP_XQ_EVENT_CODEC);
    maybe_start_stream();
}

#if XQ_HAVE_PREF_MCC
static void decide_xq(void) {
    if (s_bt.xq != XQ_UNDECIDED || !s_bt.connected) return;
    xq_device_rec_t *rec = xq_devices_find(&s_bt.store, s_bt.peer);
    bool allowed = s_bt.allow_xq && s_bt.have_caps && xq_sbc_caps_allow_xq(&s_bt.caps) &&
                   xq_devices_xq_allowed(rec, s_bt.peer_name);
    if (!allowed) {
        ESP_LOGI(TAG, "SBC-XQ not used (%s)",
                 !s_bt.allow_xq ? "disabled" : !s_bt.have_caps ? "no sink caps"
                 : !xq_sbc_caps_allow_xq(&s_bt.caps) ? "sink caps" : "device memory/blacklist");
        xq_off();
        return;
    }
    xq_sbc_cie_t want;
    xq_sbc_xq_config(&want);
    esp_a2d_mcc_t mcc;
    cie_to_esp(&want, &mcc);
    esp_err_t err = esp_a2d_source_set_pref_mcc(s_bt.conn_hdl, &mcc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_pref_mcc: %s", esp_err_to_name(err));
        xq_off();
        return;
    }
    s_bt.xq = XQ_PENDING;
    s_bt.xq_deadline_us = now_us() + (int64_t)PREF_MCC_TIMEOUT_MS * 1000;
}
#endif  // XQ_HAVE_PREF_MCC

static void xq_verdict(bool ok, bool hard_fail) {
    xq_device_rec_t *rec = xq_devices_get(&s_bt.store, s_bt.peer);
    if (!rec) return;
    xq_devices_xq_result(&s_bt.store, rec, ok);
    if (!ok && hard_fail) rec->xq = XQ_VERDICT_FAIL;
    s_bt.xq_verdict_done = true;
    ESP_LOGI(TAG, "SBC-XQ %s for %s", ok ? "works" : (rec->xq == XQ_VERDICT_FAIL ? "disabled" : "failed once"),
             s_bt.peer_name);
    save_store();
}

// ------------------------------------------------------------- connections ----

static void begin_connect(const uint8_t *addr, uint32_t max_attempts) {
    memcpy(s_bt.target, addr, 6);
    s_bt.want_connect = true;
    s_bt.attempts = 0;
    s_bt.max_attempts = max_attempts;
    s_bt.next_attempt_us = now_us();
    s_bt.pending_connect = false;
}

static void try_connect(void) {
    if (!s_bt.want_connect || s_bt.connected || s_bt.connecting) return;
    if (s_bt.scanning) {
        esp_bt_gap_cancel_discovery();
        s_bt.pending_connect = true;  // resumes on ESP_BT_GAP_DISCOVERY_STOPPED
        return;
    }
    if (s_bt.attempts >= s_bt.max_attempts) {
        s_bt.want_connect = false;
        ESP_LOGI(TAG, "giving up connecting after %u attempts", (unsigned)s_bt.attempts);
        return;
    }
    uint8_t bda[6];
    memcpy(bda, s_bt.target, 6);
    esp_err_t err = esp_a2d_source_connect(bda);
    s_bt.attempts++;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "connect: %s", esp_err_to_name(err));
        uint32_t i = s_bt.attempts - 1;
        if (i >= sizeof k_reconnect_delay_ms / sizeof k_reconnect_delay_ms[0])
            i = sizeof k_reconnect_delay_ms / sizeof k_reconnect_delay_ms[0] - 1;
        s_bt.next_attempt_us = now_us() + (int64_t)k_reconnect_delay_ms[i] * 1000;
        return;
    }
    s_bt.connecting = true;
    s_bt.next_attempt_us = now_us() + (int64_t)CONNECT_TIMEOUT_MS * 1000;
    const char *name = known_name(bda);
    char addr_str[18];
    xq_addr_format(bda, addr_str);
    ESP_LOGI(TAG, "connecting to %s (%s), attempt %u", name ? name : "?", addr_str, (unsigned)s_bt.attempts);
    update_state();
}

static void schedule_retry(void) {
    if (!s_bt.want_connect) return;
    uint32_t i = s_bt.attempts ? s_bt.attempts - 1 : 0;
    if (i >= sizeof k_reconnect_delay_ms / sizeof k_reconnect_delay_ms[0])
        i = sizeof k_reconnect_delay_ms / sizeof k_reconnect_delay_ms[0] - 1;
    s_bt.next_attempt_us = now_us() + (int64_t)k_reconnect_delay_ms[i] * 1000;
}

// ---------------------------------------------------------------- volume ----

static void volume_reset_abs(void) {
    s_bt.abs_active = false;
    s_bt.abs_pending = false;
    s_bt.abs_sent_valid = false;
    s_bt.abs_tries = 0;
}

// Worker: bring the headphones' absolute volume and the sink gain in line with the player
// volume. Called after every volume-related event and from the periodic tick.
static void volume_sync(void) {
    int32_t cdb = atomic_load(&s_want_cdb);
    if (cdb == WANT_NONE) return;  // the player has not set a volume yet
    float db = (float)cdb / 100.0f;
    bool abs = s_bt.connected && s_bt.ct_connected && s_bt.abs_volume;
    if (!abs) volume_reset_abs();
#if XQ_AVRCP
    if (abs) {
        uint8_t v = a2dp_xq_volume_from_db(db);
        int64_t now = now_us();
        bool unanswered = s_bt.abs_pending && now - s_bt.abs_sent_us > (int64_t)ABS_VOLUME_ACK_MS * 1000;
        bool give_up = unanswered && ++s_bt.abs_tries >= ABS_VOLUME_TRIES;
        if (give_up && !s_bt.abs_active) {
            // Never acknowledged: keep attenuating in the sink (the safe side if they did apply it).
            ESP_LOGW(TAG, "absolute volume not acknowledged: the player attenuates itself");
            s_bt.abs_volume = false;
            volume_reset_abs();
        } else if (give_up) {
            // They took absolute volume before and stopped answering: stop asking until the
            // player volume changes again.
            s_bt.abs_pending = false;
            s_bt.abs_tries = 0;
        } else if (!s_bt.abs_sent_valid || v != s_bt.abs_sent || unanswered) {
            if (esp_avrc_ct_send_set_absolute_volume_cmd(next_tl(), v) == ESP_OK) {
                s_bt.abs_sent = v;
                s_bt.abs_sent_valid = true;
                s_bt.abs_pending = true;
                s_bt.abs_sent_us = now;
            }
        }
    }
#endif
    // Until the headphones acknowledged absolute volume the sink attenuates itself.
    bool active = s_bt.abs_active && s_bt.abs_volume;
    atomic_store(&s_abs_active, active);
    xq_sink_set_gain_db(active ? 0.0f : db);
}

#if XQ_AVRCP
// The headphones' own volume changed (their buttons): it becomes the player volume.
static void remote_volume_changed(uint8_t v, bool proves_abs) {
    s_bt.remote_volume = v;
    if (proves_abs && s_bt.abs_volume) {
        s_bt.abs_sent = v;  // they are at v now: nothing to send back
        s_bt.abs_sent_valid = true;
        s_bt.abs_pending = false;
        s_bt.abs_tries = 0;
        s_bt.abs_active = true;
    }
    // Take it as the wanted volume right away, so volume_sync() does not push the old one
    // back before the player (which may clamp it to its limit) answers.
    atomic_store(&s_want_cdb, cdb_from_db(a2dp_xq_volume_to_db(v)));
    emit_remote(A2DP_XQ_REMOTE_VOLUME_ABS, v);
    volume_sync();
}
#endif

static void on_connected(const esp_a2d_cb_param_t *p) {
    s_bt.connected = true;
    s_bt.connecting = false;
    memcpy(s_bt.peer, p->conn_stat.remote_bda, 6);
#if XQ_HAVE_CONN_HDL
    s_bt.conn_hdl = p->conn_stat.conn_hdl;
#endif
    s_bt.connected_at_us = now_us();
    set_media(MEDIA_IDLE);
    s_bt.start_retry_at_us = 0;
    s_bt.start_fails = 0;
    s_bt.have_caps = false;
    s_bt.have_cfg = false;
    s_bt.xq = XQ_UNDECIDED;
    s_bt.xq_verdict_done = false;
    s_bt.stream_started_us = 0;
    s_bt.sink_delay_ms = 0;

    const char *name = known_name(s_bt.peer);
    char addr_str[18];
    xq_addr_format(s_bt.peer, addr_str);
    xq_strlcpy(s_bt.peer_name, name ? name : addr_str, sizeof s_bt.peer_name);
    // Audio still queued for other headphones was made at their gain (0 dB when they had
    // absolute volume): drop it rather than play it at full level on these.
    if (s_bt.have_prev_peer && !addr_eq(s_bt.prev_peer, s_bt.peer)) xq_sink_discard();
    memcpy(s_bt.prev_peer, s_bt.peer, 6);
    s_bt.have_prev_peer = true;
    volume_reset_abs();
    volume_sync();

    xq_device_rec_t *rec = xq_devices_get(&s_bt.store, s_bt.peer);
    if (rec) {
        if (name) xq_devices_set_name(&s_bt.store, rec, name);
        xq_devices_touch(&s_bt.store, rec);
    }
    save_store();
    // From now on a link loss reconnects to this device.
    memcpy(s_bt.target, s_bt.peer, 6);
    s_bt.want_connect = false;
    s_bt.attempts = 0;
#if XQ_HAVE_CONN_HDL
    ESP_LOGI(TAG, "connected to %s (%s), mtu %u", s_bt.peer_name, addr_str, (unsigned)p->conn_stat.audio_mtu);
#else
    ESP_LOGI(TAG, "connected to %s (%s)", s_bt.peer_name, addr_str);
#endif
    update_state();
#if !XQ_HAVE_PREF_MCC
    // No sink capability reports before ESP-IDF 6.0: the stack's own configuration it is.
    ESP_LOGI(TAG, "SBC-XQ needs ESP-IDF 6.0 or newer: standard SBC");
    xq_off();
#endif
}

static void on_disconnected(const esp_a2d_cb_param_t *p) {
    bool was_connected = s_bt.connected;
    bool abnormal = p->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_ABNORMAL;
    if (was_connected && s_bt.xq == XQ_ACTIVE && !s_bt.xq_verdict_done && abnormal && s_bt.stream_started_us &&
        now_us() - s_bt.stream_started_us < (int64_t)XQ_FAIL_WINDOW_MS * 1000) {
        xq_verdict(false, false);
    }
    s_bt.connected = false;
    s_bt.connecting = false;
    set_media(MEDIA_IDLE);
    s_bt.xq = XQ_UNDECIDED;
    s_bt.ct_connected = false;
    s_bt.abs_volume = false;
    s_bt.tg_play_status_registered = false;
    volume_sync();  // the sink attenuates again

    if (s_bt.pending_connect && s_bt.want_connect) {
        // Switching to another device: connect right away.
        s_bt.pending_connect = false;
        s_bt.next_attempt_us = now_us();
    } else if (was_connected) {
        ESP_LOGI(TAG, "disconnected from %s (%s)", s_bt.peer_name, abnormal ? "link lost" : "normal");
        if (abnormal && s_bt.auto_reconnect) {
            begin_connect(s_bt.peer, AUTO_RECONNECT_ATTEMPTS);
            s_bt.next_attempt_us = now_us() + (int64_t)k_reconnect_delay_ms[0] * 1000;
        }
    } else {
        schedule_retry();  // a connection attempt failed
    }
    update_state();
}

static void handle_a2d(uint16_t event, const esp_a2d_cb_param_t *p) {
    switch ((esp_a2d_cb_event_t)event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            switch (p->conn_stat.state) {
                case ESP_A2D_CONNECTION_STATE_CONNECTING:
                    s_bt.connecting = true;
                    update_state();
                    break;
                case ESP_A2D_CONNECTION_STATE_CONNECTED: on_connected(p); break;
                case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
                    // While connected, btc_av.c reports a refused incoming connection from a
                    // second device as DISCONNECTED with that device's address: not our link.
                    if (s_bt.connected && !addr_eq(p->conn_stat.remote_bda, s_bt.peer)) {
                        ESP_LOGI(TAG, "ignored connection attempt from another device");
                        break;
                    }
                    on_disconnected(p);
                    break;
                default: break;
            }
            break;
        case ESP_A2D_AUDIO_STATE_EVT:
            if (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
                set_media(MEDIA_STARTED);
                s_bt.start_fails = 0;
                if (!s_bt.stream_started_us) s_bt.stream_started_us = now_us();
            } else {
                bool ours = s_bt.media == MEDIA_SUSPENDING;
                set_media(MEDIA_IDLE);
                // Suspended by the headphones: try again later if we still have audio to play.
                if (!ours) s_bt.start_retry_at_us = now_us() + (int64_t)REMOTE_SUSPEND_RETRY_MS * 1000;
            }
            break;
        case ESP_A2D_AUDIO_CFG_EVT:
            if (p->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
                CIE_FROM_MCC(p->audio_cfg.mcc, &s_bt.cfg);
                s_bt.have_cfg = true;
                emit_event(A2DP_XQ_EVENT_CODEC);
            }
            break;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT: {
            bool ok = p->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS;
            switch (p->media_ctrl_stat.cmd) {
                case ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY:
                    if (ok && want_stream() && esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START) == ESP_OK) {
                        set_media(MEDIA_STARTING);
                    } else {
                        set_media(MEDIA_IDLE);
                        if (!ok) s_bt.start_retry_at_us = now_us() + (int64_t)START_RETRY_MS * 1000;
                    }
                    break;
                case ESP_A2D_MEDIA_CTRL_START:
                    if (ok) {
                        set_media(MEDIA_STARTED);
                        s_bt.start_fails = 0;
                        if (!s_bt.stream_started_us) s_bt.stream_started_us = now_us();
                    } else {
                        set_media(MEDIA_IDLE);
                        s_bt.start_fails++;
                        s_bt.start_retry_at_us = now_us() + (int64_t)START_RETRY_MS * 1000;
                        if (s_bt.xq == XQ_ACTIVE && s_bt.start_fails >= 2 && !s_bt.xq_verdict_done) {
                            // The sink took the XQ configuration but will not stream it: remember,
                            // reconnect, and the next connection uses Joint Stereo.
                            xq_verdict(false, true);
                            uint8_t bda[6];
                            memcpy(bda, s_bt.peer, 6);
                            begin_connect(bda, AUTO_RECONNECT_ATTEMPTS);
                            s_bt.pending_connect = true;
                            esp_a2d_source_disconnect(bda);
                        } else if (s_bt.start_fails >= START_FAILS_MAX) {
                            ESP_LOGW(TAG, "stream start refused %u times", (unsigned)s_bt.start_fails);
                        }
                    }
                    break;
                case ESP_A2D_MEDIA_CTRL_SUSPEND:
                    if (ok) set_media(MEDIA_IDLE);
                    break;
                default: break;
            }
            break;
        }
#if XQ_HAVE_PREF_MCC
        case ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT:
            if (p->a2d_report_snk_codec_caps_stat.mcc.type == ESP_A2D_MCT_SBC) {
                cie_from_esp(&p->a2d_report_snk_codec_caps_stat.mcc.cie.sbc_info, &s_bt.caps);
                s_bt.have_caps = true;
                esp_a2d_conn_hdl_t hdl = p->a2d_report_snk_codec_caps_stat.conn_hdl;
                if (hdl) s_bt.conn_hdl = hdl;
                ESP_LOGI(TAG, "sink SBC caps: sf %x ch %x blk %x sb %x alloc %x bitpool %u..%u", s_bt.caps.samp_freq,
                         s_bt.caps.ch_mode, s_bt.caps.block_len, s_bt.caps.num_subbands, s_bt.caps.alloc_mthd,
                         s_bt.caps.min_bitpool, s_bt.caps.max_bitpool);
                decide_xq();
            }
            break;
        case ESP_A2D_SRC_SET_PREF_MCC_EVT: {
            esp_bt_status_t st = p->a2d_set_pref_mcc_stat.set_status;
            if (s_bt.xq != XQ_PENDING) break;
            if (st == ESP_BT_STATUS_SUCCESS) {
                s_bt.xq = XQ_ACTIVE;
                xq_sbc_xq_config(&s_bt.cfg);
                s_bt.have_cfg = true;
                ESP_LOGI(TAG, "SBC-XQ active (Dual Channel, bitpool %u)", XQ_SBC_XQ_BITPOOL);
                emit_event(A2DP_XQ_EVENT_CODEC);
                maybe_start_stream();
            } else {
                ESP_LOGW(TAG, "SBC-XQ refused (status %d), Joint Stereo", (int)st);
                // FAIL means the sink rejected the reconfiguration; busy / not ready are timing.
                if (st == ESP_BT_STATUS_FAIL) xq_verdict(false, true);
                xq_off();
            }
            break;
        }
#endif  // XQ_HAVE_PREF_MCC
        case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT:
            s_bt.sink_delay_ms = (uint16_t)(p->a2d_report_delay_value_stat.delay_value / 10u);
            break;
        default: break;
    }
}

#if XQ_AVRCP
static void register_volume_notification(void) {
    if (s_bt.abs_volume) esp_avrc_ct_send_register_notification_cmd(next_tl(), ESP_AVRC_RN_VOLUME_CHANGE, 0);
}

static void handle_ct(uint16_t event, esp_avrc_ct_cb_param_t *p) {
    switch ((esp_avrc_ct_cb_event_t)event) {
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
            s_bt.ct_connected = p->conn_stat.connected;
            s_bt.abs_volume = false;
            volume_reset_abs();
            volume_sync();
            if (s_bt.ct_connected) esp_avrc_ct_send_get_rn_capabilities_cmd(next_tl());
            break;
        case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
            esp_avrc_rn_evt_cap_mask_t caps = p->get_rn_caps_rsp.evt_set;
            s_bt.abs_volume = esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &caps,
                                                                 ESP_AVRC_RN_VOLUME_CHANGE);
            ESP_LOGI(TAG, "AVRCP absolute volume %s", s_bt.abs_volume ? "supported" : "not supported");
            register_volume_notification();
            volume_reset_abs();
            volume_sync();  // first SetAbsoluteVolume with the player volume
            break;
        }
        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
            if (p->change_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
                remote_volume_changed(p->change_ntf.event_parameter.volume & 0x7F, true);
                register_volume_notification();  // notifications are one-shot
            }
            break;
        case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
            // Accepted: from now on the headphones apply the volume and the sink plays at 0 dB.
            s_bt.remote_volume = p->set_volume_rsp.volume & 0x7F;
            s_bt.abs_pending = false;
            s_bt.abs_tries = 0;
            s_bt.abs_active = s_bt.abs_volume;
            volume_sync();
            break;
        default: break;
    }
}

static void handle_tg(uint16_t event, esp_avrc_tg_cb_param_t *p) {
    switch ((esp_avrc_tg_cb_event_t)event) {
        case ESP_AVRC_TG_CONNECTION_STATE_EVT:
            if (!p->conn_stat.connected) s_bt.tg_play_status_registered = false;
            break;
        case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
            if (p->psth_cmd.key_state != ESP_AVRC_PT_CMD_STATE_PRESSED) break;
            switch (p->psth_cmd.key_code) {
                case ESP_AVRC_PT_CMD_PLAY: emit_remote(A2DP_XQ_REMOTE_PLAY, 0); break;
                case ESP_AVRC_PT_CMD_PAUSE: emit_remote(A2DP_XQ_REMOTE_PAUSE, 0); break;
                case ESP_AVRC_PT_CMD_STOP: emit_remote(A2DP_XQ_REMOTE_STOP, 0); break;
                case ESP_AVRC_PT_CMD_FORWARD: emit_remote(A2DP_XQ_REMOTE_NEXT, 0); break;
                case ESP_AVRC_PT_CMD_BACKWARD: emit_remote(A2DP_XQ_REMOTE_PREV, 0); break;
                case ESP_AVRC_PT_CMD_VOL_UP: emit_remote(A2DP_XQ_REMOTE_VOL_UP, 0); break;
                case ESP_AVRC_PT_CMD_VOL_DOWN: emit_remote(A2DP_XQ_REMOTE_VOL_DOWN, 0); break;
                case ESP_AVRC_PT_CMD_MUTE: emit_remote(A2DP_XQ_REMOTE_PLAY_PAUSE, 0); break;
                default: break;
            }
            break;
        }
        case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
            // The remote sets our volume (unusual for headphones): take it as the player volume.
            remote_volume_changed(p->set_abs_vol.volume & 0x7F, false);
            break;
        case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
            if (p->reg_ntf.event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE) {
                s_bt.tg_play_status_registered = true;
                tg_send_play_status(ESP_AVRC_RN_RSP_INTERIM);
            }
            break;
        default: break;
    }
}
#endif  // XQ_AVRCP

// -------------------------------------------------------------------- GAP ----

static bool cod_may_be_sink(const disc_info_t *d) {
    if (!d->has_cod || !esp_bt_gap_is_valid_cod(d->cod)) return true;  // unknown: let the name decide
    uint32_t major = esp_bt_gap_get_cod_major_dev(d->cod);
    uint32_t srvc = esp_bt_gap_get_cod_srvc(d->cod);
    if (major == ESP_BT_COD_MAJOR_DEV_AV) return true;
    if (major == ESP_BT_COD_MAJOR_DEV_PHONE || major == ESP_BT_COD_MAJOR_DEV_COMPUTER) return false;
    return (srvc & (ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO)) != 0;
}

static bool is_bonded(const uint8_t *addr) {
    esp_bd_addr_t list[XQ_DEVICES_MAX];
    int n = XQ_DEVICES_MAX;
    if (esp_bt_gap_get_bond_device_list(&n, list) != ESP_OK) return false;
    for (int i = 0; i < n && i < XQ_DEVICES_MAX; i++) {
        if (addr_eq(list[i], addr)) return true;
    }
    return false;
}

static void announce_found(const found_t *f) {
    a2dp_xq_event_t *ev = emit_event(A2DP_XQ_EVENT_DEVICE_FOUND);
    if (!ev) return;
    xq_addr_format(f->addr, ev->device.addr);
    xq_strlcpy(ev->device.name, f->name[0] ? f->name : ev->device.addr, sizeof ev->device.name);
    ev->device.rssi = f->rssi;
    ev->device.paired = is_bonded(f->addr);
    ev->device.connected = s_bt.connected && addr_eq(f->addr, s_bt.peer);
}

static void request_next_name(void) {
    if (s_bt.name_query_active || s_bt.scanning) return;
    for (uint32_t i = 0; i < s_bt.found_count; i++) {
        found_t *f = &s_bt.found[i];
        if (f->name[0] || f->name_requested) continue;
        f->name_requested = true;
        uint8_t bda[6];
        memcpy(bda, f->addr, 6);
        if (esp_bt_gap_read_remote_name(bda) == ESP_OK) {
            s_bt.name_query_active = true;
            return;
        }
    }
}

static void handle_disc_res(const disc_info_t *d) {
    if (!cod_may_be_sink(d)) return;
    found_t *f = found_find(d->addr);
    bool is_new = !f;
    if (!f) {
        if (s_bt.found_count < FOUND_MAX) {
            f = &s_bt.found[s_bt.found_count++];
        } else {
            // Replace the weakest nameless entry, else the weakest one.
            f = &s_bt.found[0];
            for (uint32_t i = 1; i < s_bt.found_count; i++) {
                found_t *c = &s_bt.found[i];
                bool better = (!c->name[0] && f->name[0]) || ((!c->name[0]) == (!f->name[0]) && c->rssi < f->rssi);
                if (better) f = c;
            }
        }
        memset(f, 0, sizeof *f);
        memcpy(f->addr, d->addr, 6);
    }
    bool name_new = d->name[0] && strcmp(f->name, d->name) != 0;
    if (d->has_rssi) f->rssi = d->rssi;
    if (d->name[0]) xq_strlcpy(f->name, d->name, sizeof f->name);
    if (!f->name[0]) {
        const char *stored = known_name(d->addr);
        if (stored) xq_strlcpy(f->name, stored, sizeof f->name);
    }
    if (is_new || name_new) announce_found(f);
}

static void handle_remote_name(const uint8_t *addr, bool ok, const char *name) {
    s_bt.name_query_active = false;
    if (ok && name[0]) {
        found_t *f = found_find(addr);
        if (f && strcmp(f->name, name) != 0) {
            xq_strlcpy(f->name, name, sizeof f->name);
            announce_found(f);
        }
        xq_device_rec_t *r = xq_devices_find(&s_bt.store, addr);
        if (r) xq_devices_set_name(&s_bt.store, r, name);
        if (s_bt.connected && addr_eq(addr, s_bt.peer)) xq_strlcpy(s_bt.peer_name, name, sizeof s_bt.peer_name);
    }
    request_next_name();
}

static void handle_auth(const uint8_t *addr, bool ok, const char *name) {
    char addr_str[18];
    xq_addr_format(addr, addr_str);
    if (!ok) {
        ESP_LOGW(TAG, "pairing with %s failed", addr_str);
        a2dp_xq_event_t *ev = emit_event(A2DP_XQ_EVENT_PAIRING_FAILED);
        if (ev) xq_strlcpy(ev->device.addr, addr_str, sizeof ev->device.addr);
        return;
    }
    xq_device_rec_t *r = xq_devices_get(&s_bt.store, addr);
    if (r && name[0]) xq_devices_set_name(&s_bt.store, r, name);
    if (name[0] && s_bt.connected && addr_eq(addr, s_bt.peer)) {
        xq_strlcpy(s_bt.peer_name, name, sizeof s_bt.peer_name);
    }
    save_store();
    ESP_LOGI(TAG, "paired with %s (%s)", name[0] ? name : "?", addr_str);
}

// --------------------------------------------------------------- commands ----

static void cmd_connect(const uint8_t *addr) {
    if (s_bt.connected && addr_eq(addr, s_bt.peer)) return;
    begin_connect(addr, USER_CONNECT_ATTEMPTS);
    if (s_bt.connected) {
        uint8_t bda[6];
        memcpy(bda, s_bt.peer, 6);
        s_bt.pending_connect = true;  // on_disconnected() connects to the target
        esp_a2d_source_disconnect(bda);
        return;
    }
    try_connect();
}

static void cmd_disconnect(void) {
    s_bt.want_connect = false;
    s_bt.pending_connect = false;
    if (s_bt.connected || s_bt.connecting) {
        uint8_t bda[6];
        memcpy(bda, s_bt.connected ? s_bt.peer : s_bt.target, 6);
        suspend_stream();
        esp_a2d_source_disconnect(bda);
    }
}

static void cmd_scan(bool start) {
    if (start) {
        if (s_bt.scanning) return;
        s_bt.found_count = 0;
        s_bt.name_query_active = false;
        esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQUIRY_LEN, 0);
        if (err != ESP_OK) ESP_LOGW(TAG, "start discovery: %s", esp_err_to_name(err));
    } else if (s_bt.scanning) {
        esp_bt_gap_cancel_discovery();
    }
}

static void cmd_forget(const uint8_t *addr) {
    uint8_t bda[6];
    memcpy(bda, addr, 6);
    if ((s_bt.connected || s_bt.connecting) && addr_eq(bda, s_bt.connected ? s_bt.peer : s_bt.target)) {
        s_bt.want_connect = false;
        s_bt.pending_connect = false;
        esp_a2d_source_disconnect(bda);
    }
    if (s_bt.want_connect && addr_eq(bda, s_bt.target)) s_bt.want_connect = false;
    esp_bt_gap_remove_bond_device(bda);
    xq_devices_remove(&s_bt.store, bda);
    save_store();
    found_t *f = found_find(bda);
    if (f) {
        size_t idx = (size_t)(f - s_bt.found);
        memmove(&s_bt.found[idx], &s_bt.found[idx + 1], (s_bt.found_count - idx - 1) * sizeof(found_t));
        s_bt.found_count--;
    }
}

static void link_watch(int64_t now) {
    // Playing into a stream nobody drains: no link, and no reconnect left to wait for.
    bool stalled = s_bt.sink_open && !s_bt.sink_paused && !s_bt.connected && !s_bt.connecting && !s_bt.want_connect;
    if (!stalled) {
        s_bt.stall_since_us = 0;
        s_bt.stall_reported = false;
        return;
    }
    if (!s_bt.stall_since_us) {
        s_bt.stall_since_us = now ? now : 1;
    } else if (!s_bt.stall_reported && now - s_bt.stall_since_us >= (int64_t)LINK_DOWN_PAUSE_MS * 1000) {
        s_bt.stall_reported = true;
        ESP_LOGI(TAG, "no headphones connected: pausing the player");
        emit_remote(A2DP_XQ_REMOTE_LINK_DOWN, 0);
    }
}

// ----------------------------------------------------------------- worker ----

static void handle_msg(bt_msg_t *m) {
    switch ((msg_type_t)m->type) {
        case MSG_A2D: handle_a2d(m->event, &m->u.a2d); break;
#if XQ_AVRCP
        case MSG_CT: handle_ct(m->event, &m->u.ct); break;
        case MSG_TG: handle_tg(m->event, &m->u.tg); break;
#endif
        case MSG_DISC_RES: handle_disc_res(&m->u.disc); break;
        case MSG_DISC_STATE:
            s_bt.scanning = m->u.disc_state.started;
            if (!s_bt.scanning) {
                if (s_bt.pending_connect && s_bt.want_connect && !s_bt.connected) {
                    s_bt.pending_connect = false;
                    s_bt.next_attempt_us = now_us();
                }
                request_next_name();
            }
            update_state();
            break;
        case MSG_REMOTE_NAME: handle_remote_name(m->u.named.addr, m->u.named.ok, m->u.named.name); break;
        case MSG_AUTH: handle_auth(m->u.named.addr, m->u.named.ok, m->u.named.name); break;
        case MSG_CMD_CONNECT: cmd_connect(m->u.addr); break;
        case MSG_CMD_DISCONNECT: cmd_disconnect(); break;
        case MSG_CMD_SCAN: cmd_scan(m->u.flag); break;
        case MSG_CMD_VOLUME: volume_sync(); break;
        case MSG_CMD_FORGET: cmd_forget(m->u.addr); break;
        case MSG_CMD_SINK_OPEN:
            s_bt.sink_open = m->u.flag;
            if (s_bt.sink_open) {
                maybe_start_stream();
            } else {
                suspend_stream();
            }
            play_status_changed();
            break;
        case MSG_CMD_SINK_PAUSED:
            s_bt.sink_paused = m->u.flag;
            s_bt.paused_at_us = now_us();
            if (!s_bt.sink_paused) {
                s_bt.start_retry_at_us = 0;
                maybe_start_stream();
            }
            play_status_changed();
            break;
        default: break;
    }
}

static void periodic(void) {
    int64_t now = now_us();
    link_watch(now);
    volume_sync();  // resends, and the sink gain after any missed message
    // Connection attempts (user request or automatic reconnect).
    if (s_bt.want_connect && !s_bt.connected) {
        if (s_bt.connecting && now >= s_bt.next_attempt_us) {
            ESP_LOGW(TAG, "connection attempt timed out");
            s_bt.connecting = false;
            schedule_retry();
            update_state();
        } else if (!s_bt.connecting && !s_bt.pending_connect && now >= s_bt.next_attempt_us) {
            try_connect();
        }
    }
    if (s_bt.connected) {
        // A media command whose acknowledgement never came: start over.
        if ((s_bt.media == MEDIA_CHECKING || s_bt.media == MEDIA_STARTING || s_bt.media == MEDIA_SUSPENDING) &&
            now - s_bt.media_since_us > (int64_t)MEDIA_ACK_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "media command %d not acknowledged", (int)s_bt.media);
            set_media(MEDIA_IDLE);
        }
        // No sink capabilities reported: decide without them.
        if (s_bt.xq == XQ_UNDECIDED && now - s_bt.connected_at_us > (int64_t)CAPS_WAIT_MS * 1000) xq_off();
        if (s_bt.xq == XQ_PENDING && now > s_bt.xq_deadline_us) {
            ESP_LOGW(TAG, "no answer to the SBC-XQ request, Joint Stereo");
            xq_off();
        }
        if (s_bt.xq == XQ_ACTIVE && !s_bt.xq_verdict_done && s_bt.media == MEDIA_STARTED && s_bt.stream_started_us &&
            now - s_bt.stream_started_us > (int64_t)XQ_OK_AFTER_MS * 1000) {
            xq_verdict(true, false);
        }
        // A long pause: let the headphones sleep their decoder (and save our radio time).
        bool long_pause = now - s_bt.paused_at_us > (int64_t)PAUSE_SUSPEND_MS * 1000;
        if (s_bt.sink_paused && s_bt.media == MEDIA_STARTED && long_pause) suspend_stream();
        maybe_start_stream();
    }
}

static void worker(void *arg) {
    (void)arg;
    bt_msg_t m;
    for (;;) {
        bool got = xQueueReceive(s_bt.queue, &m, pdMS_TO_TICKS(WORKER_TICK_MS)) == pdTRUE;
        if (got && m.type == MSG_CMD_QUIT) break;
        xSemaphoreTake(s_bt.lock, portMAX_DELAY);
        if (got) handle_msg(&m);
        periodic();
        xSemaphoreGive(s_bt.lock);
        flush_outbox();
    }
    xEventGroupSetBits(s_bt.events, EVB_WORKER_DONE);
    vTaskSuspend(NULL);  // deleted by a2dp_xq_deinit()
}

// ------------------------------------------------------------ bring-up ----

static bool wait_bits(EventBits_t bits, uint32_t timeout_ms) {
    EventBits_t got = xEventGroupWaitBits(s_bt.events, bits, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    return (got & bits) == bits;
}

static void teardown(void) {
    // Order: AVRCP before A2DP (esp_avrc_api.h), profiles before Bluedroid, Bluedroid before
    // the controller.
#if XQ_AVRCP
    if (s_bt.ct_inited) {
        xEventGroupClearBits(s_bt.events, EVB_CT_DEINIT);
        if (esp_avrc_ct_deinit() == ESP_OK && !wait_bits(EVB_CT_DEINIT, STACK_OP_TIMEOUT_MS))
            ESP_LOGW(TAG, "AVRCP CT deinit timeout");
        s_bt.ct_inited = false;
    }
    if (s_bt.tg_inited) {
        xEventGroupClearBits(s_bt.events, EVB_TG_DEINIT);
        if (esp_avrc_tg_deinit() == ESP_OK && !wait_bits(EVB_TG_DEINIT, STACK_OP_TIMEOUT_MS))
            ESP_LOGW(TAG, "AVRCP TG deinit timeout");
        s_bt.tg_inited = false;
    }
#endif
    if (s_bt.a2d_inited) {
        xEventGroupClearBits(s_bt.events, EVB_A2D_DEINIT);
        if (esp_a2d_source_deinit() == ESP_OK && !wait_bits(EVB_A2D_DEINIT, STACK_OP_TIMEOUT_MS))
            ESP_LOGW(TAG, "A2DP deinit timeout");
        s_bt.a2d_inited = false;
    }
    if (s_bt.bd_enabled) {
        esp_bluedroid_disable();
        s_bt.bd_enabled = false;
    }
    if (s_bt.bd_inited) {
        esp_bluedroid_deinit();
        s_bt.bd_inited = false;
    }
    if (s_bt.ctrl_enabled) {
        esp_bt_controller_disable();
        s_bt.ctrl_enabled = false;
    }
    if (s_bt.ctrl_inited) {
        esp_bt_controller_deinit();
        s_bt.ctrl_inited = false;
    }
    if (s_bt.sink_ready) {
        xq_sink_deinit();
        s_bt.sink_ready = false;
    }
    if (s_bt.queue) {
        QueueHandle_t q = s_bt.queue;
        s_bt.queue = NULL;
        if (s_bt.queue_caps) {
            vQueueDeleteWithCaps(q);
        } else {
            vQueueDelete(q);
        }
    }
    if (s_bt.events) {
        vEventGroupDelete(s_bt.events);
        s_bt.events = NULL;
    }
    if (s_bt.lock) {
        vSemaphoreDelete(s_bt.lock);
        s_bt.lock = NULL;
    }
}

static int stack_up(void) {
    static bool s_ble_released;
    esp_err_t err;
    if (!s_ble_released && esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        // BR/EDR only: give the BLE controller memory back to the heap (once per boot).
        err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
        if (err != ESP_OK) ESP_LOGW(TAG, "BLE memory release: %s", esp_err_to_name(err));
        s_ble_released = true;
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        err = esp_bt_controller_init(&cfg);
        if (err != ESP_OK) goto fail;
    }
    s_bt.ctrl_inited = true;
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
        if (err != ESP_OK) goto fail;
    }
    s_bt.ctrl_enabled = true;

    esp_bluedroid_status_t bd = esp_bluedroid_get_status();
    if (bd == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        esp_bluedroid_config_t bcfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        bcfg.ssp_en = true;  // Secure Simple Pairing; legacy PIN pairing still answered
        err = esp_bluedroid_init_with_cfg(&bcfg);
        if (err != ESP_OK) goto fail;
    }
    s_bt.bd_inited = true;
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        err = esp_bluedroid_enable();
        if (err != ESP_OK) goto fail;
    }
    s_bt.bd_enabled = true;
    return XQ_OK;
fail:
    ESP_LOGE(TAG, "Bluetooth stack start failed: %s", esp_err_to_name(err));
    return XQ_EIO;
}

static int profiles_up(void) {
    esp_err_t err;
    if ((err = esp_bt_gap_register_callback(gap_cb)) != ESP_OK) goto fail;
    esp_bt_gap_set_device_name(s_bt.device_name);

    // Class of Device: Audio/Video, portable audio, "audio" service.
    esp_bt_cod_t cod;
    memset(&cod, 0, sizeof cod);
    cod.major = ESP_BT_COD_MAJOR_DEV_AV;
    cod.minor = 0x07;  // portable audio (Assigned Numbers, Audio/Video minor class)
    cod.service = ESP_BT_COD_SRVC_AUDIO;
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;  // "just works" (SSP is on: bcfg.ssp_en)
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof iocap);
    esp_bt_pin_code_t pin;
    memset(pin, 0, sizeof pin);
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin);

#if XQ_AVRCP
    // AVRCP has to be initialized before A2DP (esp_avrc_api.h).
    xEventGroupClearBits(s_bt.events, EVB_CT_INIT | EVB_TG_INIT);
    if ((err = esp_avrc_ct_register_callback(ct_cb)) != ESP_OK) goto fail;
    if ((err = esp_avrc_ct_init()) != ESP_OK) goto fail;
    s_bt.ct_inited = true;
    if ((err = esp_avrc_tg_register_callback(tg_cb)) != ESP_OK) goto fail;
    if ((err = esp_avrc_tg_init()) != ESP_OK) goto fail;
    s_bt.tg_inited = true;
    if (!wait_bits(EVB_CT_INIT | EVB_TG_INIT, STACK_OP_TIMEOUT_MS)) ESP_LOGW(TAG, "AVRCP init not confirmed");

    // Target: accept the transport keys of the headphones, answer play-status registrations.
    esp_avrc_psth_bit_mask_t allowed, supported;
    memset(&supported, 0, sizeof supported);
    if (esp_avrc_tg_get_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_ALLOWED_CMD, &allowed) == ESP_OK) {
        static const esp_avrc_pt_cmd_t k_keys[] = {ESP_AVRC_PT_CMD_PLAY,    ESP_AVRC_PT_CMD_PAUSE,
                                                   ESP_AVRC_PT_CMD_STOP,    ESP_AVRC_PT_CMD_FORWARD,
                                                   ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_VOL_UP,
                                                   ESP_AVRC_PT_CMD_VOL_DOWN, ESP_AVRC_PT_CMD_MUTE};
        for (size_t i = 0; i < sizeof k_keys / sizeof k_keys[0]; i++) {
            if (esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &allowed, k_keys[i]))
                esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &supported, k_keys[i]);
        }
        esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &supported);
    }
    esp_avrc_rn_evt_cap_mask_t rn;
    memset(&rn, 0, sizeof rn);
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &rn, ESP_AVRC_RN_PLAY_STATUS_CHANGE);
    esp_avrc_tg_set_rn_evt_cap(&rn);
#endif

    // A2DP source with the internal SBC encoder fed by xq_sink_data_cb().
    xEventGroupClearBits(s_bt.events, EVB_A2D_INIT);
    if ((err = esp_a2d_register_callback(a2d_cb)) != ESP_OK) goto fail;
    if ((err = esp_a2d_source_init()) != ESP_OK) goto fail;
    s_bt.a2d_inited = true;
    if (!wait_bits(EVB_A2D_INIT, STACK_OP_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "A2DP source init not confirmed");
        err = ESP_ERR_TIMEOUT;
        goto fail;
    }
    if ((err = esp_a2d_source_register_data_callback(xq_sink_data_cb)) != ESP_OK) goto fail;

    // Bonded headphones may connect to us; strangers cannot find us.
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    return XQ_OK;
fail:
    ESP_LOGE(TAG, "Bluetooth profile start failed: %s", esp_err_to_name(err));
    return XQ_EIO;
}

bool a2dp_xq_supported(void) {
#if XQ_HAVE_PREF_MCC && !CONFIG_BT_A2DP_USE_EXTERNAL_CODEC
    return true;
#else
    return false;
#endif
}

static esp_err_t esp_err_from(int rc) {
    switch (rc) {
        case XQ_OK: return ESP_OK;
        case XQ_ENOMEM: return ESP_ERR_NO_MEM;
        case XQ_EINVAL: return ESP_ERR_INVALID_ARG;
        case XQ_EUNSUPPORTED: return ESP_ERR_NOT_SUPPORTED;
        default: return ESP_FAIL;
    }
}

static int init_impl(const a2dp_xq_config_t *cfg);

esp_err_t a2dp_xq_init(const a2dp_xq_config_t *cfg) { return esp_err_from(init_impl(cfg)); }

static int init_impl(const a2dp_xq_config_t *cfg) {
    if (s_bt.initialized) return XQ_OK;
    if (!cfg) return XQ_EINVAL;
#if CONFIG_BT_A2DP_USE_EXTERNAL_CODEC
    // With the external-codec build the stack expects encoded SBC frames
    // (esp_a2d_source_audio_data_send) and never calls the PCM data callback.
    ESP_LOGE(TAG, "CONFIG_BT_A2DP_USE_EXTERNAL_CODEC is set; this build feeds PCM to the internal encoder");
    return XQ_EUNSUPPORTED;
#endif
    memset(&s_bt, 0, sizeof s_bt);
    atomic_store(&s_want_cdb, WANT_NONE);
    atomic_store(&s_abs_active, false);
    s_bt.allow_xq = cfg->allow_sbc_xq;
    s_bt.auto_reconnect = cfg->auto_reconnect;
    xq_strlcpy(s_bt.device_name, cfg->device_name && *cfg->device_name ? cfg->device_name : "ESP32",
                 sizeof s_bt.device_name);
    if (cfg->state_dir) xq_strlcpy(s_bt.state_dir, cfg->state_dir, sizeof s_bt.state_dir);
    s_bt.remote_cb = cfg->remote_cb;
    s_bt.remote_user = cfg->remote_user;
    s_bt.event_cb = cfg->event_cb;
    s_bt.event_user = cfg->event_user;
    s_bt.worker_core = cfg->worker_core;
    s_bt.state = A2DP_XQ_STATE_OFF;

    xq_devices_init(&s_bt.store);
    if (s_bt.state_dir[0]) xq_devices_load(&s_bt.store, s_bt.state_dir);

    s_bt.lock = xSemaphoreCreateMutex();
    s_bt.events = xEventGroupCreate();
    // The message queue lives in PSRAM: it is only touched by tasks, never from an ISR.
    s_bt.queue = xQueueCreateWithCaps(QUEUE_LEN, sizeof(bt_msg_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_bt.queue_caps = s_bt.queue != NULL;
    if (!s_bt.queue) s_bt.queue = xQueueCreate(QUEUE_LEN, sizeof(bt_msg_t));
    int rc = XQ_ENOMEM;
    if (!s_bt.lock || !s_bt.events || !s_bt.queue) goto fail;
    if ((rc = xq_sink_init()) != XQ_OK) goto fail;
    s_bt.sink_ready = true;
    if ((rc = stack_up()) != XQ_OK) goto fail;
    if ((rc = profiles_up()) != XQ_OK) goto fail;

    // Initial state and automatic reconnect, before the worker exists (no concurrency yet).
    s_bt.initialized = true;
    update_state();
    // Reconnect to the last device: the configured one, else the most recently used record.
    if (s_bt.auto_reconnect) {
        uint8_t addr[6];
        bool have = cfg->last_device && xq_addr_parse(cfg->last_device, addr);
        if (!have && s_bt.store.count) {
            const xq_device_rec_t *best = &s_bt.store.rec[0];
            for (uint32_t i = 1; i < s_bt.store.count; i++) {
                if (s_bt.store.rec[i].seen > best->seen) best = &s_bt.store.rec[i];
            }
            memcpy(addr, best->addr, 6);
            have = true;
        }
        if (have) begin_connect(addr, AUTO_RECONNECT_ATTEMPTS);
    }
    flush_outbox();

    // The worker's stack goes to PSRAM when possible: it never runs code that disables the
    // flash cache (file writes go to the SD card or another file system on its own task).
    BaseType_t core = tskNO_AFFINITY;
    if (s_bt.worker_core >= 0 && s_bt.worker_core < portNUM_PROCESSORS) core = s_bt.worker_core;
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(worker, "a2dp_xq", WORKER_STACK_BYTES, NULL, WORKER_PRIORITY,
                                                    &s_bt.task, core, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_bt.task_caps = ok == pdPASS;
    if (ok != pdPASS) {
        ok = xTaskCreatePinnedToCore(worker, "a2dp_xq", WORKER_STACK_BYTES, NULL, WORKER_PRIORITY, &s_bt.task, core);
    }
    if (ok != pdPASS) {
        s_bt.initialized = false;
        rc = XQ_ENOMEM;
        goto fail;
    }
    const uint8_t *own = esp_bt_dev_get_address();
    char own_str[18];
    xq_addr_format(own, own_str);
    ESP_LOGI(TAG, "Bluetooth up as \"%s\" (%s), %u free internal", s_bt.device_name, own_str,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return XQ_OK;

fail:
    teardown();
    memset(&s_bt, 0, sizeof s_bt);
    return rc;
}

void a2dp_xq_deinit(void) {
    if (!s_bt.initialized) return;
    a2dp_xq_event_cb_t cb = s_bt.event_cb;
    void *cb_user = s_bt.event_user;
    // Close the link politely, then stop the worker.
    xSemaphoreTake(s_bt.lock, portMAX_DELAY);
    bool linked = s_bt.connected || s_bt.connecting;
    uint8_t peer[6];
    memcpy(peer, s_bt.connected ? s_bt.peer : s_bt.target, 6);
    s_bt.want_connect = false;
    s_bt.pending_connect = false;
    if (s_bt.scanning) esp_bt_gap_cancel_discovery();
    if (linked) {
        xEventGroupClearBits(s_bt.events, EVB_DISCONNECTED);
        suspend_stream();
        esp_a2d_source_disconnect(peer);
    }
    save_store();
    xSemaphoreGive(s_bt.lock);
    if (linked && !wait_bits(EVB_DISCONNECTED, DISCONNECT_WAIT_MS)) ESP_LOGW(TAG, "disconnect not confirmed");

    bt_msg_t quit;
    memset(&quit, 0, sizeof quit);
    quit.type = MSG_CMD_QUIT;
    xQueueSendToFront(s_bt.queue, &quit, portMAX_DELAY);
    wait_bits(EVB_WORKER_DONE, 5000);
    if (s_bt.task) {
        if (s_bt.task_caps) {
            vTaskDeleteWithCaps(s_bt.task);
        } else {
            vTaskDelete(s_bt.task);
        }
        s_bt.task = NULL;
    }
    s_bt.initialized = false;
    teardown();
    memset(&s_bt, 0, sizeof s_bt);
    if (cb) {
        a2dp_xq_event_t ev;
        memset(&ev, 0, sizeof ev);
        ev.type = A2DP_XQ_EVENT_STATE;
        ev.state = A2DP_XQ_STATE_OFF;
        cb(&ev, cb_user);
    }
    ESP_LOGI(TAG, "Bluetooth off, %u free internal", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

bool a2dp_xq_is_initialized(void) { return s_bt.initialized; }

// ----------------------------------------------------------------- getters ----

static void fill_codec(a2dp_xq_status_t *out) {
    xq_sbc_cie_t c;
    if (s_bt.have_cfg) {
        c = s_bt.cfg;
    } else {
        default_cfg(&c);
    }
    xq_sbc_params_t p;
    if (!xq_sbc_params_from_cie(&c, &p)) return;
    p.bitpool = xq_sbc_bluedroid_bitpool(&p, c.min_bitpool, c.max_bitpool);
    bool xq = s_bt.xq == XQ_ACTIVE && p.mode == XQ_SBC_MODE_DUAL;
    xq_strlcpy(out->codec, xq ? "SBC-XQ" : "SBC", sizeof out->codec);
    xq_strlcpy(out->mode, xq_sbc_mode_name(p.mode), sizeof out->mode);
    out->bitpool = p.bitpool;
    out->bitrate_kbps = (uint16_t)((xq_sbc_bitrate_bps(&p) + 500u) / 1000u);
    out->sample_rate = p.sample_rate;
}

void a2dp_xq_get_status(a2dp_xq_status_t *out) {
    if (!out) return;
    memset(out, 0, sizeof *out);
    out->state = A2DP_XQ_STATE_OFF;
    if (!s_bt.initialized) return;
    xSemaphoreTake(s_bt.lock, portMAX_DELAY);
    out->state = s_bt.state;
    if (s_bt.connected) {
        xq_strlcpy(out->device_name, s_bt.peer_name, sizeof out->device_name);
        xq_addr_format(s_bt.peer, out->device_addr);
        fill_codec(out);
    } else if (s_bt.connecting) {
        const char *n = known_name(s_bt.target);
        if (n) xq_strlcpy(out->device_name, n, sizeof out->device_name);
        xq_addr_format(s_bt.target, out->device_addr);
    }
    xSemaphoreGive(s_bt.lock);
}

void a2dp_xq_describe(char *out, size_t out_size) {
    if (!out || !out_size) return;
    out[0] = '\0';
    a2dp_xq_status_t st;
    a2dp_xq_get_status(&st);
    if (st.state != A2DP_XQ_STATE_CONNECTED || !st.codec[0]) return;
    snprintf(out, out_size, "%s %u kbps", st.codec, (unsigned)st.bitrate_kbps);
}

void a2dp_xq_get_stats(a2dp_xq_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof *out);
    if (!s_bt.initialized) return;
    xq_sink_get_stats(out);
    xSemaphoreTake(s_bt.lock, portMAX_DELAY);
    out->streaming = s_bt.media == MEDIA_STARTED;
    out->abs_volume = s_bt.abs_volume;
    out->remote_volume = s_bt.remote_volume;
    out->sink_delay_ms = s_bt.sink_delay_ms;
    xSemaphoreGive(s_bt.lock);
}

uint32_t a2dp_xq_devices(a2dp_xq_device_t *out, uint32_t max) {
    if (!out || !max || !s_bt.initialized) return 0;
    esp_bd_addr_t bonded[XQ_DEVICES_MAX];
    int nb = XQ_DEVICES_MAX;
    if (esp_bt_gap_get_bond_device_list(&nb, bonded) != ESP_OK || nb < 0) nb = 0;
    if (nb > XQ_DEVICES_MAX) nb = XQ_DEVICES_MAX;

    uint32_t n = 0;
    xSemaphoreTake(s_bt.lock, portMAX_DELAY);
    for (int i = 0; i < nb && n < max; i++) {
        a2dp_xq_device_t *d = &out[n++];
        memset(d, 0, sizeof *d);
        xq_addr_format(bonded[i], d->addr);
        const char *name = known_name(bonded[i]);
        xq_strlcpy(d->name, name ? name : d->addr, sizeof d->name);
        const found_t *f = found_find(bonded[i]);
        d->rssi = f ? f->rssi : 0;
        d->paired = true;
        d->connected = s_bt.connected && addr_eq(bonded[i], s_bt.peer);
    }
    for (uint32_t i = 0; i < s_bt.found_count && n < max; i++) {
        const found_t *f = &s_bt.found[i];
        bool dup = false;
        for (int j = 0; j < nb; j++) {
            if (addr_eq(bonded[j], f->addr)) dup = true;
        }
        if (dup) continue;
        a2dp_xq_device_t *d = &out[n++];
        memset(d, 0, sizeof *d);
        xq_addr_format(f->addr, d->addr);
        xq_strlcpy(d->name, f->name[0] ? f->name : d->addr, sizeof d->name);
        d->rssi = f->rssi;
        d->connected = s_bt.connected && addr_eq(f->addr, s_bt.peer);
    }
    xSemaphoreGive(s_bt.lock);

    // Connected first, then paired, then by signal strength (insertion sort, n <= 32).
    for (uint32_t i = 1; i < n; i++) {
        a2dp_xq_device_t key = out[i];
        uint32_t j = i;
        while (j > 0) {
            const a2dp_xq_device_t *p = &out[j - 1];
            int kp = key.connected * 4 + key.paired * 2, pp = p->connected * 4 + p->paired * 2;
            bool before = kp > pp || (kp == pp && key.rssi != 0 && (p->rssi == 0 || key.rssi > p->rssi));
            if (!before) break;
            out[j] = out[j - 1];
            j--;
        }
        out[j] = key;
    }
    return n;
}

// ---------------------------------------------------------------- commands ----

void a2dp_xq_scan(bool start) { post_cmd(MSG_CMD_SCAN, NULL, start, 0); }

esp_err_t a2dp_xq_connect(const char *addr) {
    uint8_t a[6];
    if (!addr || !xq_addr_parse(addr, a)) return ESP_ERR_INVALID_ARG;
    if (!s_bt.initialized) return ESP_ERR_INVALID_STATE;
    post_cmd(MSG_CMD_CONNECT, a, false, 0);
    return ESP_OK;
}

void a2dp_xq_disconnect(void) { post_cmd(MSG_CMD_DISCONNECT, NULL, false, 0); }

void a2dp_xq_forget(const char *addr) {
    uint8_t a[6];
    if (!addr || !xq_addr_parse(addr, a)) return;
    post_cmd(MSG_CMD_FORGET, a, false, 0);
}

void a2dp_xq_set_volume_db(float db) {
    if (isnan(db)) return;
    atomic_store(&s_want_cdb, cdb_from_db(db));
    // Attenuate at once unless the headphones apply the volume: the next write must not go
    // out louder than asked while the worker catches up.
    if (!atomic_load(&s_abs_active)) xq_sink_set_gain_db(db);
    QueueHandle_t q = s_bt.queue;
    if (!s_bt.initialized || !q) return;
    bt_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = MSG_CMD_VOLUME;
    xQueueSend(q, &m, 0);  // the audio task never waits; the periodic tick catches up
}

esp_err_t a2dp_xq_set_xq_allowed(const char *addr, bool allowed) {
    uint8_t a[6];
    if (!addr || !xq_addr_parse(addr, a)) return ESP_ERR_INVALID_ARG;
    if (!s_bt.initialized) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_bt.lock, portMAX_DELAY);
    xq_device_rec_t *r = xq_devices_get(&s_bt.store, a);
    int rc = XQ_ENOMEM;
    if (r) {
        r->no_xq = !allowed;
        if (allowed && r->xq == XQ_VERDICT_FAIL) {
            r->xq = XQ_VERDICT_UNKNOWN;  // give it another chance
            r->xq_fails = 0;
        }
        s_bt.store.dirty = true;
        save_store();
        rc = XQ_OK;
    }
    xSemaphoreGive(s_bt.lock);
    return esp_err_from(rc);
}

void xq_core_sink_open(bool open) { post_cmd(MSG_CMD_SINK_OPEN, NULL, open, 0); }
void xq_core_sink_paused(bool paused) { post_cmd(MSG_CMD_SINK_PAUSED, NULL, paused, 0); }

#else  // !XQ_BT_SUPPORTED ---------------------------------------------------------

// Bluetooth Classic or A2DP is disabled in sdkconfig, or the chip has no Bluetooth Classic:
// the API stays linkable and reports "not supported", so the application builds unchanged.
bool a2dp_xq_supported(void) { return false; }
esp_err_t a2dp_xq_init(const a2dp_xq_config_t *cfg) {
    (void)cfg;
    ESP_LOGW(TAG, "Bluetooth Classic A2DP is not enabled in this build (CONFIG_BT_CLASSIC_ENABLED, "
                  "CONFIG_BT_A2DP_ENABLE)");
    return ESP_ERR_NOT_SUPPORTED;
}
void a2dp_xq_deinit(void) {}
bool a2dp_xq_is_initialized(void) { return false; }
void a2dp_xq_get_status(a2dp_xq_status_t *out) {
    if (out) memset(out, 0, sizeof *out);
}
void a2dp_xq_get_stats(a2dp_xq_stats_t *out) {
    if (out) memset(out, 0, sizeof *out);
}
void a2dp_xq_scan(bool start) { (void)start; }
uint32_t a2dp_xq_devices(a2dp_xq_device_t *out, uint32_t max) {
    (void)out;
    (void)max;
    return 0;
}
esp_err_t a2dp_xq_connect(const char *addr) {
    (void)addr;
    return ESP_ERR_NOT_SUPPORTED;
}
void a2dp_xq_disconnect(void) {}
void a2dp_xq_forget(const char *addr) { (void)addr; }
esp_err_t a2dp_xq_set_xq_allowed(const char *addr, bool allowed) {
    (void)addr;
    (void)allowed;
    return ESP_ERR_NOT_SUPPORTED;
}
void xq_core_sink_open(bool open) { (void)open; }
void xq_core_sink_paused(bool paused) { (void)paused; }
void a2dp_xq_set_volume_db(float db) { (void)db; }
void a2dp_xq_describe(char *out, size_t out_size) {
    if (out && out_size) out[0] = '\0';
}
#endif  // XQ_BT_SUPPORTED
