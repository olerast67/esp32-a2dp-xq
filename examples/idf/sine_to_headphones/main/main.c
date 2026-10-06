// SPDX-License-Identifier: Apache-2.0
// Play a 440 Hz sine to Bluetooth headphones with SBC-XQ.
//
// Put the headphones into pairing mode and reset the board. Without HEADPHONES_ADDR the
// example connects to the first audio device the scan finds; later boots reconnect to it.
#include <math.h>
#include <string.h>

#include "a2dp_xq.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define HEADPHONES_ADDR ""  // "aa:bb:cc:dd:ee:ff" to skip the scan
#define TONE_HZ 440.0f
#define FRAMES 256

static const char *TAG = "example";
static volatile bool s_connect_requested;

static void on_event(const a2dp_xq_event_t *ev, void *user) {
    (void)user;
    switch (ev->type) {
        case A2DP_XQ_EVENT_DEVICE_FOUND:
            ESP_LOGI(TAG, "found %s (%s, %d dBm)%s", ev->device.name, ev->device.addr, ev->device.rssi,
                     ev->device.paired ? ", paired" : "");
            if (!s_connect_requested && !HEADPHONES_ADDR[0]) {
                s_connect_requested = true;
                a2dp_xq_connect(ev->device.addr);
            }
            break;
        case A2DP_XQ_EVENT_CODEC: {
            a2dp_xq_status_t st;
            a2dp_xq_get_status(&st);
            ESP_LOGI(TAG, "%s: %s %s, bitpool %u, %u kbps", st.device_name, st.codec, st.mode, st.bitpool,
                     st.bitrate_kbps);
            break;
        }
        case A2DP_XQ_EVENT_STATE:
            if (ev->state == A2DP_XQ_STATE_IDLE && !s_connect_requested) a2dp_xq_scan(true);
            break;
        default: break;
    }
}

void app_main(void) {
    // Bonding keys live in NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
    cfg.device_name = "a2dp-xq sine";
    cfg.last_device = HEADPHONES_ADDR[0] ? HEADPHONES_ADDR : NULL;
    cfg.event_cb = on_event;
    ESP_ERROR_CHECK(a2dp_xq_init(&cfg));
    ESP_LOGI(TAG, "SBC-XQ %s in this build", a2dp_xq_supported() ? "available" : "not available");
    if (HEADPHONES_ADDR[0]) {
        s_connect_requested = true;
        a2dp_xq_connect(HEADPHONES_ADDR);
    }

    a2dp_xq_set_volume_db(-20.0f);
    a2dp_xq_start();

    static int16_t pcm[FRAMES * 2];
    float phase = 0.0f;
    const float step = 2.0f * (float)M_PI * TONE_HZ / (float)A2DP_XQ_SAMPLE_RATE;
    for (;;) {
        for (int i = 0; i < FRAMES; i++) {
            int16_t v = (int16_t)(sinf(phase) * 12000.0f);
            pcm[2 * i] = v;
            pcm[2 * i + 1] = v;
            phase += step;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
        // Blocks while the FIFO is full: the headphones set the pace.
        a2dp_xq_write_s16(pcm, FRAMES, portMAX_DELAY);
    }
}
