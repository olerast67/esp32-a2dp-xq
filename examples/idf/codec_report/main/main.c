// SPDX-License-Identifier: Apache-2.0
// Connect to headphones and report what was negotiated: codec, channel mode, bitpool, bitrate,
// absolute volume support, sink delay and FIFO underruns, every two seconds. A quiet tone plays
// so the stream really runs.
//
// Set HEADPHONES_ADDR to your headphones, or leave it empty to take the first device the scan
// finds (put the headphones into pairing mode first).
#include <math.h>
#include <stdio.h>

#include "a2dp_xq.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define HEADPHONES_ADDR ""

static const char *TAG = "codec_report";
static volatile bool s_connect_requested;

static void on_event(const a2dp_xq_event_t *ev, void *user) {
    (void)user;
    if (ev->type == A2DP_XQ_EVENT_DEVICE_FOUND && !s_connect_requested) {
        s_connect_requested = true;
        a2dp_xq_connect(ev->device.addr);
    } else if (ev->type == A2DP_XQ_EVENT_STATE && ev->state == A2DP_XQ_STATE_IDLE && !s_connect_requested) {
        a2dp_xq_scan(true);
    }
}

static void tone_task(void *arg) {
    (void)arg;
    static int16_t pcm[256 * 2];
    float phase = 0.0f;
    for (;;) {
        for (int i = 0; i < 256; i++) {
            pcm[2 * i] = pcm[2 * i + 1] = (int16_t)(sinf(phase) * 3000.0f);
            phase += 2.0f * (float)M_PI * 1000.0f / (float)A2DP_XQ_SAMPLE_RATE;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
        a2dp_xq_write_s16(pcm, 256, portMAX_DELAY);
    }
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
    cfg.device_name = "a2dp-xq report";
    cfg.last_device = HEADPHONES_ADDR[0] ? HEADPHONES_ADDR : NULL;
    cfg.event_cb = on_event;
    ESP_ERROR_CHECK(a2dp_xq_init(&cfg));
    if (HEADPHONES_ADDR[0]) {
        s_connect_requested = true;
        a2dp_xq_connect(HEADPHONES_ADDR);
    }
    a2dp_xq_set_volume_db(-30.0f);
    a2dp_xq_start();
    xTaskCreate(tone_task, "tone", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ESP-IDF %s, SBC-XQ %s", esp_get_idf_version(), a2dp_xq_supported() ? "available" : "not available");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        a2dp_xq_status_t st;
        a2dp_xq_stats_t stats;
        a2dp_xq_get_status(&st);
        a2dp_xq_get_stats(&stats);
        if (st.state != A2DP_XQ_STATE_CONNECTED) {
            ESP_LOGI(TAG, "state %d, waiting for headphones", (int)st.state);
            continue;
        }
        ESP_LOGI(TAG, "%s [%s] %s %s bitpool %u, %u kbps @ %u Hz | stream %s, abs volume %s, delay %u ms, "
                      "FIFO %u/%u, underruns %u",
                 st.device_name, st.device_addr, st.codec, st.mode, st.bitpool, st.bitrate_kbps,
                 (unsigned)st.sample_rate, stats.streaming ? "on" : "off", stats.abs_volume ? "yes" : "no",
                 stats.sink_delay_ms, (unsigned)stats.fifo_frames, (unsigned)stats.fifo_limit_frames,
                 (unsigned)stats.underruns);
    }
}
