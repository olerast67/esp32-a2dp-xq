// SPDX-License-Identifier: Apache-2.0
// Connect to headphones and print what was negotiated (codec, mode, bitpool, bitrate) and the
// link statistics every two seconds while a quiet 1 kHz tone plays.
// The sketch fits the default partition scheme (about 1.1 MB of 1.25 MB).
#include <a2dp_xq.h>
#include <math.h>

const char *HEADPHONES = "";  // "aa:bb:cc:dd:ee:ff", or empty to take the first device found
volatile bool connectRequested = false;

void onEvent(const a2dp_xq_event_t *ev, void *) {
  if (ev->type == A2DP_XQ_EVENT_DEVICE_FOUND && !connectRequested) {
    connectRequested = true;
    a2dp_xq_connect(ev->device.addr);
  } else if (ev->type == A2DP_XQ_EVENT_STATE && ev->state == A2DP_XQ_STATE_IDLE && !connectRequested) {
    a2dp_xq_scan(true);
  }
}

void toneTask(void *) {
  static int16_t pcm[256 * 2];
  float phase = 0.0f;
  for (;;) {
    for (int i = 0; i < 256; i++) {
      pcm[2 * i] = pcm[2 * i + 1] = (int16_t)(sinf(phase) * 3000.0f);
      phase += 2.0f * PI * 1000.0f / A2DP_XQ_SAMPLE_RATE;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    a2dp_xq_write_s16(pcm, 256, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
  cfg.device_name = "a2dp-xq report";
  cfg.last_device = HEADPHONES[0] ? HEADPHONES : nullptr;
  cfg.event_cb = onEvent;
  if (a2dp_xq_init(&cfg) != ESP_OK) {
    Serial.println("Bluetooth init failed");
    return;
  }
  if (HEADPHONES[0]) {
    connectRequested = true;
    a2dp_xq_connect(HEADPHONES);
  }
  Serial.printf("SBC-XQ %s in this build (ESP-IDF %s)\n", a2dp_xq_supported() ? "available" : "not available",
                esp_get_idf_version());
  a2dp_xq_set_volume_db(-30.0f);
  a2dp_xq_start();
  xTaskCreate(toneTask, "tone", 4096, nullptr, 5, nullptr);
}

void loop() {
  delay(2000);
  a2dp_xq_status_t st;
  a2dp_xq_stats_t stats;
  a2dp_xq_get_status(&st);
  a2dp_xq_get_stats(&stats);
  if (st.state != A2DP_XQ_STATE_CONNECTED) {
    Serial.println("waiting for headphones...");
    return;
  }
  Serial.printf("%s [%s] %s %s bitpool %u, %u kbps | stream %s, abs volume %s, delay %u ms, underruns %u\n",
                st.device_name, st.device_addr, st.codec, st.mode, st.bitpool, st.bitrate_kbps,
                stats.streaming ? "on" : "off", stats.abs_volume ? "yes" : "no", stats.sink_delay_ms,
                (unsigned)stats.underruns);
}
