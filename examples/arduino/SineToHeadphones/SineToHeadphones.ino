// SPDX-License-Identifier: Apache-2.0
// Play a 440 Hz sine to Bluetooth headphones with esp32-a2dp-xq.
//
// Put the headphones into pairing mode, then reset the board. Without HEADPHONES the sketch
// connects to the first audio device the scan finds; later boots reconnect to it.
// Board: any classic ESP32 (ESP32-S3/C3/C6 have no Bluetooth Classic).
// The sketch fits the default partition scheme (about 1.1 MB of 1.25 MB).
#include <a2dp_xq.h>
#include <math.h>

const char *HEADPHONES = "";  // "aa:bb:cc:dd:ee:ff" to skip the scan
volatile bool connectRequested = false;

void onEvent(const a2dp_xq_event_t *ev, void *) {
  if (ev->type == A2DP_XQ_EVENT_DEVICE_FOUND && !connectRequested && !HEADPHONES[0]) {
    Serial.printf("found %s (%s)\n", ev->device.name, ev->device.addr);
    connectRequested = true;
    a2dp_xq_connect(ev->device.addr);
  } else if (ev->type == A2DP_XQ_EVENT_STATE && ev->state == A2DP_XQ_STATE_IDLE && !connectRequested) {
    a2dp_xq_scan(true);
  } else if (ev->type == A2DP_XQ_EVENT_CODEC) {
    a2dp_xq_status_t st;
    a2dp_xq_get_status(&st);
    Serial.printf("%s: %s %s, bitpool %u, %u kbps\n", st.device_name, st.codec, st.mode, st.bitpool, st.bitrate_kbps);
  }
}

void setup() {
  Serial.begin(115200);
  a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
  cfg.device_name = "a2dp-xq sine";
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
  a2dp_xq_set_volume_db(-20.0f);
  a2dp_xq_start();
}

void loop() {
  static int16_t pcm[256 * 2];
  static float phase = 0.0f;
  for (int i = 0; i < 256; i++) {
    int16_t v = (int16_t)(sinf(phase) * 12000.0f);
    pcm[2 * i] = pcm[2 * i + 1] = v;
    phase += 2.0f * PI * 440.0f / A2DP_XQ_SAMPLE_RATE;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;
  }
  a2dp_xq_write_s16(pcm, 256, portMAX_DELAY);  // the headphones set the pace
}
