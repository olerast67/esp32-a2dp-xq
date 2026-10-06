// SPDX-License-Identifier: Apache-2.0
// Play a WAV file from the SD card to Bluetooth headphones.
//
// The file must be PCM, 16-bit, stereo, 44.1 kHz (the rate the Bluetooth encoder takes). For
// every format (FLAC, MP3, 24-bit, other rates, gapless albums) use esp32-audio-player, which
// decodes, resamples and feeds this library for you.
//
// SD card on SPI: MOSI 23, MISO 19, SCK 18, CS 5.
#include <FS.h>
#include <SD.h>
#include <a2dp_xq.h>

const char *HEADPHONES = "";        // "aa:bb:cc:dd:ee:ff", or empty to take the first device found
const char *WAV_FILE = "/test.wav";  // on the card
const int SD_CS = 5;

File wav;
volatile bool connectRequested = false;

void onEvent(const a2dp_xq_event_t *ev, void *) {
  if (ev->type == A2DP_XQ_EVENT_DEVICE_FOUND && !connectRequested) {
    connectRequested = true;
    a2dp_xq_connect(ev->device.addr);
  } else if (ev->type == A2DP_XQ_EVENT_STATE && ev->state == A2DP_XQ_STATE_IDLE && !connectRequested) {
    a2dp_xq_scan(true);
  }
}

// Find the "data" chunk of a RIFF/WAVE file and check the format. Leaves the file at the samples.
bool openWav(const char *path) {
  wav = SD.open(path);
  if (!wav) return false;
  uint8_t hdr[12];
  if (wav.read(hdr, 12) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) return false;
  bool fmtOk = false;
  while (wav.available()) {
    uint8_t ch[8];
    if (wav.read(ch, 8) != 8) return false;
    uint32_t size = ch[4] | ch[5] << 8 | ch[6] << 16 | (uint32_t)ch[7] << 24;
    if (!memcmp(ch, "fmt ", 4)) {
      uint8_t f[16];
      if (size < 16 || wav.read(f, 16) != 16) return false;
      uint16_t format = f[0] | f[1] << 8, channels = f[2] | f[3] << 8, bits = f[14] | f[15] << 8;
      uint32_t rate = f[4] | f[5] << 8 | f[6] << 16 | (uint32_t)f[7] << 24;
      fmtOk = format == 1 && channels == 2 && bits == 16 && rate == A2DP_XQ_SAMPLE_RATE;
      wav.seek(wav.position() + size - 16 + (size & 1));
    } else if (!memcmp(ch, "data", 4)) {
      return fmtOk;
    } else {
      wav.seek(wav.position() + size + (size & 1));
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  if (!SD.begin(SD_CS, SPI, 20000000)) {
    Serial.println("no SD card");
    return;
  }
  if (!openWav(WAV_FILE)) {
    Serial.println("need a 16-bit stereo 44.1 kHz WAV file");
    return;
  }
  a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
  cfg.device_name = "a2dp-xq wav";
  cfg.last_device = HEADPHONES[0] ? HEADPHONES : nullptr;
  cfg.event_cb = onEvent;
  cfg.state_dir = "/sd";  // per-device SBC-XQ results survive reboots
  if (a2dp_xq_init(&cfg) != ESP_OK) return;
  if (HEADPHONES[0]) {
    connectRequested = true;
    a2dp_xq_connect(HEADPHONES);
  }
  a2dp_xq_set_volume_db(-15.0f);
  a2dp_xq_start();
}

void loop() {
  static int16_t pcm[512 * 2];
  if (!wav) {
    delay(100);
    return;
  }
  int bytes = wav.read((uint8_t *)pcm, sizeof pcm);  // WAV samples are little-endian, like the ESP32
  if (bytes <= 0) {
    Serial.println("end of file");
    wav.close();
    return;
  }
  a2dp_xq_write_s16(pcm, bytes / 4, portMAX_DELAY);
}
