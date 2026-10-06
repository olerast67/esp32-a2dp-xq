// SPDX-License-Identifier: Apache-2.0
// Bluetooth volume scales (see xq_volume.h).
#include "xq_volume.h"

#include <math.h>

float a2dp_xq_volume_to_db(int v127) {
    if (v127 <= 0) return -100.0f;
    if (v127 >= 127) return 0.0f;
    return -XQ_VOLUME_RANGE_DB + XQ_VOLUME_RANGE_DB * (float)v127 / 127.0f;
}

uint8_t a2dp_xq_volume_from_db(float db) {
    if (!(db > -XQ_VOLUME_RANGE_DB)) return 0;  // also NAN
    if (db >= 0.0f) return 127;
    float v = (db + XQ_VOLUME_RANGE_DB) * 127.0f / XQ_VOLUME_RANGE_DB;
    long r = lroundf(v);
    if (r < 0) r = 0;
    if (r > 127) r = 127;
    return (uint8_t)r;
}

int32_t xq_volume_gain_q30(float db) {
    if (!(db > -120.0f)) return 0;  // also NAN
    if (db >= 0.0f) return 1 << 30;
    double g = pow(10.0, (double)db / 20.0) * (double)(1 << 30);
    if (g < 0.0) g = 0.0;
    if (g > (double)(1 << 30)) g = (double)(1 << 30);
    return (int32_t)(g + 0.5);
}
