// SPDX-License-Identifier: Apache-2.0
// Volume: the AVRCP absolute volume scale (0..127) against the application volume in dB, and
// the linear gain applied in the 16-bit conversion for headphones without absolute volume.
// Pure C: covered by test/test_a2dp_xq.c. The two a2dp_xq_volume_* functions are public
// (a2dp_xq.h): the application maps the headphones' own volume changes with them.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XQ_VOLUME_RANGE_DB 60.0f  // 1..127 cover -60..0 dB in equal steps, 0 = mute

// 0 (and below) = mute (-100 dB), 127 (and above) = 0 dB, linear in dB in between.
float a2dp_xq_volume_to_db(int v127);
// Inverse: the nearest step (exact for every value a2dp_xq_volume_to_db returns). -60 dB and
// lower, NAN and -inf give 0; 0 dB and more give 127.
uint8_t a2dp_xq_volume_from_db(float db);
// Linear gain in Q30 (1 << 30 = unity) for db (<= 0; positive = unity). Below -120 dB, NAN
// or -inf: 0.
int32_t xq_volume_gain_q30(float db);

#ifdef __cplusplus
}
#endif
