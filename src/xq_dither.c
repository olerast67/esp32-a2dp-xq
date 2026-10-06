// SPDX-License-Identifier: Apache-2.0
// TPDF dither from Q31 to 16 bits (see xq_util.h). A file of its own so the host tests can
// replace it with a predictable truncation.
#include "xq_util.h"

static inline uint32_t lcg_next(uint32_t *s) {
    *s = *s * 1664525u + 1013904223u;
    return *s;
}

void xq_q31_to_s16_dither(const int32_t *in, int16_t *out, size_t samples, uint32_t *state) {
    if (!in || !out) return;
    uint32_t s = state ? *state : 0x2545F491u;
    for (size_t i = 0; i < samples; i++) {
        // TPDF: difference of two uniform 16-bit values = +-1 LSB of the 16-bit output.
        const int32_t a = (int32_t)(lcg_next(&s) >> 16);
        const int32_t b = (int32_t)(lcg_next(&s) >> 16);
        // Add dither and half an LSB, then floor: rounding to nearest with dither.
        int64_t v = ((int64_t)in[i] + (a - b) + 32768) >> 16;
        if (v > INT16_MAX) v = INT16_MAX;
        if (v < INT16_MIN) v = INT16_MIN;
        out[i] = (int16_t)v;
    }
    if (state) *state = s;
}
