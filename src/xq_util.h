// SPDX-License-Identifier: Apache-2.0
// Small private helpers of esp32-a2dp-xq: error codes of the pure-C modules, memory (PSRAM
// first on the device), logging and string helpers. Portable: the host tests build the same
// code without ESP-IDF.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Return codes of the pure-C modules: 0 or negative.
enum {
    XQ_OK = 0,
    XQ_ERR = -1,
    XQ_ENOMEM = -3,
    XQ_EINVAL = -4,
    XQ_EIO = -5,
    XQ_EUNSUPPORTED = -6,
};

#define XQ_PATH_MAX 256

const char *xq_err_name(int err);  // "ok", "out of memory", ...

// Large buffers (FIFO storage, file text): PSRAM when the board has it.
void *xq_malloc(size_t size);
void xq_free(void *ptr);

size_t xq_strlcpy(char *dst, const char *src, size_t size);  // always terminates
// "dir/name" without a double slash. XQ_EINVAL when it does not fit.
int xq_path_join(char *out, size_t out_size, const char *dir, const char *name);

void xq_log_warn(const char *tag, const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;
#define XQ_LOGW(tag, ...) xq_log_warn(tag, __VA_ARGS__)

// Interleaved Q31 to 16 bits with TPDF dither (state = LCG seed, kept per stream).
void xq_q31_to_s16_dither(const int32_t *in, int16_t *out, size_t samples, uint32_t *state);

#ifdef __cplusplus
}
#endif
