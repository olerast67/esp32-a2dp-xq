// SPDX-License-Identifier: Apache-2.0
// Private helpers (see xq_util.h).
#include "xq_util.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_log.h"
#endif

const char *xq_err_name(int err) {
    switch (err) {
        case XQ_OK: return "ok";
        case XQ_ENOMEM: return "out of memory";
        case XQ_EINVAL: return "invalid argument";
        case XQ_EIO: return "i/o error";
        case XQ_EUNSUPPORTED: return "unsupported";
        default: return "error";
    }
}

void *xq_malloc(size_t size) {
    if (!size) size = 1;
#ifdef ESP_PLATFORM
    return heap_caps_malloc_prefer(size, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
#else
    return malloc(size);
#endif
}

void xq_free(void *ptr) {
    if (!ptr) return;
#ifdef ESP_PLATFORM
    heap_caps_free(ptr);
#else
    free(ptr);
#endif
}

size_t xq_strlcpy(char *dst, const char *src, size_t size) {
    size_t len = src ? strlen(src) : 0;
    if (size) {
        size_t n = len < size - 1 ? len : size - 1;
        if (n) memcpy(dst, src, n);
        dst[n] = '\0';
    }
    return len;
}

int xq_path_join(char *out, size_t out_size, const char *dir, const char *name) {
    size_t dl = strlen(dir);
    bool need_slash = dl > 0 && dir[dl - 1] != '/';
    int n = snprintf(out, out_size, "%s%s%s", dir, need_slash ? "/" : "", name);
    return (n < 0 || (size_t)n >= out_size) ? XQ_EINVAL : XQ_OK;
}

void xq_log_warn(const char *tag, const char *fmt, ...) {
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
#ifdef ESP_PLATFORM
    ESP_LOGW(tag, "%s", msg);
#else
    fprintf(stderr, "W (%s) %s\n", tag, msg);
#endif
}
