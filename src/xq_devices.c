// SPDX-License-Identifier: Apache-2.0
// Per-headphone memory for the Bluetooth output (see xq_devices.h).
#include "xq_devices.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xq_util.h"

#define TAG "a2dp_xq"
#define FILE_MAX_BYTES 8192u

static const char *const k_xq_names[] = {"unknown", "ok", "fail"};

void xq_devices_init(xq_devices_t *d) {
    if (!d) return;
    memset(d, 0, sizeof(*d));
}

// ------------------------------------------------------------------ helpers ----

static int hex_val(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool xq_addr_parse(const char *s, uint8_t out[6]) {
    if (!s || !out) return false;
    uint8_t tmp[6];
    for (int i = 0; i < 6; i++) {
        int hi = hex_val((unsigned char)s[0]);
        int lo = hi < 0 ? -1 : hex_val((unsigned char)s[1]);
        if (hi < 0 || lo < 0) return false;
        tmp[i] = (uint8_t)(hi << 4 | lo);
        s += 2;
        if (i < 5) {
            if (*s != ':' && *s != '-') return false;
            s++;
        }
    }
    // Must end here (or at whitespace for the file format).
    if (*s && *s != '\t' && *s != ' ' && *s != '\r' && *s != '\n') return false;
    memcpy(out, tmp, 6);
    return true;
}

void xq_addr_format(const uint8_t a[6], char out[18]) {
    if (!out) return;
    if (!a) {
        out[0] = '\0';
        return;
    }
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], a[2], a[3], a[4], a[5]);
}

// Copy a device name: control characters become spaces, the result is cut on a UTF-8
// character boundary and never exceeds XQ_NAME_MAX - 1 bytes.
static void copy_name(char *dst, const char *src, size_t src_len) {
    size_t n = 0;
    for (size_t i = 0; i < src_len && src[i] && n + 1 < XQ_NAME_MAX; i++) {
        unsigned char c = (unsigned char)src[i];
        dst[n++] = (c < 0x20 || c == 0x7F) ? ' ' : (char)c;
    }
    // Do not leave half of a multi-byte sequence at the end.
    if (n > 0 && ((unsigned char)dst[n - 1] & 0x80)) {
        size_t start = n - 1;
        while (start > 0 && ((unsigned char)dst[start] & 0xC0) == 0x80) start--;
        unsigned char lead = (unsigned char)dst[start];
        size_t need = (lead & 0xE0) == 0xC0 ? 2 : (lead & 0xF0) == 0xE0 ? 3 : (lead & 0xF8) == 0xF0 ? 4 : 1;
        if (n - start < need) n = start;
    }
    while (n > 0 && dst[n - 1] == ' ') n--;
    dst[n] = '\0';
}

static int lower(int c) { return (c >= 'A' && c <= 'Z') ? c + 32 : c; }
static bool is_alnum(int c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

// Case-insensitive search for `needle` in `hay` where the match is not followed by a letter
// or digit ("Soundcore 2" must not match "Soundcore 20").
static bool contains_word(const char *hay, const char *needle) {
    size_t nl = strlen(needle);
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nl && p[i] && lower((unsigned char)p[i]) == lower((unsigned char)needle[i])) i++;
        if (i == nl && !is_alnum((unsigned char)p[nl])) return true;
    }
    return false;
}

bool xq_name_blacklisted(const char *name) {
    static const char *const k_list[] = {
        "soundcore 2", "cmf buds 2a", "dc800", "s305", "pmk tws", "pmk-tws", "phonak",
    };
    if (!name || !*name) return false;
    for (size_t i = 0; i < sizeof(k_list) / sizeof(k_list[0]); i++) {
        if (contains_word(name, k_list[i])) return true;
    }
    return false;
}

// ------------------------------------------------------------------- records ----

xq_device_rec_t *xq_devices_find(xq_devices_t *d, const uint8_t addr[6]) {
    if (!d || !addr) return NULL;
    for (uint32_t i = 0; i < d->count; i++) {
        if (memcmp(d->rec[i].addr, addr, 6) == 0) return &d->rec[i];
    }
    return NULL;
}

xq_device_rec_t *xq_devices_get(xq_devices_t *d, const uint8_t addr[6]) {
    if (!d || !addr) return NULL;
    xq_device_rec_t *r = xq_devices_find(d, addr);
    if (r) return r;
    if (d->count < XQ_DEVICES_MAX) {
        r = &d->rec[d->count++];
    } else {
        r = &d->rec[0];
        for (uint32_t i = 1; i < d->count; i++) {
            if (d->rec[i].seen < r->seen) r = &d->rec[i];
        }
    }
    memset(r, 0, sizeof(*r));
    memcpy(r->addr, addr, 6);
    d->dirty = true;
    return r;
}

void xq_devices_remove(xq_devices_t *d, const uint8_t addr[6]) {
    xq_device_rec_t *r = xq_devices_find(d, addr);
    if (!r) return;
    size_t idx = (size_t)(r - d->rec);
    memmove(&d->rec[idx], &d->rec[idx + 1], (d->count - idx - 1) * sizeof(xq_device_rec_t));
    d->count--;
    d->dirty = true;
}

void xq_devices_touch(xq_devices_t *d, xq_device_rec_t *rec) {
    if (!d || !rec) return;
    rec->seen = ++d->clock;
    d->dirty = true;
}

void xq_devices_set_name(xq_devices_t *d, xq_device_rec_t *rec, const char *name) {
    if (!d || !rec || !name || !*name) return;
    char tmp[XQ_NAME_MAX] = {0};
    copy_name(tmp, name, strlen(name));
    if (!tmp[0] || strcmp(tmp, rec->name) == 0) return;
    memcpy(rec->name, tmp, sizeof tmp);
    d->dirty = true;
}

void xq_devices_xq_result(xq_devices_t *d, xq_device_rec_t *rec, bool ok) {
    if (!d || !rec) return;
    if (ok) {
        if (rec->xq != XQ_VERDICT_OK || rec->xq_fails) d->dirty = true;
        rec->xq = XQ_VERDICT_OK;
        rec->xq_fails = 0;
        return;
    }
    if (rec->xq_fails < 255) rec->xq_fails++;
    if (rec->xq_fails >= XQ_FAILS_TO_DISABLE) rec->xq = XQ_VERDICT_FAIL;
    d->dirty = true;
}

bool xq_devices_xq_allowed(const xq_device_rec_t *rec, const char *name) {
    const char *n = (name && *name) ? name : (rec ? rec->name : NULL);
    if (n && xq_name_blacklisted(n)) return false;
    if (!rec) return true;
    return !rec->no_xq && rec->xq != XQ_VERDICT_FAIL;
}

// ------------------------------------------------------------ text format ----

static bool parse_u32(const char *s, size_t len, uint32_t *out) {
    if (!len || len > 10) return false;
    uint64_t v = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') return false;
        v = v * 10u + (uint64_t)(s[i] - '0');
    }
    if (v > 0xFFFFFFFFu) return false;
    *out = (uint32_t)v;
    return true;
}

static void parse_field(xq_device_rec_t *r, const char *key, size_t klen, const char *val, size_t vlen) {
    uint32_t n;
    if (klen == 2 && memcmp(key, "xq", 2) == 0) {
        for (uint8_t i = 0; i < 3; i++) {
            if (strlen(k_xq_names[i]) == vlen && memcmp(val, k_xq_names[i], vlen) == 0) r->xq = i;
        }
    } else if (klen == 5 && memcmp(key, "fails", 5) == 0) {
        if (parse_u32(val, vlen, &n)) r->xq_fails = n > 255 ? 255 : (uint8_t)n;
    } else if (klen == 4 && memcmp(key, "noxq", 4) == 0) {
        if (parse_u32(val, vlen, &n)) r->no_xq = n != 0;
    } else if (klen == 4 && memcmp(key, "seen", 4) == 0) {
        if (parse_u32(val, vlen, &n)) r->seen = n;
    } else if (klen == 4 && memcmp(key, "name", 4) == 0) {
        copy_name(r->name, val, vlen);
    }
}

int xq_devices_parse(xq_devices_t *d, const char *text, size_t len) {
    if (!d || !text) return XQ_EINVAL;
    int loaded = 0;
    size_t pos = 0;
    while (pos < len) {
        size_t end = pos;
        while (end < len && text[end] != '\n') end++;
        const char *line = text + pos;
        size_t llen = end - pos;
        pos = end + 1;
        while (llen && (line[llen - 1] == '\r' || line[llen - 1] == ' ')) llen--;
        while (llen && (*line == ' ' || *line == '\t')) {
            line++;
            llen--;
        }
        if (llen < 17 || line[0] == '#') continue;

        char addr_str[18];
        memcpy(addr_str, line, 17);
        addr_str[17] = '\0';
        uint8_t addr[6];
        if (!xq_addr_parse(addr_str, addr)) continue;
        if (llen > 17 && line[17] != '\t' && line[17] != ' ') continue;

        xq_device_rec_t tmp;
        memset(&tmp, 0, sizeof tmp);
        memcpy(tmp.addr, addr, 6);
        size_t i = 17;
        while (i < llen) {
            while (i < llen && (line[i] == '\t' || line[i] == ' ')) i++;
            size_t kstart = i;
            while (i < llen && line[i] != '=' && line[i] != '\t') i++;
            if (i >= llen || line[i] != '=') {
                while (i < llen && line[i] != '\t') i++;
                continue;
            }
            size_t klen = i - kstart;
            i++;  // '='
            size_t vstart = i;
            bool is_name = klen == 4 && memcmp(line + kstart, "name", 4) == 0;
            if (is_name) {
                i = llen;  // the name takes the rest of the line
            } else {
                while (i < llen && line[i] != '\t') i++;
            }
            parse_field(&tmp, line + kstart, klen, line + vstart, i - vstart);
        }
        if (tmp.xq > XQ_VERDICT_FAIL) tmp.xq = XQ_VERDICT_UNKNOWN;

        xq_device_rec_t *r = xq_devices_find(d, addr);
        if (!r) {
            if (d->count >= XQ_DEVICES_MAX) continue;
            r = &d->rec[d->count++];
        }
        *r = tmp;
        if (r->seen > d->clock) d->clock = r->seen;
        loaded++;
    }
    return loaded;
}

int xq_devices_format(const xq_devices_t *d, char *out, size_t out_size) {
    if (!d || !out || out_size < 2) return XQ_EINVAL;
    size_t used = 0;
    int n = snprintf(out, out_size, "# esp32-a2dp-xq devices v1\n");
    if (n < 0 || (size_t)n >= out_size) return XQ_EINVAL;
    used = (size_t)n;
    for (uint32_t i = 0; i < d->count; i++) {
        const xq_device_rec_t *r = &d->rec[i];
        char addr[18];
        xq_addr_format(r->addr, addr);
        n = snprintf(out + used, out_size - used, "%s\txq=%s\tfails=%u\tnoxq=%u\tseen=%u\tname=%s\n", addr,
                     k_xq_names[r->xq <= XQ_VERDICT_FAIL ? r->xq : 0], (unsigned)r->xq_fails, r->no_xq ? 1u : 0u,
                     (unsigned)r->seen, r->name);
        if (n < 0 || (size_t)n >= out_size - used) return XQ_EINVAL;
        used += (size_t)n;
    }
    return (int)used;
}

int xq_devices_load(xq_devices_t *d, const char *dir) {
    if (!d || !dir || !*dir) return XQ_EINVAL;
    char path[XQ_PATH_MAX];
    if (xq_path_join(path, sizeof path, dir, XQ_DEVICES_FILE) < 0) return XQ_EINVAL;
    FILE *fp = fopen(path, "rb");
    if (!fp) return 0;  // first run: nothing stored yet
    char *buf = (char *)xq_malloc(FILE_MAX_BYTES);
    if (!buf) {
        fclose(fp);
        return XQ_ENOMEM;
    }
    size_t len = fread(buf, 1, FILE_MAX_BYTES, fp);
    bool error = ferror(fp) != 0;
    fclose(fp);
    int rc = error ? XQ_EIO : xq_devices_parse(d, buf, len);
    xq_free(buf);
    if (rc < 0) XQ_LOGW(TAG, "cannot read %s: %s", path, xq_err_name(rc));
    d->dirty = false;
    return rc;
}

int xq_devices_save(xq_devices_t *d, const char *dir) {
    if (!d || !dir || !*dir) return XQ_EINVAL;
    if (!d->dirty) return XQ_OK;
    char path[XQ_PATH_MAX], tmp[XQ_PATH_MAX];
    if (xq_path_join(path, sizeof path, dir, XQ_DEVICES_FILE) < 0) return XQ_EINVAL;
    if (xq_path_join(tmp, sizeof tmp, dir, "a2dp_xq_devices.tmp") < 0) return XQ_EINVAL;
    char *buf = (char *)xq_malloc(FILE_MAX_BYTES);
    if (!buf) return XQ_ENOMEM;
    int len = xq_devices_format(d, buf, FILE_MAX_BYTES);
    int rc = XQ_OK;
    if (len < 0) {
        rc = len;
    } else {
        FILE *fp = fopen(tmp, "wb");
        if (!fp) {
            rc = XQ_EIO;
        } else {
            size_t wr = fwrite(buf, 1, (size_t)len, fp);
            if (fclose(fp) != 0 || wr != (size_t)len) rc = XQ_EIO;
        }
        if (rc == XQ_OK) {
            remove(path);  // FAT rename() does not replace an existing file
            if (rename(tmp, path) != 0) rc = XQ_EIO;
        } else {
            remove(tmp);
        }
    }
    xq_free(buf);
    if (rc == XQ_OK) {
        d->dirty = false;
    } else {
        XQ_LOGW(TAG, "cannot save %s: %s", path, xq_err_name(rc));
    }
    return rc;
}
