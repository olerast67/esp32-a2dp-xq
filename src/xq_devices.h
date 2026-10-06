// SPDX-License-Identifier: Apache-2.0
// Per-headphone memory, kept as a small text file in state_dir (an SD card or any mounted file
// system: <state_dir>/a2dp_xq_devices.txt), so it survives reflashing and never writes the
// internal flash during playback. One line per device:
//
//   aa:bb:cc:dd:ee:ff<TAB>xq=ok<TAB>fails=0<TAB>noxq=0<TAB>seen=12<TAB>name=Sony WH-1000XM4
//
//   xq     unknown | ok | fail   result of SBC-XQ (Dual Channel 38) on this device
//   fails  XQ failures in a row (link loss right after starting XQ); 2 marks xq=fail
//   noxq   1 = never try XQ (user choice or built-in blacklist hit)
//   seen   use counter for least-recently-used eviction
//   name   last known device name (rest of the line, control characters removed)
//
// Pure C with stdio: covered by test/test_a2dp_xq.c.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XQ_DEVICES_MAX 16
#define XQ_DEVICES_FILE "a2dp_xq_devices.txt"
#define XQ_NAME_MAX 64
#define XQ_FAILS_TO_DISABLE 2

typedef enum { XQ_VERDICT_UNKNOWN = 0, XQ_VERDICT_OK, XQ_VERDICT_FAIL } xq_verdict_t;

typedef struct {
    uint8_t addr[6];
    char name[XQ_NAME_MAX];
    uint8_t xq;          // xq_verdict_t
    uint8_t xq_fails;
    bool no_xq;
    uint32_t seen;
} xq_device_rec_t;

typedef struct {
    xq_device_rec_t rec[XQ_DEVICES_MAX];
    uint32_t count;
    uint32_t clock;  // highest `seen` value, incremented by xq_devices_touch
    bool dirty;
} xq_devices_t;

void xq_devices_init(xq_devices_t *d);
// Parse file text. Unknown keys and malformed lines are skipped. Returns records loaded.
int xq_devices_parse(xq_devices_t *d, const char *text, size_t len);
// Serialize. Returns bytes written (without the terminator) or XQ_EINVAL when out is too small.
int xq_devices_format(const xq_devices_t *d, char *out, size_t out_size);
// Load <dir>/a2dp_xq_devices.txt (a missing file is not an error: empty store). Returns records or <0.
int xq_devices_load(xq_devices_t *d, const char *dir);
// Save when dirty (write to .tmp, then replace). Returns XQ_OK or <0.
int xq_devices_save(xq_devices_t *d, const char *dir);

xq_device_rec_t *xq_devices_find(xq_devices_t *d, const uint8_t addr[6]);
// Find or create (evicting the least recently seen record when full).
xq_device_rec_t *xq_devices_get(xq_devices_t *d, const uint8_t addr[6]);
void xq_devices_remove(xq_devices_t *d, const uint8_t addr[6]);
void xq_devices_touch(xq_devices_t *d, xq_device_rec_t *rec);
void xq_devices_set_name(xq_devices_t *d, xq_device_rec_t *rec, const char *name);
// Record the outcome of an XQ attempt: ok resets the failure counter, a failure increments
// it and after XQ_FAILS_TO_DISABLE marks the device xq=fail.
void xq_devices_xq_result(xq_devices_t *d, xq_device_rec_t *rec, bool ok);
// True when XQ may be tried with this device (no failure verdict, not blacklisted).
bool xq_devices_xq_allowed(const xq_device_rec_t *rec, const char *name);

// Built-in blacklist: sinks known to break with Dual Channel SBC (collected from the PipeWire
// Bluetooth quirks list and from reports of other open-source players). Case-insensitive
// substring match on the device name.
bool xq_name_blacklisted(const char *name);

// "aa:bb:cc:dd:ee:ff" <-> bytes. Accepts upper/lower case and '-' separators.
bool xq_addr_parse(const char *str, uint8_t out[6]);
void xq_addr_format(const uint8_t addr[6], char out[18]);

#ifdef __cplusplus
}
#endif
