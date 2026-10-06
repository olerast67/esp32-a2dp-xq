// SPDX-License-Identifier: Apache-2.0
// Private interface between a2dp_xq.c (Bluetooth stack, GAP, A2DP/AVRCP state machine) and
// xq_sink.c (the PCM path written by the application and read by the A2DP data callback).
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "a2dp_xq.h"
#include "xq_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif

// PCM FIFO: 16384 frames (64 KiB) of storage, filled up to ~300 ms.
#define XQ_FIFO_CAPACITY_FRAMES 16384u
#define XQ_FIFO_LIMIT_FRAMES 13230u      // 300 ms
#define XQ_FIFO_PREBUFFER_FRAMES 4410u   // 100 ms before (re)starting after an underrun
// No write for this long = the producer stopped (end of a stream): drain instead of counting
// underruns.
#define XQ_PRODUCER_IDLE_MS 150u

// Sink side (xq_sink.c).
int xq_sink_init(void);     // allocate the FIFO and the writer wake-up semaphore
void xq_sink_deinit(void);  // free them (the A2DP callback must no longer run)
bool xq_sink_is_open(void);
bool xq_sink_is_paused(void);
// A2DP source data callback (esp_a2d_source_register_data_callback): fills `len` bytes of
// s16 stereo, silence when nothing is queued. Runs in the Bluedroid A2DP source task.
int32_t xq_sink_data_cb(uint8_t *buf, int32_t len);
void xq_sink_get_stats(a2dp_xq_stats_t *out);  // FIFO part of the statistics
// Own attenuation in the 16-bit conversion (0 dB while the headphones apply the volume). Any
// task; takes effect with the next write.
void xq_sink_set_gain_db(float db);
// Drop what is queued at the next A2DP read (the audio was made for other headphones, at
// another gain). Any task.
void xq_sink_discard(void);

// State machine side (a2dp_xq.c), called by the sink from the application task.
void xq_core_sink_open(bool open);
void xq_core_sink_paused(bool paused);

#ifdef __cplusplus
}
#endif
