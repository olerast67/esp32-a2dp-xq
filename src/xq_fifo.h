// SPDX-License-Identifier: Apache-2.0
// Single-producer single-consumer FIFO of 16-bit stereo frames between the audio task
// (writes Q31, converted with TPDF dither on the way in) and the A2DP data callback in the
// Bluetooth stack task (reads s16). Lock-free with C11 atomics, no allocation after init,
// storage in PSRAM through xq_malloc. Pure C: covered by test/test_a2dp_xq.c.
//
// On the ESP32 the xq_fifo_t itself must live in internal RAM (a static or internal heap
// object): atomic read-modify-write (S32C1I) does not work on PSRAM addresses. Only `buf`
// goes to PSRAM.
//
// Rules: exactly one producer thread calls the *_producer functions (write, flush, level,
// free_frames), exactly one consumer thread calls xq_fifo_read. init/free/reset only while
// neither side is active.
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t *buf;        // capacity * 2 samples, interleaved L/R
    uint32_t capacity;   // frames, power of two
    uint32_t mask;
    uint32_t limit;      // the producer never keeps more than this many frames queued
    uint32_t prebuffer;  // after an underrun the consumer waits for this level again

    _Atomic uint32_t wr;  // frames ever written (free-running, producer-owned)
    _Atomic uint32_t rd;  // frames ever read (free-running, consumer-owned)

    // Flush handshake: the producer publishes flush_at (a write position) and bumps
    // flush_seq; the consumer jumps its read position there and acknowledges with flush_ack.
    _Atomic uint32_t flush_at;
    _Atomic uint32_t flush_seq;
    _Atomic uint32_t flush_ack;

    // Consumer-only state.
    bool rebuffering;
    bool playing;

    // Statistics (written by the consumer, read anywhere).
    _Atomic uint32_t underruns;        // times the consumer ran dry while the producer was active
    _Atomic uint32_t underrun_frames;  // silence frames inserted by those underruns
    _Atomic uint32_t frames_out;       // real frames delivered
} xq_fifo_t;

// capacity_frames is rounded up to a power of two; limit_frames <= capacity (0 = capacity).
// Returns XQ_OK, XQ_EINVAL or XQ_ENOMEM.
int xq_fifo_init(xq_fifo_t *f, uint32_t capacity_frames, uint32_t limit_frames, uint32_t prebuffer_frames);
void xq_fifo_free(xq_fifo_t *f);
// Empty the FIFO and reset the state machine (statistics are kept). Not thread-safe.
void xq_fifo_reset(xq_fifo_t *f);
void xq_fifo_reset_stats(xq_fifo_t *f);

// ---- producer ----
// Frames that can be written now without exceeding the limit.
uint32_t xq_fifo_free_frames(const xq_fifo_t *f);
// Convert interleaved stereo Q31 to s16 with TPDF dither (xq_q31_to_s16_dither, state kept
// by the caller per stream) straight into the FIFO. Returns frames stored (<= frames).
uint32_t xq_fifo_write_q31(xq_fifo_t *f, const int32_t *pcm, uint32_t frames, uint32_t *dither_state);
// Same with a gain applied before the dither (the sink's own volume): gain_q30 is linear,
// 1 << 30 = unity (or more: no change), 0 = silence.
#define XQ_FIFO_GAIN_UNITY (1 << 30)
uint32_t xq_fifo_write_q31_gain(xq_fifo_t *f, const int32_t *pcm, uint32_t frames, uint32_t *dither_state,
                                 int32_t gain_q30);
// Store already converted s16 frames. Returns frames stored.
uint32_t xq_fifo_write_s16(xq_fifo_t *f, const int16_t *pcm, uint32_t frames);
// Drop everything written so far. Takes effect at the consumer's next read; level() already
// reports the flushed state.
void xq_fifo_flush(xq_fifo_t *f);
// Frames queued and not yet consumed (after a pending flush: frames written since it).
uint32_t xq_fifo_level(const xq_fifo_t *f);

// ---- consumer ----
// Fill exactly `frames` frames at out: real data while available, silence otherwise.
// producer_idle = the producer has stopped (end of stream, pause): the last frames below the
// prebuffer level are drained and running dry is not counted as an underrun.
// Returns the number of real frames delivered.
uint32_t xq_fifo_read(xq_fifo_t *f, int16_t *out, uint32_t frames, bool producer_idle);
// Consumer: drop everything queued now (a pending flush included) and wait for the prebuffer
// level again.
void xq_fifo_discard(xq_fifo_t *f);

#ifdef __cplusplus
}
#endif
