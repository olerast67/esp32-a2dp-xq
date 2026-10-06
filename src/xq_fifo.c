// SPDX-License-Identifier: Apache-2.0
// Lock-free SPSC FIFO of s16 stereo frames (see xq_fifo.h).
#include "xq_fifo.h"

#include <string.h>

#include "xq_util.h"

#define LOAD_RELAXED(p) atomic_load_explicit((p), memory_order_relaxed)
#define LOAD_ACQUIRE(p) atomic_load_explicit((p), memory_order_acquire)
#define STORE_RELAXED(p, v) atomic_store_explicit((p), (v), memory_order_relaxed)
#define STORE_RELEASE(p, v) atomic_store_explicit((p), (v), memory_order_release)

static uint32_t round_up_pow2(uint32_t v) {
    uint32_t p = 1;
    while (p < v && p < 0x80000000u) p <<= 1;
    return p;
}

int xq_fifo_init(xq_fifo_t *f, uint32_t capacity_frames, uint32_t limit_frames, uint32_t prebuffer_frames) {
    if (!f || capacity_frames < 2 || capacity_frames > (1u << 24)) return XQ_EINVAL;
    memset(f, 0, sizeof(*f));
    uint32_t cap = round_up_pow2(capacity_frames);
    f->buf = (int16_t *)xq_malloc((size_t)cap * 2u * sizeof(int16_t));
    if (!f->buf) return XQ_ENOMEM;
    f->capacity = cap;
    f->mask = cap - 1u;
    f->limit = (limit_frames && limit_frames <= cap) ? limit_frames : cap;
    f->prebuffer = prebuffer_frames < f->limit ? prebuffer_frames : f->limit;
    xq_fifo_reset(f);
    return XQ_OK;
}

void xq_fifo_free(xq_fifo_t *f) {
    if (!f) return;
    xq_free(f->buf);
    f->buf = NULL;
    f->capacity = f->mask = f->limit = 0;
}

void xq_fifo_reset(xq_fifo_t *f) {
    if (!f) return;
    STORE_RELAXED(&f->wr, 0);
    STORE_RELAXED(&f->rd, 0);
    STORE_RELAXED(&f->flush_at, 0);
    STORE_RELAXED(&f->flush_seq, 0);
    STORE_RELAXED(&f->flush_ack, 0);
    f->rebuffering = true;
    f->playing = false;
    atomic_thread_fence(memory_order_seq_cst);
}

void xq_fifo_reset_stats(xq_fifo_t *f) {
    if (!f) return;
    STORE_RELAXED(&f->underruns, 0);
    STORE_RELAXED(&f->underrun_frames, 0);
    STORE_RELAXED(&f->frames_out, 0);
}

// ------------------------------------------------------------------ producer ----

uint32_t xq_fifo_free_frames(const xq_fifo_t *f) {
    if (!f || !f->buf) return 0;
    uint32_t w = LOAD_RELAXED((_Atomic uint32_t *)&f->wr);
    uint32_t r = LOAD_ACQUIRE((_Atomic uint32_t *)&f->rd);
    uint32_t used = w - r;
    return used >= f->limit ? 0 : f->limit - used;
}

uint32_t xq_fifo_level(const xq_fifo_t *f) {
    if (!f || !f->buf) return 0;
    uint32_t w = LOAD_RELAXED((_Atomic uint32_t *)&f->wr);
    uint32_t seq = LOAD_RELAXED((_Atomic uint32_t *)&f->flush_seq);
    uint32_t ack = LOAD_ACQUIRE((_Atomic uint32_t *)&f->flush_ack);
    uint32_t base = (seq != ack) ? LOAD_RELAXED((_Atomic uint32_t *)&f->flush_at)
                                 : LOAD_ACQUIRE((_Atomic uint32_t *)&f->rd);
    uint32_t used = w - base;
    return used > f->capacity ? f->capacity : used;
}

void xq_fifo_flush(xq_fifo_t *f) {
    if (!f || !f->buf) return;
    uint32_t w = LOAD_RELAXED(&f->wr);
    STORE_RELAXED(&f->flush_at, w);
    atomic_fetch_add_explicit(&f->flush_seq, 1u, memory_order_release);
}

typedef void (*span_fill_fn)(int16_t *dst, uint32_t frames, const void *src, uint32_t src_offset, void *ctx);

static uint32_t write_spans(xq_fifo_t *f, uint32_t frames, span_fill_fn fill, const void *src, void *ctx) {
    uint32_t n = xq_fifo_free_frames(f);
    if (frames < n) n = frames;
    if (!n) return 0;
    uint32_t w = LOAD_RELAXED(&f->wr);
    uint32_t idx = w & f->mask;
    uint32_t first = f->capacity - idx;
    if (first > n) first = n;
    fill(f->buf + (size_t)idx * 2u, first, src, 0, ctx);
    if (n > first) fill(f->buf, n - first, src, first, ctx);
    STORE_RELEASE(&f->wr, w + n);
    return n;
}

typedef struct {
    uint32_t *dither;
    int32_t gain_q30;
} q31_ctx_t;

#define GAIN_BLOCK_SAMPLES 64u

static void fill_q31(int16_t *dst, uint32_t frames, const void *src, uint32_t off, void *ctx) {
    const q31_ctx_t *c = (const q31_ctx_t *)ctx;
    const int32_t *pcm = (const int32_t *)src + (size_t)off * 2u;
    size_t samples = (size_t)frames * 2u;
    if (c->gain_q30 >= XQ_FIFO_GAIN_UNITY) {
        xq_q31_to_s16_dither(pcm, dst, samples, c->dither);
        return;
    }
    // Attenuate in Q31 (|gain| <= 1: no overflow), then one dither to 16 bits.
    int32_t tmp[GAIN_BLOCK_SAMPLES];
    const int64_t g = c->gain_q30 > 0 ? c->gain_q30 : 0;
    for (size_t done = 0; done < samples;) {
        size_t n = samples - done < GAIN_BLOCK_SAMPLES ? samples - done : GAIN_BLOCK_SAMPLES;
        for (size_t i = 0; i < n; i++) tmp[i] = (int32_t)(((int64_t)pcm[done + i] * g + (1 << 29)) >> 30);
        xq_q31_to_s16_dither(tmp, dst + done, n, c->dither);
        done += n;
    }
}

static void fill_s16(int16_t *dst, uint32_t frames, const void *src, uint32_t off, void *ctx) {
    (void)ctx;
    memcpy(dst, (const int16_t *)src + (size_t)off * 2u, (size_t)frames * 2u * sizeof(int16_t));
}

uint32_t xq_fifo_write_q31_gain(xq_fifo_t *f, const int32_t *pcm, uint32_t frames, uint32_t *dither_state,
                                 int32_t gain_q30) {
    if (!f || !f->buf || !pcm || !frames || !dither_state) return 0;
    q31_ctx_t c = {.dither = dither_state, .gain_q30 = gain_q30};
    return write_spans(f, frames, fill_q31, pcm, &c);
}

uint32_t xq_fifo_write_q31(xq_fifo_t *f, const int32_t *pcm, uint32_t frames, uint32_t *dither_state) {
    return xq_fifo_write_q31_gain(f, pcm, frames, dither_state, XQ_FIFO_GAIN_UNITY);
}

uint32_t xq_fifo_write_s16(xq_fifo_t *f, const int16_t *pcm, uint32_t frames) {
    if (!f || !f->buf || !pcm || !frames) return 0;
    return write_spans(f, frames, fill_s16, pcm, NULL);
}

// ------------------------------------------------------------------ consumer ----

static void apply_flush(xq_fifo_t *f) {
    uint32_t seq = LOAD_ACQUIRE(&f->flush_seq);
    if (seq == LOAD_RELAXED(&f->flush_ack)) return;
    uint32_t at = LOAD_RELAXED(&f->flush_at);
    uint32_t r = LOAD_RELAXED(&f->rd);
    if ((int32_t)(at - r) > 0) STORE_RELEASE(&f->rd, at);
    STORE_RELEASE(&f->flush_ack, seq);
    f->rebuffering = true;
    f->playing = false;
}

void xq_fifo_discard(xq_fifo_t *f) {
    if (!f || !f->buf) return;
    apply_flush(f);
    STORE_RELEASE(&f->rd, LOAD_ACQUIRE(&f->wr));
    f->rebuffering = true;
    f->playing = false;
}

uint32_t xq_fifo_read(xq_fifo_t *f, int16_t *out, uint32_t frames, bool producer_idle) {
    if (!out || !frames) return 0;
    if (!f || !f->buf) {
        memset(out, 0, (size_t)frames * 2u * sizeof(int16_t));
        return 0;
    }
    apply_flush(f);

    uint32_t w = LOAD_ACQUIRE(&f->wr);
    uint32_t r = LOAD_RELAXED(&f->rd);
    uint32_t avail = w - r;
    if (avail > f->capacity) avail = 0;  // cannot happen with a single producer; stay safe

    if (f->rebuffering) {
        if (avail >= f->prebuffer || (producer_idle && avail > 0)) {
            f->rebuffering = false;
        } else {
            memset(out, 0, (size_t)frames * 2u * sizeof(int16_t));
            return 0;
        }
    }

    uint32_t n = avail < frames ? avail : frames;
    if (n) {
        uint32_t idx = r & f->mask;
        uint32_t first = f->capacity - idx;
        if (first > n) first = n;
        memcpy(out, f->buf + (size_t)idx * 2u, (size_t)first * 2u * sizeof(int16_t));
        if (n > first) memcpy(out + (size_t)first * 2u, f->buf, (size_t)(n - first) * 2u * sizeof(int16_t));
        STORE_RELEASE(&f->rd, r + n);
        atomic_fetch_add_explicit(&f->frames_out, n, memory_order_relaxed);
        f->playing = true;
    }
    if (n < frames) {
        memset(out + (size_t)n * 2u, 0, (size_t)(frames - n) * 2u * sizeof(int16_t));
        if (f->playing && !producer_idle) {
            atomic_fetch_add_explicit(&f->underruns, 1u, memory_order_relaxed);
            atomic_fetch_add_explicit(&f->underrun_frames, frames - n, memory_order_relaxed);
        }
        // Ran dry: wait for the prebuffer level before playing again (avoids stuttering
        // through a slow producer one small chunk at a time).
        f->rebuffering = true;
        f->playing = false;
    }
    return n;
}
