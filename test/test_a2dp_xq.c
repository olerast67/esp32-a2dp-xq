// SPDX-License-Identifier: Apache-2.0
// Host tests for the pure-C parts of esp32-a2dp-xq: SBC arithmetic and the XQ decision, the
// PCM FIFO, the volume scales and the per-device memory file.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "test.h"
#include "xq_devices.h"
#include "xq_fifo.h"
#include "xq_sbc.h"
#include "xq_util.h"
#include "xq_volume.h"

// Test double for the TPDF dither (the real one is src/xq_dither.c): truncation, which makes
// the FIFO contents predictable. The state is advanced so misuse would be visible.
void xq_q31_to_s16_dither(const int32_t *in, int16_t *out, size_t samples, uint32_t *state) {
    for (size_t i = 0; i < samples; i++) out[i] = (int16_t)(in[i] >> 16);
    *state += (uint32_t)samples;
}

static xq_sbc_params_t sbc(uint32_t rate, xq_sbc_mode_t mode, uint8_t bp) {
    xq_sbc_params_t p = {rate, mode, 16, 8, bp};
    return p;
}

static void fill_seq(int16_t *buf, uint32_t frames, int16_t start) {
    for (uint32_t i = 0; i < frames; i++) {
        buf[2 * i] = (int16_t)(start + (int16_t)i);
        buf[2 * i + 1] = (int16_t)-(start + (int16_t)i);
    }
}

TEST(sbc_frame_and_bitrate) {
    xq_sbc_params_t p = sbc(44100, XQ_SBC_MODE_JOINT, 53);
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 119);
    CHECK_EQ_INT(xq_sbc_bitrate_bps(&p), 327993);
    p = sbc(44100, XQ_SBC_MODE_DUAL, 38);
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 164);
    CHECK_EQ_INT(xq_sbc_bitrate_bps(&p), 452025);
    p = sbc(48000, XQ_SBC_MODE_DUAL, 38);
    CHECK_EQ_INT(xq_sbc_bitrate_bps(&p), 492000);
    p = sbc(44100, XQ_SBC_MODE_DUAL, 47);
    CHECK_EQ_INT(xq_sbc_bitrate_bps(&p) / 1000, 551);  // Dual Channel at bitpool 47 (sometimes called SBC-XQ+)
    p = sbc(44100, XQ_SBC_MODE_MONO, 31);
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 70);
    p = sbc(44100, XQ_SBC_MODE_STEREO, 53);
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 118);
    p = sbc(44100, XQ_SBC_MODE_JOINT, 0);
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 0);
    p = sbc(22050, XQ_SBC_MODE_JOINT, 53);
    CHECK_EQ_INT(xq_sbc_bitrate_bps(&p), 0);
    p = sbc(44100, XQ_SBC_MODE_JOINT, 53);
    p.blocks = 5;
    CHECK_EQ_INT(xq_sbc_frame_length(&p), 0);
    CHECK_EQ_INT(xq_sbc_frame_length(NULL), 0);
    CHECK_STR(xq_sbc_mode_name(XQ_SBC_MODE_DUAL), "Dual Channel");
    CHECK_STR(xq_sbc_mode_name(XQ_SBC_MODE_JOINT), "Joint Stereo");
}

TEST(sbc_cie_decode) {
    xq_sbc_cie_t c;
    xq_sbc_xq_config(&c);
    xq_sbc_params_t p;
    CHECK(xq_sbc_params_from_cie(&c, &p));
    CHECK_EQ_INT(p.sample_rate, 44100);
    CHECK_EQ_INT(p.mode, XQ_SBC_MODE_DUAL);
    CHECK_EQ_INT(p.blocks, 16);
    CHECK_EQ_INT(p.subbands, 8);
    CHECK_EQ_INT(c.min_bitpool, 38);
    CHECK_EQ_INT(c.max_bitpool, 38);
    xq_sbc_joint_config(&c);
    CHECK(xq_sbc_params_from_cie(&c, &p));
    CHECK_EQ_INT(p.mode, XQ_SBC_MODE_JOINT);
    CHECK_EQ_INT(c.max_bitpool, 53);
    c.ch_mode = XQ_SBC_CIE_CH_JOINT | XQ_SBC_CIE_CH_DUAL;  // capabilities, not a configuration
    CHECK(!xq_sbc_params_from_cie(&c, &p));
    c.ch_mode = 0;
    CHECK(!xq_sbc_params_from_cie(&c, &p));
    CHECK(!xq_sbc_params_from_cie(NULL, &p));
}

TEST(sbc_bluedroid_bitpool_search) {
    xq_sbc_params_t joint = sbc(44100, XQ_SBC_MODE_JOINT, 0), dual = sbc(44100, XQ_SBC_MODE_DUAL, 0);
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&joint, 2, 53), 53);   // default: 328 kbps
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&dual, 2, 53), 26);    // Dual at 328 kbps is worse than Joint 53
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&dual, 38, 38), 38);   // XQ: the pinned range forces 38
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&joint, 2, 35), 35);   // sink with a small max bitpool
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&dual, 47, 47), 47);
    uint8_t bp = xq_sbc_bluedroid_bitpool(&joint, 2, 2);
    CHECK(bp <= 2);
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&joint, 10, 5), 0);    // invalid range
    xq_sbc_params_t bad = sbc(12345, XQ_SBC_MODE_JOINT, 0);
    CHECK_EQ_INT(xq_sbc_bluedroid_bitpool(&bad, 2, 53), 0);
}

TEST(sbc_xq_caps) {
    xq_sbc_cie_t caps = {0x3, 0xF, 0xF, 0x3, 0x3, 2, 53};  // a typical sink
    CHECK(xq_sbc_caps_allow_xq(&caps));
    caps.max_bitpool = 37;  // Samsung Gear IconX
    CHECK(!xq_sbc_caps_allow_xq(&caps));
    caps.max_bitpool = 39;  // Bluedio T
    CHECK(xq_sbc_caps_allow_xq(&caps));
    caps.ch_mode = XQ_SBC_CIE_CH_JOINT | XQ_SBC_CIE_CH_STEREO;
    CHECK(!xq_sbc_caps_allow_xq(&caps));
    caps.ch_mode = 0xF;
    caps.samp_freq = XQ_SBC_CIE_SF_48K;
    CHECK(!xq_sbc_caps_allow_xq(&caps));
    caps.samp_freq = 0x3;
    caps.min_bitpool = 40;
    caps.max_bitpool = 60;
    CHECK(!xq_sbc_caps_allow_xq(&caps));
    CHECK(!xq_sbc_caps_allow_xq(NULL));
}

// ================================================================= PCM FIFO ====

TEST(fifo_init_and_limits) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 1000, 0, 100), XQ_OK);
    CHECK_EQ_INT(f.capacity, 1024);
    CHECK_EQ_INT(f.limit, 1024);
    CHECK_EQ_INT(xq_fifo_free_frames(&f), 1024);
    xq_fifo_free(&f);
    CHECK_EQ_INT(xq_fifo_init(&f, 16, 12, 4), XQ_OK);
    int16_t in[40];
    fill_seq(in, 20, 1);
    CHECK_EQ_INT(xq_fifo_write_s16(&f, in, 20), 12);  // stops at the limit
    CHECK_EQ_INT(xq_fifo_level(&f), 12);
    CHECK_EQ_INT(xq_fifo_free_frames(&f), 0);
    CHECK_EQ_INT(xq_fifo_write_s16(&f, in, 1), 0);
    xq_fifo_free(&f);
    CHECK_EQ_INT(xq_fifo_init(&f, 1, 0, 0), XQ_EINVAL);
    CHECK_EQ_INT(xq_fifo_init(NULL, 16, 0, 0), XQ_EINVAL);
}

TEST(fifo_wraparound) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 16, 16, 1), XQ_OK);
    int16_t in[64], out[64];
    int16_t next = 1, expect = 1;
    for (int round = 0; round < 20; round++) {
        uint32_t n = 3u + (uint32_t)(round * 7) % 11u;  // 3..13 frames, crosses the end often
        fill_seq(in, n, next);
        CHECK_EQ_INT(xq_fifo_write_s16(&f, in, n), n);
        next = (int16_t)(next + (int16_t)n);
        CHECK_EQ_INT(xq_fifo_read(&f, out, n, false), n);
        for (uint32_t i = 0; i < n; i++) {
            CHECK_EQ_INT(out[2 * i], expect);
            CHECK_EQ_INT(out[2 * i + 1], -expect);
            expect++;
        }
    }
    CHECK_EQ_INT(f.underruns, 0);
    xq_fifo_free(&f);
}

TEST(fifo_prebuffer_and_underrun) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 64, 64, 8), XQ_OK);
    int16_t in[40], out[40];
    fill_seq(in, 4, 100);
    xq_fifo_write_s16(&f, in, 4);
    memset(out, 0x55, sizeof out);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 10, false), 0);  // below the prebuffer level: silence
    CHECK_EQ_INT(out[0], 0);
    CHECK_EQ_INT(out[19], 0);
    CHECK_EQ_INT(f.underruns, 0);  // not playing yet: not an underrun
    fill_seq(in, 4, 104);
    xq_fifo_write_s16(&f, in, 4);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 10, false), 8);  // plays, then runs dry
    CHECK_EQ_INT(out[0], 100);
    CHECK_EQ_INT(out[14], 107);
    CHECK_EQ_INT(out[16], 0);
    CHECK_EQ_INT(f.underruns, 1);
    CHECK_EQ_INT(f.underrun_frames, 2);
    CHECK_EQ_INT(f.frames_out, 8);
    // After the underrun it waits for the prebuffer again.
    fill_seq(in, 3, 200);
    xq_fifo_write_s16(&f, in, 3);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 2, false), 0);
    CHECK_EQ_INT(f.underruns, 1);
    // End of stream: the producer is idle, the tail drains without counting an underrun.
    CHECK_EQ_INT(xq_fifo_read(&f, out, 10, true), 3);
    CHECK_EQ_INT(out[0], 200);
    CHECK_EQ_INT(f.underruns, 1);
    xq_fifo_reset_stats(&f);
    CHECK_EQ_INT(f.underruns, 0);
    xq_fifo_free(&f);
}

TEST(fifo_flush) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 64, 64, 4), XQ_OK);
    int16_t in[80], out[80];
    fill_seq(in, 20, 1);
    xq_fifo_write_s16(&f, in, 20);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 5, false), 5);
    xq_fifo_flush(&f);
    CHECK_EQ_INT(xq_fifo_level(&f), 0);  // reported before the consumer applied it
    fill_seq(in, 6, 500);
    xq_fifo_write_s16(&f, in, 6);
    CHECK_EQ_INT(xq_fifo_level(&f), 6);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 6, false), 6);  // only the audio written after the flush
    CHECK_EQ_INT(out[0], 500);
    CHECK_EQ_INT(out[10], 505);
    CHECK_EQ_INT(xq_fifo_level(&f), 0);
    CHECK_EQ_INT(f.underruns, 0);
    // A flush on an empty FIFO and a double flush are harmless.
    xq_fifo_flush(&f);
    xq_fifo_flush(&f);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 4, true), 0);
    xq_fifo_free(&f);
}

TEST(fifo_q31_conversion) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 8, 8, 1), XQ_OK);
    int32_t q31[12] = {0x12340000, -0x10000, INT32_MAX, INT32_MIN, 0, 0x7FFF0000,
                       0x00010000, -0x7FFF0000, 1, -1, 0x40000000, -0x40000000};
    uint32_t dither = 0;
    CHECK_EQ_INT(xq_fifo_write_q31(&f, q31, 6, &dither), 6);
    CHECK_EQ_INT(dither, 12);
    CHECK_EQ_INT(xq_fifo_write_q31(&f, q31, 6, &dither), 2);  // wraps and stops at the limit
    int16_t out[16];
    CHECK_EQ_INT(xq_fifo_read(&f, out, 8, false), 8);
    CHECK_EQ_INT(out[0], 0x1234);
    CHECK_EQ_INT(out[1], -1);
    CHECK_EQ_INT(out[2], 32767);
    CHECK_EQ_INT(out[3], -32768);
    CHECK_EQ_INT(out[12], 0x1234);
    CHECK_EQ_INT(xq_fifo_write_q31(&f, NULL, 2, &dither), 0);
    CHECK_EQ_INT(xq_fifo_write_q31(&f, q31, 2, NULL), 0);
    xq_fifo_free(&f);
}

// Pseudo-random interleaving of writes, reads and flushes. Invariant: the consumer only ever
// sees an increasing sequence, and after a flush nothing written before it.

TEST(fifo_interleaving_stress) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 128, 100, 16), XQ_OK);
    uint32_t seed = 12345;
    int16_t in[2 * 64], out[2 * 64];
    int16_t next = 1, last_seen = 0, flush_floor = 0;
    uint32_t bad = 0;
    for (int step = 0; step < 20000; step++) {
        seed = seed * 1664525u + 1013904223u;
        uint32_t op = seed >> 28, n = 1u + ((seed >> 8) % 60u);
        if (op < 7) {
            fill_seq(in, n, next);
            uint32_t w = xq_fifo_write_s16(&f, in, n);
            next = (int16_t)(next + (int16_t)w);
            if (next > 30000) {  // keep values in range: restart the sequence after a flush
                xq_fifo_flush(&f);
                next = 1;
                last_seen = 0;
                flush_floor = 0;
                xq_fifo_read(&f, out, 1, true);  // consumer applies the flush
                xq_fifo_read(&f, out, 64, true);
            }
        } else if (op < 15) {
            uint32_t got = xq_fifo_read(&f, out, n, (seed & 1u) != 0);
            for (uint32_t i = 0; i < got; i++) {
                int16_t v = out[2 * i];
                if (v <= last_seen || v < flush_floor || out[2 * i + 1] != -v) bad++;
                last_seen = v;
            }
            for (uint32_t i = got; i < n; i++) {
                if (out[2 * i] || out[2 * i + 1]) bad++;
            }
        } else {
            xq_fifo_flush(&f);
            flush_floor = next;
        }
        if (xq_fifo_level(&f) > f.limit) bad++;
    }
    CHECK_EQ_INT(bad, 0);
    CHECK(f.frames_out > 1000);
    xq_fifo_free(&f);
}

// ========================================================== device memory ====

TEST(fifo_q31_gain) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 256, 0, 0), XQ_OK);
    int32_t in[2 * 100];
    for (int i = 0; i < 200; i++) in[i] = (i & 1 ? -1 : 1) * (int32_t)(0x40000000 + i * 0x10000);
    uint32_t dither = 0;
    // Unity (and above): the plain conversion.
    CHECK_EQ_INT(xq_fifo_write_q31_gain(&f, in, 10, &dither, XQ_FIFO_GAIN_UNITY), 10);
    CHECK_EQ_INT(xq_fifo_write_q31_gain(&f, in, 10, &dither, INT32_MAX), 10);
    // Half gain (-6.02 dB) on 100 frames: more than one internal block of 64 samples.
    CHECK_EQ_INT(xq_fifo_write_q31_gain(&f, in, 100, &dither, XQ_FIFO_GAIN_UNITY / 2), 100);
    CHECK_EQ_INT(xq_fifo_write_q31_gain(&f, in, 10, &dither, 0), 10);  // silence
    CHECK_EQ_INT(xq_fifo_write_q31_gain(&f, in, 10, &dither, -5), 10);  // negative = silence
    int16_t out[2 * 140];
    CHECK_EQ_INT(xq_fifo_read(&f, out, 140, true), 140);
    int bad = 0;
    for (int i = 0; i < 20; i++) {
        if (out[i] != (int16_t)(in[i] >> 16)) bad++;
        if (out[20 + i] != (int16_t)(in[i] >> 16)) bad++;
    }
    for (int i = 0; i < 200; i++) {
        int32_t half = (int32_t)(((int64_t)in[i] * (XQ_FIFO_GAIN_UNITY / 2) + (1 << 29)) >> 30);
        if (out[40 + i] != (int16_t)(half >> 16)) bad++;
    }
    for (int i = 0; i < 40; i++) {
        if (out[240 + i] != 0) bad++;
    }
    CHECK_EQ_INT(bad, 0);
    CHECK_EQ_INT(dither, 2u * 140u);  // one dither pass per sample, whatever the gain
    xq_fifo_free(&f);
}

TEST(fifo_discard) {
    xq_fifo_t f;
    CHECK_EQ_INT(xq_fifo_init(&f, 64, 0, 8), XQ_OK);
    int16_t in[2 * 32], out[2 * 16];
    fill_seq(in, 32, 1);
    CHECK_EQ_INT(xq_fifo_write_s16(&f, in, 32), 32);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 8, false), 8);
    xq_fifo_flush(&f);  // pending flush and discard together
    CHECK_EQ_INT(xq_fifo_write_s16(&f, in, 4), 4);
    xq_fifo_discard(&f);
    CHECK_EQ_INT(xq_fifo_level(&f), 0);
    // Waits for the prebuffer again, then plays only what came after the discard.
    CHECK_EQ_INT(xq_fifo_write_s16(&f, in + 2 * 10, 8), 8);
    CHECK_EQ_INT(xq_fifo_read(&f, out, 16, false), 8);
    CHECK_EQ_INT(out[0], in[2 * 10]);
    xq_fifo_free(&f);
}

TEST(volume_scale) {
    CHECK_NEAR(a2dp_xq_volume_to_db(0), -100.0, 1e-6);
    CHECK_NEAR(a2dp_xq_volume_to_db(-5), -100.0, 1e-6);
    CHECK_NEAR(a2dp_xq_volume_to_db(127), 0.0, 1e-6);
    CHECK_NEAR(a2dp_xq_volume_to_db(300), 0.0, 1e-6);
    CHECK_NEAR(a2dp_xq_volume_to_db(1), -60.0 + 60.0 / 127.0, 1e-4);
    float prev = -1000.0f;
    int mismatches = 0;
    for (int v = 0; v <= 127; v++) {
        float db = a2dp_xq_volume_to_db(v);
        CHECK(db > prev);  // strictly increasing
        prev = db;
        // Round trip is exact: a volume the headphones report is not sent back changed.
        if (a2dp_xq_volume_from_db(db) != v) mismatches++;
    }
    CHECK_EQ_INT(mismatches, 0);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(0.0f), 127);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(6.0f), 127);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(-60.0f), 0);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(-96.0f), 0);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(-INFINITY), 0);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(NAN), 0);
    CHECK_EQ_INT(a2dp_xq_volume_from_db(-30.0f), 64);  // (30 / 60) * 127 = 63.5 -> 64
    // Monotonic over the whole player range.
    uint8_t pv = 0;
    for (float db = -96.0f; db <= 0.0f; db += 0.25f) {
        uint8_t v = a2dp_xq_volume_from_db(db);
        CHECK(v >= pv);
        pv = v;
    }
}

TEST(volume_gain) {
    CHECK_EQ_INT(xq_volume_gain_q30(0.0f), 1 << 30);
    CHECK_EQ_INT(xq_volume_gain_q30(3.0f), 1 << 30);
    CHECK_NEAR(xq_volume_gain_q30(-6.0206f), (1 << 29), 2000.0);
    CHECK_NEAR(xq_volume_gain_q30(-20.0f), (double)(1 << 30) / 10.0, 2.0);
    CHECK_NEAR(xq_volume_gain_q30(-60.0f), (double)(1 << 30) / 1000.0, 1.0);
    CHECK_EQ_INT(xq_volume_gain_q30(-121.0f), 0);
    CHECK_EQ_INT(xq_volume_gain_q30(-INFINITY), 0);
    CHECK_EQ_INT(xq_volume_gain_q30(NAN), 0);
    CHECK(xq_volume_gain_q30(-100.0f) > 0);
    int32_t prev = 0;
    for (float db = -110.0f; db <= 0.0f; db += 0.5f) {
        int32_t g = xq_volume_gain_q30(db);
        CHECK(g >= prev);
        prev = g;
    }
}


TEST(addr_parse_format) {
    uint8_t a[6];
    CHECK(xq_addr_parse("AA:bb:0C:dd:ee:FF", a));
    CHECK_EQ_INT(a[0], 0xAA);
    CHECK_EQ_INT(a[2], 0x0C);
    CHECK_EQ_INT(a[5], 0xFF);
    char s[18];
    xq_addr_format(a, s);
    CHECK_STR(s, "aa:bb:0c:dd:ee:ff");
    CHECK(xq_addr_parse("00-11-22-33-44-55", a));
    CHECK_EQ_INT(a[5], 0x55);
    CHECK(!xq_addr_parse("aa:bb:cc:dd:ee", a));
    CHECK(!xq_addr_parse("aa:bb:cc:dd:ee:fg", a));
    CHECK(!xq_addr_parse("aa:bb:cc:dd:ee:ff0", a));
    CHECK(!xq_addr_parse("aabbccddeeff", a));
    CHECK(!xq_addr_parse("", a));
    CHECK(!xq_addr_parse(NULL, a));
}

static const uint8_t ADDR1[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01};
static const uint8_t ADDR2[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x02};

TEST(devices_parse_file) {
    const char *text =
        "# esp32-a2dp-xq devices v1\r\n"
        "aa:bb:cc:dd:ee:01\txq=ok\tfails=0\tnoxq=0\tseen=7\tname=Sony WH-1000XM4\r\n"
        "garbage line without address\n"
        "\n"
        "zz:bb:cc:dd:ee:03\txq=ok\tname=bad address\n"
        "aa:bb:cc:dd:ee:02\txq=fail\tfails=2\tnoxq=1\tseen=9\tfuture=1\tname=Наушники\tс табом\n"
        "aa:bb:cc:dd:ee:04\txq=weird\tfails=999\tseen=abc\n";
    xq_devices_t d;
    xq_devices_init(&d);
    CHECK_EQ_INT(xq_devices_parse(&d, text, strlen(text)), 3);
    CHECK_EQ_INT(d.count, 3);
    xq_device_rec_t *r = xq_devices_find(&d, ADDR1);
    CHECK(r != NULL);
    if (r) {
        CHECK_EQ_INT(r->xq, XQ_VERDICT_OK);
        CHECK_EQ_INT(r->seen, 7);
        CHECK_STR(r->name, "Sony WH-1000XM4");
    }
    r = xq_devices_find(&d, ADDR2);
    CHECK(r != NULL);
    if (r) {
        CHECK_EQ_INT(r->xq, XQ_VERDICT_FAIL);
        CHECK_EQ_INT(r->xq_fails, 2);
        CHECK(r->no_xq);
        CHECK_STR(r->name, "Наушники с табом");  // the name is the rest of the line
    }
    uint8_t a4[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x04};
    r = xq_devices_find(&d, a4);
    CHECK(r != NULL);
    if (r) {
        CHECK_EQ_INT(r->xq, XQ_VERDICT_UNKNOWN);
        CHECK_EQ_INT(r->xq_fails, 255);
        CHECK_EQ_INT(r->seen, 0);
    }
    CHECK_EQ_INT(d.clock, 9);
    CHECK(!d.dirty);
}

TEST(devices_roundtrip_and_names) {
    xq_devices_t d, e;
    xq_devices_init(&d);
    xq_device_rec_t *r = xq_devices_get(&d, ADDR1);
    xq_devices_set_name(&d, r, "Head\tphones\nX");
    CHECK_STR(r->name, "Head phones X");
    xq_devices_touch(&d, r);
    r = xq_devices_get(&d, ADDR2);
    // 70 Cyrillic letters (2 bytes each) do not fit: cut on a character boundary.
    char longname[200] = "";
    for (int i = 0; i < 70; i++) strcat(longname, "Ж");
    xq_devices_set_name(&d, r, longname);
    size_t len = strlen(r->name);
    CHECK(len <= XQ_NAME_MAX - 1);
    CHECK_EQ_INT(len % 2, 0);
    r->xq = XQ_VERDICT_OK;
    r->no_xq = true;
    xq_devices_touch(&d, r);
    CHECK(d.dirty);

    char buf[2048];
    int n = xq_devices_format(&d, buf, sizeof buf);
    CHECK(n > 0);
    xq_devices_init(&e);
    CHECK_EQ_INT(xq_devices_parse(&e, buf, (size_t)n), 2);
    CHECK_EQ_INT(e.count, 2);
    CHECK(memcmp(&d.rec[0], &e.rec[0], sizeof d.rec[0]) == 0);
    CHECK(memcmp(&d.rec[1], &e.rec[1], sizeof d.rec[1]) == 0);
    CHECK_EQ_INT(xq_devices_format(&d, buf, 10), XQ_EINVAL);  // too small: error, no overflow
}

TEST(devices_eviction_and_remove) {
    xq_devices_t d;
    xq_devices_init(&d);
    for (int i = 0; i < XQ_DEVICES_MAX; i++) {
        uint8_t a[6] = {1, 2, 3, 4, 5, (uint8_t)i};
        xq_device_rec_t *r = xq_devices_get(&d, a);
        xq_devices_touch(&d, r);
    }
    CHECK_EQ_INT(d.count, XQ_DEVICES_MAX);
    uint8_t a3[6] = {1, 2, 3, 4, 5, 3};
    xq_devices_touch(&d, xq_devices_find(&d, a3));  // a3 is recent, a0 is now the oldest
    uint8_t anew[6] = {9, 9, 9, 9, 9, 9};
    CHECK(xq_devices_get(&d, anew) != NULL);
    CHECK_EQ_INT(d.count, XQ_DEVICES_MAX);
    uint8_t a0[6] = {1, 2, 3, 4, 5, 0};
    CHECK(xq_devices_find(&d, a0) == NULL);
    CHECK(xq_devices_find(&d, a3) != NULL);
    xq_devices_remove(&d, a3);
    CHECK(xq_devices_find(&d, a3) == NULL);
    CHECK_EQ_INT(d.count, XQ_DEVICES_MAX - 1);
    xq_devices_remove(&d, a3);  // absent: no-op
    CHECK_EQ_INT(d.count, XQ_DEVICES_MAX - 1);
}

TEST(devices_xq_policy) {
    xq_devices_t d;
    xq_devices_init(&d);
    xq_device_rec_t *r = xq_devices_get(&d, ADDR1);
    CHECK(xq_devices_xq_allowed(r, "Sony WH-1000XM4"));
    xq_devices_xq_result(&d, r, false);
    CHECK_EQ_INT(r->xq, XQ_VERDICT_UNKNOWN);
    CHECK(xq_devices_xq_allowed(r, NULL));
    xq_devices_xq_result(&d, r, false);
    CHECK_EQ_INT(r->xq, XQ_VERDICT_FAIL);
    CHECK(!xq_devices_xq_allowed(r, NULL));
    xq_devices_xq_result(&d, r, true);
    CHECK_EQ_INT(r->xq, XQ_VERDICT_OK);
    CHECK_EQ_INT(r->xq_fails, 0);
    r->no_xq = true;
    CHECK(!xq_devices_xq_allowed(r, NULL));
    CHECK(xq_devices_xq_allowed(NULL, "JBL Tune 510BT"));
    CHECK(!xq_devices_xq_allowed(NULL, "Soundcore 2"));
    CHECK(!xq_devices_xq_allowed(NULL, "SOUNDCORE 2"));
    CHECK(xq_devices_xq_allowed(NULL, "Soundcore 20i"));
    CHECK(!xq_devices_xq_allowed(NULL, "CMF Buds 2a"));
    CHECK(!xq_devices_xq_allowed(NULL, "Phonak Audeo"));
    CHECK(!xq_devices_xq_allowed(NULL, "Motorola S305"));
    CHECK(xq_name_blacklisted("PMK-TWS"));
    CHECK(!xq_name_blacklisted(""));
    CHECK(!xq_name_blacklisted(NULL));
}

TEST(devices_file_io) {
    xq_devices_t d, e;
    xq_devices_init(&d);
    xq_device_rec_t *r = xq_devices_get(&d, ADDR1);
    xq_devices_set_name(&d, r, "Test Buds");
    xq_devices_touch(&d, r);
    CHECK_EQ_INT(xq_devices_save(&d, TEST_TMP_DIR), XQ_OK);
    CHECK(!d.dirty);
    CHECK_EQ_INT(xq_devices_save(&d, TEST_TMP_DIR), XQ_OK);  // clean: nothing written
    xq_devices_init(&e);
    CHECK_EQ_INT(xq_devices_load(&e, TEST_TMP_DIR), 1);
    CHECK(xq_devices_find(&e, ADDR1) != NULL);
    // Overwrite an existing file.
    xq_devices_get(&d, ADDR2);
    CHECK_EQ_INT(xq_devices_save(&d, TEST_TMP_DIR), XQ_OK);
    xq_devices_init(&e);
    CHECK_EQ_INT(xq_devices_load(&e, TEST_TMP_DIR), 2);
    // Missing directory / file: empty store, no error.
    xq_devices_init(&e);
    CHECK_EQ_INT(xq_devices_load(&e, TEST_TMP_DIR "/does_not_exist"), 0);
    CHECK_EQ_INT(e.count, 0);
    CHECK(xq_devices_save(&d, "") < 0);
    d.dirty = true;
    CHECK(xq_devices_save(&d, TEST_TMP_DIR "/does_not_exist") < 0);
}

// Random input must never crash the parser or produce unterminated names.

TEST(devices_parse_fuzz) {
    static const char alphabet[] = "aAbBcCfF0123456789:-\t\n\r =xqokfailsnoxqseenname#Ж";
    uint32_t seed = 777;
    char buf[400];
    uint32_t bad = 0;
    for (int iter = 0; iter < 3000; iter++) {
        seed = seed * 1103515245u + 12345u;
        size_t len = (seed >> 16) % sizeof buf;
        for (size_t i = 0; i < len; i++) {
            seed = seed * 1103515245u + 12345u;
            uint32_t r = seed >> 16;
            buf[i] = (r & 7) == 0 ? (char)(r >> 3) : alphabet[(r >> 3) % (sizeof alphabet - 1)];
        }
        // Plant valid-looking addresses now and then.
        if (len > 40 && (iter % 3) == 0) memcpy(buf + 5, "\naa:bb:cc:dd:ee:ff\tname=", 24);
        xq_devices_t d;
        xq_devices_init(&d);
        int n = xq_devices_parse(&d, buf, len);
        if (n < 0 || d.count > XQ_DEVICES_MAX) bad++;
        for (uint32_t i = 0; i < d.count; i++) {
            if (memchr(d.rec[i].name, '\0', sizeof d.rec[i].name) == NULL) bad++;
            if (d.rec[i].xq > XQ_VERDICT_FAIL) bad++;
        }
    }
    CHECK_EQ_INT(bad, 0);
}

TEST(sbc_cie_from_bytes) {
    // 44.1 kHz Joint Stereo, 16 blocks, 8 subbands, loudness, bitpool 2..53, as sent on the air.
    const uint8_t joint[4] = {0x21, 0x15, 2, 53};
    xq_sbc_cie_t c;
    xq_sbc_cie_from_bytes(joint, &c);
    CHECK_EQ_INT(c.samp_freq, XQ_SBC_CIE_SF_44K);
    CHECK_EQ_INT(c.ch_mode, XQ_SBC_CIE_CH_JOINT);
    CHECK_EQ_INT(c.block_len, XQ_SBC_CIE_BLOCKS_16);
    CHECK_EQ_INT(c.num_subbands, XQ_SBC_CIE_SUBBANDS_8);
    CHECK_EQ_INT(c.alloc_mthd, XQ_SBC_CIE_ALLOC_LOUDNESS);
    CHECK_EQ_INT(c.min_bitpool, 2);
    CHECK_EQ_INT(c.max_bitpool, 53);
    xq_sbc_params_t p;
    CHECK(xq_sbc_params_from_cie(&c, &p));
    CHECK_EQ_INT(p.mode, XQ_SBC_MODE_JOINT);
    // The XQ configuration survives the byte form.
    xq_sbc_cie_t xq;
    xq_sbc_xq_config(&xq);
    uint8_t b[4] = {(uint8_t)(xq.samp_freq << 4 | xq.ch_mode),
                    (uint8_t)(xq.block_len << 4 | xq.num_subbands << 2 | xq.alloc_mthd), xq.min_bitpool, xq.max_bitpool};
    xq_sbc_cie_from_bytes(b, &c);
    CHECK(memcmp(&c, &xq, sizeof c) == 0);
    xq_sbc_cie_from_bytes(NULL, &c);  // ignored
}

TEST_MAIN(RUN(sbc_frame_and_bitrate) RUN(sbc_cie_decode) RUN(sbc_bluedroid_bitpool_search) RUN(sbc_xq_caps) RUN(sbc_cie_from_bytes) RUN(fifo_init_and_limits) RUN(fifo_wraparound) RUN(fifo_prebuffer_and_underrun) RUN(fifo_flush) RUN(fifo_q31_conversion) RUN(fifo_interleaving_stress) RUN(fifo_q31_gain) RUN(fifo_discard) RUN(volume_scale) RUN(volume_gain) RUN(addr_parse_format) RUN(devices_parse_file) RUN(devices_roundtrip_and_names) RUN(devices_eviction_and_remove) RUN(devices_xq_policy) RUN(devices_file_io) RUN(devices_parse_fuzz))
