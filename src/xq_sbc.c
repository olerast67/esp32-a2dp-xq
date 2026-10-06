// SPDX-License-Identifier: Apache-2.0
// SBC arithmetic (see xq_sbc.h).
#include "xq_sbc.h"

#include <stddef.h>

static bool valid(const xq_sbc_params_t *p) {
    if (!p) return false;
    if (p->sample_rate != 16000 && p->sample_rate != 32000 && p->sample_rate != 44100 && p->sample_rate != 48000)
        return false;
    if (p->blocks != 4 && p->blocks != 8 && p->blocks != 12 && p->blocks != 16) return false;
    if (p->subbands != 4 && p->subbands != 8) return false;
    if (p->mode > XQ_SBC_MODE_JOINT) return false;
    return true;
}

static uint32_t channels(xq_sbc_mode_t mode) { return mode == XQ_SBC_MODE_MONO ? 1u : 2u; }

uint32_t xq_sbc_frame_length(const xq_sbc_params_t *p) {
    if (!valid(p) || p->bitpool == 0) return 0;
    uint32_t nch = channels(p->mode), nsub = p->subbands, nblk = p->blocks, bp = p->bitpool;
    uint32_t len = 4u + (4u * nsub * nch) / 8u;  // header + scale factors
    uint32_t bits;
    switch (p->mode) {
        case XQ_SBC_MODE_MONO:
        case XQ_SBC_MODE_DUAL: bits = nblk * nch * bp; break;
        case XQ_SBC_MODE_STEREO: bits = nblk * bp; break;
        case XQ_SBC_MODE_JOINT:
        default: bits = nsub + nblk * bp; break;
    }
    return len + (bits + 7u) / 8u;
}

uint32_t xq_sbc_bitrate_bps(const xq_sbc_params_t *p) {
    uint32_t len = xq_sbc_frame_length(p);
    if (!len) return 0;
    uint64_t bps = (uint64_t)8u * len * p->sample_rate / ((uint64_t)p->subbands * p->blocks);
    return (uint32_t)bps;
}

const char *xq_sbc_mode_name(xq_sbc_mode_t mode) {
    switch (mode) {
        case XQ_SBC_MODE_MONO: return "Mono";
        case XQ_SBC_MODE_DUAL: return "Dual Channel";
        case XQ_SBC_MODE_STEREO: return "Stereo";
        case XQ_SBC_MODE_JOINT: return "Joint Stereo";
    }
    return "";
}

static bool one_bit(uint8_t v) { return v != 0 && (v & (uint8_t)(v - 1u)) == 0; }

bool xq_sbc_params_from_cie(const xq_sbc_cie_t *cie, xq_sbc_params_t *out) {
    if (!cie || !out) return false;
    if (!one_bit(cie->samp_freq) || !one_bit(cie->ch_mode) || !one_bit(cie->block_len) ||
        !one_bit(cie->num_subbands))
        return false;
    switch (cie->samp_freq) {
        case XQ_SBC_CIE_SF_16K: out->sample_rate = 16000; break;
        case XQ_SBC_CIE_SF_32K: out->sample_rate = 32000; break;
        case XQ_SBC_CIE_SF_44K: out->sample_rate = 44100; break;
        case XQ_SBC_CIE_SF_48K: out->sample_rate = 48000; break;
        default: return false;
    }
    switch (cie->ch_mode) {
        case XQ_SBC_CIE_CH_MONO: out->mode = XQ_SBC_MODE_MONO; break;
        case XQ_SBC_CIE_CH_DUAL: out->mode = XQ_SBC_MODE_DUAL; break;
        case XQ_SBC_CIE_CH_STEREO: out->mode = XQ_SBC_MODE_STEREO; break;
        case XQ_SBC_CIE_CH_JOINT: out->mode = XQ_SBC_MODE_JOINT; break;
        default: return false;
    }
    switch (cie->block_len) {
        case XQ_SBC_CIE_BLOCKS_4: out->blocks = 4; break;
        case XQ_SBC_CIE_BLOCKS_8: out->blocks = 8; break;
        case XQ_SBC_CIE_BLOCKS_12: out->blocks = 12; break;
        case XQ_SBC_CIE_BLOCKS_16: out->blocks = 16; break;
        default: return false;
    }
    switch (cie->num_subbands) {
        case XQ_SBC_CIE_SUBBANDS_4: out->subbands = 4; break;
        case XQ_SBC_CIE_SUBBANDS_8: out->subbands = 8; break;
        default: return false;
    }
    out->bitpool = cie->max_bitpool;
    return true;
}

// Mirrors the integer arithmetic of the internal encoder's bitpool search so the status line
// shows what is really on air. Written from the algorithm description, not copied.
uint8_t xq_sbc_bluedroid_bitpool(const xq_sbc_params_t *cfg, uint8_t min_bitpool, uint8_t max_bitpool) {
    if (!valid(cfg) || min_bitpool > max_bitpool) return 0;
    const int nsub = cfg->subbands, nblk = cfg->blocks, fs = (int)cfg->sample_rate;
    const int nch = (int)channels(cfg->mode);
    int rate = (int)XQ_SBC_BLUEDROID_TARGET_KBPS;
    unsigned moved = 0;  // bit 0: lowered the target, bit 1: raised it
    int bp = 0;
    for (int guard = 0; guard < 400; guard++) {
        if (cfg->mode == XQ_SBC_MODE_JOINT || cfg->mode == XQ_SBC_MODE_STEREO) {
            int join = cfg->mode == XQ_SBC_MODE_JOINT ? nsub : 0;
            bp = (rate * nsub * 1000 / fs) - ((32 + 4 * nsub * nch + join) / nblk);
            int frame_len = 4 + (4 * nsub * nch) / 8 + (join + nblk * bp) / 8;
            int br = (8 * frame_len * fs) / (nsub * nblk * 1000);
            if (br > rate) bp--;
            int cap = nsub == 8 ? 255 : 128;
            if (bp > cap) bp = cap;
        } else {
            bp = (nsub * rate * 1000) / (fs * nch) - ((32 / nch + 4 * nsub) / nblk);
        }
        if (bp < 0) bp = 0;
        if (bp > max_bitpool) {
            rate -= (int)XQ_SBC_BLUEDROID_RATE_STEP_KBPS;
            moved |= 1u;
        } else if (bp < min_bitpool) {
            rate += (int)XQ_SBC_BLUEDROID_RATE_STEP_KBPS;
            moved |= 2u;
        } else {
            break;
        }
        if (moved == 3u || rate <= 0) break;
    }
    if (bp < 0) bp = 0;
    if (bp > 255) bp = 255;
    return (uint8_t)bp;
}

void xq_sbc_xq_config(xq_sbc_cie_t *out) {
    if (!out) return;
    out->samp_freq = XQ_SBC_CIE_SF_44K;
    out->ch_mode = XQ_SBC_CIE_CH_DUAL;
    out->block_len = XQ_SBC_CIE_BLOCKS_16;
    out->num_subbands = XQ_SBC_CIE_SUBBANDS_8;
    out->alloc_mthd = XQ_SBC_CIE_ALLOC_LOUDNESS;
    // min = max pins the encoder: its search raises the target bitrate until the bitpool
    // reaches 38 (see xq_sbc_bluedroid_bitpool), instead of staying at 328 kbps (bitpool 26).
    out->min_bitpool = XQ_SBC_XQ_BITPOOL;
    out->max_bitpool = XQ_SBC_XQ_BITPOOL;
}

void xq_sbc_joint_config(xq_sbc_cie_t *out) {
    if (!out) return;
    out->samp_freq = XQ_SBC_CIE_SF_44K;
    out->ch_mode = XQ_SBC_CIE_CH_JOINT;
    out->block_len = XQ_SBC_CIE_BLOCKS_16;
    out->num_subbands = XQ_SBC_CIE_SUBBANDS_8;
    out->alloc_mthd = XQ_SBC_CIE_ALLOC_LOUDNESS;
    out->min_bitpool = 2;
    out->max_bitpool = XQ_SBC_JOINT_HQ_BITPOOL;
}

bool xq_sbc_caps_allow_xq(const xq_sbc_cie_t *c) {
    if (!c) return false;
    return (c->samp_freq & XQ_SBC_CIE_SF_44K) && (c->ch_mode & XQ_SBC_CIE_CH_DUAL) && (c->block_len & XQ_SBC_CIE_BLOCKS_16) &&
           (c->num_subbands & XQ_SBC_CIE_SUBBANDS_8) && (c->alloc_mthd & XQ_SBC_CIE_ALLOC_LOUDNESS) &&
           c->min_bitpool <= XQ_SBC_XQ_BITPOOL && c->max_bitpool >= XQ_SBC_XQ_BITPOOL;
}

void xq_sbc_cie_from_bytes(const uint8_t bytes[4], xq_sbc_cie_t *out) {
    if (!bytes || !out) return;
    out->samp_freq = (uint8_t)(bytes[0] >> 4);
    out->ch_mode = (uint8_t)(bytes[0] & 0x0Fu);
    out->block_len = (uint8_t)(bytes[1] >> 4);
    out->num_subbands = (uint8_t)((bytes[1] >> 2) & 0x03u);
    out->alloc_mthd = (uint8_t)(bytes[1] & 0x03u);
    out->min_bitpool = bytes[2];
    out->max_bitpool = bytes[3];
}
