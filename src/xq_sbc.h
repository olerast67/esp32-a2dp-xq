// SPDX-License-Identifier: Apache-2.0
// SBC arithmetic for the Bluetooth status line and the SBC-XQ decision: frame length and
// bitrate (A2DP 1.3 section 12.9), codec information element bit masks (A2DP section 4.3.2),
// and the bitpool that the ESP-IDF 6.1 Bluedroid internal encoder settles on. Pure C.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Codec information element bits (same values as ESP_A2D_SBC_CIE_* in esp_a2dp_api.h).
#define XQ_SBC_CIE_SF_16K 0x8u
#define XQ_SBC_CIE_SF_32K 0x4u
#define XQ_SBC_CIE_SF_44K 0x2u
#define XQ_SBC_CIE_SF_48K 0x1u
#define XQ_SBC_CIE_CH_MONO 0x8u
#define XQ_SBC_CIE_CH_DUAL 0x4u
#define XQ_SBC_CIE_CH_STEREO 0x2u
#define XQ_SBC_CIE_CH_JOINT 0x1u
#define XQ_SBC_CIE_BLOCKS_4 0x8u
#define XQ_SBC_CIE_BLOCKS_8 0x4u
#define XQ_SBC_CIE_BLOCKS_12 0x2u
#define XQ_SBC_CIE_BLOCKS_16 0x1u
#define XQ_SBC_CIE_SUBBANDS_4 0x2u
#define XQ_SBC_CIE_SUBBANDS_8 0x1u
#define XQ_SBC_CIE_ALLOC_SNR 0x2u
#define XQ_SBC_CIE_ALLOC_LOUDNESS 0x1u

// SBC-XQ (Dual Channel, bitpool 38 per channel, ~452 kbps at 44.1 kHz) and the fallback that
// every sink accepts (Joint Stereo, bitpool 53, ~328 kbps). Dual Channel codes each channel
// with its own bitpool, so 38 per channel spends more bits per channel than Joint Stereo 53 for
// both; every A2DP sink must decode Dual Channel (A2DP 1.3, 4.3.2), but some refuse a fixed
// bitpool or drop the link, hence the per-device memory and the blacklist.
#define XQ_SBC_XQ_BITPOOL 38u
#define XQ_SBC_JOINT_HQ_BITPOOL 53u
// Target bitrate of the Bluedroid internal encoder (DEFAULT_SBC_BITRATE in
// components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_source.c, IDF 6.1).
#define XQ_SBC_BLUEDROID_TARGET_KBPS 328u
#define XQ_SBC_BLUEDROID_RATE_STEP_KBPS 5u

typedef enum { XQ_SBC_MODE_MONO = 0, XQ_SBC_MODE_DUAL, XQ_SBC_MODE_STEREO, XQ_SBC_MODE_JOINT } xq_sbc_mode_t;

typedef struct {
    uint32_t sample_rate;  // 16000, 32000, 44100, 48000
    xq_sbc_mode_t mode;
    uint8_t blocks;        // 4, 8, 12, 16
    uint8_t subbands;      // 4, 8
    uint8_t bitpool;       // 2..250
} xq_sbc_params_t;

// SBC codec information element as carried in esp_a2d_cie_sbc_t (one field per byte here).
typedef struct {
    uint8_t samp_freq;
    uint8_t ch_mode;
    uint8_t block_len;
    uint8_t num_subbands;
    uint8_t alloc_mthd;
    uint8_t min_bitpool;
    uint8_t max_bitpool;
} xq_sbc_cie_t;

// Frame length in bytes, 0 for invalid parameters.
uint32_t xq_sbc_frame_length(const xq_sbc_params_t *p);
// Bitrate in bit/s, 0 for invalid parameters.
uint32_t xq_sbc_bitrate_bps(const xq_sbc_params_t *p);
// "Mono", "Dual Channel", "Stereo", "Joint Stereo".
const char *xq_sbc_mode_name(xq_sbc_mode_t mode);

// Decode a configuration (exactly one bit set per field). false when a field is not a single
// valid bit.
bool xq_sbc_params_from_cie(const xq_sbc_cie_t *cie, xq_sbc_params_t *out);

// The 4-byte SBC codec information element as carried on the air (A2DP 1.3, 4.3.2): sampling
// frequency (bits 7..4) and channel mode (3..0); block length (7..4), subbands (3..2) and
// allocation (1..0); minimum bitpool; maximum bitpool. ESP-IDF before 5.5 reports the stream
// configuration in this form (esp_a2d_mcc_t.cie.sbc).
void xq_sbc_cie_from_bytes(const uint8_t bytes[4], xq_sbc_cie_t *out);

// Bitpool the Bluedroid internal encoder picks for this configuration: it starts at
// XQ_SBC_BLUEDROID_TARGET_KBPS and moves the target in 5 kbps steps until the computed bitpool
// fits [min_bitpool, max_bitpool] (btc_a2dp_source_enc_update in IDF 6.1). With the XQ
// configuration (min = max = 38) the target climbs to 453 kbps and the bitpool is 38.
uint8_t xq_sbc_bluedroid_bitpool(const xq_sbc_params_t *cfg, uint8_t min_bitpool, uint8_t max_bitpool);

// The preferred SBC-XQ configuration: 44.1 kHz, Dual Channel, 16 blocks, 8 subbands,
// loudness allocation, bitpool fixed at XQ_SBC_XQ_BITPOOL.
void xq_sbc_xq_config(xq_sbc_cie_t *out);
// The fallback configuration: 44.1 kHz Joint Stereo, bitpool range [2, 53].
void xq_sbc_joint_config(xq_sbc_cie_t *out);
// True when a sink advertising these capabilities accepts the XQ configuration.
bool xq_sbc_caps_allow_xq(const xq_sbc_cie_t *sink_caps);

#ifdef __cplusplus
}
#endif
