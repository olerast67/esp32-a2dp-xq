# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-06

First public release. Not yet tested with real headphones: the host tests pass and the examples
build with ESP-IDF 5.3, 5.5, 6.0, 6.1 and the Arduino core 3.1.3 and newer.

### Added

- A2DP source with the Bluedroid SBC encoder, fed from a PCM FIFO (16-bit, or 32-bit with TPDF
  dither) with 300 ms of buffering in PSRAM when available.
- SBC-XQ negotiation (Dual Channel, bitpool 38, 452 kbps) with ESP-IDF 6.0 and newer, fallback to
  Joint Stereo bitpool 53 (328 kbps).
- Per-device memory of XQ results in a text file, built-in blacklist, per-device XQ switch.
- AVRCP: headphone buttons, absolute volume in both directions, own attenuation for headphones
  without absolute volume.
- Discovery with name resolution, Secure Simple Pairing and legacy PIN pairing, automatic
  reconnect, link-loss notification.
- Builds with ESP-IDF 5.3 and newer and the Arduino core 3.x; without SBC-XQ before ESP-IDF 6.0.
- Examples for ESP-IDF and Arduino.

[Unreleased]: https://github.com/olerast67/esp32-a2dp-xq/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/olerast67/esp32-a2dp-xq/releases/tag/v0.1.0
