# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

PlatformIO/Arduino firmware for an M5Stack ATOM Lite (ESP32-PICO-D4, **no PSRAM**) that receives MP3, AAC (ADTS) or µ-law audio from a host board over USB or the Grove UART and plays it to Bluetooth A2DP, a PCM5102A I2S DAC, or a null sink. README.md has the full protocol, wiring and hardware measurements.

## Commands

```sh
pio run                              # build the player (default env atom_lite)
pio run -t upload                    # build + flash
pio run -e atom_linkdiag -t upload   # link diagnostic firmware (also atom_echo)
AUDIO_MODULE_BAUD=750000 pio run -e atom_diag -t upload   # atom_diag / atom_beacon take the baud from env

python3 tools/play.py /dev/ttyUSB0 file.mp3 --output bt   # PC stand-in host (needs pyserial; ffmpeg for M4B)
python3 tools/play.py /dev/ttyUSB0 --scan 20              # list BT headsets
python3 tools/link_diag.py / tools/echo_test.py           # pair with atom_linkdiag / atom_echo
```

There is no unit test suite or linter; verification is on hardware with the `tools/` scripts. `pio device monitor` is not useful — the USB port carries the binary protocol; diagnostics come back as LOG frames (`hostlink::log`), which `play.py` prints.

## Architecture

Pipeline: **Link → Player → Outputs**, each a FreeRTOS task pinned to core 1 (Bluetooth stack owns core 0). Priorities: link 5 > output 4 > decode 3, so the UART never backs up while a frame decodes. Stack sizes, buffer sizes and priorities all live in `src/Config.h`.

- `protocol/AudioLinkProtocol.h` — the entire wire contract (framing, CRC-16/CCITT-FALSE, payload structs, `Parser`). Dependency-free and copied **verbatim** into host firmware, so keep it standalone. Changes must stay backwards compatible: append new fields at the end of payloads; receivers accept payloads at least as long as the struct they know. `MuLaw.h` / `Resample2x.h` are live-stream helpers shared with hosts. Included via `-Iprotocol`.
- `src/Link.cpp` (`hostlink`) — both transports (`Serial` = USB bridge, `Serial1` = Grove) listen at once with shared protocol state; first port heard becomes active, another takes over on PING or after `LINK_TAKEOVER_MS` silence. Host→module frames are go-back-N reliable with credit flow control (`ACK(nextSeq, rxFree)`, re-advertised every 50 ms if credit changed); PING resyncs the sequence. Also persists output/volume to NVS (`Preferences` namespace `"audio"`) and dispatches control frames to `player::` / `outputs::`.
- `src/Player.cpp` (`player`) — compressed bytes go into an `rx` stream buffer (`RX_BUFFER_BYTES`); stream start/end/stop go through a command queue to the decode task. Decodes with libhelix MP3/AAC, or µ-law for live streams (dropped if late, ~150 ms cap, `LIVE_PRIME_MS` priming), resamples to the output rate, applies Q15 gain, and writes stereo frames into `PcmRing`. Cross-task state is `std::atomic`.
- `src/Outputs.cpp` (`outputs`) — `ModuleA2dpSource` subclasses ESP32-A2DP's `BluetoothA2DPSource` to expose on-demand scan/connect (the library otherwise auto-scans); A2DP pulls from `player::readPcm` in its callback. The output task drives I2S (blocking write paces it) and the null sink. Output always runs at 44.1 kHz stereo.
- `src/main.cpp` — boot wiring and the LED state; `loop()` only drives the LED (loop stack shrunk to 3 KB to save RAM). When `AUDIO_MODULE_DIAG_MODE != 0` it builds `LinkDiag.cpp` instead (diag envs exclude libhelix and the other sources via `build_src_filter`).

## Constraints to keep in mind

- **RAM is the binding constraint**: ~217 KB free at boot, ~75 KB after Bluetooth Classic, ~50 KB with a decoder running. Don't grow buffers/stacks casually; boot logs heap via LOG. HE-AAC SBR is compiled out (`-DHELIX_NO_AAC_SBR`).
- `huge_app.csv` partition (3 MB app, no OTA) is required for BT Classic + both decoders.
- `lib/libhelix` is a vendored, **patched** codec-helix v0.9.4; record any change in `lib/libhelix/PATCHES.md`.
- USB bridge is an FTDI emulation: only 500k/750k/1.5M baud are clean (921600 and 1M garble); no hardware flow control, hence the protocol's credits/CRC/retransmit. Hosts must keep DTR/RTS deasserted or the ATOM resets.
- The firmware version reported in HELLO is `FIRMWARE_MAJOR/MINOR` in `Config.h`; hosts use it to detect features (e.g. 0.2+ reports the active transport).
- Licensing: firmware source is MIT, but the compiled binary is GPL-3.0 (libhelix). `protocol/` must not depend on either library.
