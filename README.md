# ATOM Lite Audio Module

Firmware that turns an M5Stack **ATOM Lite** (ESP32-PICO-D4) into an audio output for any
microcontroller board. The host board streams compressed audio (MP3 or AAC) or live µ-law PCM to the
module. The module decodes it and plays it to:

- **Bluetooth A2DP headphones or speakers.** It advertises as **"ATOM Lite Audio"**. This is the
  main reason the module exists: the ESP32-S3, -C3 and -C6 have only Bluetooth Low Energy, while
  the original ESP32 in the ATOM has Classic Bluetooth.
- **A PCM5102A I2S DAC** wired to the ATOM's header.
- **A null sink** that consumes audio in real time and discards it, for testing.

The output and volume are saved on the module. A paired headset is remembered and reconnects at boot.

## Connecting a host

The module listens on both transports at once, with no configuration:

| Transport | Wiring | Power | Notes |
| --- | --- | --- | --- |
| **USB** | Host's USB port (in USB host mode) to the ATOM's USB-C | Host supplies 5 V on VBUS | The host needs a USB host stack with an FTDI VCP driver (ESP-IDF `usb_host` + FTDI/CDC class drivers). It works on the ESP32-S2/S3/P4. |
| **UART** | Grove port: host TX → **G32**, host RX ← **G26**, GND | Grove 5V and GND | Any board with two free GPIOs. 3.3 V logic. |

Both run 8N1 at **500000** baud by default (`AUDIO_MODULE_BAUD` for USB, `AUDIO_MODULE_UART_BAUD`
for the Grove UART, in `platformio.ini`). The USB rate is limited by the ATOM's bridge (see below).
The Grove UART has no bridge and can be built faster if the host keeps up.

The first transport a host is heard on becomes the active one, and replies go there. Another
transport takes over when its host sends a PING, or sends anything once the active one has been
quiet for 3 s. Until a host is heard, HELLO, STATUS and LOG go out on both. Firmware 0.2 and later
report the active transport in HELLO.

## Building and flashing

Uses [PlatformIO](https://platformio.org/) with the pioarduino platform (arduino-esp32 3.3.7).

```sh
pio run -t upload          # player firmware (env atom_lite)
pio device monitor         # not useful: the USB port carries the binary protocol
```

The LED shows the state:

| LED | State |
| --- | --- |
| Blue | Idle |
| Cyan | Buffering |
| Green | Playing |
| Yellow | Paused |
| Purple | Scanning for Bluetooth |
| Red | A buffer couldn't be allocated |

Diagnostic envs (all on the USB port, see `src/LinkDiag.cpp`):

- `atom_linkdiag`: one-direction link test (`tools/link_diag.py`)
- `atom_echo`: echo (`tools/echo_test.py`)
- `atom_diag` and `atom_beacon`: run at any baud rate set by `AUDIO_MODULE_BAUD`

## Protocol

`protocol/AudioLinkProtocol.h` is the whole contract. It is dependency-free C++, and hosts include a
verbatim copy of it. `MuLaw.h` and `Resample2x.h` are the live-stream helpers.

- Frames: `A5 5A | type | seq | len u16 | payload | crc16`, with CRC-16/CCITT-FALSE.
- Host → module frames are reliable (go-back-N). The module acknowledges with `ACK(nextSeq, rxFree)`
  and the host keeps its unacknowledged audio within `rxFree`, which is the credit flow control.
  PING resynchronises the sequence and gets a HELLO back.
- Module → host frames (HELLO, ACK, STATUS every 250 ms, BT_DEVICE, LOG) are small and never resent.
- A stream starts with STREAM_START (stream id, codec, start position), continues with AUDIO_DATA
  chunks of about 512 bytes, and ends with STREAM_END to play out or STREAM_STOP to cut off.
  The position is `startMs + STATUS.playedMs`.
- Codecs:
  - MP3
  - AAC in ADTS framing: from an MP4/M4B, `ffmpeg -c:a copy -f adts`; an MPEG-TS stream is already ADTS.
  - µ-law mono at any rate, for live sound. It is kept under 150 ms of latency by dropping late audio.
- Controls: pause/resume, volume (0-100), output select, Bluetooth scan/connect/disconnect/forget.

Keep changes backwards compatible. New fields go at the end of a payload, and receivers accept
payloads at least as long as the struct they know.

### Trying it from a PC

```sh
pip install pyserial                         # and ffmpeg for M4B
python3 tools/play.py /dev/ttyUSB0 --scan 20                     # list headsets in pairing mode
python3 tools/play.py /dev/ttyUSB0 --connect AA:BB:CC:DD:EE:FF
python3 tools/play.py /dev/ttyUSB0 book.m4b --output bt
```

The port can be the ATOM's own USB, or a 3.3 V USB-UART adapter wired to the Grove port (this
tests the UART transport). From a Linux PC the ATOM's bridge tops out around 10 KB/s, so 128 kbit/s
stereo starves through USB there. ESP-IDF hosts measured 39 KB/s on the same bridge.

## Hardware notes

**PCM5102A wiring (ATOM header):**
- BCK → G22, LRCK → G19, DIN → G23. These can be changed with `AUDIO_MODULE_I2S_*`.
- SCK → GND (internal PLL), XSMT → 3V3, FMT/FLT/DEMP → GND.

**The ATOM Lite's USB bridge.** On the tested unit it is not an FTDI chip. It enumerates as `0403:6001`
("Hades2001" / "M5stack", bcdDevice 5.00), an MCU-based FTDI emulation.
- **Line rates:** 921600 and 1M come out garbled. 500k, 750k and 1.5M are clean.
- **Host → module:** 10-17 KB/s from a Linux PC, about 39 KB/s from an ESP32-S3 USB host.
- **Module → host:** a Linux host loses data unless it is trickled, while an ESP-IDF host receives
  everything.
- **Flow control:** there is none in hardware and the bridge reports no overruns, which is why the
  protocol has credits, a CRC and retransmission.
- **DTR/RTS:** a USB host should keep both deasserted, or the ATOM's auto-reset circuit resets it.

**Memory** (no PSRAM): about 217 KB is free at boot and about 75 KB once Bluetooth Classic is up.
That leaves about 50 KB with a decoder running. HE-AAC (SBR) would need another 51 KB contiguous
block, so it is compiled out (`HELIX_NO_AAC_SBR`). HE-AAC sources play their AAC-LC core.

## Layout

```
protocol/   AudioLinkProtocol.h (the wire contract), MuLaw.h, Resample2x.h
src/        Link (both transports, frames, ACK/STATUS), Player (buffers, decoders, resampler),
            Outputs (A2DP source, I2S, null sink), Config.h (pins, buffers, tasks), LinkDiag
lib/        libhelix: vendored, patched codec-helix (PATCHES.md)
tools/      play.py (PC stand-in host), link_diag.py, echo_test.py
```

## Licences

The firmware's own code is MIT (`LICENSE`). It links two libraries:
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) (Apache-2.0).
- codec-helix (GPL-3.0, vendored in `lib/libhelix`). Its Helix decoders are under the RealNetworks
  RPSL/RCSL.

So the **compiled module firmware is distributed under GPL-3.0**. Host firmware that only speaks
the protocol (including `protocol/`) doesn't link either library.
