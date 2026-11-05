#!/usr/bin/env python3
"""One-direction link test against the audio module's atom_diag firmware.

    pio run -e atom_lite -t upload   (or AUDIO_MODULE_BAUD=N pio run -e atom_diag -t upload)
    python3 tools/link_diag.py /dev/ttyUSB0 --baud 500000

module->PC: the module sends N pattern bytes as fast as it can; the PC checks them.
PC->module: the PC sends N pattern bytes as fast as it can; the module checks them and reports.
"""

import argparse
import struct
import time

import serial


def pattern(index):
    return ((index * 2654435761) & 0xFFFFFFFF) >> 24


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=500000)
    parser.add_argument("--bytes", type=int, default=32768)
    args = parser.parse_args()
    n = args.bytes
    data = bytes(pattern(i) for i in range(n))

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = args.port, args.baud, 0.05
    ser.dtr = ser.rts = False
    ser.open()
    time.sleep(0.2)
    ser.reset_input_buffer()

    ser.write(b"S" + struct.pack("<I", n))
    start = time.monotonic()
    got = bytearray()
    last = start
    while len(got) < n and time.monotonic() - last < 1.0:
        more = ser.read(4096)
        if more:
            got += more
            last = time.monotonic()
    bad = sum(1 for a, b in zip(got, data) if a != b)
    secs = max(last - start, 1e-6)
    print(f"module->PC: got {len(got)}/{n}, bad {bad} (by position), {len(got) / secs / 1024:.1f} KB/s")

    ser.reset_input_buffer()
    ser.write(b"R" + struct.pack("<I", n))
    time.sleep(0.05)
    start = time.monotonic()
    ser.write(data)
    ser.flush()
    sent_secs = time.monotonic() - start
    reply = b""
    deadline = time.monotonic() + 3.0
    while b"\n" not in reply and time.monotonic() < deadline:
        reply += ser.read(64)
    print(f"PC->module: sent {n} in {sent_secs * 1000:.0f} ms, module says: {reply.decode(errors='replace').strip()}")


if __name__ == "__main__":
    main()
