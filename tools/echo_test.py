#!/usr/bin/env python3
"""Echo test against the audio module's USB port from a PC (firmware env atom_echo).

Plug the ATOM into the PC, then:
    python3 tools/echo_test.py /dev/ttyUSB0 [--baud 500000] [--bytes 16384] [--rate 8192]

A pass here and a failure from a host board puts the fault in the host's USB path, not in the
ATOM or its bridge.
Needs pyserial (pip install pyserial).
"""

import argparse
import sys
import time

import serial


def pattern(index):
    return ((index * 2654435761) & 0xFFFFFFFF) >> 24


def open_port(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    # Deasserted before open so the ESP32 auto-reset circuit neither resets the ATOM nor
    # drops it into the bootloader (Linux asserts both on open otherwise).
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def lockstep(ser, total=2048, chunk=64):
    for offset in range(0, total, chunk):
        data = bytes(pattern(offset + i) for i in range(chunk))
        ser.write(data)
        got = b""
        deadline = time.monotonic() + 0.3
        while len(got) < chunk and time.monotonic() < deadline:
            got += ser.read(chunk - len(got))
        if got != data:
            print(f"lockstep: chunk at {offset} got {len(got)}/{chunk}: {got[:8].hex(' ')} (want {data[:8].hex(' ')})")
            return False
    print(f"lockstep {total} B: OK")
    return True


def stream(ser, total, rate):
    data = bytes(pattern(i) for i in range(total))
    ser.reset_input_buffer()
    start = time.monotonic()
    received = bytearray()
    for offset in range(0, total, 256):
        # Paced: the ATOM Lite's bridge drops data above ~15 KB/s and has no flow control.
        while rate and time.monotonic() - start < offset / rate:
            received += ser.read(ser.in_waiting or 0)
            time.sleep(0.001)
        ser.write(data[offset : offset + 256])
        received += ser.read(ser.in_waiting or 0)
    last = time.monotonic()
    while len(received) < total and time.monotonic() - last < 2.0:
        more = ser.read(4096)
        if more:
            received += more
            last = time.monotonic()
    elapsed = last - start

    expected = r = lost = gaps = corrupt = 0
    first_bad = None
    while r < len(received) and expected < total:
        if received[r] == pattern(expected):
            r += 1
            expected += 1
            continue
        if first_bad is None:
            first_bad = r
            print(f"first mismatch at {r}: got {received[r:r + 4].hex(' ')} "
                  f"want {bytes(pattern(expected + i) for i in range(4)).hex(' ')}")
        for k in range(expected + 1, min(total - 6, expected + 4096) + 1):
            if all(r + m < len(received) and received[r + m] == pattern(k + m) for m in range(6)):
                lost += k - expected
                gaps += 1
                expected = k
                break
        else:
            corrupt += 1
            r += 1
            expected += 1
    lost += total - min(expected, total)
    corrupt += len(received) - r
    ok = lost == 0 and corrupt == 0 and len(received) == total
    kbps = len(received) * 8 / elapsed / 1000 if elapsed > 0 else 0
    print(f"stream: {'OK' if ok else 'FAIL'} sent={total} got={len(received)} lost={lost} in {gaps} gaps, "
          f"corrupt={corrupt}, {elapsed * 1000:.0f} ms ({kbps:.0f} kbit/s)")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=500000)
    parser.add_argument("--rate", type=int, default=8192, help="stream pacing in bytes/s, 0 = unpaced")
    parser.add_argument("--bytes", type=int, default=16384)
    args = parser.parse_args()
    with open_port(args.port, args.baud) as ser:
        ok = lockstep(ser)
        ok = stream(ser, args.bytes, args.rate) and ok
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
