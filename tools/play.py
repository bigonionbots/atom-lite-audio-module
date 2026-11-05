#!/usr/bin/env python3
"""Play a file through the audio module from a PC, standing in for a host board.

Speaks the link protocol in protocol/AudioLinkProtocol.h (reliable go-back-N host frames,
credit flow control, STATUS/LOG/BT_DEVICE from the module) to the default atom_lite firmware:

    python3 tools/play.py /dev/ttyUSB0 book.mp3 --output null
    python3 tools/play.py /dev/ttyUSB0 book.m4b --output bt
    python3 tools/play.py /dev/ttyUSB0 --scan 20          # list Bluetooth headsets
    python3 tools/play.py /dev/ttyUSB0 --connect AA:BB:CC:DD:EE:FF

MP3 is sent as is. M4B/M4A/MP4 go through `ffmpeg -c:a copy -f adts` (no re-encoding), which is what
a host with its own MP4 reader sends. Needs pyserial. Note: from a Linux PC this bridge can drop
module->host bytes under load (README); lost ACK/STATUS frames are harmless.
"""

import argparse
import struct
import subprocess
import sys
import threading
import time

import serial

MAGIC = b"\xa5\x5a"
T = dict(Ping=0x01, StreamStart=0x10, AudioData=0x11, StreamEnd=0x12, StreamStop=0x13, Pause=0x14, Resume=0x15,
         SetVolume=0x16, SetOutput=0x17, BtScan=0x20, BtConnect=0x21, BtDisconnect=0x22, BtForget=0x23,
         Hello=0x80, Ack=0x81, Status=0x82, BtDevice=0x83, Log=0x84)
OUTPUT = dict(bt=1, i2s=2, null=3)
STATE = {0: "idle", 1: "buffering", 2: "playing", 3: "paused", 4: "draining", 5: "finished"}
BT_STATE = {0: "off", 1: "idle", 2: "scanning", 3: "connecting", 4: "connected"}
STATUS_FMT = "<BBBBBBIIIHHHI6s"
RESEND_TIMEOUT = 1.0
FAST_RESEND_GAP = 0.1
MAX_INFLIGHT = 8192
MAX_WINDOW = 48
AUDIO_CHUNK = 512


def crc16(data, crc=0xFFFF):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(frame_type, seq, payload=b""):
    body = bytes([frame_type, seq]) + struct.pack("<H", len(payload)) + payload
    return MAGIC + body + struct.pack("<H", crc16(body))


def seq_before(a, b):
    return ((a - b) & 0xFF) >= 0x80


class Host:
    def __init__(self, port, baud=500000):
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.02
        self.ser.dtr = self.ser.rts = False
        self.ser.open()
        self.lock = threading.Condition()
        self.next_seq = 0
        self.unacked = []  # [seq, frame, audio_bytes]
        self.rx_free = 0
        self.last_progress = time.monotonic()
        self.last_fast_resend = 0
        self.resends = 0
        self.status = None
        self.hello = None
        self.devices = {}
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._resender, daemon=True).start()

    def _reader(self):
        buf = bytearray()
        while self.running:
            buf += self.ser.read(4096)
            while True:
                start = buf.find(MAGIC)
                if start < 0:
                    del buf[:-1]
                    break
                del buf[:start]
                if len(buf) < 6:
                    break
                length = buf[4] | (buf[5] << 8)
                if length > 1024:
                    del buf[:2]
                    continue
                if len(buf) < 8 + length:
                    break
                body = bytes(buf[2:6 + length])
                crc = buf[6 + length] | (buf[7 + length] << 8)
                if crc != crc16(body):
                    del buf[:2]
                    continue
                del buf[:8 + length]
                self._handle(body[0], body[4:])

    def _handle(self, frame_type, payload):
        with self.lock:
            if frame_type == T["Ack"] and len(payload) >= 6:
                next_seq, rx_free, flags = struct.unpack("<BIB", payload[:6])
                before = len(self.unacked)
                self.unacked = [f for f in self.unacked if not seq_before(f[0], next_seq)]
                if len(self.unacked) < before:
                    self.last_progress = time.monotonic()
                self.rx_free = rx_free
                if flags & 1 and self.unacked and time.monotonic() - self.last_fast_resend >= FAST_RESEND_GAP:
                    self.last_fast_resend = time.monotonic()
                    self._resend_locked()
            elif frame_type == T["Status"] and len(payload) >= struct.calcsize(STATUS_FMT):
                self.status = struct.unpack(STATUS_FMT, payload[:struct.calcsize(STATUS_FMT)])
            elif frame_type == T["Hello"] and len(payload) >= 9:
                self.hello = struct.unpack("<BBBBIB", payload[:9])
                # Continue from the sequence number the module expects (see HelloPayload.nextSeq).
                self.next_seq = self.hello[5]
                self.unacked = []
                print(f"[module] hello: protocol {self.hello[0]}, firmware {self.hello[1]}.{self.hello[2]}, "
                      f"rx buffer {self.hello[4]} B")
            elif frame_type == T["Log"]:
                print(f"[module] {payload.decode(errors='replace')}")
            elif frame_type == T["BtDevice"]:
                address = ":".join(f"{b:02X}" for b in payload[:6])
                rssi = struct.unpack("<b", payload[6:7])[0]
                name = payload[7:].split(b"\0")[0].decode(errors="replace")
                if address not in self.devices:
                    print(f"[bt] {address}  {rssi:4d} dBm  {name}")
                self.devices[address] = (rssi, name)
            self.lock.notify_all()

    def _resend_locked(self):
        for frame in self.unacked:
            self.ser.write(frame[1])
        self.resends += 1
        self.last_progress = time.monotonic()

    def _resender(self):
        while self.running:
            time.sleep(0.05)
            with self.lock:
                if self.unacked and time.monotonic() - self.last_progress > RESEND_TIMEOUT:
                    self._resend_locked()

    def ping(self):
        with self.lock:
            seq = self.next_seq
            self.next_seq = (self.next_seq + 1) & 0xFF
            self.unacked = []
            self.ser.write(encode(T["Ping"], seq))
            self.lock.wait_for(lambda: self.hello is not None, timeout=2)
        return self.hello is not None

    def send(self, frame_type, payload=b"", audio_bytes=0):
        with self.lock:
            frame = encode(frame_type, self.next_seq, payload)
            self.lock.wait_for(lambda: len(self.unacked) < MAX_WINDOW and
                               (not self.unacked or sum(len(f[1]) for f in self.unacked) + len(frame) <= MAX_INFLIGHT) and
                               (audio_bytes == 0 or sum(f[2] for f in self.unacked) + audio_bytes <= self.rx_free))
            self.unacked.append([self.next_seq, frame, audio_bytes])
            if len(self.unacked) == 1:
                self.last_progress = time.monotonic()
            self.next_seq = (self.next_seq + 1) & 0xFF
            self.ser.write(frame)

    def drain(self, timeout=5):
        with self.lock:
            return self.lock.wait_for(lambda: not self.unacked, timeout=timeout)


def describe(status):
    stream, state, output, bt, volume, channels, rate, played, rx, pcm_ms, underruns, errors, heap, address = status
    return (f"{STATE.get(state, state):9s} {played / 1000:7.1f}s  {rate} Hz x{channels}  rx {rx:5d} B  pcm {pcm_ms:3d} ms  "
            f"underruns {underruns}  errors {errors}  bt {BT_STATE.get(bt, bt)}  heap {heap}")


def audio_source(path):
    if path.lower().endswith(".mp3"):
        return 1, open(path, "rb")
    proc = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-i", path, "-vn", "-c:a", "copy", "-f", "adts", "-"],
                            stdout=subprocess.PIPE)
    return 2, proc.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="the ATOM's USB port, or a 3.3 V USB-UART adapter wired to its Grove port")
    parser.add_argument("--baud", type=int, default=500000)
    parser.add_argument("file", nargs="?")
    parser.add_argument("--output", choices=OUTPUT, default=None)
    parser.add_argument("--volume", type=int, default=None)
    parser.add_argument("--scan", type=float, default=0, help="scan for Bluetooth headsets for N seconds")
    parser.add_argument("--connect", help="connect to a Bluetooth address AA:BB:CC:DD:EE:FF")
    parser.add_argument("--seconds", type=float, default=0, help="stop after N seconds of playback")
    args = parser.parse_args()

    host = Host(args.port, args.baud)
    if not host.ping():
        sys.exit("no HELLO from the module (is the atom_lite firmware flashed?)")
    if args.output:
        host.send(T["SetOutput"], bytes([OUTPUT[args.output]]))
    if args.volume is not None:
        host.send(T["SetVolume"], bytes([args.volume]))
    if args.scan:
        host.send(T["BtScan"], b"\x01")
        time.sleep(args.scan)
        host.send(T["BtScan"], b"\x00")
    if args.connect:
        host.send(T["BtConnect"], bytes(int(x, 16) for x in args.connect.split(":")))
    host.drain()
    if not args.file:
        time.sleep(1)
        if host.status:
            print(describe(host.status))
        return

    codec, source = audio_source(args.file)
    stream_id = 1
    host.send(T["StreamStart"], struct.pack("<BBI", stream_id, codec, 0))
    start = time.monotonic()
    last_print = 0
    sent = 0
    while True:
        chunk = source.read(AUDIO_CHUNK)
        if not chunk:
            break
        host.send(T["AudioData"], bytes([stream_id]) + chunk, audio_bytes=len(chunk) + 1)
        sent += len(chunk)
        now = time.monotonic()
        if now - last_print >= 1 and host.status:
            last_print = now
            print(f"{now - start:6.1f}s sent {sent:8d} B resends {host.resends:3d} | {describe(host.status)}")
        if args.seconds and now - start > args.seconds:
            break
    host.send(T["StreamEnd"], bytes([stream_id]))
    while True:
        time.sleep(1)
        if host.status:
            print(f"{time.monotonic() - start:6.1f}s sent {sent:8d} B | {describe(host.status)}")
            if host.status[1] in (0, 5):
                break
        if args.seconds and time.monotonic() - start > args.seconds + 5:
            host.send(T["StreamStop"], bytes([stream_id]))
            break


if __name__ == "__main__":
    main()
