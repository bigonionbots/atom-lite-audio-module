/**
 * @file LinkDiag.cpp
 * @brief Link diagnostics firmware modes (AUDIO_MODULE_DIAG_MODE 1-3), from the stage-2 bench.
 *
 * Diagnostics run on the USB port only.
 *
 * 1, echo: everything received is written straight back (tools/echo_test.py). The RGB LED is
 *    blue when idle, green while bytes flow and red after a receive overflow.
 * 2, beacon: transmits a fixed text line every 20 ms, to find the real transmit rate from a PC.
 * 3, link diag: one direction at a time on command (a host's link test, or
 *    tools/link_diag.py):
 *      'S' + u32 length: send `length` pattern bytes as fast as possible.
 *      'B' + u32 length + u16 burst + u16 gap_us: the same, in bursts with idle gaps.
 *      'R' + u32 length: receive `length` pattern bytes, then reply "R <got> <bad> <ms>\n".
 */

#include "LinkDiag.h"

#if AUDIO_MODULE_DIAG_MODE != 0

#include <Arduino.h>

namespace {

constexpr int LED_PIN = 27;  // SK6812 on the ATOM Lite
constexpr uint8_t LED_LEVEL = 24;
constexpr size_t RX_BUFFER_BYTES = 8192;
constexpr size_t TX_BUFFER_BYTES = 8192;
constexpr unsigned long ACTIVITY_LED_MS = 250;
// The UART's hardware RX FIFO holds 128 bytes and Arduino only empties it into the ring buffer
// at 120 by default: 8 bytes (~90 us at 921600) of slack before bytes are dropped. Empty it early.
constexpr uint8_t RX_FIFO_FULL_THRESHOLD = 32;

enum class Led : uint8_t { Idle, Active, Overflow };

Led shownLed = Led::Overflow;
bool overflowed = false;
unsigned long lastByteMs = 0;

void showLed(const Led led) {
  if (led == shownLed) return;
  shownLed = led;
  switch (led) {
    case Led::Idle:
      rgbLedWrite(LED_PIN, 0, 0, LED_LEVEL);
      break;
    case Led::Active:
      rgbLedWrite(LED_PIN, 0, LED_LEVEL, 0);
      break;
    case Led::Overflow:
      rgbLedWrite(LED_PIN, LED_LEVEL, 0, 0);
      break;
  }
}

void onUartError(const hardwareSerial_error_t error) {
  if (error == UART_BUFFER_FULL_ERROR || error == UART_FIFO_OVF_ERROR) overflowed = true;
}

#if AUDIO_MODULE_DIAG_MODE == 3
uint8_t patternByte(const uint32_t index) { return static_cast<uint8_t>((index * 2654435761u) >> 24); }

uint32_t readLength() {
  uint8_t bytes[4];
  if (Serial.readBytes(bytes, 4) != 4) return 0;
  return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

void linkDiagLoop() {
  if (Serial.available() <= 0) {
    delay(1);
    return;
  }
  const int command = Serial.read();
  if (command == 'S') {
    const uint32_t length = readLength();
    uint8_t chunk[256];
    for (uint32_t sent = 0; sent < length;) {
      const size_t n = min<size_t>(sizeof(chunk), length - sent);
      for (size_t i = 0; i < n; ++i) chunk[i] = patternByte(sent + i);
      Serial.write(chunk, n);
      sent += n;
    }
  } else if (command == 'B') {
    // 'B' + u32 length + u16 burst + u16 gap_us: send in bursts with idle gaps between them.
    const uint32_t length = readLength();
    uint8_t extra[4];
    if (Serial.readBytes(extra, 4) != 4) return;
    const uint16_t burst = static_cast<uint16_t>(extra[0] | (extra[1] << 8));
    const uint16_t gapUs = static_cast<uint16_t>(extra[2] | (extra[3] << 8));
    uint8_t chunk[256];
    for (uint32_t sent = 0; sent < length;) {
      const size_t n = min<size_t>(min<size_t>(sizeof(chunk), burst), length - sent);
      for (size_t i = 0; i < n; ++i) chunk[i] = patternByte(sent + i);
      Serial.write(chunk, n);
      Serial.flush();
      delayMicroseconds(gapUs);
      sent += n;
    }
  } else if (command == 'R') {
    const uint32_t length = readLength();
    uint32_t got = 0;
    uint32_t bad = 0;
    const unsigned long start = millis();
    unsigned long last = start;
    uint8_t chunk[256];
    while (got < length && millis() - last < 1000) {
      const int available = Serial.available();
      if (available <= 0) continue;
      const size_t n = Serial.readBytes(chunk, min<size_t>(sizeof(chunk), available));
      for (size_t i = 0; i < n; ++i) {
        if (chunk[i] != patternByte(got + i)) ++bad;
      }
      got += n;
      last = millis();
    }
    Serial.printf("R %lu %lu %lu\n", static_cast<unsigned long>(got), static_cast<unsigned long>(bad),
                  static_cast<unsigned long>(last - start));
  }
}

#elif AUDIO_MODULE_DIAG_MODE == 2
void beaconLoop() {
  Serial.print("The quick brown fox jumps over the lazy dog 0123456789\r\n");
  delay(20);
}

#else
void echoLoop() {
  uint8_t buffer[512];
  const int available = Serial.available();
  if (available > 0) {
    const size_t count = Serial.readBytes(buffer, min(static_cast<size_t>(available), sizeof(buffer)));
    Serial.write(buffer, count);
    lastByteMs = millis();
  }

  if (overflowed) {
    showLed(Led::Overflow);
  } else if (lastByteMs != 0 && millis() - lastByteMs < ACTIVITY_LED_MS) {
    showLed(Led::Active);
  } else {
    showLed(Led::Idle);
  }
  if (available <= 0) delay(1);
}

#endif

}  // namespace

void linkDiagSetup() {
  Serial.setRxBufferSize(RX_BUFFER_BYTES);
  Serial.setTxBufferSize(TX_BUFFER_BYTES);
  Serial.begin(AUDIO_MODULE_BAUD, SERIAL_8N1, -1, -1, false, 20000UL, RX_FIFO_FULL_THRESHOLD);
  Serial.onReceiveError(onUartError);
  showLed(Led::Idle);
}

void linkDiagRun() {
#if AUDIO_MODULE_DIAG_MODE == 1
  echoLoop();
#elif AUDIO_MODULE_DIAG_MODE == 2
  beaconLoop();
#else
  linkDiagLoop();
#endif
}

#endif  // AUDIO_MODULE_DIAG_MODE != 0
