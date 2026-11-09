/**
 * @file Link.cpp
 * @brief Definitions for the module end of the host link, over USB and the Grove UART.
 */

#include "Link.h"

#include <Arduino.h>
#include <AudioLinkProtocol.h>
#include <Preferences.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "Config.h"
#include "Outputs.h"
#include "Player.h"

namespace hostlink {

namespace {

using audiolink::Type;

// Cumulative, so a lost ACK costs nothing; batching keeps the module->host direction light.
constexpr uint32_t ACK_INTERVAL_MS = 10;
// A host that ran out of credit sends nothing, so it would never hear that space freed up:
// re-advertise rxFree on its own when it changed.
constexpr uint32_t CREDIT_UPDATE_MS = 50;
constexpr size_t READ_CHUNK = 256;
constexpr int FLUSH_WAIT_MS = 20;

/** One physical transport. The protocol state (sequence numbers, stream) is shared between them. */
struct Port {
  HardwareSerial* serial;
  audiolink::Transport transport;
  audiolink::Parser parser;
  bool up;
};

Port ports[] = {
    {&Serial, audiolink::Transport::Usb, {}, false},
    {&Serial1, audiolink::Transport::Uart, {}, false},
};

SemaphoreHandle_t txLock = nullptr;
// The port the current host talks on; nullptr until a host is heard, when frames go out on all.
Port* active = nullptr;
uint32_t lastActiveRxMs = 0;

uint8_t expectedSeq = 0;
int linkStream = -1;  // last STREAM_START id, -1 after STREAM_STOP
bool ackPending = false;
bool ackUrgent = false;
bool outOfSequence = false;
uint32_t lastAckMs = 0;
uint32_t lastAckRxFree = 0;
uint32_t lastStatusMs = 0;

// Diagnostics, logged every few seconds while they change: what goes wrong on this end of the link.
uint32_t gaps = 0;           // frames arriving after a lost or damaged one (the host resends)
uint32_t rejected = 0;       // audio refused: rx had no room (the host's credit was off) or another stream
std::atomic<uint32_t> uartErrors{0}; // receive overflows (the UART buffer or FIFO filled up)
uint32_t lastDiagMs = 0;
uint32_t loggedBad = 0, loggedGaps = 0, loggedRejected = 0, loggedUart = 0;
constexpr uint32_t DIAG_LOG_MS = 5000;

void onUartError(hardwareSerial_error_t error) {
  if (error == UART_BUFFER_FULL_ERROR || error == UART_FIFO_OVF_ERROR) ++uartErrors;
}

void send(const Type type, const void* payload, const size_t length) {
  uint8_t frame[audiolink::HEADER_BYTES + 128 + audiolink::CRC_BYTES];
  if (length > 128) return;
  const size_t size = audiolink::encode(frame, type, 0, payload, length);
  xSemaphoreTake(txLock, portMAX_DELAY);
  if (active) {
    active->serial->write(frame, size);
  } else {
    for (Port& port : ports) {
      if (port.up) port.serial->write(frame, size);
    }
  }
  xSemaphoreGive(txLock);
}

void sendHello() {
  audiolink::HelloPayload hello{};
  hello.protocolVersion = audiolink::PROTOCOL_VERSION;
  hello.firmwareMajor = config::FIRMWARE_MAJOR;
  hello.firmwareMinor = config::FIRMWARE_MINOR;
  hello.outputs = outputs::supported();
  hello.rxBufferBytes = config::RX_BUFFER_BYTES;
  hello.nextSeq = expectedSeq;
  hello.transport = static_cast<uint8_t>(active ? active->transport : audiolink::Transport::Unknown);
  send(Type::Hello, &hello, sizeof(hello));
}

void sendAck() {
  audiolink::AckPayload ack{};
  ack.nextSeq = expectedSeq;
  ack.rxFree = static_cast<uint32_t>(player::rxFree());
  ack.flags = outOfSequence ? audiolink::ACK_OUT_OF_SEQUENCE : 0;
  outOfSequence = false;
  lastAckRxFree = ack.rxFree;
  send(Type::Ack, &ack, sizeof(ack));
  ackPending = false;
  ackUrgent = false;
  lastAckMs = millis();
}

void sendStatus() {
  audiolink::StatusPayload status{};
  player::fillStatus(status);
  status.output = static_cast<uint8_t>(outputs::current());
  status.btState = static_cast<uint8_t>(outputs::btState());
  outputs::btAddress(status.btAddress);
  send(Type::Status, &status, sizeof(status));
  lastStatusMs = millis();
}

void saveSetting(const char* key, const uint8_t value) {
  Preferences prefs;
  if (prefs.begin("audio", false)) {
    prefs.putUChar(key, value);
    prefs.end();
  }
}

/** Handles an in-sequence host frame. False leaves it unaccepted so the host resends it later. */
bool handleHostFrame(const Type type, const uint8_t* payload, const uint16_t length) {
  switch (type) {
    case Type::AudioData: {
      if (length < 1) return true;
      const int streamId = payload[0];
      if (streamId != linkStream) return true;  // a stream that was stopped or replaced: drop it
      // Right after STREAM_START the decoder task (lower priority) still has to reset its buffers.
      // Yield to it briefly rather than refusing, which would cost the host a resend timeout.
      for (int i = 0; i < FLUSH_WAIT_MS && player::acceptingStream() != streamId; ++i) vTaskDelay(1);
      if (player::acceptingStream() != streamId) return false;
      return player::pushData(payload + 1, length - 1);
    }
    case Type::StreamStart: {
      // Protocol 2 hosts send only streamId, codec and startMs; the rest is then 0.
      if (length < offsetof(audiolink::StreamStartPayload, sampleRate)) return true;
      audiolink::StreamStartPayload start{};
      std::memcpy(&start, payload, std::min<size_t>(length, sizeof(start)));
      linkStream = start.streamId;
      player::streamStart(start.streamId, static_cast<audiolink::Codec>(start.codec), start.startMs, start.sampleRate,
                          start.channels);
      return true;
    }
    case Type::StreamEnd:
      if (length >= 1) player::streamEnd(payload[0]);
      return true;
    case Type::StreamStop:
      if (length >= 1 && payload[0] == linkStream) {
        player::streamStop(payload[0]);
        linkStream = -1;
      }
      return true;
    case Type::Pause:
      player::pause();
      return true;
    case Type::Resume:
      player::resume();
      return true;
    case Type::SetVolume:
      if (length >= 1) {
        player::setVolume(payload[0]);
        saveSetting("volume", payload[0]);
      }
      return true;
    case Type::SetOutput:
      if (length >= 1) {
        outputs::select(static_cast<audiolink::Output>(payload[0]));
        saveSetting("output", payload[0]);
      }
      return true;
    case Type::BtScan:
      outputs::btScan(length >= 1 && payload[0] != 0);
      return true;
    case Type::BtConnect:
      if (length >= 6) outputs::btConnect(payload);
      return true;
    case Type::BtDisconnect:
      outputs::btDisconnect();
      return true;
    case Type::BtForget:
      outputs::btForget();
      return true;
    default:
      return true;  // unknown command: accept and ignore, so a newer host cannot stall the link
  }
}

/**
 * Whether a frame from `port` is for us. The first port a host is heard on becomes the active one.
 * Another port takes over with a PING (a host starting a session), or with any frame once the
 * active port has been silent for LINK_TAKEOVER_MS.
 */
bool claim(Port& port, const Type type, const uint32_t now) {
  if (active == &port) return true;
  if (active && type != Type::Ping && now - lastActiveRxMs < config::LINK_TAKEOVER_MS) return false;
  xSemaphoreTake(txLock, portMAX_DELAY);
  active = &port;
  xSemaphoreGive(txLock);
  return true;
}

void handleFrame(Port& port) {
  audiolink::Parser& parser = port.parser;
  const Type type = parser.type();
  if (static_cast<uint8_t>(type) >= 0x80) return;  // module->host types echoed back: ignore
  const uint32_t now = millis();
  if (!claim(port, type, now)) return;
  lastActiveRxMs = now;

  if (type == Type::Ping) {
    expectedSeq = static_cast<uint8_t>(parser.seq() + 1);
    sendHello();
    sendAck();
    return;
  }
  if (parser.seq() != expectedSeq) {
    // A gap means a frame was lost or damaged: ask for a resend from expectedSeq. A duplicate
    // (an older sequence number) is the host already resending, so just acknowledge.
    if (!audiolink::seqBefore(parser.seq(), expectedSeq)) {
      if (!outOfSequence) ++gaps;
      outOfSequence = true;
    }
    ackPending = true;
    ackUrgent = true;
    return;
  }
  if (handleHostFrame(type, parser.payload(), parser.length())) {
    ++expectedSeq;
  } else {
    if (type == Type::AudioData) ++rejected;
    ackUrgent = true;
  }
  ackPending = true;
}

/** Feeds what one port has waiting to its parser. Returns the bytes read. */
size_t pollPort(Port& port, uint8_t* chunk) {
  if (!port.up) return 0;
  const int available = port.serial->available();
  if (available <= 0) return 0;
  const size_t n = port.serial->readBytes(chunk, std::min(static_cast<size_t>(available), READ_CHUNK));
  for (size_t i = 0; i < n; ++i) {
    if (port.parser.push(chunk[i])) handleFrame(port);
  }
  return n;
}

void linkTask(void*) {
  uint8_t chunk[READ_CHUNK];
  sendHello();
  for (;;) {
    size_t read = 0;
    for (Port& port : ports) read += pollPort(port, chunk);
    const uint32_t now = millis();
    if (ackPending && (ackUrgent || now - lastAckMs >= ACK_INTERVAL_MS)) {
      sendAck();
    } else if (now - lastAckMs >= CREDIT_UPDATE_MS && player::rxFree() != lastAckRxFree) {
      sendAck();
    }
    if (now - lastStatusMs >= audiolink::STATUS_PERIOD_MS) sendStatus();
    if (now - lastDiagMs >= DIAG_LOG_MS) {
      lastDiagMs = now;
      uint32_t bad = 0;
      for (const Port& port : ports) bad += port.parser.badFrames();
      const uint32_t uart = uartErrors;
      if (bad != loggedBad || gaps != loggedGaps || rejected != loggedRejected || uart != loggedUart) {
        log("link: %lu bad frames, %lu gaps, %lu audio refused, %lu UART overflows in 5 s",
            static_cast<unsigned long>(bad - loggedBad), static_cast<unsigned long>(gaps - loggedGaps),
            static_cast<unsigned long>(rejected - loggedRejected), static_cast<unsigned long>(uart - loggedUart));
        loggedBad = bad;
        loggedGaps = gaps;
        loggedRejected = rejected;
        loggedUart = uart;
      }
    }
    if (read == 0) vTaskDelay(1);
  }
}

void beginPort(Port& port, const uint32_t baud, const int rxPin, const int txPin) {
  port.serial->setRxBufferSize(config::UART_RX_BUFFER);
  port.serial->setTxBufferSize(config::UART_TX_BUFFER);
  port.serial->begin(baud, SERIAL_8N1, rxPin, txPin, false, 20000UL, config::UART_RX_FIFO_FULL);
  port.serial->onReceiveError(onUartError);
  port.up = true;
}

}  // namespace

bool begin() {
  txLock = xSemaphoreCreateMutex();
  if (!txLock) return false;
  // UART0 is wired to the ATOM's USB bridge, on its default pins.
  beginPort(ports[0], AUDIO_MODULE_BAUD, -1, -1);
#if AUDIO_MODULE_UART_RX >= 0 && AUDIO_MODULE_UART_TX >= 0
  beginPort(ports[1], AUDIO_MODULE_UART_BAUD, AUDIO_MODULE_UART_RX, AUDIO_MODULE_UART_TX);
#endif
  return xTaskCreatePinnedToCore(linkTask, "link", config::LINK_TASK_STACK, nullptr, config::LINK_TASK_PRIORITY,
                                 nullptr, config::APP_CORE) == pdPASS;
}

void log(const char* format, ...) {
  char text[120];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  if (length > 0) send(Type::Log, text, std::min(static_cast<size_t>(length), sizeof(text) - 1));
}

void sendBtDevice(const uint8_t address[6], const int8_t rssi, const char* name) {
  audiolink::BtDevicePayload device{};
  std::memcpy(device.address, address, sizeof(device.address));
  device.rssi = rssi;
  std::strncpy(device.name, name, sizeof(device.name) - 1);
  send(Type::BtDevice, &device, sizeof(device));
}

audiolink::Transport activeTransport() { return active ? active->transport : audiolink::Transport::Unknown; }

}  // namespace hostlink
