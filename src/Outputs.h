#pragma once

/**
 * @file Outputs.h
 * @brief Audio outputs: Bluetooth A2DP source (headphones), I2S (PCM5102A) and a null sink.
 *
 * Every output runs at 44.1 kHz stereo; the player resamples to it. Bluetooth starts on first use
 * and stays up (its stack cannot be torn down and restarted cheaply); switching away from it just
 * stops feeding it. The remembered headset lives in NVS (ESP32-A2DP stores it) and reconnects at
 * boot when Bluetooth is the selected output.
 */

#include <AudioLinkProtocol.h>

#include <cstdint>

namespace outputs {

using DeviceCallback = void (*)(const uint8_t address[6], int8_t rssi, const char* name);

bool begin(audiolink::Output initial, DeviceCallback onDevice);
void select(audiolink::Output output);
audiolink::Output current();
/** Bitmask of (1 << Output) this build supports. */
uint8_t supported();

void btScan(bool start);
void btConnect(const uint8_t address[6]);
void btDisconnect();
void btForget();
audiolink::BtState btState();
/** Connected or remembered device; zeros when none. */
void btAddress(uint8_t out[6]);

}  // namespace outputs
