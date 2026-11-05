/**
 * @file main.cpp
 * @brief ATOM Lite audio module: plays audio a host board streams to it over USB or UART.
 *
 * Link (Link.h) -> player (Player.h) -> output (Outputs.h): Bluetooth A2DP headphones, a PCM5102A on
 * I2S, or a null sink for testing. The selected output and the volume persist in NVS.
 *
 * LED: blue idle, cyan buffering, green playing, yellow paused, purple Bluetooth scanning,
 * red if a buffer could not be allocated.
 *
 * Builds with AUDIO_MODULE_DIAG_MODE != 0 are the stage-2 link diagnostics instead (LinkDiag.cpp).
 */

#include <Arduino.h>

#include "LinkDiag.h"

#if AUDIO_MODULE_DIAG_MODE == 0

#include <Preferences.h>

#include "Config.h"
#include "Link.h"
#include "Outputs.h"
#include "Player.h"

// loop() only drives the LED; the default 8 KB loop stack is RAM the decoder needs.
SET_LOOP_TASK_STACK_SIZE(3 * 1024);

namespace {

uint32_t shownColor = 0xFFFFFFFF;
bool failed = false;

void showColor(const uint32_t rgb) {
  if (rgb == shownColor) return;
  shownColor = rgb;
  rgbLedWrite(config::LED_PIN, (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
}

uint32_t stateColor() {
  constexpr uint32_t L = config::LED_LEVEL;
  if (failed) return L << 16;
  if (outputs::btState() == audiolink::BtState::Scanning) return (L << 16) | L;
  switch (player::state()) {
    case audiolink::PlayState::Buffering:
      return (L << 8) | L;
    case audiolink::PlayState::Playing:
    case audiolink::PlayState::Draining:
      return L << 8;
    case audiolink::PlayState::Paused:
      return (L << 16) | (L << 8);
    default:
      return L;
  }
}

void onBtDevice(const uint8_t address[6], const int8_t rssi, const char* name) {
  hostlink::sendBtDevice(address, rssi, name);
}

}  // namespace

void setup() {
  Preferences prefs;
  uint8_t output = static_cast<uint8_t>(audiolink::Output::Bluetooth);
  uint8_t volume = 70;
  if (prefs.begin("audio", true)) {
    output = prefs.getUChar("output", output);
    volume = prefs.getUChar("volume", volume);
    prefs.end();
  }

  const uint32_t heapAtBoot = ESP.getFreeHeap();
  const uint32_t blockAtBoot = ESP.getMaxAllocHeap();
  failed = !player::begin();
  player::setVolume(volume);
  failed = !hostlink::begin() || failed;
  failed = !outputs::begin(static_cast<audiolink::Output>(output), onBtDevice) || failed;
  hostlink::log("module %u.%u up: output %u, volume %u%s", config::FIRMWARE_MAJOR, config::FIRMWARE_MINOR, output,
                volume, failed ? ", INIT FAILED" : "");
  hostlink::log("heap: boot %lu (block %lu), now %lu (block %lu)", static_cast<unsigned long>(heapAtBoot),
                static_cast<unsigned long>(blockAtBoot), static_cast<unsigned long>(ESP.getFreeHeap()),
                static_cast<unsigned long>(ESP.getMaxAllocHeap()));
}

void loop() {
  showColor(stateColor());
  delay(100);
}

#else

void setup() { linkDiagSetup(); }

void loop() { linkDiagRun(); }

#endif
