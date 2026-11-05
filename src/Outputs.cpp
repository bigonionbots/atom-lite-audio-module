/**
 * @file Outputs.cpp
 * @brief Definitions for the module's audio outputs.
 */

#include "Outputs.h"

#include <Arduino.h>
#include <BluetoothA2DPSource.h>
#include <ESP_I2S.h>

#include <atomic>
#include <cstring>

#include "Config.h"
#include "Player.h"

namespace outputs {

namespace {

using audiolink::BtState;
using audiolink::Output;

constexpr size_t OUTPUT_CHUNK_FRAMES = 256;
constexpr uint32_t NULL_SINK_PERIOD_MS = 10;
constexpr uint32_t SCAN_RETRY_MS = 2000;

/**
 * ESP32-A2DP's source scans until a device is accepted and only reconnects on its own. This
 * exposes on-demand scanning and connecting to an address picked on the host.
 */
class ModuleA2dpSource : public BluetoothA2DPSource {
 public:
  void startScan() {
    is_end = false;
    if (!discovery_active) esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
  }

  void stopScan() {
    is_end = true;
    if (discovery_active) esp_bt_gap_cancel_discovery();
  }

  bool scanning() const { return discovery_active; }
  bool connecting() const { return s_a2d_state == APP_AV_STATE_CONNECTING; }

  void connectAddress(const uint8_t address[6]) {
    esp_bd_addr_t peer;
    std::memcpy(peer, address, sizeof(peer));
    stopScan();
    std::memcpy(peer_bd_addr, peer, sizeof(peer));
    set_last_connection(peer);
    reconnect_status = AutoReconnect;
    is_autoreconnect_allowed = true;
    reconnect_retries = max_reconnect_retries;
    s_a2d_state = APP_AV_STATE_CONNECTING;
    connect_to(peer);
  }
};

ModuleA2dpSource a2dp;
I2SClass i2s;

std::atomic<Output> selected{Output::Bluetooth};
std::atomic<bool> btStarted{false};
std::atomic<bool> scanRequested{false};
DeviceCallback deviceCallback = nullptr;

// Called from the Bluetooth task while discovering. Report every audio sink; never auto-connect.
bool onSsid(const char* name, esp_bd_addr_t address, int rssi) {
  if (deviceCallback) deviceCallback(address, static_cast<int8_t>(rssi), name ? name : "");
  return false;
}

int32_t onFrames(Frame* frames, const int32_t count) {
  size_t got = 0;
  if (selected == Output::Bluetooth) {
    got = player::readPcm(reinterpret_cast<StereoFrame*>(frames), static_cast<size_t>(count));
  }
  std::memset(frames + got, 0, (count - got) * sizeof(Frame));
  return count;
}

void startBluetooth() {
  if (btStarted.exchange(true)) return;
  a2dp.set_local_name(config::BT_LOCAL_NAME);
  a2dp.set_ssid_callback(onSsid);
  a2dp.set_data_callback_in_frames(onFrames);
  a2dp.set_auto_reconnect(true);
  // BLE is unused: give its controller memory back to the heap.
  a2dp.set_reset_ble(true);
  a2dp.start();
  // Our own gain does the volume; keep the library's at unity (headset buttons can still lower it).
  a2dp.set_volume(127);
}

bool i2sStarted = false;

void startI2s() {
  if (i2sStarted) return;
  i2s.setPins(AUDIO_MODULE_I2S_BCK, AUDIO_MODULE_I2S_WS, AUDIO_MODULE_I2S_DOUT);
  i2sStarted = i2s.begin(I2S_MODE_STD, config::BT_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
}

void stopI2s() {
  if (!i2sStarted) return;
  i2s.end();
  i2sStarted = false;
}

// Drives I2S and the null sink, and keeps a requested Bluetooth scan alive.
void outputTask(void*) {
  StereoFrame chunk[OUTPUT_CHUNK_FRAMES];
  uint32_t lastScanTry = 0;
  for (;;) {
    if (btStarted) {
      if (scanRequested && !a2dp.scanning() && !a2dp.is_connected() && millis() - lastScanTry > SCAN_RETRY_MS) {
        lastScanTry = millis();
        a2dp.startScan();
      } else if (!scanRequested && a2dp.scanning()) {
        // The library starts scanning on its own when no headset is remembered; only scan on request.
        a2dp.stopScan();
      }
    }

    const Output output = selected;
    if (output == Output::I2s) {
      startI2s();
      if (!i2sStarted) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      const size_t got = player::readPcm(chunk, OUTPUT_CHUNK_FRAMES);
      std::memset(chunk + got, 0, (OUTPUT_CHUNK_FRAMES - got) * sizeof(StereoFrame));
      // Blocks until the DMA has room, which paces this loop at the sample rate.
      i2s.write(reinterpret_cast<const uint8_t*>(chunk), sizeof(chunk));
      continue;
    }
    stopI2s();

    if (output == Output::Null) {
      // Consume in real time and discard: exercises the link and the decoder without hardware.
      size_t remaining = config::BT_SAMPLE_RATE * NULL_SINK_PERIOD_MS / 1000;
      while (remaining > 0) {
        const size_t got = player::readPcm(chunk, std::min(remaining, OUTPUT_CHUNK_FRAMES));
        if (got == 0) break;
        remaining -= got;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(NULL_SINK_PERIOD_MS));
  }
}

}  // namespace

bool begin(const Output initial, const DeviceCallback onDevice) {
  deviceCallback = onDevice;
  player::setOutputRate(config::BT_SAMPLE_RATE);
  select(initial);
  return xTaskCreatePinnedToCore(outputTask, "output", config::OUTPUT_TASK_STACK, nullptr, config::OUTPUT_TASK_PRIORITY,
                                 nullptr, config::APP_CORE) == pdPASS;
}

void select(const Output output) {
  if (output != Output::Bluetooth && output != Output::I2s && output != Output::Null) return;
  selected = output;
  if (output == Output::Bluetooth) startBluetooth();
}

Output current() { return selected; }

uint8_t supported() {
  return (1u << static_cast<uint8_t>(Output::Bluetooth)) | (1u << static_cast<uint8_t>(Output::I2s)) |
         (1u << static_cast<uint8_t>(Output::Null));
}

void btScan(const bool start) {
  scanRequested = start;
  if (start) {
    startBluetooth();
    a2dp.startScan();
  } else if (btStarted) {
    a2dp.stopScan();
  }
}

void btConnect(const uint8_t address[6]) {
  scanRequested = false;
  startBluetooth();
  a2dp.connectAddress(address);
}

void btDisconnect() {
  if (btStarted) a2dp.disconnect();
}

void btForget() {
  if (!btStarted) return;
  a2dp.disconnect();
  a2dp.clean_last_connection();
}

BtState btState() {
  if (!btStarted) return BtState::Off;
  if (a2dp.is_connected()) return BtState::Connected;
  if (a2dp.scanning()) return BtState::Scanning;
  if (a2dp.connecting()) return BtState::Connecting;
  return BtState::Idle;
}

void btAddress(uint8_t out[6]) {
  if (!btStarted) {
    std::memset(out, 0, 6);
    return;
  }
  std::memcpy(out, *a2dp.get_last_peer_address(), 6);
}

}  // namespace outputs
