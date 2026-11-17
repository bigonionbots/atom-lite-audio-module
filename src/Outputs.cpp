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

// ESP32-A2DP's private constants (BluetoothA2DPSource.cpp).
constexpr uint16_t HEART_BEAT_EVT = 0xff00;
constexpr uint16_t STACK_UP_EVT = 0;
constexpr int MEDIA_STATE_IDLE = 0;

/**
 * ESP32-A2DP's source scans until a device is accepted and only reconnects on its own. This
 * exposes on-demand scanning and connecting to an address picked on the host, and fixes the
 * library's slow connection handling:
 * - Its stack-up handler sleeps 10 s before the first connect; ours connects at once.
 * - Its heartbeat is 10 s, so a failed page cost 10-20 s; ours is BT_HEARTBEAT_MS.
 * - A host connect request that arrived before the stack was up failed (ESP_ERR_INVALID_STATE)
 *   and left the state machine stuck in "connecting". Requests now run on the Bluetooth app
 *   task, queued behind the stack-up event.
 * - Reconnects from the unconnected state never set "connecting", and a connection that came
 *   up there was never noticed, so audio waited for luck. Both are handled now.
 * - Media start waited for the next heartbeat after connecting; it is requested at once.
 * - When retries ran out the library fell back to discovery and stopped reconnecting. A
 *   remembered headset is now retried until the host disconnects or forgets it.
 */
class ModuleA2dpSource : public BluetoothA2DPSource {
 public:
  ModuleA2dpSource() { self = this; }

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

  /** Connects to a headset. Safe from any task, and before the Bluetooth stack is up. */
  void connectAddress(const uint8_t address[6]) {
    stopScan();
    bt_app_work_dispatch(onConnectWork, 0, const_cast<uint8_t*>(address), ESP_BD_ADDR_LEN, nullptr);
  }

 protected:
  void av_hdl_stack_evt(uint16_t event, void* param) override {
    if (event != STACK_UP_EVT) {
      BluetoothA2DPSource::av_hdl_stack_evt(event, param);
      return;
    }
    // The library's stack-up setup, minus its 10 s sleep and its automatic discovery.
    esp_bt_gap_set_device_name(dev_name);
    esp_bt_gap_register_callback(ccall_app_gap_callback);
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(ccall_app_rc_ct_callback);
    if (is_passthru_active) {
      esp_avrc_tg_init();
      esp_avrc_tg_register_callback(ccall_app_rc_tg_callback);
    }
    esp_avrc_rn_evt_cap_mask_t events = {0};
    for (auto id : avrc_rn_events) esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &events, id);
    esp_avrc_tg_set_rn_evt_cap(&events);
    esp_a2d_source_init();
    esp_a2d_register_callback(ccall_app_a2d_callback);
    esp_a2d_source_register_data_callback(ccall_bt_app_a2d_data_cb);
    // Not connectable: only we start connections, so the state machine stays in step.
    set_scan_mode_connectable(false);

    s_a2d_state = APP_AV_STATE_UNCONNECTED;
    if (wantsPeer()) startConnect();

    s_tmr = xTimerCreate("connTmr", pdMS_TO_TICKS(config::BT_HEARTBEAT_MS), pdTRUE, nullptr,
                         ccall_a2d_app_heart_beat);
    xTimerStart(s_tmr, portMAX_DELAY);
  }

  void bt_app_av_state_unconnected_hdlr(uint16_t event, void* param) override {
    if (event == ESP_A2D_CONNECTION_STATE_EVT) {
      const auto state = static_cast<esp_a2d_cb_param_t*>(param)->conn_stat.state;
      if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        onConnected();
      } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        s_a2d_state = APP_AV_STATE_CONNECTING;
        s_connecting_heatbeat_count = 0;
      } else {
        retryAfterFailure();
      }
      return;
    }
    if (event == HEART_BEAT_EVT) {
      if (wantsPeer()) startConnect();
      return;
    }
    BluetoothA2DPSource::bt_app_av_state_unconnected_hdlr(event, param);
  }

  // Replaces the library's handler, which sent CHECK_SRC_RDY on each heartbeat while paging and
  // took the (failed) reply as proof of a connection: the state flipped to "connected", and the
  // page failure that followed then went unretried until the next heartbeat.
  void bt_app_av_state_connecting_hdlr(uint16_t event, void* param) override {
    switch (event) {
      case ESP_A2D_CONNECTION_STATE_EVT: {
        const auto state = static_cast<esp_a2d_cb_param_t*>(param)->conn_stat.state;
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
          onConnected();
        } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
          s_a2d_state = APP_AV_STATE_UNCONNECTED;
          retryAfterFailure();
        }
        break;
      }
      case HEART_BEAT_EVT:
        // A page answers or times out (about 5 s) on its own; this only catches a lost event.
        if (++s_connecting_heatbeat_count >= 4) {
          s_a2d_state = APP_AV_STATE_UNCONNECTED;
          s_connecting_heatbeat_count = 0;
        }
        break;
      default:
        break;
    }
  }

  // A headset that drops (out of range, switched off) is paged again at once.
  void bt_app_av_state_connected_hdlr(uint16_t event, void* param) override {
    if (event == ESP_A2D_CONNECTION_STATE_EVT &&
        static_cast<esp_a2d_cb_param_t*>(param)->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
      s_a2d_state = APP_AV_STATE_UNCONNECTED;
      retryAfterFailure();
      return;
    }
    BluetoothA2DPSource::bt_app_av_state_connected_hdlr(event, param);
  }

 private:
  static ModuleA2dpSource* self;
  uint32_t lastPageMs = 0;

  // Runs on the Bluetooth app task, after the stack-up event.
  static void onConnectWork(uint16_t, void* param) { self->connectNow(static_cast<uint8_t*>(param)); }

  bool wantsPeer() { return is_autoreconnect_allowed && has_last_connection(); }

  void connectNow(uint8_t* peer) {
    const bool same = std::memcmp(peer, last_connection, ESP_BD_ADDR_LEN) == 0;
    set_last_connection(peer);
    reconnect_status = AutoReconnect;
    is_autoreconnect_allowed = true;
    if (s_a2d_state == APP_AV_STATE_CONNECTED || s_a2d_state == APP_AV_STATE_DISCONNECTING) {
      // Connected to another headset: drop it; the next heartbeat connects the new one.
      if (!same) esp_a2d_source_disconnect(peer_bd_addr);
      return;
    }
    if (s_a2d_state == APP_AV_STATE_CONNECTING && same) return;  // already on it
    startConnect();
  }

  // A failed page (often the first one after boot) is retried at once instead of on the next
  // heartbeat; a failure within a second of the attempt is left to the heartbeat.
  void retryAfterFailure() {
    if (wantsPeer() && millis() - lastPageMs >= 1000) startConnect();
  }

  void startConnect() {
    lastPageMs = millis();
    std::memcpy(peer_bd_addr, last_connection, ESP_BD_ADDR_LEN);
    s_connecting_heatbeat_count = 0;
    s_a2d_state = connect_to(last_connection) ? APP_AV_STATE_CONNECTING : APP_AV_STATE_UNCONNECTED;
  }

  void onConnected() {
    ESP_LOGI(BT_AV_TAG, "a2dp connected");
    s_a2d_state = APP_AV_STATE_CONNECTED;
    s_media_state = MEDIA_STATE_IDLE;
    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
  }
};

ModuleA2dpSource* ModuleA2dpSource::self = nullptr;

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
