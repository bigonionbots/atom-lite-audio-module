#pragma once

/**
 * @file Config.h
 * @brief Pins, buffer sizes and task layout for the ATOM Lite audio module.
 */

#include <cstddef>
#include <cstdint>

// PCM5102A I2S DAC on the ATOM Lite's header. Its SCK pin goes to GND (internal PLL), XSMT high.
#ifndef AUDIO_MODULE_I2S_BCK
#define AUDIO_MODULE_I2S_BCK 22
#endif
#ifndef AUDIO_MODULE_I2S_WS
#define AUDIO_MODULE_I2S_WS 19
#endif
#ifndef AUDIO_MODULE_I2S_DOUT
#define AUDIO_MODULE_I2S_DOUT 23
#endif

// USB link: UART0 through the ATOM Lite's USB bridge. This bridge (an FTDI emulation) only works
// at some rates; 500000 is the tested one (README, "USB bridge").
#ifndef AUDIO_MODULE_BAUD
#define AUDIO_MODULE_BAUD 500000
#endif

// UART link on the Grove port (Serial1). Wire host TX -> RX pin, host RX -> TX pin, and GND. Set
// either pin to -1 to build without it.
#ifndef AUDIO_MODULE_UART_RX
#define AUDIO_MODULE_UART_RX 32
#endif
#ifndef AUDIO_MODULE_UART_TX
#define AUDIO_MODULE_UART_TX 26
#endif
#ifndef AUDIO_MODULE_UART_BAUD
#define AUDIO_MODULE_UART_BAUD 500000
#endif

namespace config {

constexpr uint8_t FIRMWARE_MAJOR = 0;
constexpr uint8_t FIRMWARE_MINOR = 2;

// RAM is the constraint: Bluetooth Classic takes ~50 KB of heap and the AAC decoder ~30-60 KB, on
// an ESP32 without PSRAM. The host holds a large queue, so the module only needs to ride out jitter.
/** Compressed audio waiting to be decoded: about 2 s at 64 kbit/s. */
constexpr size_t RX_BUFFER_BYTES = 16 * 1024;
/** Decoded stereo frames waiting for the output: about 46 ms at 44.1 kHz. */
constexpr size_t PCM_RING_FRAMES = 2048;
/** Playback starts once the PCM ring is this full, so the first frames do not underrun. */
constexpr size_t PCM_PRIME_FRAMES = PCM_RING_FRAMES / 2;
/**
 * A live stream (game sound) starts, and restarts after running dry, once this much is waiting
 * (PCM ring plus undecoded bytes). Its data comes in bursts, a frame of the game at a time over
 * a USB link, and Bluetooth takes it in bursts too; less than this underran constantly.
 */
constexpr uint32_t LIVE_PRIME_MS = 60;

constexpr size_t UART_RX_BUFFER = 4096;
constexpr size_t UART_TX_BUFFER = 2048;
constexpr uint8_t UART_RX_FIFO_FULL = 32;

/** A2DP sources always send 44.1 kHz stereo (SBC). */
constexpr uint32_t BT_SAMPLE_RATE = 44100;
constexpr const char* BT_LOCAL_NAME = "ATOM Lite Audio";

/** Another port takes over the link once the active one has been silent this long (Link.h). */
constexpr uint32_t LINK_TAKEOVER_MS = 3000;

constexpr int LED_PIN = 27;
constexpr uint8_t LED_LEVEL = 20;

// Tasks. The Bluetooth stack runs on core 0; ours share core 1. The link outranks the decoder so
// the UART never backs up while a frame decodes.
constexpr uint32_t LINK_TASK_STACK = 4096;
constexpr uint32_t DECODE_TASK_STACK = 8192;
constexpr uint32_t OUTPUT_TASK_STACK = 4096;
constexpr int LINK_TASK_PRIORITY = 5;
constexpr int DECODE_TASK_PRIORITY = 3;
constexpr int OUTPUT_TASK_PRIORITY = 4;
constexpr int APP_CORE = 1;

}  // namespace config
