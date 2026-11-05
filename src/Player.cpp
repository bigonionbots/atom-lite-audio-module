/**
 * @file Player.cpp
 * @brief Definitions for the module's player: buffers, decoder task, resampler.
 */

#include "Player.h"

#include <AACDecoderHelix.h>
#include <Arduino.h>
#include <MP3DecoderHelix.h>
#include <MuLaw.h>
#include <Resample2x.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>

#include <atomic>

#include "Config.h"
#include "Link.h"

namespace player {

namespace {

using audiolink::Codec;
using audiolink::PlayState;

struct Command {
  enum class Kind : uint8_t { Start, End, Stop } kind;
  uint8_t streamId;
  Codec codec;
  uint32_t sampleRate;  // live streams
  uint8_t channels;
};

constexpr size_t DECODE_CHUNK = 512;
constexpr size_t EMIT_FRAMES = 256;

StreamBufferHandle_t rx = nullptr;
QueueHandle_t commands = nullptr;
PcmRing pcm;

libhelix::MP3DecoderHelix mp3;
libhelix::AACDecoderHelix aac;
libhelix::CommonHelix* decoder = nullptr;

// Decoder task only: the live (µ-law) stream, when one is playing instead of a compressed one.
std::atomic<bool> live{false};
std::atomic<uint32_t> liveRate{0};
int16_t liveSamples[DECODE_CHUNK];
resample2x::Interpolator liveInterpolator;  // when the output runs at exactly twice the stream
std::atomic<uint16_t> liveDrops{0};         // chunks dropped

// Written by the link task, read everywhere.
std::atomic<int> currentStream{-1};
std::atomic<bool> accepting{false};
std::atomic<bool> abortDecode{false};
std::atomic<bool> paused{false};
std::atomic<int32_t> gainQ15{32768};
std::atomic<uint8_t> volumePercent{100};

// Written by the decoder task.
std::atomic<PlayState> playState{PlayState::Idle};
std::atomic<uint32_t> sourceRate{0};
std::atomic<uint8_t> sourceChannels{0};
std::atomic<bool> endReceived{false};
std::atomic<uint16_t> decodeErrors{0};

// Written by the output side.
std::atomic<uint32_t> outputRate{config::BT_SAMPLE_RATE};
std::atomic<bool> primed{false};
std::atomic<uint32_t> playedFrames{0};
std::atomic<uint32_t> playedMsBase{0};
std::atomic<uint16_t> underruns{0};

// Decoder task only: resampler state.
uint32_t resampleStep = 1u << 16;  // source frames per output frame, Q16
uint32_t resamplePhase = 0;
StereoFrame resamplePrev{0, 0};
uint32_t resampleOutputRate = 0;
StereoFrame emitBuffer[EMIT_FRAMES];
size_t emitCount = 0;

int16_t applyGain(const int32_t sample) { return static_cast<int16_t>((sample * gainQ15.load()) >> 15); }

void flushEmit() {
  size_t written = 0;
  while (written < emitCount) {
    if (abortDecode) break;
    written += pcm.write(emitBuffer + written, emitCount - written);
    if (written < emitCount) vTaskDelay(pdMS_TO_TICKS(2));
  }
  emitCount = 0;
}

void emit(const StereoFrame& frame) {
  emitBuffer[emitCount++] = {applyGain(frame.left), applyGain(frame.right)};
  if (emitCount == EMIT_FRAMES) flushEmit();
}

void resetResampler() {
  resampleOutputRate = outputRate.load();
  const uint32_t rate = sourceRate.load();
  resampleStep = rate && resampleOutputRate
                     ? static_cast<uint32_t>((static_cast<uint64_t>(rate) << 16) / resampleOutputRate)
                     : (1u << 16);
  resamplePhase = 0;
  resamplePrev = {0, 0};
}

// Linear interpolation is plenty for speech, and costs almost nothing.
void handlePcm(const int16_t* samples, const size_t sampleCount, const uint32_t rate, const uint8_t channels) {
  if (abortDecode || channels == 0 || rate == 0) return;
  if (rate != sourceRate || channels != sourceChannels) {
    sourceRate = rate;
    sourceChannels = channels;
    resetResampler();
  }
  if (outputRate != resampleOutputRate) resetResampler();

  const size_t frames = sampleCount / channels;
  const bool passThrough = resampleStep == (1u << 16);
  for (size_t i = 0; i < frames && !abortDecode; ++i) {
    const StereoFrame current = channels == 1 ? StereoFrame{samples[i], samples[i]}
                                              : StereoFrame{samples[i * channels], samples[i * channels + 1]};
    if (passThrough) {
      emit(current);
      continue;
    }
    while (resamplePhase < (1u << 16)) {
      // Q15 so (difference up to 65535) * phase stays inside int32.
      const int32_t phase = static_cast<int32_t>(resamplePhase >> 1);
      emit({static_cast<int16_t>(resamplePrev.left + (((current.left - resamplePrev.left) * phase) >> 15)),
            static_cast<int16_t>(resamplePrev.right + (((current.right - resamplePrev.right) * phase) >> 15))});
      resamplePhase += resampleStep;
    }
    resamplePhase -= 1u << 16;
    resamplePrev = current;
  }
  flushEmit();
}

void onMp3(MP3FrameInfo& info, short* samples, const size_t length, void*) {
  handlePcm(samples, length, static_cast<uint32_t>(info.samprate), static_cast<uint8_t>(info.nChans));
}

void onAac(_AACFrameInfo& info, short* samples, const size_t length, void*) {
  handlePcm(samples, length, static_cast<uint32_t>(info.sampRateOut), static_cast<uint8_t>(info.nChans));
}

void endDecoder() {
  if (decoder) decoder->end();
  decoder = nullptr;
  live = false;
}

/** Audio of the live stream not yet played: samples waiting in rx, plus the PCM ring. */
uint32_t liveBufferedMs() {
  const size_t rxSamples = config::RX_BUFFER_BYTES - xStreamBufferSpacesAvailable(rx);
  const uint32_t rate = outputRate;
  return (liveRate ? static_cast<uint32_t>(rxSamples * 1000 / liveRate) : 0) +
         (rate ? static_cast<uint32_t>(pcm.available() * 1000 / rate) : 0);
}

/** Live stream: decodes what arrived, dropping it instead while too much is waiting. */
void serviceLive(uint8_t* chunk) {
  const size_t got = xStreamBufferReceive(rx, chunk, DECODE_CHUNK, pdMS_TO_TICKS(20));
  if (got == 0) return;
  // Only drop once playback has started: until then the buffer is filling to the prime level.
  if (primed && liveBufferedMs() > audiolink::LIVE_MAX_BUFFERED_MS) {
    liveDrops = static_cast<uint16_t>(liveDrops + 1);
    return;
  }
  if (outputRate == 2 * liveRate) {
    // The usual case (22050 Hz game sound, 44.1 kHz out): a real low-pass, not linear steps,
    // which let the harmonics of square waves through as crackle (Resample2x.h).
    for (size_t i = 0; i < got && !abortDecode; ++i) {
      int16_t first = 0;
      int16_t second = 0;
      liveInterpolator.push(mulaw::decode(chunk[i]), first, second);
      emit({first, first});
      emit({second, second});
    }
    flushEmit();
    return;
  }
  for (size_t i = 0; i < got; ++i) liveSamples[i] = mulaw::decode(chunk[i]);
  handlePcm(liveSamples, got, liveRate, 1);
}

void resetPlayback() {
  xStreamBufferReset(rx);
  pcm.clear();
  emitCount = 0;
  sourceRate = 0;
  sourceChannels = 0;
  primed = false;
  endReceived = false;
  playedFrames = 0;
  playedMsBase = 0;
  underruns = 0;
  decodeErrors = 0;
  resetResampler();
}

void handleCommand(const Command& command, bool& active, bool& flushed) {
  switch (command.kind) {
    case Command::Kind::Start:
      endDecoder();
      resetPlayback();
      if (command.codec == Codec::MuLaw) {
        live = command.sampleRate > 0 && command.channels == 1;
        liveRate = command.sampleRate;
        liveInterpolator = resample2x::Interpolator();
        sourceRate = command.sampleRate;  // STATUS shows it
        sourceChannels = 1;
        resetResampler();  // for handlePcm(), when the output is not exactly twice the rate
        liveDrops = 0;
        if (live) {
          hostlink::log("live stream: mu-law %lu Hz mono", static_cast<unsigned long>(liveRate.load()));
        } else {
          hostlink::log("live stream rejected: %lu Hz, %u channels", static_cast<unsigned long>(command.sampleRate),
                        command.channels);
        }
      } else {
        decoder = command.codec == Codec::AacAdts ? static_cast<libhelix::CommonHelix*>(&aac)
                                                  : static_cast<libhelix::CommonHelix*>(&mp3);
      }
      if (decoder) {
        const uint32_t heapBefore = ESP.getFreeHeap();
        if (!decoder->begin()) {
          hostlink::log("decoder start failed, free heap %lu, largest block %lu",
                        static_cast<unsigned long>(heapBefore), static_cast<unsigned long>(ESP.getMaxAllocHeap()));
        } else {
          hostlink::log("%s decoder: %lu B, free heap %lu", command.codec == Codec::AacAdts ? "AAC" : "MP3",
                        static_cast<unsigned long>(heapBefore - ESP.getFreeHeap()),
                        static_cast<unsigned long>(ESP.getFreeHeap()));
        }
      }
      active = true;
      flushed = false;
      playState = paused ? PlayState::Paused : PlayState::Buffering;
      abortDecode = false;
      accepting = true;
      break;
    case Command::Kind::End:
      if (command.streamId == currentStream) endReceived = true;
      break;
    case Command::Kind::Stop:
      endDecoder();
      resetPlayback();
      active = false;
      playState = PlayState::Idle;
      abortDecode = false;
      break;
  }
}

void decodeTask(void*) {
  bool active = false;
  bool flushed = false;
  uint8_t chunk[DECODE_CHUNK];
  for (;;) {
    Command command;
    TickType_t wait = active ? 0 : portMAX_DELAY;
    while (xQueueReceive(commands, &command, wait) == pdTRUE) {
      handleCommand(command, active, flushed);
      wait = 0;
    }
    if (!active) continue;

    if (live) {
      serviceLive(chunk);
      if (!endReceived) continue;
    } else if (decoder) {
      const size_t got = xStreamBufferReceive(rx, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
      if (got > 0) {
        if (decoder->write(chunk, got) < got) ++decodeErrors;
        continue;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));  // a stream that could not start: nothing to decode
    }
    if (endReceived && !flushed) {
      if (decoder) decoder->flush();
      flushEmit();
      flushed = true;
      primed = true;  // play out whatever is left even if it never filled the prime level
      if (playState != PlayState::Paused) playState = PlayState::Draining;
    }
    if (flushed && pcm.available() == 0 && playState != PlayState::Paused) {
      playState = PlayState::Finished;
      active = false;
    }
  }
}

}  // namespace

bool begin() {
  rx = xStreamBufferCreate(config::RX_BUFFER_BYTES, 1);
  commands = xQueueCreate(8, sizeof(Command));
  if (!rx || !commands || !pcm.begin(config::PCM_RING_FRAMES)) return false;
  mp3.setDataCallback(onMp3);
  aac.setDataCallback(onAac);
  return xTaskCreatePinnedToCore(decodeTask, "decode", config::DECODE_TASK_STACK, nullptr, config::DECODE_TASK_PRIORITY,
                                 nullptr, config::APP_CORE) == pdPASS;
}

void streamStart(const uint8_t streamId, const audiolink::Codec codec, const uint32_t, const uint32_t sampleRate,
                 const uint8_t channels) {
  // Stop accepting first: the decoder task resets the buffer, and the link task is its only writer.
  accepting = false;
  abortDecode = true;
  currentStream = streamId;
  const Command command{Command::Kind::Start, streamId, codec, sampleRate, channels};
  xQueueSend(commands, &command, portMAX_DELAY);
}

void streamEnd(const uint8_t streamId) {
  const Command command{Command::Kind::End, streamId, audiolink::Codec::Mp3, 0, 0};
  xQueueSend(commands, &command, portMAX_DELAY);
}

void streamStop(const uint8_t streamId) {
  if (streamId != currentStream) return;
  accepting = false;
  abortDecode = true;
  currentStream = -1;
  const Command command{Command::Kind::Stop, streamId, audiolink::Codec::Mp3, 0, 0};
  xQueueSend(commands, &command, portMAX_DELAY);
}

void pause() {
  paused = true;
  const PlayState now = playState;
  if (now != PlayState::Idle && now != PlayState::Finished) playState = PlayState::Paused;
}

void resume() {
  paused = false;
  if (playState == PlayState::Paused) {
    playState = endReceived ? PlayState::Draining : (primed ? PlayState::Playing : PlayState::Buffering);
  }
}

void setVolume(uint8_t percent) {
  percent = percent > 100 ? 100 : percent;
  volumePercent = percent;
  // Square law: close enough to perceived loudness, and 100 % is unity gain.
  gainQ15 = static_cast<int32_t>(percent) * percent * 32768 / 10000;
}

int acceptingStream() { return accepting ? currentStream.load() : -1; }

bool pushData(const uint8_t* data, const size_t length) {
  if (xStreamBufferSpacesAvailable(rx) < length) return false;
  return xStreamBufferSend(rx, data, length, 0) == length;
}

size_t rxFree() { return accepting ? xStreamBufferSpacesAvailable(rx) : 0; }

void setOutputRate(const uint32_t rate) {
  const uint32_t old = outputRate.exchange(rate);
  if (old == rate || old == 0) return;
  playedMsBase += static_cast<uint32_t>(static_cast<uint64_t>(playedFrames.exchange(0)) * 1000 / old);
}

size_t readPcm(StereoFrame* out, const size_t frames) {
  if (paused || currentStream < 0) return 0;
  const PlayState now = playState;
  if (now == PlayState::Idle || now == PlayState::Finished) return 0;
  const bool isLive = live;
  if (!primed) {
    if (pcm.available() < config::PCM_PRIME_FRAMES) return 0;
    if (isLive && liveBufferedMs() < config::LIVE_PRIME_MS) return 0;
    primed = true;
  }
  const size_t got = pcm.read(out, frames);
  if (got < frames && !endReceived) {
    ++underruns;
    // Live: go quiet and build the cushion up again, rather than stutter through many small gaps.
    if (isLive) primed = false;
  }
  playedFrames += static_cast<uint32_t>(got);
  if (got > 0 && now == PlayState::Buffering) playState = PlayState::Playing;
  return got;
}

bool wantsOutput() {
  const PlayState now = playState;
  return !paused && (now == PlayState::Buffering || now == PlayState::Playing || now == PlayState::Draining);
}

void fillStatus(audiolink::StatusPayload& status) {
  const int stream = currentStream;
  const uint32_t rate = outputRate;
  status.streamId = stream < 0 ? 0 : static_cast<uint8_t>(stream);
  status.playState = static_cast<uint8_t>(stream < 0 ? PlayState::Idle : playState.load());
  status.volume = volumePercent;
  status.channels = sourceChannels;
  status.sampleRate = sourceRate;
  status.playedMs =
      playedMsBase + (rate ? static_cast<uint32_t>(static_cast<uint64_t>(playedFrames) * 1000 / rate) : 0);
  status.rxBuffered = static_cast<uint32_t>(config::RX_BUFFER_BYTES - xStreamBufferSpacesAvailable(rx));
  status.pcmBufferedMs = static_cast<uint16_t>(rate ? pcm.available() * 1000 / rate : 0);
  status.underruns = underruns;
  status.decodeErrors = live ? liveDrops.load() : decodeErrors.load();
  status.freeHeap = ESP.getFreeHeap();
}

audiolink::PlayState state() { return currentStream < 0 ? PlayState::Idle : playState.load(); }

}  // namespace player
