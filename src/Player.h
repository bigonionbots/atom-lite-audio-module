#pragma once

/**
 * @file Player.h
 * @brief Compressed-audio buffer, decoder task and PCM hand-off to the active output.
 *
 * Data path: Link (AUDIO_DATA) -> rx stream buffer -> decoder task (MP3 / AAC-ADTS via libhelix,
 * or µ-law for live game sound) -> resample to the output rate, mono to stereo, volume
 * -> PCM ring -> output (Bluetooth A2DP, I2S or the null sink) pulling with readPcm().
 *
 * Live (µ-law) streams are kept near real time: when more than LIVE_MAX_BUFFERED_MS of audio is
 * waiting, the oldest is dropped instead of played late.
 *
 * Threads: the link task calls the stream/control functions, the decoder task owns the decoders,
 * outputs call readPcm() from their own context (the A2DP callback runs in the Bluetooth task).
 */

#include <AudioLinkProtocol.h>

#include <cstddef>
#include <cstdint>

#include "PcmRing.h"

namespace player {

bool begin();

// --- Link task ---
/** sampleRate and channels: for Codec::MuLaw (live) streams; 0 for compressed ones. */
void streamStart(uint8_t streamId, audiolink::Codec codec, uint32_t startMs, uint32_t sampleRate, uint8_t channels);
void streamEnd(uint8_t streamId);
void streamStop(uint8_t streamId);
void pause();
void resume();
void setVolume(uint8_t percent);

/** The stream id that AUDIO_DATA must carry, or -1 when no stream is accepting data. */
int acceptingStream();
/** Appends compressed bytes; false (and nothing written) when they do not fit. */
bool pushData(const uint8_t* data, size_t length);
size_t rxFree();

// --- Outputs ---
/** The sample rate the active output runs at; the decoder resamples to it. */
void setOutputRate(uint32_t rate);
/** Pulls up to `frames` frames; returns fewer (possibly 0) when paused, priming or starved. */
size_t readPcm(StereoFrame* out, size_t frames);
/** True while a stream is playing out (not idle, paused or finished). */
bool wantsOutput();

// --- Status ---
void fillStatus(audiolink::StatusPayload& status);
audiolink::PlayState state();

}  // namespace player
