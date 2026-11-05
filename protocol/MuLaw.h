#pragma once

/**
 * @file MuLaw.h
 * @brief G.711 µ-law, the codec of live audio on the module link (Codec::MuLaw, game sound).
 *
 * Shared verbatim by both firmwares, like AudioLinkProtocol.h, so keep it dependency-free C++.
 *
 * One byte per 16-bit sample, with no state, so any byte boundary is a sample boundary and either
 * end may drop bytes (to keep a live stream's latency down) without disturbing the rest. Chosen
 * over IMA ADPCM, which halves the size again but smears the hard edges of Game Boy square waves:
 * on two square waves at 22050 Hz, ADPCM measured 14 dB SNR, µ-law 37.6 dB
 * (test/gb_host/mulaw_test.cpp checks the µ-law figure).
 */

#include <cstdint>

namespace mulaw {

/** One 16-bit sample to a µ-law byte (G.711: bias 0x84, clipped at 32635, bits inverted). */
inline uint8_t encode(const int16_t sample) {
  constexpr int BIAS = 0x84;
  constexpr int CLIP = 32635;
  int magnitude = sample;
  const int sign = magnitude < 0 ? 0x80 : 0;
  if (sign) magnitude = -magnitude;
  if (magnitude > CLIP) magnitude = CLIP;
  magnitude += BIAS;
  int exponent = 7;
  for (int mask = 0x4000; (magnitude & mask) == 0 && exponent > 0; mask >>= 1) --exponent;
  const int mantissa = (magnitude >> (exponent + 3)) & 0x0F;
  return static_cast<uint8_t>(~(sign | (exponent << 4) | mantissa));
}

/** A µ-law byte back to a 16-bit sample. */
inline int16_t decode(const uint8_t byte) {
  constexpr int BIAS = 0x84;
  const uint8_t code = static_cast<uint8_t>(~byte);
  const int exponent = (code >> 4) & 0x07;
  const int mantissa = code & 0x0F;
  const int magnitude = (((mantissa << 3) + BIAS) << exponent) - BIAS;
  return static_cast<int16_t>((code & 0x80) ? -magnitude : magnitude);
}

}  // namespace mulaw
