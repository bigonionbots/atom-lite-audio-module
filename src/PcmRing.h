#pragma once

/**
 * @file PcmRing.h
 * @brief Interleaved 16-bit stereo frame ring shared by the decoder (writer) and an output (reader).
 *
 * Index updates and copies happen under a spinlock, so either side may also clear() it. Copies are
 * at most a few hundred frames, which keeps the critical sections in the microseconds.
 */

#include <Arduino.h>
#include <esp_heap_caps.h>

#include <algorithm>
#include <cstdint>
#include <cstring>

struct StereoFrame {
  int16_t left;
  int16_t right;
};

class PcmRing {
 public:
  bool begin(const size_t frames) {
    buffer_ = static_cast<StereoFrame*>(heap_caps_malloc(frames * sizeof(StereoFrame), MALLOC_CAP_8BIT));
    capacity_ = buffer_ ? frames : 0;
    return buffer_ != nullptr;
  }

  size_t write(const StereoFrame* in, size_t frames) {
    portENTER_CRITICAL(&mux_);
    frames = std::min(frames, capacity_ - count_);
    for (size_t done = 0; done < frames;) {
      const size_t run = std::min(frames - done, capacity_ - head_);
      std::memcpy(buffer_ + head_, in + done, run * sizeof(StereoFrame));
      head_ = (head_ + run) % capacity_;
      done += run;
    }
    count_ += frames;
    portEXIT_CRITICAL(&mux_);
    return frames;
  }

  size_t read(StereoFrame* out, size_t frames) {
    portENTER_CRITICAL(&mux_);
    frames = std::min(frames, count_);
    for (size_t done = 0; done < frames;) {
      const size_t run = std::min(frames - done, capacity_ - tail_);
      std::memcpy(out + done, buffer_ + tail_, run * sizeof(StereoFrame));
      tail_ = (tail_ + run) % capacity_;
      done += run;
    }
    count_ -= frames;
    portEXIT_CRITICAL(&mux_);
    return frames;
  }

  void clear() {
    portENTER_CRITICAL(&mux_);
    head_ = tail_ = count_ = 0;
    portEXIT_CRITICAL(&mux_);
  }

  size_t available() const { return count_; }
  size_t space() const { return capacity_ - count_; }
  size_t capacity() const { return capacity_; }

 private:
  StereoFrame* buffer_ = nullptr;
  size_t capacity_ = 0;
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t count_ = 0;  // indices change only inside the critical sections
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
