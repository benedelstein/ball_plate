#pragma once

// Fixed-window moving average over the last N samples.
// Uses an incremental sum so add() is O(1) regardless of window size.
template <int N>
class MovingAverage {
public:
  MovingAverage() { reset(); }

  // Push `value` into the window and return the current mean.
  // During warmup (fewer than N samples added), the mean is taken over the
  // samples added so far rather than dividing by the full window — this avoids
  // artificially damping early readings toward zero.
  float add(float value) {
    sum_ -= buffer_[head_];
    buffer_[head_] = value;
    sum_ += value;
    head_ = (head_ + 1) % N;
    if (count_ < N) ++count_;
    return sum_ / (float)count_;
  }

  void reset() {
    sum_ = 0.0f;
    head_ = 0;
    count_ = 0;
    for (int i = 0; i < N; ++i) buffer_[i] = 0.0f;
  }

  int count() const { return count_; }

private:
  float buffer_[N];
  float sum_;
  int head_;
  int count_;
};
