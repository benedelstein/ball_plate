#pragma once

// Clamp `value` into the closed interval [lo, hi].
template <typename T>
inline T clip(T value, T lo, T hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}
