#pragma once

#include "Clip.h"

// Discrete-time PID controller with clipped integral and a first-step guard
// on the derivative term (so the first update() can't generate a spurious
// derivative kick from an uninitialized previous error).
class PidController {
public:
  PidController(float kp, float ki, float kd,
                float integralMin, float integralMax)
      : kp_(kp), ki_(ki), kd_(kd),
        integralMin_(integralMin), integralMax_(integralMax) {
    reset();
  }

  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    hasPrevious_ = false;
  }

  // Compute control output for one timestep.
  //   setpoint   — target value
  //   measurement — current measured value
  //   dt         — elapsed seconds since the previous update
  float update(float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;

    const float p = kp_ * error;

    integral_ = clip(integral_ + ki_ * error * dt, integralMin_, integralMax_);

    float d = 0.0f;
    if (hasPrevious_ && dt > 0.0f) {
      d = kd_ * (error - previousError_) / dt;
    }

    previousError_ = error;
    hasPrevious_ = true;

    return p + integral_ + d;
  }

  // Accessors useful for tests and diagnostics.
  float integral() const { return integral_; }
  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

private:
  float kp_, ki_, kd_;
  float integralMin_, integralMax_;
  float integral_;
  float previousError_;
  bool hasPrevious_;
};
