#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

template <typename T>
T clamp(T value, T minimum, T maximum) {
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

class PIDController {
public:
    PIDController(double kp, double ki, double kd, float integralMin, float integralMax)
        : kp_(kp), ki_(ki), kd_(kd),
          integralMin_(integralMin), integralMax_(integralMax),
          integral_(0), previousError_(0) {}

    // Compute PID output given current error and time delta
    float compute(float error, float dt) {
        float p = kp_ * error;

        integral_ += ki_ * error * dt;
        integral_ = clamp(integral_, integralMin_, integralMax_);

        float d = kd_ * (error - previousError_) / dt;

        previousError_ = error;
        return p + integral_ + d;
    }

    void reset() {
        integral_ = 0;
        previousError_ = 0;
    }

    // Accessors for testing/debugging
    float getP(float error) const { return kp_ * error; }
    float getIntegral() const { return integral_; }
    float getPreviousError() const { return previousError_; }

private:
    double kp_, ki_, kd_;
    float integralMin_, integralMax_;
    float integral_;
    float previousError_;
};

#endif
