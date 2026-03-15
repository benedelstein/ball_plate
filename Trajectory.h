#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point2D {
    float x;
    float y;
};

// All trajectory functions are pure: they return a setpoint given parameters.
// No global state mutation.

namespace Trajectory {

    inline Point2D center() {
        return {0, 0};
    }

    inline Point2D circle(float radius, int index, int pointsPerCycle) {
        float angle = float(index) / pointsPerCycle * M_PI * 2;
        return {radius * (float)cos(angle), radius * (float)sin(angle)};
    }

    inline Point2D ellipse(float a, float b, int index, int pointsPerCycle) {
        float angle = float(index) / pointsPerCycle * M_PI * 2;
        return {a * (float)cos(angle), b * (float)sin(angle)};
    }

    inline Point2D line(float length, int index, int pointsPerCycle) {
        float x;
        if (index < pointsPerCycle / 2) {
            x = float(index) / (pointsPerCycle / 2) * (length / 2);
        } else {
            x = (1.0f - float(index - pointsPerCycle / 2) / (pointsPerCycle / 2)) * (length / 2);
        }
        return {x, 0};
    }

    inline Point2D fourCorners(float w, float h, int cornerIndex) {
        // cornerIndex: 0-3
        switch (cornerIndex % 4) {
            case 0: return { w,  h};
            case 1: return {-w,  h};
            case 2: return {-w, -h};
            case 3: return { w, -h};
        }
        return {0, 0};
    }

} // namespace Trajectory

// Manages trajectory state (index, timing, corner tracking)
class TrajectoryController {
public:
    enum Mode {
        MODE_CENTER = 0,
        MODE_CIRCLE = 1,
        MODE_FOUR_CORNERS = 2,
        MODE_ELLIPSE = 3
    };

    TrajectoryController(int pointsPerCycle, float radialVelocity)
        : pointsPerCycle_(pointsPerCycle), radialVelocity_(radialVelocity),
          index_(0), cornerIndex_(0), lastUpdateTime_(0), mode_(MODE_FOUR_CORNERS) {}

    void setMode(Mode mode) { mode_ = mode; }
    Mode getMode() const { return mode_; }

    void setLastUpdateTime(float t) { lastUpdateTime_ = t; }

    // Returns updated setpoint. Call each loop iteration with current time.
    Point2D update(float currentTime) {
        float dt = (currentTime - lastUpdateTime_) / 1000.0f;
        float updateIncrement = 1.0f / radialVelocity_ / pointsPerCycle_;

        switch (mode_) {
            case MODE_CENTER:
                return Trajectory::center();

            case MODE_CIRCLE:
                if (dt > updateIncrement) {
                    lastUpdateTime_ = currentTime;
                    index_ += int(round(dt / updateIncrement));
                    if (index_ > pointsPerCycle_) index_ = 0;
                }
                return Trajectory::circle(10, index_, pointsPerCycle_);

            case MODE_FOUR_CORNERS:
                if (dt > 2.0f) {
                    lastUpdateTime_ = currentTime;
                    cornerIndex_ = (cornerIndex_ + 1) % 4;
                }
                return Trajectory::fourCorners(32, 13, cornerIndex_);

            case MODE_ELLIPSE:
                if (dt > updateIncrement) {
                    lastUpdateTime_ = currentTime;
                    index_ += int(round(dt / updateIncrement));
                    if (index_ > pointsPerCycle_) index_ = 0;
                }
                return Trajectory::ellipse(15, 10, index_, pointsPerCycle_);

            default:
                return Trajectory::center();
        }
    }

private:
    int pointsPerCycle_;
    float radialVelocity_;
    int index_;
    int cornerIndex_;
    float lastUpdateTime_;
    Mode mode_;
};

#endif
