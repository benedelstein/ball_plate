#pragma once

#include <math.h>

struct Setpoint {
  float x;
  float y;
};

enum TrajectoryMode {
  TRAJECTORY_CENTER       = 0,
  TRAJECTORY_CIRCLE       = 1,
  TRAJECTORY_FOUR_CORNERS = 2,
  TRAJECTORY_ELLIPSE      = 3,
};

// Generates time-varying (x, y) setpoints for the plate.
//
// update(nowMs) is meant to be called every loop iteration; each mode has its
// own internal cadence and only advances when enough time has elapsed.
//
// The static helpers (circle/ellipse/fourCorners) are pure functions and can
// be unit tested in isolation.
class TrajectoryGenerator {
public:
  TrajectoryGenerator()
      : mode_(TRAJECTORY_CENTER),
        circleRadius_(10.0f),
        ellipseA_(15.0f), ellipseB_(10.0f),
        cornerHalfWidth_(32.0f), cornerHalfHeight_(13.0f),
        pointsPerCycle_(150),
        radialVelocity_(1.0f),
        cornerDwellSeconds_(2.0f),
        index_(0),
        cornerIndex_(0),
        lastUpdateMs_(0),
        started_(false),
        current_{0.0f, 0.0f} {}

  void setMode(TrajectoryMode mode) { mode_ = mode; }
  TrajectoryMode mode() const { return mode_; }

  void setCircle(float radius)              { circleRadius_ = radius; }
  void setEllipse(float a, float b)         { ellipseA_ = a; ellipseB_ = b; }
  void setCorners(float halfW, float halfH) { cornerHalfWidth_ = halfW; cornerHalfHeight_ = halfH; }
  void setRadialVelocity(float hz)          { radialVelocity_ = hz; }
  void setPointsPerCycle(int n)             { pointsPerCycle_ = n; }
  void setCornerDwell(float seconds)        { cornerDwellSeconds_ = seconds; }

  // Mark `nowMs` as the trajectory start time. Call from setup() so the first
  // update() doesn't see a giant dt.
  void begin(unsigned long nowMs) {
    lastUpdateMs_ = nowMs;
    started_ = true;
    index_ = 0;
    cornerIndex_ = 0;
    current_ = {0.0f, 0.0f};
  }

  Setpoint update(unsigned long nowMs) {
    if (!started_) begin(nowMs);

    const float dt = (nowMs - lastUpdateMs_) / 1000.0f;
    const float pointPeriod = 1.0f / (radialVelocity_ * (float)pointsPerCycle_);

    switch (mode_) {
      case TRAJECTORY_CENTER:
        current_ = {0.0f, 0.0f};
        break;

      case TRAJECTORY_CIRCLE:
        if (dt >= pointPeriod) {
          current_ = circle(circleRadius_, index_, pointsPerCycle_);
          advanceIndex(dt, pointPeriod);
          lastUpdateMs_ = nowMs;
        }
        break;

      case TRAJECTORY_ELLIPSE:
        if (dt >= pointPeriod) {
          current_ = ellipse(ellipseA_, ellipseB_, index_, pointsPerCycle_);
          advanceIndex(dt, pointPeriod);
          lastUpdateMs_ = nowMs;
        }
        break;

      case TRAJECTORY_FOUR_CORNERS:
        if (dt >= cornerDwellSeconds_) {
          current_ = fourCorners(cornerHalfWidth_, cornerHalfHeight_, cornerIndex_);
          cornerIndex_ = (cornerIndex_ + 1) & 3;
          lastUpdateMs_ = nowMs;
        }
        break;
    }
    return current_;
  }

  // --- Pure trajectory math (testable without Arduino) ---

  static Setpoint circle(float radius, int i, int pointsPerCycle) {
    const float angle = (float)i / (float)pointsPerCycle * 2.0f * (float)M_PI;
    return { radius * cosf(angle), radius * sinf(angle) };
  }

  static Setpoint ellipse(float a, float b, int i, int pointsPerCycle) {
    const float angle = (float)i / (float)pointsPerCycle * 2.0f * (float)M_PI;
    return { a * cosf(angle), b * sinf(angle) };
  }

  // Walks the four quadrants counter-clockwise starting at +x/+y.
  static Setpoint fourCorners(float halfWidth, float halfHeight, int cornerIndex) {
    switch (cornerIndex & 3) {
      case 0:  return {  halfWidth,  halfHeight };
      case 1:  return { -halfWidth,  halfHeight };
      case 2:  return { -halfWidth, -halfHeight };
      default: return {  halfWidth, -halfHeight };
    }
  }

private:
  // If multiple point periods have elapsed since the last update, advance the
  // index by the corresponding integer number of steps so angular velocity
  // doesn't sag when the control loop is busy.
  void advanceIndex(float dt, float pointPeriod) {
    int step = (int)(dt / pointPeriod + 0.5f);
    if (step < 1) step = 1;
    index_ = (index_ + step) % pointsPerCycle_;
  }

  TrajectoryMode mode_;
  float circleRadius_;
  float ellipseA_, ellipseB_;
  float cornerHalfWidth_, cornerHalfHeight_;
  int pointsPerCycle_;
  float radialVelocity_;
  float cornerDwellSeconds_;
  int index_;
  int cornerIndex_;
  unsigned long lastUpdateMs_;
  bool started_;
  Setpoint current_;
};
