// Host-compilable unit tests for the ball_plate control modules.
//
// Build & run:
//   cd test && make
//
// These tests intentionally avoid any Arduino dependency so they can be run
// on a workstation without flashing hardware.

#include <cmath>
#include <cstdio>

#include "../Clip.h"
#include "../MovingAverage.h"
#include "../PidController.h"
#include "../Trajectory.h"

namespace {

int g_checks   = 0;
int g_failures = 0;

#define CHECK(cond)                                                  \
  do {                                                               \
    ++g_checks;                                                      \
    if (!(cond)) {                                                   \
      ++g_failures;                                                  \
      std::fprintf(stderr, "FAIL %s:%d  %s\n",                       \
                   __FILE__, __LINE__, #cond);                       \
    }                                                                \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                        \
  do {                                                               \
    ++g_checks;                                                      \
    const double _a = (double)(a);                                   \
    const double _b = (double)(b);                                   \
    if (std::fabs(_a - _b) > (double)(tol)) {                        \
      ++g_failures;                                                  \
      std::fprintf(stderr, "FAIL %s:%d  |%g - %g| > %g\n",           \
                   __FILE__, __LINE__, _a, _b, (double)(tol));       \
    }                                                                \
  } while (0)

// ------------------ Clip ------------------

void testClipInt() {
  CHECK(clip(5, 0, 10) == 5);
  CHECK(clip(-1, 0, 10) == 0);
  CHECK(clip(11, 0, 10) == 10);
  CHECK(clip(0, 0, 10) == 0);
  CHECK(clip(10, 0, 10) == 10);
}

void testClipFloat() {
  CHECK_NEAR(clip(0.5f, 0.0f, 1.0f), 0.5f, 1e-6f);
  CHECK_NEAR(clip(-0.5f, 0.0f, 1.0f), 0.0f, 1e-6f);
  CHECK_NEAR(clip(1.5f, 0.0f, 1.0f), 1.0f, 1e-6f);
}

// ------------------ MovingAverage ------------------

void testMovingAverageWarmup() {
  // During warmup, mean is over samples-added-so-far, not over the window.
  MovingAverage<3> ma;
  CHECK_NEAR(ma.add(3.0f), 3.0f, 1e-6f);
  CHECK_NEAR(ma.add(6.0f), 4.5f, 1e-6f);
  CHECK_NEAR(ma.add(9.0f), 6.0f, 1e-6f);
}

void testMovingAverageEviction() {
  MovingAverage<3> ma;
  ma.add(3.0f);
  ma.add(6.0f);
  ma.add(9.0f);
  // Window full: next add evicts 3, mean = (6+9+12)/3
  CHECK_NEAR(ma.add(12.0f), 9.0f, 1e-6f);
  // Evict 6, mean = (9+12+15)/3
  CHECK_NEAR(ma.add(15.0f), 12.0f, 1e-6f);
}

void testMovingAverageReset() {
  MovingAverage<3> ma;
  ma.add(100.0f);
  ma.add(200.0f);
  ma.reset();
  CHECK(ma.count() == 0);
  CHECK_NEAR(ma.add(1.0f), 1.0f, 1e-6f);
}

void testMovingAverageNumericStability() {
  // A long run of identical values shouldn't drift due to incremental sum.
  MovingAverage<5> ma;
  for (int i = 0; i < 10000; ++i) ma.add(7.5f);
  CHECK_NEAR(ma.add(7.5f), 7.5f, 1e-3f);
}

// ------------------ PidController ------------------

void testPidProportionalOnly() {
  PidController pid(2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
  // error = 10 - 4 = 6, P = 2*6 = 12
  CHECK_NEAR(pid.update(10.0f, 4.0f, 0.1f), 12.0f, 1e-6f);
}

void testPidIntegralAccumulates() {
  PidController pid(0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
  pid.update(10.0f, 0.0f, 1.0f);
  CHECK_NEAR(pid.integral(), 10.0f, 1e-6f);
  pid.update(10.0f, 0.0f, 1.0f);
  CHECK_NEAR(pid.integral(), 20.0f, 1e-6f);
}

void testPidIntegralClipped() {
  PidController pid(0.0f, 1.0f, 0.0f, -5.0f, 5.0f);
  pid.update(100.0f, 0.0f, 1.0f); // 100 -> clipped to 5
  CHECK_NEAR(pid.integral(), 5.0f, 1e-6f);
  pid.update(-100.0f, 0.0f, 1.0f); // -100 -> clipped to -5
  CHECK_NEAR(pid.integral(), -5.0f, 1e-6f);
}

void testPidDerivativeSkipsFirstStep() {
  PidController pid(0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
  // First call: no previous error, derivative term must be 0.
  CHECK_NEAR(pid.update(10.0f, 0.0f, 0.1f), 0.0f, 1e-6f);
  // Error went from 10 to 5 over 0.1s -> derivative = -50
  CHECK_NEAR(pid.update(10.0f, 5.0f, 0.1f), -50.0f, 1e-5f);
}

void testPidZeroDt() {
  // dt = 0 must not divide-by-zero in the derivative term.
  PidController pid(1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
  pid.update(10.0f, 0.0f, 0.1f);
  const float out = pid.update(10.0f, 5.0f, 0.0f);
  CHECK(std::isfinite(out));
}

void testPidReset() {
  PidController pid(1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
  pid.update(10.0f, 0.0f, 1.0f);
  pid.update(10.0f, 0.0f, 1.0f);
  CHECK(pid.integral() != 0.0f);
  pid.reset();
  CHECK_NEAR(pid.integral(), 0.0f, 1e-6f);
  // After reset, derivative is 0 again on next call.
  // P = 10, I += 10 -> I = 10, D = 0 -> output = 20
  CHECK_NEAR(pid.update(10.0f, 0.0f, 1.0f), 20.0f, 1e-6f);
}

// ------------------ Trajectory ------------------

void testTrajectoryCircle() {
  Setpoint s0 = TrajectoryGenerator::circle(10.0f, 0, 100);
  CHECK_NEAR(s0.x, 10.0f, 1e-4f);
  CHECK_NEAR(s0.y,  0.0f, 1e-4f);
  Setpoint sq = TrajectoryGenerator::circle(10.0f, 25, 100);
  CHECK_NEAR(sq.x, 0.0f, 1e-4f);
  CHECK_NEAR(sq.y, 10.0f, 1e-4f);
  Setpoint sh = TrajectoryGenerator::circle(10.0f, 50, 100);
  CHECK_NEAR(sh.x, -10.0f, 1e-4f);
  CHECK_NEAR(sh.y,   0.0f, 1e-4f);
}

void testTrajectoryEllipseDegeneratesToCircle() {
  Setpoint c = TrajectoryGenerator::circle(7.0f, 13, 60);
  Setpoint e = TrajectoryGenerator::ellipse(7.0f, 7.0f, 13, 60);
  CHECK_NEAR(c.x, e.x, 1e-6f);
  CHECK_NEAR(c.y, e.y, 1e-6f);
}

void testTrajectoryFourCorners() {
  Setpoint c0 = TrajectoryGenerator::fourCorners(30.0f, 10.0f, 0);
  CHECK_NEAR(c0.x,  30.0f, 1e-6f); CHECK_NEAR(c0.y,  10.0f, 1e-6f);
  Setpoint c1 = TrajectoryGenerator::fourCorners(30.0f, 10.0f, 1);
  CHECK_NEAR(c1.x, -30.0f, 1e-6f); CHECK_NEAR(c1.y,  10.0f, 1e-6f);
  Setpoint c2 = TrajectoryGenerator::fourCorners(30.0f, 10.0f, 2);
  CHECK_NEAR(c2.x, -30.0f, 1e-6f); CHECK_NEAR(c2.y, -10.0f, 1e-6f);
  Setpoint c3 = TrajectoryGenerator::fourCorners(30.0f, 10.0f, 3);
  CHECK_NEAR(c3.x,  30.0f, 1e-6f); CHECK_NEAR(c3.y, -10.0f, 1e-6f);
  // Wraps modulo 4.
  Setpoint c4 = TrajectoryGenerator::fourCorners(30.0f, 10.0f, 4);
  CHECK_NEAR(c4.x, c0.x, 1e-6f); CHECK_NEAR(c4.y, c0.y, 1e-6f);
}

void testTrajectoryCenterMode() {
  TrajectoryGenerator t;
  t.setMode(TRAJECTORY_CENTER);
  t.begin(1000);
  Setpoint s = t.update(2000);
  CHECK_NEAR(s.x, 0.0f, 1e-6f);
  CHECK_NEAR(s.y, 0.0f, 1e-6f);
}

void testTrajectoryFourCornersAdvancesOnDwell() {
  TrajectoryGenerator t;
  t.setMode(TRAJECTORY_FOUR_CORNERS);
  t.setCorners(20.0f, 5.0f);
  t.setCornerDwell(2.0f);
  t.begin(0);

  // Before dwell elapses: still at initial (0, 0).
  Setpoint s0 = t.update(1000);
  CHECK_NEAR(s0.x, 0.0f, 1e-6f);
  CHECK_NEAR(s0.y, 0.0f, 1e-6f);

  // After dwell: first corner.
  Setpoint s1 = t.update(3000);
  CHECK_NEAR(s1.x,  20.0f, 1e-6f);
  CHECK_NEAR(s1.y,   5.0f, 1e-6f);

  // After another dwell: second corner.
  Setpoint s2 = t.update(5500);
  CHECK_NEAR(s2.x, -20.0f, 1e-6f);
  CHECK_NEAR(s2.y,   5.0f, 1e-6f);
}

}  // namespace

int main() {
  testClipInt();
  testClipFloat();
  testMovingAverageWarmup();
  testMovingAverageEviction();
  testMovingAverageReset();
  testMovingAverageNumericStability();
  testPidProportionalOnly();
  testPidIntegralAccumulates();
  testPidIntegralClipped();
  testPidDerivativeSkipsFirstStep();
  testPidZeroDt();
  testPidReset();
  testTrajectoryCircle();
  testTrajectoryEllipseDegeneratesToCircle();
  testTrajectoryFourCorners();
  testTrajectoryCenterMode();
  testTrajectoryFourCornersAdvancesOnDwell();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
