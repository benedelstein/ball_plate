#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// Minimal test framework
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do { \
    tests_run++; \
    if (expr) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL: %s (line %d)\n", #expr, __LINE__); } \
} while(0)

#define ASSERT_FLOAT_EQ(a, b, eps) do { \
    tests_run++; \
    if (fabs((a) - (b)) < (eps)) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL: %s == %s (got %.6f vs %.6f, line %d)\n", #a, #b, (double)(a), (double)(b), __LINE__); } \
} while(0)

#define ASSERT_INT_EQ(a, b) do { \
    tests_run++; \
    if ((a) == (b)) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL: %s == %s (got %d vs %d, line %d)\n", #a, #b, (int)(a), (int)(b), __LINE__); } \
} while(0)

#define TEST(name) void name(); \
    static struct Register_##name { Register_##name() { test_registry[test_count++] = {#name, name}; } } reg_##name; \
    void name()

struct TestEntry { const char* name; void (*fn)(); };
static TestEntry test_registry[100];
static int test_count = 0;

// Include the headers under test
#include "../PIDController.h"
#include "../MovingAverage.h"
#include "../Trajectory.h"

// ==================== clamp tests ====================

TEST(test_clamp_int_within_range) {
    ASSERT_INT_EQ(clamp(5, 0, 10), 5);
}

TEST(test_clamp_int_below_min) {
    ASSERT_INT_EQ(clamp(-5, 0, 10), 0);
}

TEST(test_clamp_int_above_max) {
    ASSERT_INT_EQ(clamp(15, 0, 10), 10);
}

TEST(test_clamp_int_at_boundaries) {
    ASSERT_INT_EQ(clamp(0, 0, 10), 0);
    ASSERT_INT_EQ(clamp(10, 0, 10), 10);
}

TEST(test_clamp_float_within_range) {
    ASSERT_FLOAT_EQ(clamp(5.0f, 0.0f, 10.0f), 5.0f, 0.001);
}

TEST(test_clamp_float_below_min) {
    ASSERT_FLOAT_EQ(clamp(-5.0f, -2.0f, 10.0f), -2.0f, 0.001);
}

TEST(test_clamp_float_above_max) {
    ASSERT_FLOAT_EQ(clamp(15.5f, 0.0f, 10.0f), 10.0f, 0.001);
}

TEST(test_clamp_negative_range) {
    ASSERT_INT_EQ(clamp(-3, -10, -1), -3);
    ASSERT_INT_EQ(clamp(5, -10, -1), -1);
}

// ==================== PIDController tests ====================

TEST(test_pid_proportional_only) {
    PIDController pid(1.0, 0.0, 0.0, -100, 100);
    float out = pid.compute(10.0, 0.1);
    ASSERT_FLOAT_EQ(out, 10.0, 0.001);
}

TEST(test_pid_proportional_negative_error) {
    PIDController pid(2.0, 0.0, 0.0, -100, 100);
    float out = pid.compute(-5.0, 0.1);
    ASSERT_FLOAT_EQ(out, -10.0, 0.001);
}

TEST(test_pid_integral_accumulates) {
    PIDController pid(0.0, 1.0, 0.0, -100, 100);
    pid.compute(10.0, 1.0);  // integral = 10
    float out = pid.compute(10.0, 1.0);  // integral = 20
    ASSERT_FLOAT_EQ(out, 20.0, 0.001);
    ASSERT_FLOAT_EQ(pid.getIntegral(), 20.0, 0.001);
}

TEST(test_pid_integral_clamp) {
    PIDController pid(0.0, 1.0, 0.0, -5, 5);
    pid.compute(100.0, 1.0);  // would be 100, but clamped to 5
    ASSERT_FLOAT_EQ(pid.getIntegral(), 5.0, 0.001);
}

TEST(test_pid_integral_clamp_negative) {
    PIDController pid(0.0, 1.0, 0.0, -5, 5);
    pid.compute(-100.0, 1.0);
    ASSERT_FLOAT_EQ(pid.getIntegral(), -5.0, 0.001);
}

TEST(test_pid_derivative) {
    PIDController pid(0.0, 0.0, 1.0, -100, 100);
    pid.compute(0.0, 1.0);   // previous error = 0
    float out = pid.compute(10.0, 1.0);  // d = (10-0)/1 = 10
    ASSERT_FLOAT_EQ(out, 10.0, 0.001);
}

TEST(test_pid_derivative_decreasing_error) {
    PIDController pid(0.0, 0.0, 1.0, -100, 100);
    pid.compute(10.0, 1.0);
    float out = pid.compute(5.0, 1.0);  // d = (5-10)/1 = -5
    ASSERT_FLOAT_EQ(out, -5.0, 0.001);
}

TEST(test_pid_full_output) {
    PIDController pid(0.5, 0.1, 0.2, -100, 100);
    // First call: error=10, dt=0.5
    // P = 0.5*10 = 5, I = 0.1*10*0.5 = 0.5, D = 0.2*(10-0)/0.5 = 4
    float out = pid.compute(10.0, 0.5);
    ASSERT_FLOAT_EQ(out, 5.0 + 0.5 + 4.0, 0.01);
}

TEST(test_pid_reset) {
    PIDController pid(1.0, 1.0, 1.0, -100, 100);
    pid.compute(10.0, 1.0);
    pid.reset();
    ASSERT_FLOAT_EQ(pid.getIntegral(), 0.0, 0.001);
    ASSERT_FLOAT_EQ(pid.getPreviousError(), 0.0, 0.001);
}

TEST(test_pid_zero_error) {
    PIDController pid(1.0, 1.0, 1.0, -100, 100);
    float out = pid.compute(0.0, 1.0);
    ASSERT_FLOAT_EQ(out, 0.0, 0.001);
}

TEST(test_pid_small_dt) {
    PIDController pid(1.0, 0.5, 0.1, -100, 100);
    // dt=0.01, error=20
    // P=20, I=0.5*20*0.01=0.1, D=0.1*(20-0)/0.01=200
    float out = pid.compute(20.0, 0.01);
    ASSERT_FLOAT_EQ(out, 20.0 + 0.1 + 200.0, 0.01);
}

// ==================== MovingAverage tests ====================

TEST(test_moving_average_single_value) {
    MovingAverage ma;
    ma.init(3);
    float result = ma.update(9.0);
    // buffer: [0, 0, 9], sum=9, avg=3
    ASSERT_FLOAT_EQ(result, 3.0, 0.001);
}

TEST(test_moving_average_fills_window) {
    MovingAverage ma;
    ma.init(3);
    ma.update(3.0);  // [0,0,3] avg=1
    ma.update(6.0);  // [0,3,6] avg=3
    float result = ma.update(9.0);  // [3,6,9] avg=6
    ASSERT_FLOAT_EQ(result, 6.0, 0.001);
}

TEST(test_moving_average_steady_state) {
    MovingAverage ma;
    ma.init(3);
    for (int i = 0; i < 10; i++) {
        ma.update(5.0);
    }
    ASSERT_FLOAT_EQ(ma.getValue(), 5.0, 0.001);
}

TEST(test_moving_average_window_sliding) {
    MovingAverage ma;
    ma.init(3);
    ma.update(1.0);  // [0,0,1]
    ma.update(2.0);  // [0,1,2]
    ma.update(3.0);  // [1,2,3]
    float result = ma.update(4.0);  // [2,3,4] avg=3
    ASSERT_FLOAT_EQ(result, 3.0, 0.001);
}

TEST(test_moving_average_reset) {
    MovingAverage ma;
    ma.init(3);
    ma.update(10.0);
    ma.update(20.0);
    ma.reset();
    ASSERT_FLOAT_EQ(ma.getValue(), 0.0, 0.001);
    float result = ma.update(6.0);
    ASSERT_FLOAT_EQ(result, 2.0, 0.001);  // [0,0,6] avg=2
}

TEST(test_moving_average_negative_values) {
    MovingAverage ma;
    ma.init(2);
    ma.update(-4.0);  // [0,-4] avg=-2
    float result = ma.update(-6.0);  // [-4,-6] avg=-5
    ASSERT_FLOAT_EQ(result, -5.0, 0.001);
}

TEST(test_moving_average_window_size_1) {
    MovingAverage ma;
    ma.init(1);
    ASSERT_FLOAT_EQ(ma.update(42.0), 42.0, 0.001);
    ASSERT_FLOAT_EQ(ma.update(99.0), 99.0, 0.001);
}

// ==================== Trajectory tests ====================

TEST(test_trajectory_center) {
    Point2D p = Trajectory::center();
    ASSERT_FLOAT_EQ(p.x, 0.0, 0.001);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_circle_at_zero) {
    // index=0 -> angle=0 -> cos(0)=1, sin(0)=0
    Point2D p = Trajectory::circle(10.0, 0, 100);
    ASSERT_FLOAT_EQ(p.x, 10.0, 0.001);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_circle_quarter) {
    // index=25, points=100 -> angle=pi/2 -> cos=0, sin=1
    Point2D p = Trajectory::circle(10.0, 25, 100);
    ASSERT_FLOAT_EQ(p.x, 0.0, 0.01);
    ASSERT_FLOAT_EQ(p.y, 10.0, 0.01);
}

TEST(test_trajectory_circle_half) {
    // index=50, points=100 -> angle=pi -> cos=-1, sin=0
    Point2D p = Trajectory::circle(10.0, 50, 100);
    ASSERT_FLOAT_EQ(p.x, -10.0, 0.01);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.01);
}

TEST(test_trajectory_circle_radius) {
    // All points on a circle should have distance == radius from origin
    for (int i = 0; i < 100; i++) {
        Point2D p = Trajectory::circle(15.0, i, 100);
        float dist = sqrt(p.x * p.x + p.y * p.y);
        ASSERT_FLOAT_EQ(dist, 15.0, 0.01);
    }
}

TEST(test_trajectory_ellipse_at_zero) {
    Point2D p = Trajectory::ellipse(20.0, 10.0, 0, 100);
    ASSERT_FLOAT_EQ(p.x, 20.0, 0.001);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_ellipse_quarter) {
    Point2D p = Trajectory::ellipse(20.0, 10.0, 25, 100);
    ASSERT_FLOAT_EQ(p.x, 0.0, 0.01);
    ASSERT_FLOAT_EQ(p.y, 10.0, 0.01);
}

TEST(test_trajectory_ellipse_on_curve) {
    // Verify points satisfy ellipse equation: (x/a)^2 + (y/b)^2 = 1
    float a = 20.0, b = 10.0;
    for (int i = 0; i < 100; i++) {
        Point2D p = Trajectory::ellipse(a, b, i, 100);
        float eq = (p.x / a) * (p.x / a) + (p.y / b) * (p.y / b);
        ASSERT_FLOAT_EQ(eq, 1.0, 0.01);
    }
}

TEST(test_trajectory_line_start) {
    Point2D p = Trajectory::line(20.0, 0, 100);
    ASSERT_FLOAT_EQ(p.x, 0.0, 0.001);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_line_midpoint) {
    // At halfway through first half, should be at half of half-length
    Point2D p = Trajectory::line(20.0, 25, 100);
    ASSERT_FLOAT_EQ(p.x, 5.0, 0.01);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_line_y_always_zero) {
    for (int i = 0; i < 100; i++) {
        Point2D p = Trajectory::line(30.0, i, 100);
        ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
    }
}

TEST(test_trajectory_four_corners_cycle) {
    Point2D p0 = Trajectory::fourCorners(10.0, 5.0, 0);
    Point2D p1 = Trajectory::fourCorners(10.0, 5.0, 1);
    Point2D p2 = Trajectory::fourCorners(10.0, 5.0, 2);
    Point2D p3 = Trajectory::fourCorners(10.0, 5.0, 3);

    ASSERT_FLOAT_EQ(p0.x,  10.0, 0.001); ASSERT_FLOAT_EQ(p0.y,  5.0, 0.001);
    ASSERT_FLOAT_EQ(p1.x, -10.0, 0.001); ASSERT_FLOAT_EQ(p1.y,  5.0, 0.001);
    ASSERT_FLOAT_EQ(p2.x, -10.0, 0.001); ASSERT_FLOAT_EQ(p2.y, -5.0, 0.001);
    ASSERT_FLOAT_EQ(p3.x,  10.0, 0.001); ASSERT_FLOAT_EQ(p3.y, -5.0, 0.001);
}

TEST(test_trajectory_four_corners_wraps) {
    Point2D p4 = Trajectory::fourCorners(10.0, 5.0, 4);
    Point2D p0 = Trajectory::fourCorners(10.0, 5.0, 0);
    ASSERT_FLOAT_EQ(p4.x, p0.x, 0.001);
    ASSERT_FLOAT_EQ(p4.y, p0.y, 0.001);
}

// ==================== TrajectoryController tests ====================

TEST(test_trajectory_controller_center_mode) {
    TrajectoryController tc(100, 1.0);
    tc.setMode(TrajectoryController::MODE_CENTER);
    tc.setLastUpdateTime(0);
    Point2D p = tc.update(1000);
    ASSERT_FLOAT_EQ(p.x, 0.0, 0.001);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.001);
}

TEST(test_trajectory_controller_mode_change) {
    TrajectoryController tc(100, 1.0);
    tc.setMode(TrajectoryController::MODE_CENTER);
    ASSERT_TRUE(tc.getMode() == TrajectoryController::MODE_CENTER);
    tc.setMode(TrajectoryController::MODE_CIRCLE);
    ASSERT_TRUE(tc.getMode() == TrajectoryController::MODE_CIRCLE);
}

TEST(test_trajectory_controller_circle_starts_at_radius) {
    TrajectoryController tc(100, 1.0);
    tc.setMode(TrajectoryController::MODE_CIRCLE);
    tc.setLastUpdateTime(0);
    // At t=0, no time has passed, index stays at 0 -> circle(10, 0, 100) = (10, 0)
    Point2D p = tc.update(0);
    ASSERT_FLOAT_EQ(p.x, 10.0, 0.01);
    ASSERT_FLOAT_EQ(p.y, 0.0, 0.01);
}

// ==================== Integration-style tests ====================

TEST(test_pid_converges_toward_zero_error) {
    // Simulate a simple 1D system where position += pidOutput * dt
    PIDController pid(0.5, 0.01, 0.1, -50, 50);
    float position = 50.0;  // start at 50
    float setpoint = 0.0;
    float dt = 0.05;

    for (int i = 0; i < 200; i++) {
        float error = setpoint - position;
        float output = pid.compute(error, dt);
        position += output * dt;  // simplified plant model
    }

    // Should be near setpoint after 200 steps
    ASSERT_TRUE(fabs(position - setpoint) < 5.0);
}

TEST(test_moving_average_smooths_noise) {
    MovingAverage ma;
    ma.init(5);
    // Alternating values around 10
    float noisy[] = {8, 12, 9, 11, 10, 8, 12, 9, 11, 10};
    float lastVal = 0;
    for (int i = 0; i < 10; i++) {
        lastVal = ma.update(noisy[i]);
    }
    // After filling with values around 10, output should be close to 10
    ASSERT_FLOAT_EQ(lastVal, 10.0, 0.5);
}

// ==================== Main ====================

int main() {
    printf("Running %d tests...\n\n", test_count);

    for (int i = 0; i < test_count; i++) {
        printf("  [%s]\n", test_registry[i].name);
        test_registry[i].fn();
    }

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed, %d total\n", tests_passed, tests_failed, tests_run);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
