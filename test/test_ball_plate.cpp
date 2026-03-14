// Mocks must come before the source so Arduino symbols are available.
#include "mocks/Arduino.h"
#include "mocks/Servo.h"
#include "mocks/TouchScreen.h"

// Forward declarations the Arduino IDE normally auto-generates.
void updateSetpoint();
void circle(float radius, int i);
void ellipse(float a, float b, int i);
void line(float length, int i);
void fourCorners(float l);
int   clip (int   value, int   minimum, int   maximum);
float clip2(float value, float minimum, float maximum);

// Pull in the firmware as a header (all functions + globals become available).
#include "../ball_plate.ino"

#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------
static int g_run = 0, g_pass = 0, g_fail = 0;

static void _assert(bool ok, const char* msg, const char* file, int line) {
    ++g_run;
    if (ok) { ++g_pass; printf("  PASS  %s\n", msg); }
    else     { ++g_fail; printf("  FAIL  %s  (%s:%d)\n", msg, file, line); }
}

#define ASSERT(cond, msg)           _assert((cond), (msg), __FILE__, __LINE__)
#define ASSERT_EQ(a, b, msg)        ASSERT((a) == (b), (msg))
#define ASSERT_NEAR(a, b, eps, msg) ASSERT(std::fabs((double)(a)-(double)(b)) < (eps), (msg))

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void reset_globals() {
    setpointX    = 0.0f;
    setpointY    = 0.0f;
    cornerIndex  = 1;
    index        = 0;
}

// ---------------------------------------------------------------------------
// clip() — integer clamping
// ---------------------------------------------------------------------------
static void test_clip() {
    printf("\n[clip]\n");
    ASSERT_EQ(clip(5,   0,  10),  5,  "within range unchanged");
    ASSERT_EQ(clip(0,   0,  10),  0,  "at minimum unchanged");
    ASSERT_EQ(clip(10,  0,  10), 10,  "at maximum unchanged");
    ASSERT_EQ(clip(-5, -10, 10), -5,  "negative within range unchanged");
    ASSERT_EQ(clip(-1,  0,  10),  0,  "below min → min");
    ASSERT_EQ(clip(11,  0,  10), 10,  "above max → max");
    ASSERT_EQ(clip(-60, -50, 50), -50, "far below min → min");
    ASSERT_EQ(clip( 60, -50, 50),  50, "far above max → max");
}

// ---------------------------------------------------------------------------
// clip2() — float clamping
// ---------------------------------------------------------------------------
static void test_clip2() {
    printf("\n[clip2]\n");
    ASSERT_NEAR(clip2(5.0f,   0.0f, 10.0f),  5.0f, 1e-6, "within range unchanged");
    ASSERT_NEAR(clip2(0.0f,   0.0f, 10.0f),  0.0f, 1e-6, "at minimum unchanged");
    ASSERT_NEAR(clip2(10.0f,  0.0f, 10.0f), 10.0f, 1e-6, "at maximum unchanged");
    ASSERT_NEAR(clip2(-5.5f, -10.0f, 10.0f), -5.5f, 1e-6, "negative within range");
    ASSERT_NEAR(clip2(-0.1f,  0.0f, 10.0f),  0.0f, 1e-6, "just below min → min");
    ASSERT_NEAR(clip2(10.1f,  0.0f, 10.0f), 10.0f, 1e-6, "just above max → max");
    ASSERT_NEAR(clip2(-100.0f, -10.0f, 10.0f), -10.0f, 1e-6, "far below min → min");
    ASSERT_NEAR(clip2( 100.0f, -10.0f, 10.0f),  10.0f, 1e-6, "far above max → max");
}

// ---------------------------------------------------------------------------
// circle()
// ---------------------------------------------------------------------------
static void test_circle() {
    printf("\n[circle]\n");
    const float r   = 10.0f;
    const float tol = 1e-4f;

    reset_globals();
    circle(r, 0);
    ASSERT_NEAR(setpointX,  r,    tol,  "i=0: setpointX = radius");
    ASSERT_NEAR(setpointY,  0.0f, tol,  "i=0: setpointY = 0");

    reset_globals();
    circle(r, pointsPerCycle / 2);  // angle = π
    ASSERT_NEAR(setpointX, -r,    tol,  "i=half: setpointX = -radius");
    ASSERT_NEAR(setpointY,  0.0f, 1e-3, "i=half: setpointY ≈ 0");

    reset_globals();
    circle(r, pointsPerCycle / 4);  // angle ≈ π/2
    ASSERT_NEAR(setpointX,  0.0f, 0.5f, "i=quarter: setpointX ≈ 0");
    ASSERT_NEAR(setpointY,  r,    0.5f, "i=quarter: setpointY ≈ radius");

    reset_globals();
    circle(r, pointsPerCycle);      // angle = 2π ≡ 0
    ASSERT_NEAR(setpointX,  r,    1e-3, "i=full: returns to start X");
    ASSERT_NEAR(setpointY,  0.0f, 1e-3, "i=full: returns to start Y");
}

// ---------------------------------------------------------------------------
// ellipse()
// ---------------------------------------------------------------------------
static void test_ellipse() {
    printf("\n[ellipse]\n");
    const float a = 15.0f, b = 10.0f;

    reset_globals();
    ellipse(a, b, 0);
    ASSERT_NEAR(setpointX,  a,    1e-4, "i=0: setpointX = a");
    ASSERT_NEAR(setpointY,  0.0f, 1e-4, "i=0: setpointY = 0");

    reset_globals();
    ellipse(a, b, pointsPerCycle / 2);  // angle = π
    ASSERT_NEAR(setpointX, -a,    1e-4, "i=half: setpointX = -a");
    ASSERT_NEAR(setpointY,  0.0f, 1e-3, "i=half: setpointY ≈ 0");

    reset_globals();
    ellipse(a, b, pointsPerCycle / 4);  // angle ≈ π/2
    ASSERT_NEAR(setpointX,  0.0f, 0.5f, "i=quarter: setpointX ≈ 0");
    ASSERT_NEAR(setpointY,  b,    0.5f, "i=quarter: setpointY ≈ b");
}

// ---------------------------------------------------------------------------
// fourCorners() — state machine
// ---------------------------------------------------------------------------
static void test_four_corners_sequence() {
    printf("\n[fourCorners — sequence]\n");
    reset_globals();
    cornerIndex = 1;

    fourCorners(30.0f);
    ASSERT_NEAR(setpointX,  32.0f, 1e-4, "corner 1: X =  32");
    ASSERT_NEAR(setpointY,  13.0f, 1e-4, "corner 1: Y =  13");

    fourCorners(30.0f);
    ASSERT_NEAR(setpointX, -32.0f, 1e-4, "corner 2: X = -32");
    ASSERT_NEAR(setpointY,  13.0f, 1e-4, "corner 2: Y =  13");

    fourCorners(30.0f);
    ASSERT_NEAR(setpointX, -32.0f, 1e-4, "corner 3: X = -32");
    ASSERT_NEAR(setpointY, -13.0f, 1e-4, "corner 3: Y = -13");

    fourCorners(30.0f);
    ASSERT_NEAR(setpointX,  32.0f, 1e-4, "corner 4: X =  32");
    ASSERT_NEAR(setpointY, -13.0f, 1e-4, "corner 4: Y = -13");
}

static void test_four_corners_wrap() {
    printf("\n[fourCorners — wrap]\n");
    reset_globals();
    cornerIndex = 4;

    fourCorners(30.0f);                   // processes corner 4, then wraps to 1
    ASSERT_EQ(cornerIndex, 1, "cornerIndex resets to 1 after corner 4");

    fourCorners(30.0f);                   // now processes corner 1
    ASSERT_NEAR(setpointX, 32.0f, 1e-4,  "after wrap: corner 1 X =  32");
    ASSERT_NEAR(setpointY, 13.0f, 1e-4,  "after wrap: corner 1 Y =  13");
}

// ---------------------------------------------------------------------------
// line() — documents known bug: uses global `index` instead of parameter `i`
// ---------------------------------------------------------------------------
static void test_line_bug_uses_global_index() {
    printf("\n[line — known bug: uses global `index` not parameter `i`]\n");
    reset_globals();
    index = 10;         // global
    int i = 0;          // first half (i < pointsPerCycle/2)
    line(5.0f, i);
    // Expected if bug-free: setpointX = i/5.0f/2 = 0
    // Actual (due to bug):  setpointX = index/5.0f/2 = 1.0
    float expected_buggy = (float)index / 5.0f / 2.0f;
    ASSERT_NEAR(setpointX, expected_buggy, 1e-4,
                "line: setpointX uses global `index` (bug), not parameter `i`");
    ASSERT_NEAR(setpointY, 0.0f, 1e-4, "line: setpointY always 0");
}

static void test_line_second_half_negative() {
    printf("\n[line — second half negates X]\n");
    reset_globals();
    index = 10;
    line(5.0f, 80);   // 80 >= pointsPerCycle/2 → negative branch
    float expected = -(float)index / 5.0f / 2.0f;
    ASSERT_NEAR(setpointX, expected, 1e-4, "i>=half: setpointX negative");
    ASSERT_NEAR(setpointY,   0.0f, 1e-4,  "i>=half: setpointY = 0");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    printf("=== ball_plate tests ===\n");

    test_clip();
    test_clip2();
    test_circle();
    test_ellipse();
    test_four_corners_sequence();
    test_four_corners_wrap();
    test_line_bug_uses_global_index();
    test_line_second_half_negative();

    printf("\n=== %d/%d passed", g_pass, g_run);
    if (g_fail) printf(", %d FAILED", g_fail);
    printf(" ===\n");
    return g_fail ? 1 : 0;
}
