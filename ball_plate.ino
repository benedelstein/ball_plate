// Ball-on-plate PID controller.
//
// A resistive touchscreen acts as the position sensor; two servos tilt the
// plate along X and Y. Each axis runs an independent PID controller because
// the plate's moments of inertia differ along each axis.
//
// This .ino file is intentionally thin: control math, filtering, and
// trajectory generation live in the headers next to it and are exercised by
// the host-compilable tests under test/.

#include "TouchScreen.h"
#include <Servo.h>

#include "Clip.h"
#include "MovingAverage.h"
#include "PidController.h"
#include "Trajectory.h"

// ---------- Touchscreen pins ----------
// YP / XM must be analog pins (An notation). YM / XP may be digital.
#define YP A2
#define XM A3
#define YM 8
#define XP 9

// ---------- Plate geometry ----------
// Plate is 165 x 105 mm. Origin is the center; bottom-left corner is
// (-PLATE_HALF_WIDTH, -PLATE_HALF_HEIGHT).
static const float PLATE_WIDTH       = 165.0f;
static const float PLATE_HEIGHT      = 105.0f;
static const float PLATE_HALF_WIDTH  = PLATE_WIDTH  / 2.0f;
static const float PLATE_HALF_HEIGHT = PLATE_HEIGHT / 2.0f;

// ---------- Touchscreen ----------
// 500 = measured X-axis resistance in ohms (for pressure calculation).
static const int TOUCH_RESISTANCE_OHMS    = 500;
static const int TOUCH_PRESSURE_THRESHOLD = 10;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, TOUCH_RESISTANCE_OHMS);

// ---------- Servos ----------
static const int X_SERVO_PIN  = 6;
static const int Y_SERVO_PIN  = 11;
static const int X_FLAT_ANGLE = 95; // angle that holds the plate level on X
static const int Y_FLAT_ANGLE = 88; // angle that holds the plate level on Y
// Maximum deflection from the flat angle, in degrees, per axis. X needs more
// range than Y to achieve the same physical tilt.
static const int X_OUTPUT_RANGE = 50;
static const int Y_OUTPUT_RANGE = 40;
Servo xServo;
Servo yServo;

// ---------- Control loop ----------
static const float CONTROL_DT_SECONDS = 0.02f; // ~50 Hz

// ---------- Filtering and validity gating ----------
static const int FILTER_WINDOW            = 10;
static const int MIN_VALID_BEFORE_CONTROL = 3;   // require this many touches in a row before acting
static const int RESET_AFTER_INVALID      = 100; // ball off plate: re-level and reset integrals

MovingAverage<FILTER_WINDOW> filterX;
MovingAverage<FILTER_WINDOW> filterY;

// ---------- PID controllers ----------
// Tuning values are empirical; see commit history for context.
PidController pidX(/*kp=*/0.55f,  /*ki=*/0.05f, /*kd=*/0.275f,
                   /*iMin=*/-10.0f,              /*iMax=*/10.0f);
PidController pidY(/*kp=*/0.35f,  /*ki=*/0.05f, /*kd=*/0.16f,
                   /*iMin=*/-PLATE_HALF_HEIGHT,  /*iMax=*/PLATE_HALF_HEIGHT);

// ---------- Trajectory generator ----------
TrajectoryGenerator trajectory;

// ---------- Loop state ----------
unsigned long lastLoopMs    = 0;
int           validStreak   = 0;
int           invalidStreak = 0;

// Arduino's map() is integer-only, which silently truncates float bounds.
static inline float mapFloat(float x, float inMin, float inMax,
                             float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static void writeFlat() {
  xServo.write(X_FLAT_ANGLE);
  yServo.write(Y_FLAT_ANGLE);
}

void setup() {
  Serial.begin(9600);

  xServo.attach(X_SERVO_PIN);
  yServo.attach(Y_SERVO_PIN);
  writeFlat();

  trajectory.setMode(TRAJECTORY_FOUR_CORNERS);
  trajectory.begin(millis());

  lastLoopMs = millis();
}

void loop() {
  const unsigned long now = millis();

  // Trajectory has its own internal cadence; update it every iteration so
  // mode timing isn't gated by the control loop period.
  const Setpoint target = trajectory.update(now);

  const float dt = (now - lastLoopMs) / 1000.0f;
  if (dt < CONTROL_DT_SECONDS) return;
  lastLoopMs = now;

  const TSPoint p = ts.getPoint();

  // Update validity streaks.
  if (p.z == 0) {
    validStreak = 0;
    ++invalidStreak;
  } else {
    ++validStreak;
    invalidStreak = 0;
  }

  // Ball off plate for too long: re-level and let integrals decay.
  if (invalidStreak >= RESET_AFTER_INVALID) {
    writeFlat();
    pidX.reset();
    pidY.reset();
    return;
  }

  // Wait until we've seen a few consecutive valid touches to avoid acting on
  // a single spurious reading.
  if (validStreak < MIN_VALID_BEFORE_CONTROL) return;
  if (p.z < TOUCH_PRESSURE_THRESHOLD)         return;

  // Map ADC reading to mm. The touchscreen never reads all the way to the
  // physical edges (observed range ~75-950), but we map the full 0-1024
  // range anyway — using the empirical range would inflate near-edge readings
  // and the ball can't reach those positions in practice.
  const float rawX = mapFloat((float)p.x, 0.0f, 1024.0f,
                              -PLATE_HALF_WIDTH,  PLATE_HALF_WIDTH);
  const float rawY = mapFloat((float)p.y, 0.0f, 1024.0f,
                              -PLATE_HALF_HEIGHT, PLATE_HALF_HEIGHT);

  const float x = filterX.add(rawX);
  const float y = filterY.add(rawY);

  const float pidOutX = pidX.update(target.x, x, dt);
  const float pidOutY = pidY.update(target.y, y, dt);

  // Map PID output (in position-error units) to a servo deflection in degrees.
  const float xRange = (float)X_OUTPUT_RANGE;
  const float yRange = (float)Y_OUTPUT_RANGE;
  const int xOffset = (int)roundf(
      clip(mapFloat(pidOutX, -PLATE_HALF_WIDTH,  PLATE_HALF_WIDTH,
                    -xRange, xRange),
           -xRange, xRange));
  const int yOffset = (int)roundf(
      clip(mapFloat(pidOutY, -PLATE_HALF_HEIGHT, PLATE_HALF_HEIGHT,
                    -yRange, yRange),
           -yRange, yRange));

  xServo.write(X_FLAT_ANGLE + xOffset);
  yServo.write(Y_FLAT_ANGLE + yOffset);
}
