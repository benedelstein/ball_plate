#ifndef CONFIG_H
#define CONFIG_H

// Touchscreen pins
#define YP A2  // Y+ (analog)
#define XM A3  // X- (analog)
#define YM 8   // Y- (digital)
#define XP 9   // X+ (digital)

// Touchscreen dimensions (mm)
const float PLATE_WIDTH  = 165.0;
const float PLATE_HEIGHT = 105.0;
const float HALF_WIDTH   = PLATE_WIDTH / 2.0;
const float HALF_HEIGHT  = PLATE_HEIGHT / 2.0;

// Touchscreen ADC range
const int TS_ADC_MIN = 0;
const int TS_ADC_MAX = 1024;
const int TS_RESISTANCE = 500;  // ohms across touchscreen

// Minimum pressure to register a touch
const int MIN_PRESSURE = 10;

// PID constants - X axis
const double KP_X = 0.55;
const double KI_X = 0.05;
const double KD_X = 0.275;

// PID constants - Y axis
const double KP_Y = 0.35;
const double KI_Y = 0.05;
const double KD_Y = 0.16;

// Servo configuration
const int X_SERVO_PIN   = 6;
const int Y_SERVO_PIN   = 11;
const int FLAT_X_ANGLE  = 95;
const int FLAT_Y_ANGLE  = 88;
const int X_OUTPUT_RANGE = 50;  // max servo offset for X axis
const int Y_OUTPUT_RANGE = 40;  // max servo offset for Y axis

// Timing
const float MIN_LOOP_DT = 0.02;  // minimum dt in seconds (50 Hz)

// Input filtering
const int INPUT_WINDOW_SIZE = 10;

// Touch validation
const int MIN_VALID_POINTS   = 3;    // consecutive valid points before acting
const int RESET_THRESHOLD    = 100;  // invalid points before resetting servos
const int DETACH_THRESHOLD   = 300;  // invalid points before detaching servos

// Integral clamp
const float IX_CLAMP = 10.0;
const float IY_CLAMP = HALF_HEIGHT;

// Trajectory
const int POINTS_PER_CYCLE     = 150;
const float RADIAL_VELOCITY    = 1.0;  // rotations per second

#endif
