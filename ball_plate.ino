#include <Servo.h>
#include "TouchScreen.h"
#include "Config.h"
#include "PIDController.h"
#include "MovingAverage.h"
#include "Trajectory.h"

// Touchscreen
TouchScreen ts = TouchScreen(XP, YP, XM, YM, TS_RESISTANCE);

// Servos
Servo xServo;
Servo yServo;

// PID controllers
PIDController pidX(KP_X, KI_X, KD_X, -IX_CLAMP, IX_CLAMP);
PIDController pidY(KP_Y, KI_Y, KD_Y, -IY_CLAMP, IY_CLAMP);

// Input filters
MovingAverage filterX;
MovingAverage filterY;

// Trajectory
TrajectoryController trajectory(POINTS_PER_CYCLE, RADIAL_VELOCITY);

// Timing
float currentTime, previousTime;

// Touch validation
int numValidPoints = 0;
int numInvalidPoints = 0;

void setup() {
    Serial.begin(9600);
    xServo.attach(X_SERVO_PIN);
    yServo.attach(Y_SERVO_PIN);
    xServo.write(FLAT_X_ANGLE);
    yServo.write(FLAT_Y_ANGLE);

    filterX.init(INPUT_WINDOW_SIZE);
    filterY.init(INPUT_WINDOW_SIZE);

    currentTime = millis();
    previousTime = currentTime;
    trajectory.setLastUpdateTime(currentTime);
    trajectory.setMode(TrajectoryController::MODE_FOUR_CORNERS);
}

void loop() {
    currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;

    Point2D setpoint = trajectory.update(currentTime);

    if (dt < MIN_LOOP_DT) return;
    previousTime = currentTime;

    TSPoint p = ts.getPoint();

    // Touch validation
    if (p.z == 0) {
        numValidPoints = 0;
        numInvalidPoints++;
    } else {
        numValidPoints++;
        numInvalidPoints = 0;
    }

    // Reset servos to flat if ball is absent
    if (numInvalidPoints >= DETACH_THRESHOLD) {
        xServo.detach();
        yServo.detach();
        return;
    }
    if (numInvalidPoints >= RESET_THRESHOLD) {
        xServo.write(FLAT_X_ANGLE);
        yServo.write(FLAT_Y_ANGLE);
        pidX.reset();
        pidY.reset();
        return;
    }

    // Wait for enough consecutive valid readings
    if (numValidPoints < MIN_VALID_POINTS) return;

    if (p.z < MIN_PRESSURE) return;

    // Convert ADC readings to mm
    float rawX = map(p.x, TS_ADC_MIN, TS_ADC_MAX, -HALF_WIDTH, HALF_WIDTH);
    float rawY = map(p.y, TS_ADC_MIN, TS_ADC_MAX, -HALF_HEIGHT, HALF_HEIGHT);

    // Apply moving average filter
    float filteredX = filterX.update(rawX);
    float filteredY = filterY.update(rawY);

    // Compute PID
    float errorX = setpoint.x - filteredX;
    float errorY = setpoint.y - filteredY;

    float pidOutX = pidX.compute(errorX, dt);
    float pidOutY = pidY.compute(errorY, dt);

    Serial.print(pidX.getP(errorX)); Serial.print(",");
    Serial.print(pidX.getIntegral()); Serial.print(",");
    Serial.println(pidOutX - pidX.getP(errorX) - pidX.getIntegral());

    // Map PID output to servo angle offsets
    int xOutput = int(round(map(pidOutX, -HALF_WIDTH, HALF_WIDTH, -X_OUTPUT_RANGE, X_OUTPUT_RANGE)));
    int yOutput = int(round(map(pidOutY, -HALF_HEIGHT, HALF_HEIGHT, -Y_OUTPUT_RANGE, Y_OUTPUT_RANGE)));
    xOutput = clamp(xOutput, -X_OUTPUT_RANGE, X_OUTPUT_RANGE);
    yOutput = clamp(yOutput, -Y_OUTPUT_RANGE, Y_OUTPUT_RANGE);

    xServo.write(FLAT_X_ANGLE + xOutput);
    yServo.write(FLAT_Y_ANGLE + yOutput);
}
