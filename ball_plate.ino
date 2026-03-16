#include "TouchScreen.h"
#include <Servo.h>

#define YP A2  // Y plus. must be an analog pin, use "An" notation!
#define XM A3  // X minus. must be an analog pin, use "An" notation!
#define YM 8   // Y minus. can be a digital pin
#define XP 9   // X plus. can be a digital pin

// full dimensions of screen is 165x105mm.
float setpointX = 0; // x setpoint in mm. center of screen = (0,0)
float setpointY = 0; // y setpoint in mm
const float width = 165; // x direction (mm)
const float height = 105; // y direction (mm)
float time, timePrev;
float errorX, errorY, previousErrorX, previousErrorY;
TSPoint p;
float Px, Ix, Dx, Py, Iy, Dy;
int numValidPoints = 0;
int numInvalidPoints = 0;

/////////////////PID CONSTANTS/////////////////
const double Kpx = .55;
const double Kix = 0.05;
const double Kdx = .275;

const double Kpy = .35;
const double Kiy = 0.05;
const double Kdy = .16;
///////////////////////////////////////////////

// SERVOS
const int xServoPin = 6;
const int yServoPin = 11;
Servo xServo;
Servo yServo;
const int flatXAngle = 95;
const int flatYAngle = 88;

// touchscreen (resistance across x is 274 ohms)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 500);

const int pointsPerCycle = 150;
float radialVelocity = 1; // rotations per second
int index = 0;
float trajectoryUpdateTime, lastTrajectoryUpdateTime;

// input smoothing
const int inputWindowSize = 10;
float filteredX = 0;
float filteredY = 0;
float sumX = 0;
float sumY = 0;
float readingsX[inputWindowSize];
float readingsY[inputWindowSize];

int mode = 2;

// helpers
int clip(int value, int minimum, int maximum) {
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

float clip(float value, float minimum, float maximum) {
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

// moving average filter - shifts readings and returns new filtered value
float movingAverage(float readings[], float &sum, float newValue) {
  sum -= readings[0];
  for (int i = 0; i < inputWindowSize - 1; i++) {
    readings[i] = readings[i + 1];
  }
  readings[inputWindowSize - 1] = newValue;
  sum += newValue;
  return sum / inputWindowSize;
}

// compute one axis of PID. updates integral and returns PID output.
float computePID(float error, float previousError, float &I,
                 double Kp, double Ki, double Kd,
                 float iMin, float iMax, float dt) {
  float P = Kp * error;
  I += Ki * error * dt;
  I = clip(I, iMin, iMax);
  float D = Kd * (error - previousError) / dt;
  return P + I + D;
}

void setup() {
  Serial.begin(9600);
  xServo.attach(xServoPin);
  yServo.attach(yServoPin);
  xServo.write(flatXAngle);
  yServo.write(flatYAngle);
  time = millis();
  lastTrajectoryUpdateTime = millis();
}

void loop() {
  time = millis();
  float dt = (time - timePrev) / 1000;
  updateSetpoint();

  if (dt > 0.02) {
    timePrev = time;
    p = ts.getPoint();

    if (p.z == 0) {
      numValidPoints = 0;
      numInvalidPoints++;
    } else {
      numValidPoints++;
      numInvalidPoints = 0;
    }

    // detach servos after prolonged no-touch
    if (numInvalidPoints >= 300) {
      xServo.detach();
      yServo.detach();
      return;
    }

    // reset motors to flat after moderate no-touch
    if (numInvalidPoints >= 100) {
      xServo.write(flatXAngle);
      yServo.write(flatYAngle);
      Ix = 0;
      Iy = 0;
      return;
    }

    // wait for accumulation of readings
    if (numValidPoints < 3) {
      return;
    }

    if (p.z >= 10) {
      // convert readings to mm
      float x = map(p.x, 0, 1024, -82.5, 82.5);
      float y = map(p.y, 0, 1024, -52.5, 52.5);

      filteredX = movingAverage(readingsX, sumX, x);
      filteredY = movingAverage(readingsY, sumY, y);

      errorX = setpointX - filteredX;
      errorY = setpointY - filteredY;

      float PIDx = computePID(errorX, previousErrorX, Ix, Kpx, Kix, Kdx, -10, 10, dt);
      float PIDy = computePID(errorY, previousErrorY, Iy, Kpy, Kiy, Kdy, -height/2, height/2, dt);

      int xOutput = int(round(map(PIDx, -width/2, width/2, -50, 50)));
      int yOutput = int(round(map(PIDy, -height/2, height/2, -40, 40)));
      xOutput = clip(xOutput, -50, 50);
      yOutput = clip(yOutput, -40, 40);

      xServo.write(flatXAngle + xOutput);
      yServo.write(flatYAngle + yOutput);

      previousErrorX = errorX;
      previousErrorY = errorY;
    }
  }
}

// trajectory patterns
void circle(float radius, int i) {
  float angle = float(i) / pointsPerCycle * M_PI * 2;
  setpointX = radius * cos(angle);
  setpointY = radius * sin(angle);
}

void ellipse(float a, float b, int i) {
  float angle = float(i) / pointsPerCycle * M_PI * 2;
  setpointX = a * cos(angle);
  setpointY = b * sin(angle);
}

void line(float length, int i) {
  if (i < pointsPerCycle / 2) {
    setpointX = length / 2;
  } else {
    setpointX = -length / 2;
  }
  setpointY = 0;
}

int cornerIndex = 1;
void fourCorners(float l) {
  float w = l;
  float h = l;
  switch (cornerIndex) {
    case 1: setpointX = w;  setpointY = h;  break;
    case 2: setpointX = -w; setpointY = h;  break;
    case 3: setpointX = -w; setpointY = -h; break;
    case 4: setpointX = w;  setpointY = -h; break;
  }
  cornerIndex++;
  if (cornerIndex > 4) cornerIndex = 1;
}

void updateSetpoint() {
  float dt = (time - lastTrajectoryUpdateTime) / 1000;
  float updateIncrement = 1 / radialVelocity / pointsPerCycle;

  switch (mode) {
    case 0:
      setpointX = 0;
      setpointY = 0;
      break;
    case 1:
      if (dt > updateIncrement) {
        circle(10, index);
        lastTrajectoryUpdateTime = time;
        index += int(round(dt / updateIncrement));
        if (index > pointsPerCycle) index = 0;
      }
      break;
    case 2:
      if (dt > 2) {
        fourCorners(30);
        lastTrajectoryUpdateTime = time;
      }
      break;
    case 3:
      if (dt > updateIncrement) {
        ellipse(15, 10, index);
        lastTrajectoryUpdateTime = time;
        index += int(round(dt / updateIncrement));
        if (index > pointsPerCycle) index = 0;
      }
      break;
    default:
      setpointX = 0;
      setpointY = 0;
      break;
  }
}
