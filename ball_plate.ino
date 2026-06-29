#include <stdint.h> // not sure if needed - imports integer types
#include "TouchScreen.h"
#include <Servo.h>

#define YP A2  // Y plus. must be an analog pin, use "An" notation!
#define XM A3  // X minus. must be an analog pin, use "An" notation!
#define YM 8   // Y minus. can be a digital pin
#define XP 9   // X plus. can be a digital pin

// full dimensions of screen is 165x105mm.
float setpointX = 0; // x setpoint in mm. let center of screen = (0,0). bottom left edge = (-82.5,-52.5)
float setpointY = 0; // y setpoint in mm
const float width = 165; // x direction
const float height = 105; // y direction (mm)
float time, timePrev;
float errorX, errorY, previousErrorX, previousErrorY;
TSPoint p; // current point of touchscreen
float Ix, Iy; // integral terms for each axis
int numValidPoints = 0; // number of consecutive valid points. used to discard random measurements that may swing the motors
int numInvalidPoints = 0; // number of consecutive no-touch points. if crosses a threshold, motors are reset

/////////////////PID CONSTANTS/////////////////
// New approach: PID output is servo tilt in degrees.
// Tune these as deg/mm, deg/(mm*s), and deg*s/mm.
// Conservative starting tune for hardware testing. Increase Kp first if it
// feels too slow, increase Kd if it overshoots, and add Ki only after P/D
// behavior is stable.
const float Kpx = .22;
const float Kix = .012;
const float Kdx = .09;

const float Kpy = .20;
const float Kiy = .014;
const float Kdy = .075;

const float maxXTilt = 28;
const float maxYTilt = 24;
const float maxIntegralTilt = 4;
const float integralActiveError = 22; // only integrate near the target
///////////////////////////////////////////////

// SERVOS (doesn't need pwm pins)
const int xServoPin = 6;
const int yServoPin = 11;
Servo xServo;
Servo yServo;
const int flatXAngle = 95;
const int flatYAngle = 88;

// initialize touchscreen
// resistance across x is 274 ohms (measured)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 500);

const int pointsPerCycle = 150;
float radialVelocity = 1; // rotations per second
int index = 0;
float trajectoryUpdateTime, lastTrajectoryUpdateTime;

// input smoothing
const float inputAlpha = 0.25; // higher = faster response, lower = smoother signal
const int touchThreshold = 10;
float filteredX = 0;
float filteredY = 0;
bool filterInitialized = false;
bool havePreviousError = false;
bool servosAttached = false;

int mode = 2;

void updateSetpoint();
void circle(float radius, int i);
void ellipse(float a, float b, int i);
void line(float length, int i);
void fourCorners(float l);
void attachServos();
void detachServos();
void resetPidAndFilter();
float fmap(float value, float in_min, float in_max, float out_min, float out_max);
float calculateTilt(float error, float previousError, float *integral, float kp, float ki, float kd, float maxTilt, float dt);
int clip(int value, int minimum, int maximum);
float clip2(float value, float minimum, float maximum);

void setup() {
  Serial.begin(9600); // is this needed at a diff baud?
  attachServos();

  // todo: write the servos to their starting points (flat). determine what the starting points should be
  xServo.write(flatXAngle); // might not be 90
  yServo.write(flatYAngle);
  time = millis();
  timePrev = time;
  lastTrajectoryUpdateTime = time;
//  setpointX = 30;
}

void loop() {
  // put your main code here, to run repeatedly:

  // read time
  time = millis();
  float dt = (time - timePrev) / 1000; // get to seconds from milliseconds
//  Serial.print("dt: "); Serial.println(dt*1000);
//  Serial.println(time-lastTrajectoryUpdateTime);
  updateSetpoint();

//  Serial.println(setpointX);
  if (dt > 0.02) {
    timePrev = time;
    // read current position
    // get x and y position of ball on touchscreen
    p = ts.getPoint();

  //  Serial.print("X = "); Serial.print(p.x);
  //  Serial.print("\tY = "); Serial.print(p.y);
  //  Serial.print("\tPressure = "); Serial.println(p.z);

    // nothing is touching, discard this point
      if(p.z < touchThreshold) {
    //    Serial.println("discarded");
        numValidPoints = 0;
        numInvalidPoints++;
        havePreviousError = false;
    //    Serial.print("invalid points count: "); Serial.println(numInvalidPoints);
      } else {
        attachServos();
        numValidPoints++;
        numInvalidPoints = 0; // reset to zero because we have a valid point now
      }

      // reset motors if the ball has been missing for a while
      if(numInvalidPoints >= 100) {
        xServo.write(flatXAngle);
        yServo.write(flatYAngle);
        resetPidAndFilter();

        if(numInvalidPoints >= 300) {
          detachServos();
        }
        return;
      }

      // wait for a few consecutive valid readings before moving
      if(numValidPoints < 3) {
        return;
      }

      // valid point, continue
      if (p.z >= touchThreshold) {
      // convert readings to mm
      // the readings never get that close to the edges
      // actual range: x: 75-950, y: 100-870
      // using full range still because then that doesn't inflate the xy readings.
      // if i used a range of 75-950, then a reading of 950 is 82.5, but it cant read your finger that close, its
      // really just a reading of about ~75mm
      float x = fmap(p.x, 0, 1024, -width/2.0, width/2.0); // x is 165 mm wide
      float y = fmap(p.y, 0, 1024, -height/2.0, height/2.0); // y is 105mm wide
  //    Serial.println(x);

      // Exponential filter: less lag than a moving average, no startup bias.
      if(!filterInitialized) {
        filteredX = x;
        filteredY = y;
        filterInitialized = true;
      } else {
        filteredX += inputAlpha * (x - filteredX);
        filteredY += inputAlpha * (y - filteredY);
      }
  //    Serial.print(x); Serial.print(",");Serial.println(filteredX);
  //    Serial.print(y); Serial.print(","); Serial.println(filteredY);

      // calculate error
      errorX = setpointX - filteredX;
      errorY = setpointY - filteredY;
    //  Serial.print("x error = "); Serial.println(errorX);
    //  Serial.print("y error = "); Serial.println(errorY);

      // New approach: the controller directly outputs a requested platform
      // tilt in degrees. No extra map-from-distance-to-servo step.
      float xTilt = calculateTilt(errorX, previousErrorX, &Ix, Kpx, Kix, Kdx, maxXTilt, dt);
      float yTilt = calculateTilt(errorY, previousErrorY, &Iy, Kpy, Kiy, Kdy, maxYTilt, dt);

      int xAngle = clip(int(round(flatXAngle + xTilt)), 0, 180);
      int yAngle = clip(int(round(flatYAngle + yTilt)), 0, 180);

//      Serial.print("X angle: ");
//      Serial.println(xAngle);
//      Serial.print("Y angle: ");
//      Serial.println(yAngle);

      xServo.write(xAngle);
      yServo.write(yAngle);

      previousErrorX = errorX;
      previousErrorY = errorY;
      havePreviousError = true;
    }
  }
}

// setpoint draw a circle
// radius (mm)
// take an index and number of points, then calculate x and y setpoint based on index and trig
// so for index 1 and num points = 100, 1/100*360 = angle in degrees. calculate based on that
// how many indices should there be for a reasonable rotation rate?
void circle(float radius, int i) {
    float angle = float(i)/pointsPerCycle * 2.0 * PI;
    setpointX = radius * cos(angle);
    setpointY = radius * sin(angle);
}

void ellipse(float a, float b, int i) {
    float angle = float(i)/pointsPerCycle * 2.0 * PI;
    setpointX = a * cos(angle);
    setpointY = b * sin(angle);
}

void line(float length, int i) {
  int halfCycle = pointsPerCycle / 2;
  int wrappedIndex = i % pointsPerCycle;
  float progress;

  if (wrappedIndex < halfCycle) {
    progress = float(wrappedIndex) / halfCycle;
  } else {
    progress = 1.0 - float(wrappedIndex - halfCycle) / halfCycle;
  }

  setpointX = -length / 2.0 + progress * length;
  setpointY = 0;
}

int cornerIndex = 1;
void fourCorners(float l) {
  // todo
  float w = 32;
  float h = 13;
  switch(cornerIndex) {
    case 1:
      // quad 1
      setpointX = w;
      setpointY = h;
      break;
    case 2:
      setpointX = -w;
      setpointY = h;
      break;
    case 3:
      setpointX = -w;
      setpointY = -h;
      break;
    case 4:
      setpointX = w;
      setpointY = -h;
      break;
  }
  cornerIndex++;
  if(cornerIndex > 4) cornerIndex = 1; // reset back to beginning
}

void updateSetpoint() {
  float dt = (time - lastTrajectoryUpdateTime)/1000;
  float updateIncrement = 1/radialVelocity/pointsPerCycle;

  switch(mode) {
    case 0:
      // center
      setpointX = 0;
      setpointY = 0;
      break;
    case 1:
      // circle
      if (dt > updateIncrement) {
        circle(10, index); // set setpoint to circle trajectory
        lastTrajectoryUpdateTime = time;
        index+=int(round(dt/updateIncrement)); // often dt is larger than the ideal update time (skipping updates)
        // bc of this, update index to nearest integer. this avoids artificially lowering the angular velocity.
        if (index > pointsPerCycle) {
          index = 0;
        }
      }
      break;
    case 2:
      // four corners
      if(dt > 2) {
        fourCorners(30);
        lastTrajectoryUpdateTime = time;
      }
      break;
    case 3:
      // ellipse
      if (dt > 1/radialVelocity/pointsPerCycle) {
        ellipse(15,10, index); // set setpoint to circle trajectory
        lastTrajectoryUpdateTime = time;
        index+=int(round(dt/updateIncrement)); // if dt is more than the update time, then increments index by more than 1
        if (index > pointsPerCycle) {
          index = 0;
        }
      }
      break;
    default:
      setpointX = 0;
      setpointY = 0;
      break;
  }
}

void attachServos() {
  if(!servosAttached) {
    xServo.attach(xServoPin);
    yServo.attach(yServoPin);
    servosAttached = true;
  }
}

void detachServos() {
  if(servosAttached) {
    xServo.detach();
    yServo.detach();
    servosAttached = false;
  }
}

void resetPidAndFilter() {
  Ix = 0;
  Iy = 0;
  previousErrorX = 0;
  previousErrorY = 0;
  havePreviousError = false;
  filterInitialized = false;
  filteredX = 0;
  filteredY = 0;
}

float fmap(float value, float in_min, float in_max, float out_min, float out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateTilt(float error, float previousError, float *integral, float kp, float ki, float kd, float maxTilt, float dt) {
  float proportional = kp * error;

  // Anti-windup: only integrate when the ball is reasonably close to the target.
  if(abs(error) <= integralActiveError) {
    *integral += ki * error * dt;
    *integral = clip2(*integral, -maxIntegralTilt, maxIntegralTilt);
  } else {
    *integral = 0;
  }

  float derivative = 0;
  if(havePreviousError) {
    derivative = kd * (error - previousError) / dt;
  }

  return clip2(proportional + *integral + derivative, -maxTilt, maxTilt);
}

// helper
int clip(int value, int minimum, int maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

// helper
float clip2(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}
