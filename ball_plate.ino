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
float Px, Ix, Dx, Py, Iy, Dy; // pid values for each axis
int numValidPoints = 0; // number of consecutive valid points. used to discard random measurements that may swing the motors
int numInvalidPoints = 0; // number of consecutive no-touch points. if crosses a threshold, motors are reset

/////////////////PID CONSTANTS/////////////////
// TODO: MAY NEED DIFFERENT CONSTANTS FOR X AND Y. 
// each axis has a different length, so different moment of inertia, etc.
const double Kpx = .9;
const double Kix = 0.01; //.1
const double Kdx = .25; //.25

const double Kpy = .7;
const double Kiy = .01;
const double Kdy = .15; //.15
///////////////////////////////////////////////

// SERVOS (doesn't need pwm pins)
const int xServoPin = 6;
const int yServoPin = 11;
Servo xServo;
Servo yServo;
const int flatXAngle = 102;
const int flatYAngle = 80;

// initialize touchscreen
// resistance across x is 274 ohms (measured)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 500);

const int pointsPerCycle = 100;
float radialVelocity = .1; // .1 rotations per second
int index = 0;
float trajectoryUpdateTime, lastTrajectoryUpdateTime;

void setup() {
  Serial.begin(9600); // is this needed at a diff baud?
  xServo.attach(xServoPin);
  yServo.attach(yServoPin);

  // todo: write the servos to their starting points (flat). determine what the starting points should be
  xServo.write(flatXAngle); // might not be 90
  yServo.write(flatYAngle);
  time = millis();
  lastTrajectoryUpdateTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  // read time
  timePrev = time;
  time = millis();
  float dt = (time - timePrev) / 1000; // get to seconds from milliseconds
//  Serial.println(time-lastTrajectoryUpdateTime);
// (1/radialVelocity)/pointsPerCycle
//  if ((time - lastTrajectoryUpdateTime)/1000 > 3) {
////    circle(5, index); // set setpoint to circle trajectory
////    line(10, index);
////    setpointX = setpointX == 10 ? -10 : 10;
////    setpointY = 0;
//    
//    lastTrajectoryUpdateTime = time;
//    index++;
//    if (index > pointsPerCycle) {
//      index = 0;
//    }
//  }

//  Serial.println(setpointX);
  
  // read current position
  // get x and y position of ball on touchscreen
  p = ts.getPoint();

//  Serial.print("X = "); Serial.print(p.x);
//  Serial.print("\tY = "); Serial.print(p.y);
//  Serial.print("\tPressure = "); Serial.println(p.z);
  
  // nothing is touching, discard this point
  if(p.z == 0) {
//    Serial.println("discarded");
    numValidPoints = 0;
    numInvalidPoints++;
//    Serial.print("invalid points count: "); Serial.println(numInvalidPoints);
  } else {
    numValidPoints++;
    numInvalidPoints = 0; // reset to zero because we have a a valid point now
  }

  // reset motors if not long enough
  if(numInvalidPoints >= 100) {
    xServo.write(flatXAngle);
    yServo.write(flatYAngle);
    return;
  }

  if(numInvalidPoints >= 300) {
    xServo.detach();
    yServo.detach();
  }
  
  // wait for accumulation of readings to do something
  if(numValidPoints < 3) {
    return;
  }

  // valid point, continue
  if (p.z >= 10) {
    // convert readings to mm
    // the readings never get that close to the edges
    // actual range: x: 75-950, y: 100-870
    // using full range still because then that doesn't inflate the xy readings.
    // if i used a range of 75-950, then a reading of 950 is 82.5, but it cant read your finger that close, its
    // really just a reading of about ~75mm
    float x = map(p.x, 0, 1024, -82.5, 82.5); // x is 165 mm wide
    float y = map(p.y, 0, 1024, -52.5, 52.5); // y is 105mm wide
//    Serial.println(x);
//    Serial.print("\t");
//    Serial.println(y);
    // calculate error
    errorX = setpointX - x;
    errorY = setpointY - y;
  //  Serial.print("x error = "); Serial.println(errorX);
  //  Serial.print("y error = "); Serial.println(errorY);
    // calculate x and y motor PID independently ??
    // calculate error and PID output
    Px = Kpx*errorX;
    Ix = Ix + Kix*errorX*dt;
    Dx = Kdx*(errorX-previousErrorX)/dt;
    float PIDx = Px+Ix+Dx;
  
    Py = Kpy*errorY;
    Iy = Iy + Kiy*errorY*dt;
    Dy = Kdy*(errorY-previousErrorY)/dt;
    float PIDy = Py+Iy+Dy;
//    Serial.print("PIDx = "); Serial.println(PIDx);
//    Serial.print("PIDy = "); Serial.println(PIDy);
    
    // transform and output based on PID output
    
    // HOW DO I MAP AN ERROR IN DISTANCE (FROM TOUCHSCREEN) TO AN OUTPUT IN MOTOR ANGLE?
    // IS IT ALL IN THE PID CONSTANTS? IS THERE A TRANSFER FUNCTION?
    
    // e.g. map to output range of 0-180
    // if x is less than setpoint x, then move one servo to tilt x 
    // same for y
    // map output values to 0-180 or smaller range
    // TODO: WHAT ARE THE INPUT LIMITS?
    // do I take a linear mapping, or just clip the values?
    int xOutput = int(round(map(PIDx, -width/2, width/2, -50, 50))); // x needs larger range
    int yOutput = int(round(map(PIDy, -height/2, height/2, -40, 40)));
    xOutput = clip(xOutput,-50,50);
    yOutput = clip(yOutput,-40,40);
    Serial.print("X angle: "); 
    Serial.println(xOutput);
    Serial.print("Y angle: "); 
    Serial.println(yOutput);

    // even point +/- whatever the PID was. so if even point is 90 degrees and PID output is 10, write 90+10 to servo
    // TODO: maybe use writeMicroseconds() to get more resolution. 1000-2000 microseconds range corresponds to 0-180
    xServo.write(flatXAngle + xOutput);
    yServo.write(flatYAngle + yOutput);
    
    // set this for the next loop
    previousErrorX = errorX;
    previousErrorY = errorY;
  }
}

// setpoint draw a circle
// radius (mm)
// take an index and number of points, then calculate x and y setpoint based on index and trig
// so for index 1 and num points = 100, 1/100*360 = angle in degrees. calculate based on that
// how many indices should there be for a reasonable rotation rate?
void circle(float radius, int i) {
    float angle = float(i)/pointsPerCycle * M_PI * 2;
    setpointX = radius * cos(angle);
    setpointY = radius * sin(angle);
}

void line(float length, int i) {
  if (i < pointsPerCycle/2) {
    setpointX = index/length/2;
  } else {
    setpointX = -index/length/2;
  }
  setpointY = 0;
}

void fourCorners(float sideLength) {
  // todo
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
