# Ball-on-plate PID controller

Arduino sketch that balances a ball on a tilting plate using a resistive
touchscreen as the position sensor and two servos to tilt the plate.

## Layout

```
ball_plate.ino     Arduino entry point: pin wiring, setup(), loop().
Clip.h             Templated clamp helper.
MovingAverage.h    Fixed-window incremental moving average.
PidController.h    Per-axis PID controller with clipped integral.
Trajectory.h       Setpoint trajectory generator (center / circle / ellipse
                   / four-corners modes), plus pure trajectory math helpers.
test/              Host-compilable unit tests for the modules above.
```

The headers are deliberately free of Arduino dependencies so they can be
exercised on a workstation with `g++`.

## Hardware

- Resistive touchscreen on pins YP=A2, XM=A3, YM=8, XP=9
  (uses the `TouchScreen` library).
- Two servos on pins 6 (X) and 11 (Y) (uses the `Servo` library).
- Plate dimensions: 165 x 105 mm.

## Building the firmware

Open `ball_plate.ino` in the Arduino IDE — the other headers in the sketch
folder are picked up automatically.

## Running the tests

```
cd test
make
```

The test binary prints `N checks, 0 failures` on success and exits non-zero
on any failure.
