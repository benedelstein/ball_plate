# Ball & Plate Balancer

An Arduino-based [ball-and-plate](https://en.wikipedia.org/wiki/Ball_balancing) control system. A resistive touchscreen senses the position of a ball rolling on top of it, and two servos tilt the plate to keep the ball at a target position or drive it along a trajectory. Each axis is controlled independently with its own PID loop.

## How it works

1. **Sense** — A 4-wire resistive touchscreen reports the ball's `(x, y)` position and contact pressure. Coordinates are mapped from raw ADC readings into millimeters, with the center of the screen as the origin `(0, 0)`.
2. **Filter** — Raw readings are passed through a moving-average filter (window of 10 samples) to smooth out noise. Consecutive-valid / consecutive-invalid point counters reject spurious readings and detect when the ball has left the plate.
3. **Control** — A PID controller per axis computes a correction from the error between the ball's filtered position and the current setpoint. The integral term is clamped to limit windup.
4. **Actuate** — The PID output is mapped to a servo angle offset and added to each axis's "flat" angle, tilting the plate to roll the ball toward the setpoint.

If no touch is detected for ~100 loops, the servos return to flat and the integrators reset; after ~300 loops the servos detach.

## Hardware

| Component | Notes |
|-----------|-------|
| Arduino (Uno-class) | Runs the control loop |
| 4-wire resistive touchscreen | ~165 mm × 105 mm active area; measured X resistance ≈ 274 Ω |
| 2× hobby servos | One per tilt axis |

### Pin assignments

| Signal | Pin | Notes |
|--------|-----|-------|
| Touchscreen Y+ (`YP`) | A2 | Must be an analog pin |
| Touchscreen X− (`XM`) | A3 | Must be an analog pin |
| Touchscreen Y− (`YM`) | 8  | Digital |
| Touchscreen X+ (`XP`) | 9  | Digital |
| X servo | 6  | |
| Y servo | 11 | |

The plate's level ("flat") positions are `flatXAngle = 95°` and `flatYAngle = 88°` — adjust these to match your mechanical assembly.

## Dependencies

Install these libraries through the Arduino Library Manager:

- [`Adafruit TouchScreen`](https://github.com/adafruit/Adafruit_TouchScreen) — provides `TouchScreen.h`
- `Servo` — bundled with the Arduino IDE

## Building & uploading

1. Open `ball_plate.ino` in the Arduino IDE (or build with `arduino-cli`).
2. Install the dependencies above.
3. Select your board and port, then upload.
4. Open the Serial Monitor at **9600 baud** to view PID telemetry.

## Trajectory modes

The `mode` variable selects what the ball is asked to do. Change it and re-upload:

| `mode` | Behavior |
|--------|----------|
| `0` | Hold at center `(0, 0)` |
| `1` | Trace a circle (`circle()`, radius 10 mm) |
| `2` | Cycle through four corners (`fourCorners()`) — **default** |
| `3` | Trace an ellipse (`ellipse()`, 15 × 10 mm) |

`radialVelocity` (rotations per second) and `pointsPerCycle` control how fast the setpoint moves around a trajectory.

## Tuning

PID gains are defined near the top of the sketch and are tuned per axis, since each axis has a different length and therefore different dynamics:

```c
const double Kpx = .55,  Kix = 0.05, Kdx = .275;  // X axis
const double Kpy = .35,  Kiy = 0.05, Kdy = .16;   // Y axis
```

Start by tuning `Kp` for a responsive-but-not-oscillating response, add `Kd` to damp overshoot, then a small `Ki` to remove steady-state offset. The integral terms are clamped (`clip2`) to prevent windup.

## Coordinate system

The origin `(0, 0)` is the center of the plate. The active area is 165 mm wide (X) by 105 mm tall (Y), so the corners span roughly `(±82.5, ±52.5)` mm.
