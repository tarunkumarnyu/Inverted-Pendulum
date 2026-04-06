# Inverted Pendulum — LQR Balance Controller

[![Platform](https://img.shields.io/badge/Platform-Arduino%20Mega%202560-00979D)](https://store.arduino.cc/products/arduino-mega-2560-rev3)
[![Build](https://img.shields.io/badge/Build-PlatformIO-orange)](https://platformio.org/)
[![Control](https://img.shields.io/badge/Control-LQR%20%40%20200%20Hz-D85A30)](#control-architecture)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](#license)

A real-time inverted pendulum balance controller running on an Arduino Mega 2560. The system uses LQR state feedback for angle stabilization combined with PI cart centering, running at 200 Hz with quadrature encoder feedback. The angle controller keeps the pendulum within ±2° of vertical.

## Demo



https://github.com/user-attachments/assets/bd4a8e72-5cd6-4e73-9694-4a535366f85d


## Architecture

![Control architecture](architecture.png)

Two encoders feed a sliding-window state estimator. The controller splits into an LQR angle correction (with dead-zone feed-forward to overcome motor static friction) and a PI cart centering term (linear bias only, so it cannot overpower the angle correction). The sum is clamped to ±220 PWM and sent through an L298N H-bridge to a geared DC motor driving the cart over a GT2 timing belt.

## Hardware

| Component | Model | Notes |
|---|---|---|
| Microcontroller | Arduino Mega 2560 | 4 hardware interrupt pins for dual quadrature encoders |
| DC motor | CQR37D (12 V, 6.25:1 gear) | Built-in Hall encoder, 64 CPR → 1600 counts/rev quadrature |
| Motor driver | L298N dual H-bridge | PWM speed control, ~35 PWM minimum to overcome friction |
| Pendulum encoder | E38S6 (600 PPR) | NPN open-collector — needs `INPUT_PULLUP` |
| Drive | GT2 timing belt + 20-tooth pulley | r ≈ 6.37 mm, converts rotation to linear cart motion |
| Rail | 30 cm linear | ±120 mm software travel limit |

### Wiring

```
CQR37D Motor Encoder        Arduino Mega
  Yellow (A) ─────────────── Pin 2  (INT0)
  White  (B) ─────────────── Pin 3  (INT1)
  Blue   (VCC) ───────────── 3.3 V
  Gray   (GND) ───────────── GND

E38S6 Pendulum Encoder      Arduino Mega
  Green  (A) ─────────────── Pin 18 (INT5)
  White  (B) ─────────────── Pin 19 (INT4)
  VCC ────────────────────── 5 V
  GND ────────────────────── GND

L298N Motor Driver          Arduino Mega
  IN1 ────────────────────── Pin 8
  IN2 ────────────────────── Pin 9
  ENA ────────────────────── Pin 10 (PWM)
```

### Physical parameters

- Cart mass: 266 g
- Rod mass: 16 g · length 30.4 cm (COM at 15.2 cm)
- Linearized natural frequency at the upright equilibrium: ωₙ ≈ 7.36 rad/s
- Motor encoder: 1600 counts/rev (×4 quadrature)
- Pendulum encoder: 2400 counts/rev (×4 quadrature)

## Control Architecture

The control loop runs at **200 Hz** (5 ms period) and is structured as two additive terms.

**State estimator.** Encoder counts are converted to angle (rad) and cart position (mm). Velocities are estimated by comparing the current sample to the value 10 cycles (50 ms) ago — this gives roughly 10× better resolution than a single-step finite difference and avoids amplifying encoder quantization noise.

**Angle correction (LQR).** State feedback on pendulum angle and angular velocity, with a dead-zone feed-forward term that adds `MIN_PWM` whenever the desired command is non-trivial. This compensates for the L298N + motor static friction (~35 PWM dead band) so the controller can react to small angle errors.

```
u_angle = -(K1 * theta + K2 * theta_dot)
if |u_angle| > 1:  u_angle += sign(u_angle) * MIN_PWM
```

**Cart centering (PI on position).** Proportional + derivative on cart position and velocity, plus an integral term that eliminates steady-state drift caused by belt slop and asymmetric friction. Crucially, no dead-zone boost is applied here — the cart-centering term is meant to be a *gentle bias* on top of angle correction, not a dominant input.

```
u_cart = -(K3 * cart_pos + K4 * cart_vel + KI * cart_integral)
```

**Output.** `u = u_angle + u_cart`, clamped to ±`MAX_BALANCE_PWM` (220) to prevent violent recoveries that snap the rod or rip the cart off the rail.

### Current gains

| Gain | Value | Maps |
|---|---|---|
| K1 | 800 | angle (rad) → PWM |
| K2 | 30 | angular velocity (rad/s) → PWM |
| K3 | 0.8 | cart position (mm) → PWM |
| K4 | 0.3 | cart velocity (mm/s) → PWM |
| KI | 0.5 | cart position integral → PWM |

The integral is clamped to ±500 to prevent windup if the cart hits a rail.

### Safety

- **Fall detection.** If `|theta| > 30°` (`BALANCE_EXIT_ANGLE`), the controller drops to `STATE_ESTOP` and brakes.
- **Cart limit.** If `|cart_pos| > 120 mm`, same — the cart is about to hit the end of the rail.
- **PWM clamp.** Output is hard-limited to ±220 PWM.

## Serial Interface

| Key | Action |
|---|---|
| `s` | Start balancing (hold pendulum upright first) |
| `x` | Emergency stop |
| `r` | Reset encoders to zero |
| `d` | Toggle diagnostic mode (raw encoder counts + pin states) |
| `t` | Tilt test — pure-P control to verify motor direction |
| `m` | Motor test (forward/reverse at PWM 60) |
| `p` | Print current state |

In balance mode the firmware streams tab-separated telemetry at 50 Hz suitable for the Arduino Serial Plotter:

```
theta(deg)    cart_pos(mm)    motor_cmd(PWM)    theta_dot(rad/s)
```

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
pio run              # build
pio run -t upload    # flash
pio device monitor   # serial monitor (115200 baud)
```

## Bring-up & Tuning

1. **Verify encoders** (`d`). Rotate each encoder by hand and confirm both counts move and pin states toggle. If A and B never both change, you have a wiring issue.
2. **Tilt test** (`t`). Tilt the pendulum slowly by hand; the motor should push the cart **toward** the tilt. If it pushes away, flip `MOTOR_DIR` to `-1` in `config.h`.
3. **Angle-only first.** Set `K3 = K4 = KI = 0` and tune K1, K2 until the pendulum balances on its own (the cart will drift).
   - Increase K1 if the rod falls easily.
   - Increase K2 if it oscillates rapidly.
4. **Add cart centering.** Bring K3 up from ~0.5 until cart drift stops, then add KI to eliminate steady-state offset, and finally K4 to damp the return-to-center motion.

## Status

Angle stabilization holds the rod within ±2° of vertical. Cart centering via the integral term works but needs at least ±50 mm of travel for the integral to converge before the rail-limit ESTOP triggers — useful range, not a hard constraint.

## Project Structure

```
include/
  config.h          # pins, physical constants, gains, limits
  controller.h      # LQR + state machine API
  motor.h           # H-bridge API
src/
  main.cpp          # 200 Hz loop, serial CLI, state machine
  controller.cpp    # lqr_balance() + integral, in_balance_region()
  motor.cpp         # motor_set/brake/coast with deadband + boost
platformio.ini      # Arduino Mega 2560, paulstoffregen/Encoder
```

## License

MIT.
