# Inverted Pendulum — LQR Balance Controller

A real-time inverted pendulum balance controller running on an Arduino Mega 2560. The system uses LQR (Linear Quadratic Regulator) state feedback with integral cart centering to keep a pendulum balanced upright on a linear rail.

## Hardware

| Component | Model | Notes |
|---|---|---|
| Microcontroller | Arduino Mega 2560 | 5 interrupt pins for dual encoders |
| DC Motor | CQR37D (12V, 6.25:1 gear ratio) | Built-in Hall-effect encoder, 64 CPR |
| Motor Driver | L298N Dual H-Bridge | PWM speed control, min ~35 PWM to move |
| Pendulum Encoder | E38S6 (600 PPR) | NPN open-collector, needs INPUT_PULLUP |
| Drive System | GT2 timing belt + pulleys | Converts rotation to linear cart motion |
| Rail Length | 30 cm | ~±30mm effective cart travel |

## Wiring

```
CQR37D Motor Encoder        Arduino Mega
  Yellow (A) ──────────────── Pin 2  (INT0)
  White  (B) ──────────────── Pin 3  (INT1)
  Blue   (VCC) ────────────── 3.3V
  Gray   (GND) ────────────── GND

E38S6 Pendulum Encoder       Arduino Mega
  Green  (A) ──────────────── Pin 18 (INT5)
  White  (B) ──────────────── Pin 19 (INT4)
  VCC ─────────────────────── 5V
  GND ─────────────────────── GND

L298N Motor Driver            Arduino Mega
  IN1 ─────────────────────── Pin 8
  IN2 ─────────────────────── Pin 9
  ENA ─────────────────────── Pin 10 (PWM)
```

## Control Architecture

The controller uses a split control structure running at **200 Hz**:

**Angle correction** — LQR state feedback on pendulum angle and angular velocity, with dead-zone feed-forward compensation to overcome motor static friction:

```
u_angle = -(K1 * theta + K2 * theta_dot)
if |u_angle| > 1:  u_angle += sign(u_angle) * MIN_PWM
```

**Cart centering** — Proportional + Integral control on cart position, applied as a linear bias (no dead-zone boost) to avoid overpowering the angle correction:

```
u_cart = -(K3 * cart_pos + K4 * cart_vel + KI * cart_integral)
```

**Velocity estimation** uses a 10-sample sliding window (50ms) for 10x better resolution compared to single-step finite difference.

### Current Gains

| Gain | Value | Description |
|---|---|---|
| K1 | 800 | Angle (rad) → PWM |
| K2 | 30 | Angular velocity (rad/s) → PWM |
| K3 | 0.8 | Cart position (mm) → PWM |
| K4 | 0.3 | Cart velocity (mm/s) → PWM |
| KI | 0.5 | Cart position integral → PWM |

## Physical Parameters

- Cart mass: 266 g
- Rod mass: 16 g
- Rod length: 30.4 cm (COM at 15.2 cm)
- Motor encoder: 1600 counts/rev (with quadrature)
- Pendulum encoder: 2400 counts/rev (with quadrature)
- Pulley radius: 6.37 mm (GT2 20-tooth)

## Serial Commands

| Key | Action |
|---|---|
| `s` | Start balancing (hold pendulum upright first) |
| `x` | Emergency stop |
| `r` | Reset encoders to zero |
| `d` | Toggle diagnostic mode (raw encoder counts) |
| `t` | Tilt test (verify motor direction) |
| `m` | Motor test (forward/reverse at PWM 60) |
| `p` | Print current state |

## Serial Output (Balance Mode)

Tab-separated at 50 Hz:
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

## Tuning Guide

1. Run tilt test (`t`) — motor should push cart **toward** the tilt
2. If direction is wrong, flip `MOTOR_DIR` to `-1` in `config.h`
3. Start with K3=0, K4=0, KI=0 — get angle balancing first
4. Increase K1 if pendulum falls too easily
5. Increase K2 if it oscillates rapidly
6. Add K3 for cart centering (start ~0.5, increase until drift stops)
7. Add KI to eliminate steady-state cart drift
8. Add K4 to damp cart oscillation when returning to center

## Status

The angle controller successfully keeps the pendulum within ±2° of vertical. Cart centering via integral action is functional but requires sufficient cart travel (~±50mm minimum) for the integral to converge before hitting rail limits.
