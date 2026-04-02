#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

/// System operating states
enum SystemState {
    STATE_IDLE,
    STATE_BALANCE,
    STATE_ESTOP
};

/// Compute LQR balance control output
/// State: [theta, theta_dot, cart_pos_mm, cart_vel_mm_s]
/// Returns motor command (-255 to +255)
int16_t lqr_balance(float theta, float theta_dot,
                    float cart_pos, float cart_vel);

/// Reset controller state (call when starting balance)
void controller_reset();

/// Check if pendulum angle is within controllable region
bool in_balance_region(float theta);

#endif // CONTROLLER_H
