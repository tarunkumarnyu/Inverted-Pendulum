#include "controller.h"
#include "config.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  LQR Balance Controller
// ─────────────────────────────────────────────────────────────────────────────
//
//  State vector: x = [theta, theta_dot, cart_pos (mm), cart_vel (mm/s)]
//  Control law:  u = -K * x  (with integral on cart position)
//
//  theta is measured from VERTICAL (0 = upright, positive = one direction)
//  The output u is scaled to PWM range (-255 to +255)
//

static float cart_integral = 0.0f;

void controller_reset() {
    cart_integral = 0.0f;
}

int16_t lqr_balance(float theta, float theta_dot,
                    float cart_pos, float cart_vel) {
    // Integrate cart position for steady-state drift correction
    cart_integral += cart_pos * (1.0f / CONTROL_FREQ_HZ);
    if (cart_integral >  500.0f) cart_integral =  500.0f;
    if (cart_integral < -500.0f) cart_integral = -500.0f;

    // Angle correction with dead zone feed-forward
    float u_angle = -(LQR_K1 * theta + LQR_K2 * theta_dot);
    if (u_angle > 1.0f)       u_angle += MIN_PWM;
    else if (u_angle < -1.0f) u_angle -= MIN_PWM;
    else                       u_angle = 0.0f;

    // Cart centering with integral
    float u_cart = -(LQR_K3 * cart_pos + LQR_K4 * cart_vel + LQR_KI * cart_integral);

    float u = u_angle + u_cart;

    // Clamp to safe balance PWM (not full 255 — prevents violent crashes)
    if (u > MAX_BALANCE_PWM)  u = MAX_BALANCE_PWM;
    if (u < -MAX_BALANCE_PWM) u = -MAX_BALANCE_PWM;

    return (int16_t)u;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Region Check
// ─────────────────────────────────────────────────────────────────────────────

bool in_balance_region(float theta) {
    return fabsf(theta) < BALANCE_EXIT_ANGLE;
}
