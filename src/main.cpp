// =============================================================================
//  Inverted Pendulum — LQR Balance Controller (with Diagnostics)
//  Arduino Mega 2560 + CQR37D Motor + E38S6 Encoder + L298N Driver
// =============================================================================

#include <Arduino.h>
#include <Encoder.h>
#include "config.h"
#include "motor.h"
#include "controller.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Encoders (PJRC Encoder library — quadrature via hardware interrupts)
// ─────────────────────────────────────────────────────────────────────────────
Encoder motorEnc(MOTOR_ENC_A, MOTOR_ENC_B);
Encoder pendEnc(PEND_ENC_A, PEND_ENC_B);

// ─────────────────────────────────────────────────────────────────────────────
//  State variables
// ─────────────────────────────────────────────────────────────────────────────
SystemState state = STATE_IDLE;

float theta      = 0.0f;
float theta_dot  = 0.0f;
float theta_prev = 0.0f;

float cart_pos      = 0.0f;
float cart_vel      = 0.0f;
float cart_pos_prev = 0.0f;

unsigned long last_control_us = 0;
unsigned long last_serial_ms  = 0;
float dt = 0.005f;

int16_t motor_cmd = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Velocity estimation via sliding window (50ms window = 10 cycles at 200Hz)
//  This gives 10x better resolution than single-step finite difference
// ─────────────────────────────────────────────────────────────────────────────
#define VEL_WINDOW  10   // number of control cycles for velocity computation
float theta_history[VEL_WINDOW];
float cart_history[VEL_WINDOW];
int   vel_idx = 0;
float vel_window_dt;

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: normalize angle to [-PI, PI]
// ─────────────────────────────────────────────────────────────────────────────
float normalize_angle(float angle) {
    while (angle >  PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    motor_init();

    // E38S6 NPN open-collector — ensure pull-ups
    pinMode(PEND_ENC_A, INPUT_PULLUP);
    pinMode(PEND_ENC_B, INPUT_PULLUP);

    // CQR37D motor encoder — also set pull-ups (Hall-effect, 3.3V)
    pinMode(MOTOR_ENC_A, INPUT_PULLUP);
    pinMode(MOTOR_ENC_B, INPUT_PULLUP);

    motorEnc.write(0);
    pendEnc.write(0);

    // Initialize history buffers to zero
    for (int i = 0; i < VEL_WINDOW; i++) {
        theta_history[i] = 0.0f;
        cart_history[i]  = 0.0f;
    }
    vel_window_dt = VEL_WINDOW * (CONTROL_PERIOD_US * 1e-6f);

    Serial.println(F("=== Inverted Pendulum — LQR Balance ==="));
    Serial.println(F("Commands:"));
    Serial.println(F("  d - Diagnostic mode (raw encoder counts)"));
    Serial.println(F("  s - Start balancing (hold upright first!)"));
    Serial.println(F("  x - Emergency stop"));
    Serial.println(F("  r - Reset encoders"));
    Serial.println(F("  m - Motor test"));
    Serial.println(F("  p - Print state"));
    Serial.println(F(""));
    Serial.println(F(">>> Press 'd' first to verify encoders <<<"));
    Serial.println(F("State: IDLE"));

    last_control_us = micros();
    last_serial_ms  = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main Loop
// ─────────────────────────────────────────────────────────────────────────────

bool diag_mode = false;

void loop() {

    // ── Serial commands ─────────────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'd':
            case 'D':
                diag_mode = !diag_mode;
                if (diag_mode) {
                    Serial.println(F("DIAG ON — rotate encoders by hand:"));
                    Serial.println(F("  motor_raw  pend_raw  mA mB pA pB"));
                } else {
                    Serial.println(F("DIAG OFF"));
                }
                break;

            case 's':
            case 'S':
                diag_mode = false;
                motorEnc.write(0);
                pendEnc.write(0);
                cart_pos = 0; cart_vel = 0;
                theta = 0; theta_dot = 0;
                // Clear history buffers
                for (int i = 0; i < VEL_WINDOW; i++) {
                    theta_history[i] = 0.0f;
                    cart_history[i]  = 0.0f;
                }
                vel_idx = 0;
                controller_reset();
                state = STATE_BALANCE;
                Serial.println(F("State → BALANCE"));
                Serial.println(F("Release pendulum NOW!"));
                break;

            case 'x':
            case 'X':
                state = STATE_ESTOP;
                motor_brake();
                diag_mode = false;
                Serial.println(F("State → EMERGENCY STOP"));
                break;

            case 'r':
            case 'R':
                motorEnc.write(0);
                pendEnc.write(0);
                cart_pos = 0; cart_vel = 0;
                theta = 0; theta_dot = 0;
                Serial.println(F("Encoders reset"));
                break;

            case 'm':
            case 'M': {
                motorEnc.write(0);
                Serial.println(F("Motor test: forward PWM 60..."));
                motor_set(60);
                delay(300);
                motor_coast();
                delay(300);
                Serial.print(F("  Encoder after fwd: "));
                Serial.println(motorEnc.read());
                Serial.println(F("Motor test: reverse PWM 60..."));
                motor_set(-60);
                delay(300);
                motor_coast();
                delay(300);
                Serial.print(F("  Encoder after rev: "));
                Serial.println(motorEnc.read());
                break;
            }

            case 't':
            case 'T': {
                // TILT TEST: pure proportional control, no velocity.
                // Tilt pendulum by hand. Motor should push cart TOWARDS the tilt.
                // If motor pushes AWAY from tilt, MOTOR_DIR is wrong.
                Serial.println(F("TILT TEST — tilt pendulum slowly by hand."));
                Serial.println(F("Motor should push cart TOWARD the tilt."));
                Serial.println(F("Press any key to stop."));
                pendEnc.write(0);
                while (!Serial.available()) {
                    long pc = pendEnc.read();
                    float angle = pc * PEND_COUNTS_TO_RAD * PEND_DIR;
                    // Simple proportional: 500 * angle in radians
                    int16_t cmd = (int16_t)(-500.0f * angle);
                    // Clamp
                    if (cmd > 80) cmd = 80;
                    if (cmd < -80) cmd = -80;
                    motor_set(cmd * MOTOR_DIR);
                    // Print every 100ms
                    static unsigned long last_print = 0;
                    if (millis() - last_print > 100) {
                        last_print = millis();
                        Serial.print(F("  angle="));
                        Serial.print(angle * 180.0f / PI, 1);
                        Serial.print(F("deg  cmd="));
                        Serial.print(cmd);
                        Serial.print(F("  sent="));
                        Serial.println(cmd * MOTOR_DIR);
                    }
                    delay(5);
                }
                Serial.read(); // consume the key
                motor_coast();
                Serial.println(F("Tilt test stopped."));
                break;
            }

            case 'p':
            case 'P':
                Serial.print(F("State=")); Serial.print(state);
                Serial.print(F(" theta=")); Serial.print(theta * 180.0f / PI, 1);
                Serial.print(F("deg tdot=")); Serial.print(theta_dot, 1);
                Serial.print(F(" cart=")); Serial.print(cart_pos, 1);
                Serial.print(F("mm cmd=")); Serial.println(motor_cmd);
                break;

            default:
                break;
        }
    }

    // ── 200 Hz Control Loop ─────────────────────────────────────────────────
    unsigned long now_us = micros();
    if ((now_us - last_control_us) >= CONTROL_PERIOD_US) {
        dt = (now_us - last_control_us) * 1e-6f;
        last_control_us = now_us;

        // ── Read encoders ───────────────────────────────────────────────────
        long motor_counts = motorEnc.read();
        long pend_counts  = pendEnc.read();

        // Cart position in mm (negated — motor encoder reads backwards)
        cart_pos = -motor_counts * MOTOR_COUNTS_TO_MM;

        // Pendulum angle — zeroed when 's' pressed (upright = 0)
        theta = normalize_angle(pend_counts * PEND_COUNTS_TO_RAD * PEND_DIR);

        // ── Velocity via sliding window ─────────────────────────────────────
        // Compare current value to value from VEL_WINDOW cycles ago
        // This gives velocity averaged over 50ms — much less noisy than 5ms
        int oldest_idx = vel_idx;  // this slot has the oldest value
        theta_dot = (theta - theta_history[oldest_idx]) / vel_window_dt;
        cart_vel  = (cart_pos - cart_history[oldest_idx]) / vel_window_dt;

        // Store current values in circular buffer
        theta_history[vel_idx] = theta;
        cart_history[vel_idx]  = cart_pos;
        vel_idx = (vel_idx + 1) % VEL_WINDOW;

        // ── State machine ───────────────────────────────────────────────────
        switch (state) {
            case STATE_IDLE:
                motor_cmd = 0;
                motor_coast();
                break;

            case STATE_BALANCE:
                // Check if pendulum has fallen too far
                if (!in_balance_region(theta)) {
                    state = STATE_ESTOP;
                    motor_brake();
                    Serial.print(F("ESTOP: fell ("));
                    Serial.print(theta * 180.0f / PI, 1);
                    Serial.println(F("deg). 's' to retry."));
                    break;
                }

                // Safety: cart travel limit
                if (fabsf(cart_pos) > CART_LIMIT_MM) {
                    state = STATE_ESTOP;
                    motor_brake();
                    Serial.print(F("ESTOP: cart limit ("));
                    Serial.print(cart_pos, 1);
                    Serial.println(F("mm)"));
                    break;
                }

                // LQR control — MOTOR_DIR flips sign if motor wired backwards
                motor_cmd = lqr_balance(theta, theta_dot, cart_pos, cart_vel);
                motor_set(motor_cmd * MOTOR_DIR);
                break;

            case STATE_ESTOP:
                motor_cmd = 0;
                motor_brake();
                break;
        }
    }

    // ── Serial output ───────────────────────────────────────────────────────
    unsigned long now_ms = millis();
    if ((now_ms - last_serial_ms) >= (1000 / SERIAL_OUTPUT_HZ)) {
        last_serial_ms = now_ms;

        if (diag_mode) {
            Serial.print(motorEnc.read());
            Serial.print('\t');
            Serial.print(pendEnc.read());
            Serial.print('\t');
            Serial.print(digitalRead(MOTOR_ENC_A));
            Serial.print('\t');
            Serial.print(digitalRead(MOTOR_ENC_B));
            Serial.print('\t');
            Serial.print(digitalRead(PEND_ENC_A));
            Serial.print('\t');
            Serial.println(digitalRead(PEND_ENC_B));
        }
        else if (state == STATE_BALANCE) {
            // angle(deg)  cart_pos(mm)  motor_cmd  theta_dot
            Serial.print(theta * 180.0f / PI, 1);
            Serial.print('\t');
            Serial.print(cart_pos, 1);
            Serial.print('\t');
            Serial.print(motor_cmd);
            Serial.print('\t');
            Serial.println(theta_dot, 2);
        }
    }
}
