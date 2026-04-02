#include "motor.h"
#include "config.h"

void motor_init() {
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);

    // Start with motor stopped
    motor_coast();
}

void motor_set(int16_t speed) {
    // Clamp to valid range
    if (speed > MAX_PWM)  speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;

    // Dead-zone handling:
    // 1. If command is tiny (noise), zero it out (deadband)
    // 2. If command is real but below motor minimum, boost it
    if (speed > 0) {
        if (speed < 5) speed = 0;           // deadband — ignore noise
        else if (speed < MIN_PWM) speed = MIN_PWM;  // boost to overcome friction
    }
    if (speed < 0) {
        if (speed > -5) speed = 0;
        else if (speed > -MIN_PWM) speed = -MIN_PWM;
    }

    if (speed > 0) {
        // Forward
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_ENA, speed);
    } else if (speed < 0) {
        // Reverse
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_ENA, -speed);
    } else {
        // Stop (coast)
        motor_coast();
    }
}

void motor_brake() {
    // Active brake: both inputs HIGH
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, MAX_PWM);
}

void motor_coast() {
    // Coast: both inputs LOW
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, 0);
}
