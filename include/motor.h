#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/// Initialize motor driver pins
void motor_init();

/// Set motor speed: -255 (full reverse) to +255 (full forward)
void motor_set(int16_t speed);

/// Brake the motor (both pins HIGH)
void motor_brake();

/// Coast the motor (both pins LOW)
void motor_coast();

#endif // MOTOR_H
