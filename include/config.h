#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
//  PIN DEFINITIONS — Arduino Mega 2560
// =============================================================================

// CQR37D Motor Encoder (Hall-effect, quadrature)
// Connect Yellow → Pin 2, White → Pin 3
#define MOTOR_ENC_A  2   // INT0  (Yellow wire)
#define MOTOR_ENC_B  3   // INT1  (White wire)
// NOTE: CQR37D encoder needs VCC (Blue→3.3V) and GND (Gray→GND)

// E38S6 Pendulum Encoder (600 PPR, NPN open-collector → use INPUT_PULLUP)
// Connect Green → Pin 18, White → Pin 19
#define PEND_ENC_A   18  // INT5
#define PEND_ENC_B   19  // INT4

// L298N Motor Driver
#define MOTOR_IN1    8
#define MOTOR_IN2    9
#define MOTOR_ENA    10  // PWM capable

// Direction multipliers: set to -1 to reverse direction
// Flip MOTOR_DIR if the cart moves the wrong way relative to pendulum tilt
// Flip PEND_DIR if the pendulum angle reads backwards
#define MOTOR_DIR     1     // confirmed correct via tilt test
#define PEND_DIR      1     // 1 = normal encoder direction

// =============================================================================
//  PHYSICAL CONSTANTS
// =============================================================================

// System parameters (for reference — used in LQR gain computation)
#define CART_MASS_KG         0.266f   // cart mass
#define ROD_MASS_KG          0.016f   // rod mass (trimmed)
#define ROD_LENGTH_M         0.304f   // full rod length
#define ROD_COM_M            0.152f   // center of mass (L/2)

// CQR37D motor encoder: 64 CPR (motor shaft) × 6.25 gear ratio = 400 counts/rev
// With quadrature (×4 decoding from Encoder library): 400 × 4 = 1600 counts/rev
#define MOTOR_CPR_QUAD       1600.0f

// Timing belt pulley radius (mm) — ADJUST TO YOUR PULLEY
// Common GT2 20-tooth: 20 × 2mm / (2π) ≈ 6.37mm radius
// Common GT2 16-tooth: 16 × 2mm / (2π) ≈ 5.09mm radius
#define PULLEY_RADIUS_MM     6.37f

// E38S6 encoder: 600 PPR × 4 (quadrature) = 2400 counts/rev
#define PEND_CPR_QUAD        2400.0f

// Conversion factors
#define PEND_COUNTS_TO_RAD   (2.0f * 3.14159265f / PEND_CPR_QUAD)
#define MOTOR_COUNTS_TO_MM   (2.0f * 3.14159265f * PULLEY_RADIUS_MM / MOTOR_CPR_QUAD)

// =============================================================================
//  CONTROL PARAMETERS
// =============================================================================

// Control loop frequency
#define CONTROL_FREQ_HZ      200
#define CONTROL_PERIOD_US    (1000000L / CONTROL_FREQ_HZ)

// Serial output frequency for plotting (Hz)
#define SERIAL_OUTPUT_HZ     50

// Maximum cart travel from center (mm) — ADJUST TO YOUR RAIL LENGTH
#define CART_LIMIT_MM        120.0f   // ±12cm from center

// Maximum PWM output (0–255)
#define MAX_PWM              255

// Minimum PWM to overcome L298N dead zone + motor friction
// Below this value the motor won't actually move
#define MIN_PWM              35

// Maximum PWM allowed during balance (prevents violent crashes)
#define MAX_BALANCE_PWM      220

// =============================================================================
//  LQR GAINS
// =============================================================================
// State vector: x = [theta, theta_dot, cart_pos(mm), cart_vel(mm/s)]
// Control law:  u = -K * x
//
// Linearized system: ωn ≈ 7.36 rad/s (natural frequency of upright equilibrium)
// Rod: 44g, 30.4cm | Cart: 266g
//
// STARTING GAINS — tune on real hardware via Serial commands
// Increase K1 if pendulum doesn't stay upright
// Increase K2 if it oscillates too fast
// Increase K3 if cart drifts too far from center
// Increase K4 if cart overshoots when returning
//
#define LQR_K1   800.0f    // angle gain (tighter angle control → less drift)
#define LQR_K2    30.0f    // velocity damping (higher to prevent oscillation)
#define LQR_K3     0.8f    // cart centering
#define LQR_K4     0.3f    // cart velocity damping
#define LQR_KI     0.5f    // cart integral

// =============================================================================
//  BALANCE PARAMETERS
// =============================================================================

// If pendulum tilts beyond this angle (radians), stop balancing
// ~30 degrees — pendulum has fallen too far to recover
#define BALANCE_EXIT_ANGLE   0.52f

#endif // CONFIG_H
