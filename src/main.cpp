#include <Arduino.h>
#include <Encoder.h>
#include <math.h>

// =============================================================================
// HARDWARE PINS
// =============================================================================

// Motor encoder (CQR37D)
#define MOTOR_ENC_A  2
#define MOTOR_ENC_B  3

// Pendulum encoder (E38S6)
#define PEND_ENC_A   18
#define PEND_ENC_B   19

// L298N
#define MOTOR_IN1    8
#define MOTOR_IN2    9
#define MOTOR_ENA    10

// Direction flips if needed
#define MOTOR_DIR    1
#define PEND_DIR     1

// =============================================================================
// PHYSICAL PARAMETERS
// =============================================================================

// Cart + pendulum
// 24g uniform rod (45cm) + 31g bob at the top
#define CART_MASS_KG         0.266f
#define ROD_MASS_KG          0.055f      // 24g rod + 31g top bob
#define ROD_LENGTH_M         0.450f
#define ROD_COM_M            0.352f      // shifted up by the top bob

// Inertia about COM, computed analytically for rod + point mass:
// I_pivot = (1/3)m_r L^2 + m_top L^2 = 0.00790
// I_cg = I_pivot - M*L_cg^2 = 0.00108
#define ROD_INERTIA_KGM2     0.00108f

#define GRAVITY              9.81f

// Encoder conversions
#define MOTOR_CPR_QUAD       1600.0f
#define PEND_CPR_QUAD        2400.0f
#define PULLEY_RADIUS_M      0.00637f

#define PEND_COUNTS_TO_RAD   (2.0f * PI / PEND_CPR_QUAD)
#define MOTOR_COUNTS_TO_M    (2.0f * PI * PULLEY_RADIUS_M / MOTOR_CPR_QUAD)

// =============================================================================
// CONTROL TIMING
// =============================================================================

#define CONTROL_FREQ_HZ      200
#define CONTROL_PERIOD_US    (1000000UL / CONTROL_FREQ_HZ)
#define SERIAL_OUTPUT_HZ     50

// =============================================================================
// SAFETY / MOTOR LIMITS
// =============================================================================

#define MAX_PWM              255
#define MIN_PWM              35
#define MAX_BALANCE_PWM      255

#define BALANCE_EXIT_ANGLE_RAD   0.52f     // ~30 deg
#define CART_LIMIT_M             0.25f     // 25 cm — 5 cm safety margin from physical 30 cm rail

// =============================================================================
// LQR DESIGN PARAMETERS
// =============================================================================
//
// State order for the LQR model is:
// x = [cart_pos(m), cart_vel(m/s), theta(rad), theta_dot(rad/s)]
//
// Cost:
// J = sum (x'Qx + u'Ru)
//
// Q weights state error
// R penalizes control effort
//
// INPUT_GAIN_N_PER_PWM maps PWM command -> equivalent horizontal force
// This is the hardest real-world parameter. Tune this if response is too weak/strong.
//

#define Q_X           3000.0f
#define Q_XDOT        100.0f
#define Q_THETA      8000.0f
#define Q_THETADOT     20.0f
#define R_INPUT         0.10f

#define INPUT_GAIN_N_PER_PWM   0.008f

// Riccati iteration settings
#define LQR_MAX_ITERS   200
#define LQR_TOL         1e-4f

// =============================================================================
// SYSTEM STATE
// =============================================================================

enum SystemState {
  STATE_IDLE,
  STATE_BALANCE,
  STATE_ESTOP
};

SystemState state = STATE_IDLE;

// =============================================================================
// ENCODERS
// =============================================================================

Encoder motorEnc(MOTOR_ENC_A, MOTOR_ENC_B);
Encoder pendEnc(PEND_ENC_A, PEND_ENC_B);

// =============================================================================
// MEASURED STATES
// =============================================================================

// Physical measured states
float cart_pos = 0.0f;      // m
float cart_vel = 0.0f;      // m/s
float theta = 0.0f;         // rad
float theta_dot = 0.0f;     // rad/s

// Sliding window velocity estimation
#define VEL_WINDOW 10
float cart_hist[VEL_WINDOW];
float theta_hist[VEL_WINDOW];
int vel_idx = 0;
float vel_window_dt = 0.0f;

// Timing
unsigned long last_control_us = 0;
unsigned long last_serial_ms = 0;

// Motor command
int16_t motor_cmd = 0;

// LQR matrices
float Ad[4][4];
float Bd[4];
float Qmat[4][4];
float K[4];     // final gain row vector

bool diag_mode = false;

// =============================================================================
// BASIC HELPERS
// =============================================================================

float normalize_angle(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

float fabs_max_diff_4x4(float A[4][4], float B[4][4]) {
  float mx = 0.0f;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float d = fabsf(A[i][j] - B[i][j]);
      if (d > mx) mx = d;
    }
  }
  return mx;
}

// =============================================================================
// MOTOR FUNCTIONS
// =============================================================================

void motor_coast() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}

void motor_brake() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENA, MAX_PWM);
}

void motor_init() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  motor_coast();
}

void motor_set(int16_t speed) {
  // Smooth dead zone bias: ramp from 0 to MIN_PWM over the first 40 PWM
  // counts, then full MIN_PWM bias above that. Longer ramp = gentler
  // transition through the medium-command range.
  const int16_t RAMP = 40;
  if (speed > 2) {
    int16_t bias = (speed >= RAMP) ? MIN_PWM : (int16_t)((int32_t)MIN_PWM * speed / RAMP);
    speed += bias;
  } else if (speed < -2) {
    int16_t mag = -speed;
    int16_t bias = (mag >= RAMP) ? MIN_PWM : (int16_t)((int32_t)MIN_PWM * mag / RAMP);
    speed -= bias;
  } else {
    speed = 0;
  }

  if (speed > MAX_PWM)  speed = MAX_PWM;
  if (speed < -MAX_PWM) speed = -MAX_PWM;

  if (speed > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, -speed);
  } else {
    motor_coast();
  }
}

// =============================================================================
// MATRIX / LQR UTILITIES
// =============================================================================

void mat4_zero(float M[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      M[i][j] = 0.0f;
}

void mat4_identity(float M[4][4]) {
  mat4_zero(M);
  for (int i = 0; i < 4; i++) M[i][i] = 1.0f;
}

void mat4_copy(float dst[4][4], float src[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      dst[i][j] = src[i][j];
}

void mat4_add(float C[4][4], float A[4][4], float B[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      C[i][j] = A[i][j] + B[i][j];
}

void mat4_sub(float C[4][4], float A[4][4], float B[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      C[i][j] = A[i][j] - B[i][j];
}

void mat4_mul(float C[4][4], float A[4][4], float B[4][4]) {
  float T[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      T[i][j] = 0.0f;
      for (int k = 0; k < 4; k++) {
        T[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  mat4_copy(C, T);
}

void mat4_transpose(float T[4][4], float A[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      T[i][j] = A[j][i];
}

void mat4_vec_mul(float y[4], float A[4][4], float x[4]) {
  for (int i = 0; i < 4; i++) {
    y[i] = 0.0f;
    for (int j = 0; j < 4; j++) {
      y[i] += A[i][j] * x[j];
    }
  }
}

float vecT_vec4(float a[4], float b[4]) {
  float s = 0.0f;
  for (int i = 0; i < 4; i++) s += a[i] * b[i];
  return s;
}

void outer4(float C[4][4], float a[4], float b[4], float scale) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      C[i][j] = scale * a[i] * b[j];
}

// =============================================================================
// BUILD MODEL (continuous -> discrete Euler)
// =============================================================================

void build_model_and_weights() {
  // State order:
  // x = [cart_pos, cart_vel, theta, theta_dot]
  //
  // Continuous-time linearized upright inverted pendulum:
  //
  // x_dot = A x + B u
  //
  // u is PWM command here, not physical force directly.
  // So we convert force input model using INPUT_GAIN_N_PER_PWM.

  const float M = CART_MASS_KG;
  const float m = ROD_MASS_KG;
  const float l = ROD_COM_M;
  const float I = ROD_INERTIA_KGM2;
  const float g = GRAVITY;
  const float alpha = INPUT_GAIN_N_PER_PWM;   // N per PWM count

  const float p = I * (M + m) + M * m * l * l;

  // Continuous-time A and B for upright equilibrium
  float A[4][4];
  float B_force[4];

  mat4_zero(A);

  A[0][1] = 1.0f;
  A[1][2] = (m * m * g * l * l) / p;
  A[2][3] = 1.0f;
  A[3][2] = (m * g * l * (M + m)) / p;

  B_force[0] = 0.0f;
  B_force[1] = (I + m * l * l) / p;
  B_force[2] = 0.0f;
  B_force[3] = (m * l) / p;

  // Convert force-input B to PWM-input B
  float Bc[4];
  for (int i = 0; i < 4; i++) {
    Bc[i] = B_force[i] * alpha;
  }

  // Discretize with forward Euler:
  // Ad = I + A*dt
  // Bd = B*dt
  const float dt = 1.0f / CONTROL_FREQ_HZ;

  mat4_identity(Ad);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Ad[i][j] += A[i][j] * dt;
    }
  }

  for (int i = 0; i < 4; i++) {
    Bd[i] = Bc[i] * dt;
  }

  // Build diagonal Q
  mat4_zero(Qmat);
  Qmat[0][0] = Q_X;
  Qmat[1][1] = Q_XDOT;
  Qmat[2][2] = Q_THETA;
  Qmat[3][3] = Q_THETADOT;
}

// =============================================================================
// DISCRETE LQR: solve DARE iteratively
// =============================================================================

bool compute_lqr_gain() {
  // Solve:
  // P = A'PA - A'PB(R + B'PB)^-1 B'PA + Q
  //
  // Then:
  // K = (R + B'PB)^-1 B'PA

  float P[4][4];
  float Pn[4][4];
  mat4_copy(P, Qmat);   // reasonable initial guess

  float At[4][4];
  mat4_transpose(At, Ad);

  for (int iter = 0; iter < LQR_MAX_ITERS; iter++) {
    float PA[4][4];
    float AtPA[4][4];

    float PB[4];
    float BtPA[4];
    float AtPB[4];
    float correction[4][4];

    // PA = P*A
    mat4_mul(PA, P, Ad);

    // AtPA = A'*P*A
    mat4_mul(AtPA, At, PA);

    // PB = P*B
    mat4_vec_mul(PB, P, Bd);

    // scalar S = R + B'*P*B
    float BtPB = vecT_vec4(Bd, PB);
    float S = R_INPUT + BtPB;

    if (fabsf(S) < 1e-8f) {
      return false;
    }

    // BtPA = B' * P * A = B' * PA
    for (int j = 0; j < 4; j++) {
      BtPA[j] = 0.0f;
      for (int i = 0; i < 4; i++) {
        BtPA[j] += Bd[i] * PA[i][j];
      }
    }

    // AtPB = (B'PA)' for symmetric P
    for (int i = 0; i < 4; i++) {
      AtPB[i] = BtPA[i];
    }

    // correction = AtPB * (1/S) * BtPA
    outer4(correction, AtPB, BtPA, 1.0f / S);

    // Pn = AtPA - correction + Q
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        Pn[i][j] = AtPA[i][j] - correction[i][j] + Qmat[i][j];
      }
    }

    // convergence check
    float err = fabs_max_diff_4x4(Pn, P);
    mat4_copy(P, Pn);

    if (err < LQR_TOL) {
      break;
    }

    if (iter == LQR_MAX_ITERS - 1) {
      // still continue to compute K, but warn later
    }
  }

  // Final K = (R + B'PB)^-1 B'PA
  float PA[4][4];
  float PB[4];
  mat4_mul(PA, P, Ad);
  mat4_vec_mul(PB, P, Bd);

  float BtPB = vecT_vec4(Bd, PB);
  float S = R_INPUT + BtPB;
  if (fabsf(S) < 1e-8f) {
    return false;
  }

  for (int j = 0; j < 4; j++) {
    K[j] = 0.0f;
    for (int i = 0; i < 4; i++) {
      K[j] += Bd[i] * PA[i][j];
    }
    K[j] /= S;
  }

  return true;
}

// =============================================================================
// CONTROLLER
// =============================================================================

bool in_balance_region(float th) {
  return fabsf(th) < BALANCE_EXIT_ANGLE_RAD;
}

int16_t lqr_control(float cart_pos_m, float cart_vel_mps,
                    float theta_rad, float theta_dot_rps) {
  // state vector order must match model order:
  // [x, x_dot, theta, theta_dot]
  float x[4];
  x[0] = cart_pos_m;
  x[1] = cart_vel_mps;
  x[2] = theta_rad;
  x[3] = theta_dot_rps;

  float u = 0.0f;
  for (int i = 0; i < 4; i++) {
    u += K[i] * x[i];
  }
  u = -u;

  // Saturate for safety
  if (u > MAX_BALANCE_PWM)  u = MAX_BALANCE_PWM;
  if (u < -MAX_BALANCE_PWM) u = -MAX_BALANCE_PWM;

  return (int16_t)u;
}

// =============================================================================
// RESET RUNTIME STATE
// =============================================================================

void reset_runtime_state() {
  cart_pos = 0.0f;
  cart_vel = 0.0f;
  theta = 0.0f;
  theta_dot = 0.0f;
  motor_cmd = 0;

  for (int i = 0; i < VEL_WINDOW; i++) {
    cart_hist[i] = 0.0f;
    theta_hist[i] = 0.0f;
  }
  vel_idx = 0;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);

  motor_init();

  pinMode(PEND_ENC_A, INPUT_PULLUP);
  pinMode(PEND_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR_ENC_B, INPUT_PULLUP);

  motorEnc.write(0);
  pendEnc.write(0);

  reset_runtime_state();

  vel_window_dt = VEL_WINDOW * (1.0f / CONTROL_FREQ_HZ);

  build_model_and_weights();
  bool ok = compute_lqr_gain();

  // Hand-override gains. The DARE solver gives gains that don't quite
  // match what works on the real hardware. K_theta_dot is too high
  // (causes vibration); K_x and K_xdot are too LOW (cart correction is
  // weak — only ~7 PWM at 76mm of drift). We scale them to match values
  // known to balance the physical system.
  K[0] *= 8.0f;     // cart position: 8x stronger
  K[1] *= 5.0f;     // cart velocity: 5x stronger
  if (K[3] > 60.0f) K[3] = 60.0f;

  Serial.println(F("=== Inverted Pendulum LQR (Q,R -> K on Arduino) ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  d : toggle diagnostic encoder print"));
  Serial.println(F("  s : zero encoders and start balance"));
  Serial.println(F("  x : emergency stop"));
  Serial.println(F("  r : reset encoders"));
  Serial.println(F("  p : print K gain"));
  Serial.println(F("  m : motor test"));
  Serial.println(F("  t : tilt test"));
  Serial.println();

  if (!ok) {
    Serial.println(F("ERROR: LQR gain calculation failed."));
    state = STATE_ESTOP;
  } else {
    Serial.println(F("LQR gain calculated."));
    Serial.print(F("K = [ "));
    for (int i = 0; i < 4; i++) {
      Serial.print(K[i], 6);
      Serial.print(' ');
    }
    Serial.println(F("]"));
    Serial.println(F("State: IDLE"));
  }

  last_control_us = micros();
  last_serial_ms = millis();
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // --------------------------------------------------------------------------
  // SERIAL COMMANDS
  // --------------------------------------------------------------------------
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'd':
      case 'D':
        diag_mode = !diag_mode;
        Serial.println(diag_mode ? F("DIAG ON") : F("DIAG OFF"));
        break;

      case 's':
      case 'S':
        diag_mode = false;
        motorEnc.write(0);
        pendEnc.write(0);
        reset_runtime_state();
        state = STATE_BALANCE;
        Serial.println(F("State -> BALANCE"));
        Serial.println(F("Hold upright, then release."));
        break;

      case 'x':
      case 'X':
        state = STATE_ESTOP;
        motor_brake();
        Serial.println(F("State -> ESTOP"));
        break;

      case 'r':
      case 'R':
        motorEnc.write(0);
        pendEnc.write(0);
        reset_runtime_state();
        Serial.println(F("Encoders reset."));
        break;

      case 'p':
      case 'P':
        Serial.print(F("K = [ "));
        for (int i = 0; i < 4; i++) {
          Serial.print(K[i], 6);
          Serial.print(' ');
        }
        Serial.println(F("]"));
        break;

      case 'm':
      case 'M':
        Serial.println(F("Motor test +60"));
        motor_set(60);
        delay(300);
        motor_coast();
        delay(300);
        Serial.print(F("Motor encoder = "));
        Serial.println(motorEnc.read());

        Serial.println(F("Motor test -60"));
        motor_set(-60);
        delay(300);
        motor_coast();
        delay(300);
        Serial.print(F("Motor encoder = "));
        Serial.println(motorEnc.read());
        break;

      case 'a':
      case 'A': {
        Serial.println(F("=== CART ACCEL TEST ==="));
        Serial.println(F("Center the cart. Starting full PWM in 3s..."));
        delay(3000);

        const int N_SAMPLES = 30;
        long pos_samples[N_SAMPLES];
        unsigned long t_samples[N_SAMPLES];

        motorEnc.write(0);
        unsigned long t0 = micros();
        motor_set(255 * MOTOR_DIR);

        for (int i = 0; i < N_SAMPLES; i++) {
          unsigned long target = (unsigned long)i * 5000UL;
          while ((micros() - t0) < target) { /* spin */ }
          t_samples[i] = micros() - t0;
          pos_samples[i] = motorEnc.read();
        }

        motor_brake();
        delay(300);
        motor_coast();

        Serial.println(F("t_ms\tpos_mm\tvel_mm/s"));
        for (int i = 0; i < N_SAMPLES; i++) {
          float pos_mm = -pos_samples[i] * MOTOR_COUNTS_TO_M * 1000.0f;
          float vel_mm_s = 0.0f;
          if (i > 0) {
            float dt_s = (t_samples[i] - t_samples[i-1]) * 1e-6f;
            float dpos_mm = -(pos_samples[i] - pos_samples[i-1]) * MOTOR_COUNTS_TO_M * 1000.0f;
            vel_mm_s = dpos_mm / dt_s;
          }
          Serial.print(t_samples[i] / 1000.0f, 1);
          Serial.print('\t');
          Serial.print(pos_mm, 2);
          Serial.print('\t');
          Serial.println(vel_mm_s, 1);
        }
        Serial.println(F("=== END ==="));
        break;
      }

      case 't':
      case 'T':
        Serial.println(F("Tilt test: motor should move TOWARD tilt."));
        Serial.println(F("Press any key to stop."));
        pendEnc.write(0);
        while (!Serial.available()) {
          long pc = pendEnc.read();
          float angle = normalize_angle(pc * PEND_COUNTS_TO_RAD * PEND_DIR);

          int16_t cmd2 = (int16_t)(-500.0f * angle);
          if (cmd2 > 80) cmd2 = 80;
          if (cmd2 < -80) cmd2 = -80;

          motor_set(cmd2 * MOTOR_DIR);

          static unsigned long lastp = 0;
          if (millis() - lastp > 100) {
            lastp = millis();
            Serial.print(F("angle(deg)="));
            Serial.print(angle * 180.0f / PI, 1);
            Serial.print(F(" cmd="));
            Serial.println(cmd2 * MOTOR_DIR);
          }
          delay(5);
        }
        Serial.read();
        motor_coast();
        Serial.println(F("Tilt test stopped."));
        break;

      default:
        break;
    }
  }

  // --------------------------------------------------------------------------
  // CONTROL LOOP
  // --------------------------------------------------------------------------
  unsigned long now_us = micros();
  if ((now_us - last_control_us) >= CONTROL_PERIOD_US) {
    float dt = (now_us - last_control_us) * 1e-6f;
    last_control_us = now_us;

    long motor_counts = motorEnc.read();
    long pend_counts = pendEnc.read();

    // cart position in meters
    cart_pos = -motor_counts * MOTOR_COUNTS_TO_M;

    // pendulum angle in radians, zeroed when user pressed 's'
    theta = normalize_angle(pend_counts * PEND_COUNTS_TO_RAD * PEND_DIR);

    // Sliding-window derivative
    int oldest_idx = vel_idx;

    cart_vel = (cart_pos - cart_hist[oldest_idx]) / vel_window_dt;
    theta_dot = (theta - theta_hist[oldest_idx]) / vel_window_dt;

    cart_hist[vel_idx] = cart_pos;
    theta_hist[vel_idx] = theta;
    vel_idx = (vel_idx + 1) % VEL_WINDOW;

    // State machine
    switch (state) {
      case STATE_IDLE:
        motor_cmd = 0;
        motor_coast();
        break;

      case STATE_BALANCE:
        if (!in_balance_region(theta)) {
          state = STATE_ESTOP;
          motor_brake();
          Serial.print(F("ESTOP: angle = "));
          Serial.print(theta * 180.0f / PI, 1);
          Serial.println(F(" deg"));
          break;
        }

        if (fabsf(cart_pos) > CART_LIMIT_M) {
          state = STATE_ESTOP;
          motor_brake();
          Serial.print(F("ESTOP: cart limit = "));
          Serial.print(cart_pos * 1000.0f, 1);
          Serial.println(F(" mm"));
          break;
        }

        motor_cmd = lqr_control(cart_pos, cart_vel, theta, theta_dot);
        motor_set(motor_cmd * MOTOR_DIR);
        break;

      case STATE_ESTOP:
        motor_cmd = 0;
        motor_brake();
        break;
    }
  }

  // --------------------------------------------------------------------------
  // SERIAL OUTPUT
  // --------------------------------------------------------------------------
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
      Serial.print(theta * 180.0f / PI, 2);
      Serial.print('\t');
      Serial.print(theta_dot, 3);
      Serial.print('\t');
      Serial.print(cart_pos * 1000.0f, 2);
      Serial.print('\t');
      Serial.print(cart_vel * 1000.0f, 2);
      Serial.print('\t');
      Serial.println(motor_cmd);
    }
  }
}