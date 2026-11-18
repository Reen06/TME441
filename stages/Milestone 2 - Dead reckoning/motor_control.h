/* Motor Control Library
*  Utilities for configuring and controlling DC motors.
*
*  @author Matthew Ford
*  @version 2024.1
*/

// Define motor control pins
#define RMOTOR 1
#define RMOTOR_DIR_A 12
#define RMOTOR_DIR_B 11
#define RMOTOR_SPD 10

#define LMOTOR 0
#define LMOTOR_DIR_A 7
#define LMOTOR_DIR_B 8
#define LMOTOR_SPD 9

// Define PID options
#define PID_INT 20       // PID update interval [ms]
#define PID_REP_INT 250  // PID report interval [ms]

/* Global variables for PID motor control. You will need to tune these
*  parameters for your own robot. */
const float K_P = .1;    // Proportional gain
const float K_I = 0.001;  // Integral gain
const float K_D = 0.000;    // Derivative gain (damping)

long last_enc_l = 0;  // Last encoder position [steps]
long last_enc_r = 0;
long last_pid_time = 0;  // Time of last control loop update [ms]
long last_pid_rep_time = 0;
long err_int_l = 0;  // Error integrals
long err_int_r = 0;
long last_speed_l = 0;  // Previous measured speed [steps/sec]
long last_speed_r = 0;

// Attach encoders
Encoder enc_r(21, 20);  // Change pin numbers to match your own circuit
Encoder enc_l(18, 19);

// Initialize motors
void initMotors() {
  pinMode(LMOTOR_DIR_A, OUTPUT);
  pinMode(LMOTOR_DIR_B, OUTPUT);
  pinMode(LMOTOR_SPD, OUTPUT);

  pinMode(RMOTOR_DIR_A, OUTPUT);
  pinMode(RMOTOR_DIR_B, OUTPUT);
  pinMode(RMOTOR_SPD, OUTPUT);
}

/* Turn motor on at desired speed
*
*  @param motor Which motor to turn on (LMOTOR or RMOTOR).
*  @param speed Desired speed, -255 to 255. Negative = backwards.
*/
void motorOn(int motor, int speed) {
  if (motor == LMOTOR) {
    digitalWrite(LMOTOR_DIR_A, (speed > 0));
    digitalWrite(LMOTOR_DIR_B, !(speed > 0));
    analogWrite(LMOTOR_SPD, abs(speed));
  } else if (motor == RMOTOR) {
    digitalWrite(RMOTOR_DIR_A, (speed > 0));
    digitalWrite(RMOTOR_DIR_B, !(speed > 0));
    analogWrite(RMOTOR_SPD, abs(speed));
  }
}

/* Stop motor FAST (electronic braking)
*
*  @ param motor Which motor to turn on (LMOTOR or RMOTOR).
*/
void motorHalt(int motor) {
  if (motor == LMOTOR) {
    digitalWrite(LMOTOR_DIR_A, LOW);
    digitalWrite(LMOTOR_DIR_B, LOW);
    digitalWrite(LMOTOR_SPD, HIGH);
  } else if (motor == RMOTOR) {
    digitalWrite(RMOTOR_DIR_A, LOW);
    digitalWrite(RMOTOR_DIR_B, LOW);
    digitalWrite(RMOTOR_SPD, HIGH);
  }
}

/* Update motor voltages using feedback control
*
*  This function should be called repeatedly in a fast loop to update
*  the control signals. The PID update interval can be adjusted by
*  changing the constant PID_INT.
*
*  @param speed_l Desired speed of left motor [ticks/second].
*  @param speed_r ...              right motor.
*  @param report Print data to Serial Monitor periodically.
*  A zero speed immediately brakes that wheel and clears its PID memory,
*  even when the next timed PID update is not due yet.
*/
void updatePID(long speed_l, long speed_r, bool report) {

  // Handle stopped wheels before the timing gate. Never apply PID output to a
  // stopped wheel: stored integral error could otherwise turn it on again.
  if (speed_l == 0) {
    motorHalt(LMOTOR);
    err_int_l = 0;
    last_speed_l = 0;
  }
  if (speed_r == 0) {
    motorHalt(RMOTOR);
    err_int_r = 0;
    last_speed_r = 0;
  }

  long curr_time = millis();  // Get current clock time
  long dt = curr_time - last_pid_time;

  if (dt >= PID_INT) {
    // Calculate motor speeds
    long curr_enc_l = enc_l.read();
    long curr_enc_r = enc_r.read();

    long speed_meas_l = (1000 * (curr_enc_l - last_enc_l)) / dt;
    long speed_meas_r = (1000 * (curr_enc_r - last_enc_r)) / dt;

    // Calculate error signals
    long err_l = speed_l - speed_meas_l;
    long err_r = speed_r - speed_meas_r;

    // Accumulate error only for wheels with nonzero speed commands.
    if (speed_l != 0) err_int_l += err_l * dt;
    if (speed_r != 0) err_int_r += err_r * dt;

    // Calculate control signals
    int u_P_l = K_P * err_l;
    int u_P_r = K_P * err_r;
    int u_I_l = K_I * err_int_l;
    int u_I_r = K_I * err_int_r;
    int u_D_l = K_D * ((speed_meas_l - last_speed_l) / dt);
    int u_D_r = K_D * ((speed_meas_r - last_speed_r) / dt);

    int u_l = (speed_l == 0) ? 0 : constrain(int(u_P_l + u_I_l + u_D_l), -255, 255);
    int u_r = (speed_r == 0) ? 0 : constrain(int(u_P_r + u_I_r + u_D_r), -255, 255);

    // Update "memory" variables
    last_enc_l = curr_enc_l;
    last_enc_r = curr_enc_r;
    last_pid_time = curr_time;
    last_speed_l = (speed_l == 0) ? 0 : speed_meas_l;
    last_speed_r = (speed_r == 0) ? 0 : speed_meas_r;

    // Stopped wheels stay braked; only moving wheels receive PWM commands.
    if (speed_l != 0) motorOn(LMOTOR, u_l);
    if (speed_r != 0) motorOn(RMOTOR, u_r);

    // Report values to Serial Monitor
    if (report && (curr_time - last_pid_rep_time) > PID_REP_INT) {
      Serial.print("time:");
      Serial.print(curr_time);
      Serial.print(", v_meas_l:");
      Serial.print(speed_meas_l);
      Serial.print(", v_meas_r:");
      Serial.print(speed_meas_r);
      Serial.print(", voltage_l:");
      Serial.print(u_l);
      Serial.print(", voltage_r:");
      Serial.println(u_r);

      last_pid_rep_time = curr_time;
    }
  }
}

// Reset PID "memory" terms
void resetPID() {
  last_speed_l = 0;
  last_speed_r = 0;
  err_int_l = 0;
  err_int_r = 0;
  last_enc_l = enc_l.read();
  last_enc_r = enc_r.read();
  last_pid_time = millis();
}
