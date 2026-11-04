/*
 * Drive straight or turn with smooth speed ramps; stop at encoder targets.
 *   driveDistance(150.0f);  // Forward 150 cm; use -50.0f for backward 50 cm.
 *   rotateCCW(90.0f);      // CCW 90 degrees; use -90.0f for clockwise.
 *   stopDriving();         // Brake both wheels.
 *
 * Moves wait until finished or stopped by a fault: true = success, false = fault.
 * Encoders measure travel; the PID in motor_control.h controls wheel speed.
 * Long moves: ramp up, cruise, ramp down. Short moves: steady minimum speed.
 * Names: l/r = left/right; ticks = encoder counts.
 */

// --- Robot measurements: calibrate for the actual robot ---
const float WHEEL_DIAMETER_CM = 8.0;                // Tire diameter under load.
const float TRACK_WIDTH_CM = 17.5f;                 // Distance between tire centers.
const float ENCODER_TICKS_PER_WHEEL_REV = 1632.67f;  // Counts per full wheel turn.

// --- Speed settings: tune after calibration and motor PID tuning ---
const float DRIVE_SPEED_CM_S = 20.0f;      // Straight driving speed.
const float TURN_SPEED_CM_S = 6.0f;        // Each wheel's speed during a turn.
const float MIN_MOTION_SPEED_CM_S = 3.0f;  // Minimum speed to overcome friction.
const float SLEW_ACCEL_CM_S2 = 10.0f;      // Ramp acceleration (cm/s^2); lower = gentler.

// --- Timing and debugging ---
// Encoders determine distance and angle; timeouts only stop failed movements.
const unsigned long MOVEMENT_TIMEOUT_MS = 60000UL;   // 60 s maximum per move.
const unsigned long NO_PROGRESS_TIMEOUT_MS = 3000UL; // 3 s without progress = stall.
const unsigned long BRAKE_SETTLE_MS = 200UL;         // Let braking settle after success.
const bool REPORT_DRIVING_PID = true;              // Print PID data to Serial when true.

// --- Stop ---
// Zero speed brakes immediately and clears stored PID error.
void stopDriving() {
  updatePID(0, 0, false);
}

// --- Speed profile: all distances in cm, returned speed in cm/s ---
float rampSpeed(float progress_cm, float target_cm, float max_speed_cm_s) {
  const float min_speed = MIN_MOTION_SPEED_CM_S;
  const float min_speed_sq = min_speed * min_speed;

  // v^2 = u^2 + 2*a*d gives the distance for ONE ramp from min to max speed.
  const float ramp_cm =
      (max_speed_cm_s * max_speed_cm_s - min_speed_sq) / (2.0f * SLEW_ACCEL_CM_S2);

  // Not enough room for both ramps? Use a steady low speed for the whole move.
  if (target_cm < 2.0f * ramp_cm) {
    return min_speed;
  }

  progress_cm = constrain(progress_cm, 0.0f, target_cm);
  const float remaining_cm = target_cm - progress_cm;

  // Near the start, progress limits speed; near the end, distance remaining does.
  const float slew_cm = min(progress_cm, remaining_cm);
  const float speed_cm_s = sqrt(min_speed_sq + 2.0f * SLEW_ACCEL_CM_S2 * slew_cm);
  return min(speed_cm_s, max_speed_cm_s);
}

// --- Shared movement: used by driveDistance() and rotateCCW() ---
/*
 * Move each wheel by its own distance: positive = forward, negative = backward.
 * Each wheel brakes at its target; a zero distance keeps that wheel braked.
 */
bool moveWheels(float distance_l_cm, float distance_r_cm, float max_speed_cm_s) {
  // 1. Check calibration and speed limits.
  if (WHEEL_DIAMETER_CM <= 0.0f || TRACK_WIDTH_CM <= 0.0f ||
      ENCODER_TICKS_PER_WHEEL_REV <= 0.0f || MIN_MOTION_SPEED_CM_S <= 0.0f ||
      max_speed_cm_s < MIN_MOTION_SPEED_CM_S || SLEW_ACCEL_CM_S2 <= 0.0f) {
    stopDriving();
    Serial.println(F("Check calibration and speed limits in driving.ino."));
    return false;
  }

  // 2. Convert distance targets from cm to ticks. Circumference = PI * diameter.
  const float ticks_per_cm =
      ENCODER_TICKS_PER_WHEEL_REV / (PI * WHEEL_DIAMETER_CM);
  const float target_l = abs(distance_l_cm) * ticks_per_cm;
  const float target_r = abs(distance_r_cm) * ticks_per_cm;

  // Targets are positive tick counts; direction holds the forward/backward sign.
  const int direction_l = (distance_l_cm < 0.0f) ? -1 : 1;
  const int direction_r = (distance_r_cm < 0.0f) ? -1 : 1;

  // 3. Save starting readings; leave the encoder counts running between moves.
  const long start_l = enc_l.read();
  const long start_r = enc_r.read();
  resetPID();

  bool finished_l = (target_l == 0.0f);
  bool finished_r = (target_r == 0.0f);
  bool success = true;
  long furthest_l = 0;
  long furthest_r = 0;
  const unsigned long start_ms = millis();
  unsigned long last_progress_l_ms = start_ms;
  unsigned long last_progress_r_ms = start_ms;

  // 4. Read progress, check for faults, and update speeds until both wheels finish.
  while (!finished_l || !finished_r) {
    const unsigned long now_ms = millis();

    // Direction makes intended travel positive. Wrong-way travel reduces progress.
    const long progress_l = direction_l * (enc_l.read() - start_l);
    const long progress_r = direction_r * (enc_r.read() - start_r);

    // Reset stall timers only for new progress, so rocking back and forth won't count.
    if (progress_l > furthest_l) {
      furthest_l = progress_l;
      last_progress_l_ms = now_ms;
    }
    if (progress_r > furthest_r) {
      furthest_r = progress_r;
      last_progress_r_ms = now_ms;
    }

    // Once finished, a wheel stays finished even if it rolls back slightly.
    if (progress_l >= target_l) {
      finished_l = true;
    }
    if (progress_r >= target_r) {
      finished_r = true;
    }
    if (finished_l && finished_r) {
      break;
    }

    const bool timed_out = now_ms - start_ms >= MOVEMENT_TIMEOUT_MS;
    const bool stalled_l =
        !finished_l && now_ms - last_progress_l_ms >= NO_PROGRESS_TIMEOUT_MS;
    const bool stalled_r =
        !finished_r && now_ms - last_progress_r_ms >= NO_PROGRESS_TIMEOUT_MS;

    if (timed_out || stalled_l || stalled_r) {
      success = false;
      break;
    }

    // Plan each wheel's ramp from encoder travel. Finished wheels get zero.
    long speed_l = 0;
    long speed_r = 0;

    if (!finished_l) {
      const float speed_cm_s = rampSpeed(progress_l / ticks_per_cm,
                                         abs(distance_l_cm), max_speed_cm_s);
      // PID expects whole ticks/s. Keep a moving command from rounding to zero.
      speed_l = direction_l * max(1L, (long)(speed_cm_s * ticks_per_cm));
    }

    if (!finished_r) {
      const float speed_cm_s = rampSpeed(progress_r / ticks_per_cm,
                                         abs(distance_r_cm), max_speed_cm_s);
      speed_r = direction_r * max(1L, (long)(speed_cm_s * ticks_per_cm));
    }

    updatePID(speed_l, speed_r, REPORT_DRIVING_PID);
    delay(1);  // Check often; PID updates on its own 20 ms interval.
  }

  // 5. Brake both wheels after completion or a fault.
  stopDriving();
  if (success) {
    delay(BRAKE_SETTLE_MS);
  } else {
    Serial.println(F("Movement stopped: timeout or no encoder progress."));
  }
  return success;
}

// --- Drive straight ---
// Same distance for both wheels: positive = forward, negative = backward.
bool driveDistance(float distance_cm) {
  return moveWheels(distance_cm, distance_cm, DRIVE_SPEED_CM_S);
}

// --- Turn in place ---
/*
 * Viewed from above: positive angle = CCW, negative = clockwise.
 * Each wheel's arc = PI * track width * (angle / 360).
 */
bool rotateCCW(float angle_degrees) {
  const float wheel_arc_cm = PI * TRACK_WIDTH_CM * angle_degrees / 360.0f;

  // Opposite wheel directions make the robot spin around its center.
  return moveWheels(-wheel_arc_cm, wheel_arc_cm, TURN_SPEED_CM_S);
}
