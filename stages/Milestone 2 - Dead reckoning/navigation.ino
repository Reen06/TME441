/* navigation.ino
*  Functions for navigating the robot to April tags using the HuskyLens camera
*
*  Starter author: Matthew Ford
*/

const float SEARCH_ANGLE_DEG = 30.0f;
const unsigned long SEARCH_TIMEOUT_MS = 30000UL;
const unsigned long APPROACH_TIMEOUT_MS = 120000UL;
const float HEADING_TOLERANCE_DEG = 2.0f;
const float ALIGNMENT_GAIN = 0.70f;
const float MAX_ALIGNMENT_TURN_DEG = 10.0f;
const int CENTERED_READINGS_REQUIRED = 3;
const unsigned long ALIGNMENT_TIMEOUT_MS = 30000UL;
const unsigned long ALIGNMENT_SETTLE_MS = 150UL;
const float REACQUIRE_ANGLE_DEG = 15.0f;
const float MAX_FINAL_CLEARANCE_CM = 10.0f;
const float FINAL_APPROACH_CM = 15.0f;
const float APPROACH_SPEED_CM_S = 20.0f; // Maximum continuous forward speed.
const float FINAL_SPEED_CM_S = 5.0f;    // Slow speed near the target.
const float SLOWDOWN_RANGE_CM = 40.0f;
const unsigned long RANGE_SAMPLE_INTERVAL_MS = 60UL;
const unsigned long RANGE_REPORT_INTERVAL_MS = 250UL;
const float MAX_APPROACH_TRAVEL_CM = 250.0f;  // Maximum encoder-measured wheel travel per tag

// HC-SR04-style ultrasonic sensor wired to Arduino Mega pins 22 and 23.
const bool USE_ULTRASONIC = true;
const int ULTRASONIC_TRIG_PIN = 22;
const int ULTRASONIC_ECHO_PIN = 23;

void initRangeSensor() {
  if (!USE_ULTRASONIC) return;
  if (ULTRASONIC_TRIG_PIN < 0 || ULTRASONIC_ECHO_PIN < 0 ||
      ULTRASONIC_TRIG_PIN == ULTRASONIC_ECHO_PIN) {
    stopDriving();
    Serial.println(F("Set valid ultrasonic pins before enabling sensor."));
    while (true) { delay(100); }
  }
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}

// Distance from the ultrasonic sensor face to a reflecting object [cm].
// Return -1 if no valid echo. Only called when sensor is enabled.
float ultrasonicRangeCM() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return -1.0f;
  float distance = duration * 0.0343f / 2.0f;
  return (distance >= 2.0f && distance <= 400.0f) ? distance : -1.0f;
}

/* Find and approach the specified April tag
*
*  @param tag_id ID number of the desired tag
*  @return
*     1 = Success (valid clearance <= 10 cm after target alignment)
*    -1 = Tag search timeout
*    -2 = Communication, range sensor, or movement fault
*/
int driveToTag(int tag_id) {
  stopDriving();
  unsigned long start_time = millis();
  TagPose pose = {0, 0.0f, 0.0f};

  // 1. Search for the desired tag and calculate range and heading.
  while (millis() - start_time < SEARCH_TIMEOUT_MS) {
    pose = rangeAndHeadingToTag(tag_id);
    if (pose.status == -1) {
      Serial.println(F("ERROR: Camera communication or measurement failed."));
      stopDriving(); return -2;
    }
    if (pose.status == 1) break;
    if (!rotateCCW(SEARCH_ANGLE_DEG)) { stopDriving(); return -2; }
    odometryDelay(150);
  }
  if (pose.status != 1) { stopDriving(); return -1; }

  // 2. Close the camera heading loop BEFORE enabling forward movement.
  //    Use short, slow corrections and remeasure after each turn.
  Serial.print(F("Tag detected: ")); Serial.println(tag_id);
  start_time = millis();
  int centered_readings = 0;
  float last_correction = 0.0f;
  bool alignment_confirmed = false;
  while (millis() - start_time < ALIGNMENT_TIMEOUT_MS) {
    if (pose.status == -1) {
      stopDriving();
      Serial.println(F("ERROR: Camera failed during alignment."));
      return -2;
    }
    if (pose.status == 0) {
      // Never drive forward without confirmation. If a correction lost
      // the tag, first undo half that correction; then scan in small steps.
      centered_readings = 0;
      float recover = last_correction != 0.0f ? -0.5f * last_correction : REACQUIRE_ANGLE_DEG;
      last_correction = 0.0f;
      Serial.println(F("Tag lost during alignment. Searching again."));
      if (!rotateCCW(recover)) { stopDriving(); return -2; }
    } else {
      Serial.print(F("Heading error: ")); Serial.println(pose.heading);
      if (abs(pose.heading) <= HEADING_TOLERANCE_DEG) {
        centered_readings++;
        Serial.print(F("Centered readings: ")); Serial.println(centered_readings);
        if (centered_readings >= CENTERED_READINGS_REQUIRED) {
          alignment_confirmed = true;
          break;
        }
      } else {
        centered_readings = 0;
        float correction = constrain(ALIGNMENT_GAIN * pose.heading,
                                    -MAX_ALIGNMENT_TURN_DEG, MAX_ALIGNMENT_TURN_DEG);
        last_correction = correction;
        Serial.print(F("Alignment turn: ")); Serial.println(correction);
        // Same differential-drive geometry as rotateCCW(), but use the
        // minimum motion speed (5 cm/s in driving.ino) for fine alignment.
        const float arc_cm = PI * TRACK_WIDTH_CM * correction / 360.0f;
        if (!moveWheels(-arc_cm, arc_cm, MIN_MOTION_SPEED_CM_S)) {
          stopDriving(); return -2;
        }
      }
    }
    // Allow the chassis and camera image to settle between readings.
    odometryDelay(ALIGNMENT_SETTLE_MS);
    pose = rangeAndHeadingToTag(tag_id);
  }
  if (!alignment_confirmed) {
    stopDriving();
    Serial.println(F("ERROR: Alignment not confirmed. Forward movement blocked."));
    return -2;
  }
  Serial.println(F("Center confirmed. Forward movement enabled."));

  // 3. Lock onto this target. Do not request another camera result
  //    until the next tag. The tag may leave the camera view up close.
  Serial.println(F("Target locked. Approaching using ultrasonic sensor."));
  if (!USE_ULTRASONIC) {
    Serial.println(F("ERROR: Ultrasonic sensor must be enabled."));
    stopDriving(); return -2;
  }
  // 4. Run one continuous approach. No driveDistance() calls or intermediate
  //    braking; PID stays active while the ultrasonic sensor measures clearance.
  const float ticks_per_cm = ENCODER_TICKS_PER_WHEEL_REV / (PI * WHEEL_DIAMETER_CM);
  const long start_l = enc_l.read();
  const long start_r = enc_r.read();
  long furthest_l = 0;
  long furthest_r = 0;
  resetPID();
  start_time = millis();
  unsigned long last_progress_l_ms = start_time;
  unsigned long last_progress_r_ms = start_time;
  unsigned long last_range_ms = start_time - RANGE_SAMPLE_INTERVAL_MS;
  unsigned long last_report_ms = start_time - RANGE_REPORT_INTERVAL_MS;
  unsigned long last_speed_ms = start_time;
  float measured = -1.0f;
  float commanded_speed = FINAL_SPEED_CM_S;
  float desired_speed = FINAL_SPEED_CM_S;

  while (millis() - start_time < APPROACH_TIMEOUT_MS) {
    updateOdometry();
    unsigned long now_ms = millis();

    // Read ultrasonic clearance approximately every 60 ms while moving.
    // Stop immediately on an invalid echo rather than driving without range data.
    if (now_ms - last_range_ms >= RANGE_SAMPLE_INTERVAL_MS) {
      last_range_ms = now_ms;
      measured = ultrasonicRangeCM();
      if (measured < 2.0f) {
        stopDriving();
        Serial.println(F("ERROR: No valid ultrasonic echo."));
        return -2;
      }
      if (measured <= MAX_FINAL_CLEARANCE_CM) {
        stopDriving();
        Serial.print(F("Arrival confirmed at "));
        Serial.print(measured); Serial.println(F(" cm"));
        return 1;
      }

      // Reduce speed smoothly from 20 cm/s at 40 cm to 5 cm/s at 15 cm.
      float fraction = constrain((measured - FINAL_APPROACH_CM) /
          (SLOWDOWN_RANGE_CM - FINAL_APPROACH_CM), 0.0f, 1.0f);
      desired_speed = FINAL_SPEED_CM_S +
          fraction * (APPROACH_SPEED_CM_S - FINAL_SPEED_CM_S);
      if (now_ms - last_report_ms >= RANGE_REPORT_INTERVAL_MS) {
        last_report_ms = now_ms;
        Serial.print(F("Ultrasonic distance: ")); Serial.println(measured);
      }
    }

    // 5. Keep the original encoder stall and travel-limit protection.
    now_ms = millis();
    const long progress_l = enc_l.read() - start_l;
    const long progress_r = enc_r.read() - start_r;
    if (progress_l > furthest_l) {
      furthest_l = progress_l;
      last_progress_l_ms = now_ms;
    }
    if (progress_r > furthest_r) {
      furthest_r = progress_r;
      last_progress_r_ms = now_ms;
    }
    if (abs(progress_l) / ticks_per_cm >= MAX_APPROACH_TRAVEL_CM ||
        abs(progress_r) / ticks_per_cm >= MAX_APPROACH_TRAVEL_CM ||
        now_ms - last_progress_l_ms >= NO_PROGRESS_TIMEOUT_MS ||
        now_ms - last_progress_r_ms >= NO_PROGRESS_TIMEOUT_MS) {
      stopDriving();
      Serial.println(F("ERROR: Approach travel limit or encoder stall."));
      return -2;
    }

    // 6. Ramp up gradually; reduce speed immediately when approaching the tag.
    float dt = (now_ms - last_speed_ms) / 1000.0f;
    last_speed_ms = now_ms;
    commanded_speed = min(desired_speed,
                          commanded_speed + SLEW_ACCEL_CM_S2 * dt);
    long speed_ticks = max(1L, (long)(commanded_speed * ticks_per_cm));
    updatePID(speed_ticks, speed_ticks, false);
    delay(1);
  }

  stopDriving();
  Serial.println(F("ERROR: Approach timeout."));
  return -2;
}
