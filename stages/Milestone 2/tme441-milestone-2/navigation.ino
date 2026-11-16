/* Milestone 2: camera locks the tag; HC-SR04 controls the close approach.
 * driveToTag(int): 1 = arrived, -1 = search timeout, -2 = other fault.
 */
enum NavigationMotion { SEARCH_MOTION, ALIGN_MOTION, APPROACH_MOTION, ULTRASONIC_MOTION };
NavigationMotion navigation_motion = SEARCH_MOTION;
int watched_tag_id = FIRST_TAG_ID;
TagPose watched_pose = {};
float initial_heading_deg = 0.0f;
unsigned long last_camera_ms = 0;
unsigned long navigation_started_ms = 0;
unsigned long search_started_ms = 0;
bool searching = false;
int monitor_result = 0;

bool navigationSettingsValid() {
  const float search_wheel_speed = SEARCH_TURN_SPEED_DEG_S * PI * TRACK_WIDTH_CM / 360.0f;
  return drivingSettingsValid() &&
      isfinite(SEARCH_TURN_SPEED_DEG_S) && SEARCH_TURN_SPEED_DEG_S > 0.0f &&
      search_wheel_speed >= MIN_MOTION_SPEED_CM_S &&
      (SEARCH_DIRECTION == 1 || SEARCH_DIRECTION == -1) &&
      STOP_TARGET_CM > ULTRASONIC_MIN_CM && STOP_TARGET_CM < MAX_STOP_CLEARANCE_CM &&
      MAX_STOP_CLEARANCE_CM <= 10.0f &&
      isfinite(CAMERA_TO_STOP_REFERENCE_CM) &&
      isfinite(APPROACH_SPEED_CM_S) && APPROACH_SPEED_CM_S >= MIN_MOTION_SPEED_CM_S &&
      APPROACH_SPEED_CM_S <= DRIVE_SPEED_CM_S &&
      isfinite(MAX_APPROACH_STEP_CM) && MAX_APPROACH_STEP_CM > 0.0f &&
      isfinite(CAMERA_MIN_VISIBLE_CM) && CAMERA_MIN_VISIBLE_CM > MAX_STOP_CLEARANCE_CM &&
      isfinite(CAMERA_ULTRASONIC_HANDOFF_CM) && CAMERA_ULTRASONIC_HANDOFF_CM > CAMERA_MIN_VISIBLE_CM &&
      isfinite(HANDOFF_RANGE_TOLERANCE_CM) && HANDOFF_RANGE_TOLERANCE_CM > 0.0f &&
      isfinite(ULTRASONIC_APPROACH_SPEED_CM_S) && ULTRASONIC_APPROACH_SPEED_CM_S >= MIN_MOTION_SPEED_CM_S &&
      ULTRASONIC_APPROACH_SPEED_CM_S <= APPROACH_SPEED_CM_S &&
      isfinite(ULTRASONIC_APPROACH_STEP_CM) && ULTRASONIC_APPROACH_STEP_CM > 0.0f && ULTRASONIC_APPROACH_STEP_CM <= 1.0f &&
      isfinite(ULTRASONIC_STOP_CONFIRM_CM) && ULTRASONIC_STOP_CONFIRM_CM >= STOP_TARGET_CM &&
      ULTRASONIC_STOP_CONFIRM_CM < MAX_STOP_CLEARANCE_CM &&
      isfinite(ULTRASONIC_MAX_APPROACH_TRAVEL_CM) && ULTRASONIC_MAX_APPROACH_TRAVEL_CM > 0.0f &&
      isfinite(ULTRASONIC_TRAVEL_MARGIN_CM) && ULTRASONIC_TRAVEL_MARGIN_CM >= 0.0f &&
      ULTRASONIC_TRIG_PIN >= 0 && ULTRASONIC_TRIG_PIN < 70 &&
      ULTRASONIC_ECHO_PIN >= 0 && ULTRASONIC_ECHO_PIN < 70 && ULTRASONIC_TRIG_PIN != ULTRASONIC_ECHO_PIN &&
      ULTRASONIC_PING_INTERVAL_MS > 60UL && ULTRASONIC_ECHO_TIMEOUT_US > 0 && ULTRASONIC_ECHO_TIMEOUT_US <= 20000UL &&
      ULTRASONIC_FRESHNESS_MS >= ULTRASONIC_PING_INTERVAL_MS &&
      ULTRASONIC_CONFIRM_TIMEOUT_MS >= 5 * ULTRASONIC_PING_INTERVAL_MS &&
      isfinite(ULTRASONIC_MIN_CM) && ULTRASONIC_MIN_CM >= 2.0f &&
      isfinite(ULTRASONIC_MAX_CM) && ULTRASONIC_MAX_CM > CAMERA_ULTRASONIC_HANDOFF_CM &&
      isfinite(ULTRASONIC_STABLE_SPREAD_CM) && ULTRASONIC_STABLE_SPREAD_CM > 0.0f &&
      isfinite(ALIGN_TOLERANCE_DEG) && ALIGN_TOLERANCE_DEG > 0.0f &&
      isfinite(RANGE_HEIGHT_SCALE_CM_PX) && RANGE_HEIGHT_SCALE_CM_PX > 0.0f &&
      isfinite(RANGE_OFFSET_CM) && isfinite(RANGE_BIAS_CORRECTION_CM) &&
      isfinite(CAMERA_CENTER_X_PX) && CAMERA_CENTER_X_PX > 0.0f && CAMERA_CENTER_X_PX < 319.0f &&
      isfinite(CAMERA_FOCAL_LENGTH_PX) && CAMERA_FOCAL_LENGTH_PX > 0.0f &&
      CAMERA_POLL_INTERVAL_MS > 0 && CAMERA_TIMEOUT_MS > 0 && CAMERA_RETRY_COUNT > 0 &&
      TAG_SEARCH_TIMEOUT_MS > 0 && TAG_NAVIGATION_TIMEOUT_MS > TAG_SEARCH_TIMEOUT_MS &&
      FIRST_TAG_ID > 0 && LAST_TAG_ID >= FIRST_TAG_ID && LAST_TAG_ID < 32767;
}

float tagClearanceCm() {
  return watched_pose.range_cm - CAMERA_TO_STOP_REFERENCE_CM;
}

TagPose readTagWithRetries(int tag_id) {
  TagPose pose = {};
  for (unsigned int attempt = 0; attempt < CAMERA_RETRY_COUNT; ++attempt) {
    pose = rangeAndHeadingToTag(tag_id);
    if (pose.status != TAG_ERROR) return pose;
    stopDriving();
    Serial.print(F("[camera] failed read, target=")); Serial.print(tag_id);
    Serial.print(F(" attempt=")); Serial.println(attempt + 1);
    delay(CAMERA_POLL_INTERVAL_MS);
  }
  return pose;
}

bool monitorTagMotion() {
  if (millis() - navigation_started_ms >= TAG_NAVIGATION_TIMEOUT_MS) {
    monitor_result = TAG_NAVIGATION_ERROR;
    return false;
  }
  if (navigation_motion == SEARCH_MOTION && millis() - search_started_ms >= TAG_SEARCH_TIMEOUT_MS) {
    monitor_result = TAG_SEARCH_FAILED;
    return false;
  }
  if (navigation_motion == ULTRASONIC_MOTION) {
    const UltrasonicReading reading = readUltrasonic();
    if (!ultrasonicReadingFresh(reading)) {
      monitor_result = TAG_NAVIGATION_ERROR;
      return false;
    }
    return reading.distance_cm > STOP_TARGET_CM;
  }
  if (millis() - last_camera_ms < CAMERA_POLL_INTERVAL_MS) return true;
  watched_pose = rangeAndHeadingToTag(watched_tag_id);
  last_camera_ms = millis();
  if (watched_pose.status == TAG_ERROR) return false;
  if (navigation_motion == SEARCH_MOTION) return watched_pose.status != TAG_VISIBLE;
  if (watched_pose.status != TAG_VISIBLE) return false;
  if (navigation_motion == ALIGN_MOTION) {
    return abs(watched_pose.heading_deg) > ALIGN_TOLERANCE_DEG &&
        watched_pose.heading_deg * initial_heading_deg > 0.0f;
  }
  // Do not count an unrelated close obstacle as arrival at a distant tag.
  const UltrasonicReading obstacle = readUltrasonic();
  if (ultrasonicReadingFresh(obstacle) && obstacle.distance_cm <= MAX_STOP_CLEARANCE_CM) {
    monitor_result = TAG_NAVIGATION_ERROR;
    return false;
  }
  return abs(watched_pose.heading_deg) <= ALIGN_TOLERANCE_DEG &&
      tagClearanceCm() > CAMERA_ULTRASONIC_HANDOFF_CM;
}

int finishTagNavigation(int status) {
  setMotionMonitor(0);
  stopDriving();
  Serial.print(F("[tag] ID=")); Serial.print(watched_tag_id);
  Serial.print(F(" status=")); Serial.println(status);
  return status;
}

int endUltrasonicApproach(int status) {
  setMotionMonitor(0);
  stopDriving();
  return status;
}

// Internal status 0 requests camera re-alignment of the SAME target.
int driveUltrasonicApproach(float camera_clearance_cm) {
  stopDriving();
  UltrasonicReading range = readStableUltrasonic();
  if (!ultrasonicReadingFresh(range)) {
    Serial.println(F("[fault] no stable HC-SR04 reading at handoff"));
    return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
  }
  if (abs(range.distance_cm - camera_clearance_cm) > HANDOFF_RANGE_TOLERANCE_CM) {
    Serial.println(F("[fault] camera/ultrasonic distances disagree; possible different obstacle"));
    return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
  }
  Serial.print(F("[handoff] target=")); Serial.print(watched_tag_id);
  Serial.print(F(" camera_cm=")); Serial.print(camera_clearance_cm);
  Serial.print(F(" ultrasonic_cm=")); Serial.println(range.distance_cm);
  const long start_l = enc_l.read(), start_r = enc_r.read();
  const float cm_per_tick = PI * WHEEL_DIAMETER_CM / ENCODER_TICKS_PER_WHEEL_REV;
  const float travel_budget = min(ULTRASONIC_MAX_APPROACH_TRAVEL_CM,
      max(0.0f, range.distance_cm - STOP_TARGET_CM) + ULTRASONIC_TRAVEL_MARGIN_CM);

  while (true) {
    if (millis() - navigation_started_ms >= TAG_NAVIGATION_TIMEOUT_MS) {
      Serial.println(F("[fault] ultrasonic approach deadline"));
      return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    }
    // Camera queries and multi-ping confirmation happen with motors braked.
    watched_pose = readTagWithRetries(watched_tag_id);
    last_camera_ms = millis();
    if (watched_pose.status == TAG_ERROR) return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    if (watched_pose.status == TAG_VISIBLE && abs(watched_pose.heading_deg) > ALIGN_TOLERANCE_DEG) {
      Serial.println(F("[handoff] visible tag drifted; re-align before continuing"));
      return endUltrasonicApproach(0);
    }
    range = readStableUltrasonic();
    if (!ultrasonicReadingFresh(range) || millis() - navigation_started_ms >= TAG_NAVIGATION_TIMEOUT_MS) {
      Serial.println(F("[fault] invalid ultrasonic confirmation or deadline"));
      return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    }
    if (watched_pose.status == TAG_VISIBLE &&
        abs(range.distance_cm - tagClearanceCm()) > HANDOFF_RANGE_TOLERANCE_CM) {
      Serial.println(F("[fault] visible tag and ultrasonic ranges diverged"));
      return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    }
    Serial.print(F("[ultrasonic] target=")); Serial.print(watched_tag_id);
    Serial.print(F(" clearance_cm=")); Serial.println(range.distance_cm);
    if (range.distance_cm <= ULTRASONIC_STOP_CONFIRM_CM && range.maximum_cm <= MAX_STOP_CLEARANCE_CM) {
      Serial.println(F("[arrival] confirmed by three ultrasonic pings, motors braked"));
      return endUltrasonicApproach(TAG_REACHED);
    }
    const float travelled_cm =
        (abs(enc_l.read() - start_l) + abs(enc_r.read() - start_r)) * 0.5f * cm_per_tick;
    const float budget_left_cm = travel_budget - travelled_cm;
    if (budget_left_cm <= 0.0f) {
      Serial.println(F("[fault] ultrasonic travel budget exhausted; range not decreasing"));
      return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    }
    const float step_cm = min(ULTRASONIC_APPROACH_STEP_CM,
        min(range.distance_cm - STOP_TARGET_CM, budget_left_cm));
    navigation_motion = ULTRASONIC_MOTION;
    monitor_result = 0;
    setMotionMonitor(monitorTagMotion);
    const bool completed = driveDistance(step_cm, ULTRASONIC_APPROACH_SPEED_CM_S);
    setMotionMonitor(0);
    if (monitor_result != 0 || (!completed && !wasMotionInterrupted())) {
      Serial.println(F("[fault] ultrasonic echo, deadline, or encoder failure during close movement"));
      return endUltrasonicApproach(TAG_NAVIGATION_ERROR);
    }
    // A low reading only interrupts movement. The next stationary three-ping
    // median must confirm arrival before this target is marked reached.
  }
}

int driveToTag(int tag_id) {
  watched_tag_id = tag_id;
  setMotionMonitor(0);
  stopDriving();
  if (tag_id <= 0 || !navigationSettingsValid()) return finishTagNavigation(TAG_NAVIGATION_ERROR);
  navigation_started_ms = millis();
  searching = false;
  monitor_result = 0;
  while (true) {
    if (millis() - navigation_started_ms >= TAG_NAVIGATION_TIMEOUT_MS) {
      Serial.println(F("[fault] navigation deadline exceeded"));
      return finishTagNavigation(TAG_NAVIGATION_ERROR);
    }
    watched_pose = readTagWithRetries(tag_id);
    last_camera_ms = millis();
    if (watched_pose.status == TAG_ERROR) return finishTagNavigation(TAG_NAVIGATION_ERROR);
    bool completed = false;
    monitor_result = 0;
    if (watched_pose.status != TAG_VISIBLE) {
      if (!searching) {
        searching = true; search_started_ms = millis();
        Serial.print(F("[search] target=")); Serial.println(tag_id);
      }
      if (millis() - search_started_ms >= TAG_SEARCH_TIMEOUT_MS) return finishTagNavigation(TAG_SEARCH_FAILED);
      navigation_motion = SEARCH_MOTION;
      setMotionMonitor(monitorTagMotion);
      completed = rotateContinuously(SEARCH_DIRECTION * SEARCH_TURN_SPEED_DEG_S);
    } else {
      searching = false;
      Serial.print(F("[pose] target=")); Serial.print(tag_id);
      Serial.print(F(" clearance_cm=")); Serial.print(tagClearanceCm());
      Serial.print(F(" heading_deg=")); Serial.println(watched_pose.heading_deg);
      if (abs(watched_pose.heading_deg) > ALIGN_TOLERANCE_DEG) {
        navigation_motion = ALIGN_MOTION;
        initial_heading_deg = watched_pose.heading_deg;
        setMotionMonitor(monitorTagMotion);
        completed = rotateCCW(initial_heading_deg);
      } else if (tagClearanceCm() <= CAMERA_ULTRASONIC_HANDOFF_CM) {
        const int close_status = driveUltrasonicApproach(tagClearanceCm());
        if (close_status == 0) continue;
        return finishTagNavigation(close_status);
      } else {
        navigation_motion = APPROACH_MOTION;
        setMotionMonitor(monitorTagMotion);
        const float step_cm = min(MAX_APPROACH_STEP_CM, tagClearanceCm() - CAMERA_ULTRASONIC_HANDOFF_CM);
        completed = driveDistance(step_cm, APPROACH_SPEED_CM_S);
      }
    }
    setMotionMonitor(0);
    if (monitor_result != 0) return finishTagNavigation(monitor_result);
    if (!completed && !wasMotionInterrupted()) return finishTagNavigation(TAG_NAVIGATION_ERROR);
  }
}

int runMilestone2() {
  for (int tag_id = FIRST_TAG_ID; tag_id <= LAST_TAG_ID; ++tag_id) {
    Serial.print(F("[route] starting target=")); Serial.println(tag_id);
    const int status = driveToTag(tag_id);
    stopDriving();
    if (status != TAG_REACHED) {
      Serial.print(F("[route] HALTED at target=")); Serial.print(tag_id);
      Serial.print(F(" status=")); Serial.println(status);
      return status;
    }
    Serial.print(F("[route] reached target=")); Serial.println(tag_id);
    if (tag_id < LAST_TAG_ID) delay(TAG_PAUSE_MS);
  }
  Serial.println(F("[route] COMPLETE: tags 1 -> 2 -> 3. Motors braked until reset."));
  return TAG_REACHED;
}

void printCameraCalibrationSample() {
  const TagPose pose = rangeAndHeadingToTag(CALIBRATION_TAG_ID);
  const UltrasonicReading range = readUltrasonic();
  Serial.print(millis()); Serial.print(',');
  Serial.print(CALIBRATION_TAG_ID); Serial.print(',');
  Serial.print((int)pose.status); Serial.print(',');
  Serial.print(pose.x_px); Serial.print(','); Serial.print(pose.height_px); Serial.print(',');
  Serial.print(pose.range_cm); Serial.print(','); Serial.print(pose.heading_deg); Serial.print(',');
  Serial.print((int)range.status); Serial.print(','); Serial.println(range.distance_cm);
}
