/* Milestone 2: bounded search, same-frame geometry, monitored encoder moves.
 * driveToTag(int): 1 = arrived, -1 = search timeout, -2 = other fault.
 */
enum NavigationMotion { SEARCH_MOTION, ALIGN_MOTION, APPROACH_MOTION, FINAL_MOTION };
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
  const float search_wheel_speed =
      SEARCH_TURN_SPEED_DEG_S * PI * TRACK_WIDTH_CM / 360.0f;
  return drivingSettingsValid() &&
      isfinite(SEARCH_TURN_SPEED_DEG_S) && SEARCH_TURN_SPEED_DEG_S > 0.0f &&
      search_wheel_speed >= MIN_MOTION_SPEED_CM_S &&
      (SEARCH_DIRECTION == 1 || SEARCH_DIRECTION == -1) &&
      STOP_TARGET_CM > 0.0f && STOP_TARGET_CM < MAX_STOP_CLEARANCE_CM &&
      MAX_STOP_CLEARANCE_CM <= 10.0f &&
      isfinite(CAMERA_TO_STOP_REFERENCE_CM) && CAMERA_TO_STOP_REFERENCE_CM >= 0.0f &&
      isfinite(APPROACH_SPEED_CM_S) && APPROACH_SPEED_CM_S >= MIN_MOTION_SPEED_CM_S &&
      APPROACH_SPEED_CM_S <= DRIVE_SPEED_CM_S &&
      isfinite(MAX_APPROACH_STEP_CM) && MAX_APPROACH_STEP_CM > 0.0f &&
      isfinite(MAX_FINAL_APPROACH_CM) && MAX_FINAL_APPROACH_CM > 0.0f &&
      MAX_FINAL_APPROACH_CM <= 5.0f &&
      isfinite(ALIGN_TOLERANCE_DEG) && ALIGN_TOLERANCE_DEG > 0.0f &&
      isfinite(RANGE_HEIGHT_SCALE_CM_PX) && RANGE_HEIGHT_SCALE_CM_PX > 0.0f &&
      isfinite(RANGE_OFFSET_CM) &&
      isfinite(CAMERA_CENTER_X_PX) && CAMERA_CENTER_X_PX > 0.0f && CAMERA_CENTER_X_PX < 319.0f &&
      isfinite(CAMERA_FOCAL_LENGTH_PX) && CAMERA_FOCAL_LENGTH_PX > 0.0f &&
      CAMERA_POLL_INTERVAL_MS > 0 && CAMERA_TIMEOUT_MS > 0 && CAMERA_RETRY_COUNT > 0 &&
      TAG_SEARCH_TIMEOUT_MS > 0 && TAG_NAVIGATION_TIMEOUT_MS > TAG_SEARCH_TIMEOUT_MS &&
      FIRST_TAG_ID > 0 && LAST_TAG_ID >= FIRST_TAG_ID && LAST_TAG_ID <= 32767;
}

float tagClearanceCm() {
  return watched_pose.range_cm - CAMERA_TO_STOP_REFERENCE_CM;
}

bool tagIsCloseEnough() {
  return watched_pose.status == TAG_VISIBLE &&
      tagClearanceCm() <= MAX_STOP_CLEARANCE_CM &&
      abs(watched_pose.heading_deg) <= ALIGN_TOLERANCE_DEG;
}

// Every error retry happens with the motors braked.
TagPose readTagWithRetries(int tag_id) {
  TagPose pose = {};
  for (unsigned int attempt = 0; attempt < CAMERA_RETRY_COUNT; ++attempt) {
    pose = rangeAndHeadingToTag(tag_id);
    if (pose.status != TAG_ERROR) return pose;
    stopDriving();
    Serial.print(F("[camera] failed read, target="));
    Serial.print(tag_id);
    Serial.print(F(" attempt="));
    Serial.println(attempt + 1);
    delay(CAMERA_POLL_INTERVAL_MS);
  }
  return pose;
}

// false always stops the current move before the route decides what to do next.
bool monitorTagMotion() {
  if (millis() - navigation_started_ms >= TAG_NAVIGATION_TIMEOUT_MS) {
    monitor_result = TAG_NAVIGATION_ERROR;
    return false;
  }
  if (navigation_motion == SEARCH_MOTION &&
      millis() - search_started_ms >= TAG_SEARCH_TIMEOUT_MS) {
    monitor_result = TAG_SEARCH_FAILED;
    return false;
  }
  if (millis() - last_camera_ms < CAMERA_POLL_INTERVAL_MS) return true;
  watched_pose = rangeAndHeadingToTag(watched_tag_id);
  last_camera_ms = millis();
  if (watched_pose.status == TAG_ERROR) return false;
  if (navigation_motion == SEARCH_MOTION) return watched_pose.status != TAG_VISIBLE;
  if (watched_pose.status != TAG_VISIBLE) {
    // Only a pre-planned, <=5 cm final move may outlast camera visibility.
    return navigation_motion == FINAL_MOTION;
  }
  if (tagIsCloseEnough()) return false;
  if (navigation_motion == ALIGN_MOTION) {
    return abs(watched_pose.heading_deg) > ALIGN_TOLERANCE_DEG &&
        watched_pose.heading_deg * initial_heading_deg > 0.0f;
  }
  return abs(watched_pose.heading_deg) <= ALIGN_TOLERANCE_DEG;
}

int finishTagNavigation(int status) {
  setMotionMonitor(0);
  stopDriving();
  Serial.print(F("[tag] ID="));
  Serial.print(watched_tag_id);
  Serial.print(F(" status="));
  Serial.println(status);
  return status;
}

int driveToTag(int tag_id) {
  watched_tag_id = tag_id;
  setMotionMonitor(0);
  stopDriving();
  if (tag_id <= 0 || !navigationSettingsValid()) {
    return finishTagNavigation(TAG_NAVIGATION_ERROR);
  }
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
    if (watched_pose.status == TAG_ERROR) {
      Serial.println(F("[fault] persistent camera error"));
      return finishTagNavigation(TAG_NAVIGATION_ERROR);
    }
    if (tagIsCloseEnough()) return finishTagNavigation(TAG_REACHED);

    bool completed = false;
    bool final_move = false;
    monitor_result = 0;
    if (watched_pose.status != TAG_VISIBLE) {
      if (!searching) {
        searching = true;
        search_started_ms = millis();
        Serial.print(F("[search] target="));
        Serial.println(tag_id);
      }
      if (millis() - search_started_ms >= TAG_SEARCH_TIMEOUT_MS) {
        return finishTagNavigation(TAG_SEARCH_FAILED);
      }
      navigation_motion = SEARCH_MOTION;
      setMotionMonitor(monitorTagMotion);
      completed = rotateContinuously(SEARCH_DIRECTION * SEARCH_TURN_SPEED_DEG_S);
    } else {
      searching = false;
      Serial.print(F("[pose] target="));
      Serial.print(tag_id);
      Serial.print(F(" clearance_cm="));
      Serial.print(tagClearanceCm());
      Serial.print(F(" heading_deg="));
      Serial.println(watched_pose.heading_deg);
      setMotionMonitor(monitorTagMotion);
      if (abs(watched_pose.heading_deg) > ALIGN_TOLERANCE_DEG) {
        navigation_motion = ALIGN_MOTION;
        initial_heading_deg = watched_pose.heading_deg;
        completed = rotateCCW(initial_heading_deg);
      } else {
        const float remaining_cm = tagClearanceCm() - STOP_TARGET_CM;
        final_move = remaining_cm <= MAX_FINAL_APPROACH_CM;
        navigation_motion = final_move ? FINAL_MOTION : APPROACH_MOTION;
        const float step_cm = final_move ? remaining_cm : min(remaining_cm, MAX_APPROACH_STEP_CM);
        Serial.print(final_move ? F("[final] encoder distance_cm=") : F("[approach] step_cm="));
        Serial.println(step_cm);
        completed = driveDistance(step_cm, APPROACH_SPEED_CM_S);
      }
    }
    setMotionMonitor(0);
    if (monitor_result != 0) return finishTagNavigation(monitor_result);
    if (!completed && !wasMotionInterrupted()) {
      Serial.println(F("[fault] movement stalled or timed out"));
      return finishTagNavigation(TAG_NAVIGATION_ERROR);
    }
    if (final_move && completed) {
      Serial.println(F("[arrival] final encoder target reached from last valid camera pose"));
      return finishTagNavigation(TAG_REACHED);
    }
    // A camera error, lost tag, or changed heading interrupted a move while
    // braking. Re-read/retry stopped, then search or align for the SAME target.
  }
}

int runMilestone2() {
  for (int tag_id = FIRST_TAG_ID; tag_id <= LAST_TAG_ID; ++tag_id) {
    Serial.print(F("[route] starting target="));
    Serial.println(tag_id);
    const int status = driveToTag(tag_id);
    stopDriving();
    if (status != TAG_REACHED) {
      Serial.print(F("[route] HALTED at target="));
      Serial.print(tag_id);
      Serial.print(F(" status="));
      Serial.println(status);
      return status;  // Never skip an unvisited tag.
    }
    Serial.print(F("[route] reached target="));
    Serial.println(tag_id);
    if (tag_id < LAST_TAG_ID) delay(TAG_PAUSE_MS);
  }
  Serial.println(F("[route] COMPLETE: tags 1 -> 2 -> 3. Motors braked until reset."));
  return TAG_REACHED;
}

void printCameraCalibrationSample() {
  const TagPose pose = rangeAndHeadingToTag(CALIBRATION_TAG_ID);
  Serial.print(millis()); Serial.print(',');
  Serial.print(CALIBRATION_TAG_ID); Serial.print(',');
  Serial.print((int)pose.status); Serial.print(',');
  Serial.print(pose.x_px); Serial.print(',');
  Serial.print(pose.height_px); Serial.print(',');
  Serial.print(pose.range_cm); Serial.print(',');
  Serial.println(pose.heading_deg);
}
