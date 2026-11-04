/* Search continuously, turn by camera bearing, then approach in short V2 moves.
 * The motion monitor also reads the camera DURING each move.
 * driveToTag returns 1 = reached stopping distance, -2 = camera/movement fault.
 */

enum NavigationMotion { SEARCH_MOTION, ALIGN_MOTION, APPROACH_MOTION };
NavigationMotion navigation_motion = SEARCH_MOTION;
int watched_tag_id = FIRST_TAG_ID;
TagPose watched_pose = {};
float initial_bearing_deg = 0.0f;
unsigned long last_camera_ms = 0;

bool navigationSettingsValid() {
  return isfinite(SEARCH_TURN_SPEED_DEG_S) && SEARCH_TURN_SPEED_DEG_S > 0.0f &&
      (SEARCH_DIRECTION == 1 || SEARCH_DIRECTION == -1) &&
      isfinite(STOP_DISTANCE_CM) && STOP_DISTANCE_CM > 0.0f &&
      isfinite(STOP_MARGIN_CM) && STOP_MARGIN_CM >= 0.0f &&
      isfinite(CAMERA_TO_FRONT_CM) && CAMERA_TO_FRONT_CM >= 0.0f &&
      isfinite(APPROACH_SPEED_CM_S) && APPROACH_SPEED_CM_S > 0.0f &&
      isfinite(MAX_APPROACH_STEP_CM) && MAX_APPROACH_STEP_CM > 0.0f &&
      isfinite(ALIGN_TOLERANCE_DEG) && ALIGN_TOLERANCE_DEG > 0.0f &&
      isfinite(RANGE_HEIGHT_SCALE_CM_PX) && RANGE_HEIGHT_SCALE_CM_PX > 0.0f &&
      isfinite(RANGE_OFFSET_CM) &&
      isfinite(CAMERA_CENTER_X_PX) && CAMERA_CENTER_X_PX > 0.0f &&
      CAMERA_CENTER_X_PX < 319.0f &&
      isfinite(CAMERA_FOCAL_LENGTH_PX) && CAMERA_FOCAL_LENGTH_PX > 0.0f &&
      CAMERA_POLL_INTERVAL_MS > 0 && CAMERA_TIMEOUT_MS > 0 &&
      FIRST_TAG_ID > 0 && LAST_TAG_ID >= FIRST_TAG_ID;
}

float tagClearanceCm() {
  return watched_pose.range_cm - CAMERA_TO_FRONT_CM;
}

bool tagIsCloseEnough() {
  return watched_pose.status == TAG_VISIBLE &&
      tagClearanceCm() <= STOP_DISTANCE_CM + STOP_MARGIN_CM;
}

// Returning false interrupts the current move and brakes both wheels.
bool monitorTagMotion() {
  if (millis() - last_camera_ms < CAMERA_POLL_INTERVAL_MS) {
    return true;
  }
  watched_pose = readTagPose(watched_tag_id);
  last_camera_ms = millis();
  if (watched_pose.status == TAG_ERROR) {
    return false;
  }
  if (navigation_motion == SEARCH_MOTION) {
    return watched_pose.status == TAG_MISSING;
  }
  if (watched_pose.status != TAG_VISIBLE || tagIsCloseEnough()) {
    return false;
  }
  if (navigation_motion == ALIGN_MOTION) {
    // Stop early when centered or when the measured bearing crosses zero.
    return abs(watched_pose.bearing_deg) > ALIGN_TOLERANCE_DEG &&
        watched_pose.bearing_deg * initial_bearing_deg > 0.0f;
  }
  return abs(watched_pose.bearing_deg) <= ALIGN_TOLERANCE_DEG;
}

int driveToTag(int tag_id) {
  watched_tag_id = tag_id;
  while (true) {
    watched_pose = readTagPose(tag_id);
    last_camera_ms = millis();
    if (watched_pose.status == TAG_ERROR) {
      stopDriving();
      return -2;
    }
    if (tagIsCloseEnough()) {
      stopDriving();
      return 1;
    }

    bool completed = false;
    setMotionMonitor(monitorTagMotion);
    if (watched_pose.status == TAG_MISSING) {
      navigation_motion = SEARCH_MOTION;
      Serial.println(F("Searching continuously for tag."));
      completed = rotateContinuously(SEARCH_DIRECTION * SEARCH_TURN_SPEED_DEG_S);
    } else if (abs(watched_pose.bearing_deg) > ALIGN_TOLERANCE_DEG) {
      navigation_motion = ALIGN_MOTION;
      initial_bearing_deg = watched_pose.bearing_deg;
      Serial.print(F("Aligning, angle deg: "));
      Serial.println(initial_bearing_deg);
      completed = rotateCCW(initial_bearing_deg);
    } else {
      navigation_motion = APPROACH_MOTION;
      const float remaining_cm =
          tagClearanceCm() - STOP_DISTANCE_CM - STOP_MARGIN_CM;
      const float step_cm = min(remaining_cm, MAX_APPROACH_STEP_CM);
      Serial.print(F("Approaching, clearance cm: "));
      Serial.print(tagClearanceCm());
      Serial.print(F(", step cm: "));
      Serial.println(step_cm);
      completed = driveDistance(step_cm, APPROACH_SPEED_CM_S);
    }
    setMotionMonitor(0);

    if (watched_pose.status == TAG_ERROR ||
        (!completed && !wasMotionInterrupted())) {
      stopDriving();
      return -2;
    }
    // Re-read after braking: lost tag -> search; changed bearing -> align;
    // enough clearance -> another bounded step; close enough -> final stop.
  }
}
