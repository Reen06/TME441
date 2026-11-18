/* camera.ino
*  Functions for communicating with the HuskyLens camera
*
*  Starter author: Matthew Ford
*/

// --- Camera calibration: replace provisional values using measurements ---
const float CAMERA_CENTER_X = 160.0f;  // pixels, nominal frame center
const float CAMERA_FOCAL_X = 250.0f;   // pixels, provisional; measure on your robot
const float RANGE_SCALE = 3489.0f;     // cm * pixels, instructor's initial fit
const float RANGE_OFFSET = -3.0f;      // cm, instructor's initial fit

// Print a HuskyLens result
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println(F("Object unknown!"));
  }
}

/* Estimate range and heading to a tag from the same camera result.
*
*  @param tag_id ID of the desired tag
*  @return TagPose: status 1 = found, 0 = not found, -1 = error
*/
TagPose rangeAndHeadingToTag(int tag_id) {
  TagPose pose = {0, 0.0f, 0.0f};

  if (!cam.request()) {
    pose.status = -1;  // Communication error
    return pose;
  }

  while (cam.available()) {
    HUSKYLENSResult result = cam.read();
    if (result.command != COMMAND_RETURN_BLOCK || result.ID != tag_id) {
      continue;
    }

    if (result.height <= 0 || CAMERA_FOCAL_X <= 0.0f || RANGE_SCALE <= 0.0f) {
      pose.status = -1;
      return pose;
    }

    // Positive heading turns left (CCW); negative turns right (CW).
    float heading_rad = atan((CAMERA_CENTER_X - result.xCenter) / CAMERA_FOCAL_X);
    pose.heading = heading_rad * 180.0f / PI;

    // Height-based fit gives approximate forward distance.
    // Calibrate the fit using the front of the ultrasonic sensor as reference.
    float forward_range = RANGE_SCALE / result.height + RANGE_OFFSET;
    if (forward_range <= 0.0f) {
      pose.status = -1;
      return pose;
    }

    // Off-axis slant range estimate; assumes aligned camera/sensor axes.
    // Their actual origins differ, so remeasure after turning before driving.
    pose.range = forward_range / cos(heading_rad);
    pose.status = 1;

    if (CALIBRATION_MODE) {
      printResult(result);
      Serial.print(F("range_cm=")); Serial.print(pose.range);
      Serial.print(F(", heading_deg=")); Serial.println(pose.heading);
    }
    return pose;
  }

  return pose;  // Target tag not found
}
