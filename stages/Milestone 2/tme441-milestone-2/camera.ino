/* One camera frame supplies the requested tag's range AND heading.
 * Partial detections are unusable for range, but are not communication errors.
 */
TagPose rangeAndHeadingToTag(int tag_id) {
  TagPose pose = {};
  pose.status = TAG_ERROR;
  if (tag_id <= 0 || !cam.requestBlocks(tag_id)) {
    return pose;
  }

  pose.status = TAG_MISSING;
  while (cam.available()) {
    const HUSKYLENSResult result = cam.read();
    if (result.command != COMMAND_RETURN_BLOCK || result.ID != tag_id) {
      continue;
    }
    if (result.xCenter < 0 || result.xCenter >= 320 ||
        result.yCenter < 0 || result.yCenter >= 240 ||
        result.width <= 0 || result.height <= 0) {
      pose.status = TAG_ERROR;
      return pose;
    }
    if (result.xCenter - result.width / 2 <= 0 ||
        result.xCenter + result.width / 2 >= 319 ||
        result.yCenter - result.height / 2 <= 0 ||
        result.yCenter + result.height / 2 >= 239) {
      pose.status = TAG_PARTIAL;
      continue;  // A second, fully visible detection can still be usable.
    }

    pose.x_px = result.xCenter;
    pose.height_px = result.height;
    pose.range_cm = RANGE_HEIGHT_SCALE_CM_PX / result.height + RANGE_OFFSET_CM;
    pose.heading_deg = atan2(CAMERA_CENTER_X_PX - result.xCenter,
                            CAMERA_FOCAL_LENGTH_PX) * 180.0f / PI;
    if (!isfinite(pose.range_cm) || pose.range_cm <= 0.0f ||
        !isfinite(pose.heading_deg)) {
      pose.status = TAG_ERROR;
      return pose;
    }
    pose.status = TAG_VISIBLE;
    return pose;
  }
  return pose;
}
