/* Camera geometry adapted from Matthew Ford's Milestone 2 camera.ino.
 * Range and bearing come from the SAME frame of the requested tag.
 */

TagPose readTagPose(int tag_id) {
  TagPose pose = {};
  pose.status = TAG_ERROR;
  if (!cam.requestBlocks(tag_id)) {
    return pose;
  }

  pose.status = TAG_MISSING;
  while (cam.available()) {
    const HUSKYLENSResult result = cam.read();
    if (result.command != COMMAND_RETURN_BLOCK || result.ID != tag_id) {
      continue;
    }
    // A clipped tag can look smaller and falsely appear farther away.
    if (result.xCenter < 0 || result.xCenter >= 320 ||
        result.yCenter < 0 || result.yCenter >= 240 ||
        result.width <= 0 || result.height <= 0 ||
        result.xCenter - result.width / 2 <= 0 ||
        result.xCenter + result.width / 2 >= 319 ||
        result.yCenter - result.height / 2 <= 0 ||
        result.yCenter + result.height / 2 >= 239) {
      pose.status = TAG_ERROR;
      return pose;
    }

    pose.x_px = result.xCenter;
    pose.height_px = result.height;
    pose.range_cm = RANGE_HEIGHT_SCALE_CM_PX / result.height + RANGE_OFFSET_CM;
    pose.bearing_deg = atan2(CAMERA_CENTER_X_PX - result.xCenter,
                            CAMERA_FOCAL_LENGTH_PX) * 180.0f / PI;
    if (!isfinite(pose.range_cm) || pose.range_cm <= 0.0f ||
        !isfinite(pose.bearing_deg)) {
      pose.status = TAG_ERROR;
      return pose;
    }
    pose.status = TAG_VISIBLE;
    return pose;
  }
  return pose;
}
