/* Camera helpers adapted from Matthew Ford's Milestone 2 camera.ino.
 * Returns -1 when the tag is absent, -2 on communication/invalid data errors.
 */

int getObjCenterX(int obj_id) {
  if (!cam.request()) {
    return -2;
  }

  while (cam.available()) {
    const HUSKYLENSResult result = cam.read();
    if (result.command == COMMAND_RETURN_BLOCK && result.ID == obj_id) {
      return (result.xCenter >= 0 && result.xCenter < 320) ? result.xCenter : -2;
    }
  }
  return -1;
}

int rangeToTag(int tag_id) {
  if (!cam.request()) {
    return -2;
  }

  while (cam.available()) {
    const HUSKYLENSResult result = cam.read();
    if (result.command == COMMAND_RETURN_BLOCK && result.ID == tag_id) {
      if (result.height <= 0) {
        return -2;
      }
      // Original Milestone 2 empirical fit; recalibrate for your printed tags.
      const int range_cm = 3489 / result.height - 3;
      return (range_cm > 0) ? range_cm : -2;
    }
  }
  return -1;
}
