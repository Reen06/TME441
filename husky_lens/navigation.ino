/* Tag navigation adapted from Matthew Ford's Milestone 2 navigation.ino.
 * Uses V2's blocking, encoder-based driveDistance() and rotateCCW().
 * Returns 1 = success, -1 = tag missing/search timeout, -2 = camera/motion fault.
 */

int driveToTag(int tag_id) {
  const int status = pointToTag(tag_id);
  if (status != 1) {
    stopDriving();
    return status;
  }

  const int range_cm = rangeToTag(tag_id);
  if (range_cm < 0) {
    stopDriving();
    return range_cm;  // Never turn an error code into a reverse drive command.
  }

  // Original approach: one encoder move, stopping an estimated 10 cm short.
  if (range_cm <= 10) {
    stopDriving();
    return 1;
  }
  return driveDistance(range_cm - 10) ? 1 : -2;
}

int pointToTag(int tag_id) {
  const unsigned long start_ms = millis();
  while (millis() - start_ms < 30000UL) {
    const int x = getObjCenterX(tag_id);
    if (x == -2) {
      stopDriving();
      return -2;
    }
    if (x == -1) {
      if (!rotateCCW(30.0f)) {
        return -2;
      }
    } else if (abs(x - 160) < 10) {
      stopDriving();
      return 1;
    } else {
      if (!rotateCCW((x < 160) ? 8.0f : -8.0f)) {
        return -2;
      }
    }
    delay(100);
  }

  stopDriving();
  return -1;
}
