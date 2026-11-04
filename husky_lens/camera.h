#ifndef HUSKY_CAMERA_H
#define HUSKY_CAMERA_H

enum TagStatus { TAG_ERROR = -2, TAG_MISSING = 0, TAG_VISIBLE = 1 };

struct TagPose {
  TagStatus status;
  int x_px;
  int height_px;
  float range_cm;     // Forward distance from camera to the tag plane.
  float bearing_deg;  // Positive = left/CCW; negative = right/CW.
};

#endif
