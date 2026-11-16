#ifndef MILESTONE_2_CAMERA_H
#define MILESTONE_2_CAMERA_H

enum TagStatus {
  TAG_ERROR = -2, TAG_MISSING = 0, TAG_VISIBLE = 1, TAG_PARTIAL = 2
};

struct TagPose {
  TagStatus status;
  int x_px;
  int height_px;
  float range_cm;    // Lens to tag plane, before mounting-offset correction.
  float heading_deg; // Positive = left/CCW; negative = right/CW.
};

// Both quantities come from one camera response for exactly this tag ID.
TagPose rangeAndHeadingToTag(int tag_id);
#endif
