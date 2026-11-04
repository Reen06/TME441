#ifndef HUSKY_NAVIGATION_CONFIG_H
#define HUSKY_NAVIGATION_CONFIG_H

// Search: +1 = counterclockwise, -1 = clockwise, viewed from above.
const float SEARCH_TURN_SPEED_DEG_S = 30.0f;
const int SEARCH_DIRECTION = 1;

// Distance from the FRONT of the robot to the tag. The margin stops it earlier.
const float STOP_DISTANCE_CM = 20.0f;
const float STOP_MARGIN_CM = 5.0f;
// Measure how far the camera lens sits BEHIND the robot's front bumper.
const float CAMERA_TO_FRONT_CM = 0.0f;
const float APPROACH_SPEED_CM_S = 6.0f;
const float MAX_APPROACH_STEP_CM = 5.0f;
const float ALIGN_TOLERANCE_DEG = 3.0f;

// range_cm = RANGE_HEIGHT_SCALE_CM_PX / tag_height_px + RANGE_OFFSET_CM.
// Initial range fit inherited from the supplied TME code; recalibrate for your tag.
const float RANGE_HEIGHT_SCALE_CM_PX = 3489.0f;
const float RANGE_OFFSET_CM = -3.0f;
const float CAMERA_CENTER_X_PX = 160.0f;
// Initial estimate, NOT a measured camera specification. Calibration:
// focal_length_px = abs(tag_x - center_x) / tan(known_bearing_in_radians).
const float CAMERA_FOCAL_LENGTH_PX = 280.0f;

const unsigned long CAMERA_POLL_INTERVAL_MS = 100UL;
const unsigned long CAMERA_TIMEOUT_MS = 100UL;
const unsigned long TAG_PAUSE_MS = 1000UL;

// Original TME route. Set both IDs to 1 to approach tag 1 and stay stopped.
const int FIRST_TAG_ID = 1;
const int LAST_TAG_ID = 3;

#endif
