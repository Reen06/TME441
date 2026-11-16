#ifndef MILESTONE_2_NAVIGATION_CONFIG_H
#define MILESTONE_2_NAVIGATION_CONFIG_H

// Autonomous route: visit each ID once and HALT after the last tag.
const int FIRST_TAG_ID = 1;
const int LAST_TAG_ID = 3;

// Preserve the slower search speed selected during robot testing.
const float SEARCH_TURN_SPEED_DEG_S = 20.0f;
const int SEARCH_DIRECTION = 1;  // +1 = CCW, -1 = CW, viewed from above.
const unsigned long TAG_SEARCH_TIMEOUT_MS = 45000UL;
const unsigned long TAG_NAVIGATION_TIMEOUT_MS = 120000UL;

// Aim inside the required 10 cm clearance, allowing 2 cm for stopping error.
const float STOP_TARGET_CM = 8.0f;
const float MAX_STOP_CLEARANCE_CM = 10.0f;
// Lens distance BEHIND the front face of the ultrasonic sensor (or robot front
// until the sensor is installed). Zero is an UNMEASURED placeholder: measure it.
const float CAMERA_TO_STOP_REFERENCE_CM = 0.0f;
const float APPROACH_SPEED_CM_S = 4.0f;
const float MAX_APPROACH_STEP_CM = 3.0f;
const float ALIGN_TOLERANCE_DEG = 3.0f;

// A tag may leave the field of view near the robot. Only the last <=5 cm may
// finish on encoders, from a fresh, centered, fully visible tag measurement.
// The frozen encoder target is NEVER extended when a tag disappears.
const float MAX_FINAL_APPROACH_CM = 5.0f;

// Provisional fit inherited from the starter: calibrate with the actual tags.
const float RANGE_HEIGHT_SCALE_CM_PX = 3489.0f;
const float RANGE_OFFSET_CM = -3.0f;
const float CAMERA_CENTER_X_PX = 160.0f;
const float CAMERA_FOCAL_LENGTH_PX = 280.0f;  // Estimate, not measured FOV.

const unsigned long CAMERA_POLL_INTERVAL_MS = 100UL;
const unsigned long CAMERA_TIMEOUT_MS = 100UL;
const unsigned long CAMERA_CONNECT_TIMEOUT_MS = 10000UL;
const unsigned int CAMERA_RETRY_COUNT = 3;
const unsigned long TAG_PAUSE_MS = 1000UL;

// true = camera CSV only, motors stay braked; false = autonomous 1 -> 2 -> 3.
const bool CAMERA_CALIBRATION_MODE = false;
const int CALIBRATION_TAG_ID = 1;

// Reserved for a later HC-SR04 addition: TRIG D22, ECHO D23, VCC 5V, GND.
// No ultrasonic measurements or pin changes are used in this camera-only stage.
#endif
