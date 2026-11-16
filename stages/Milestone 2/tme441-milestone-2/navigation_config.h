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
// Lens distance BEHIND the HC-SR04 front face. Confirmed 2026-11-16:
// camera lens and sensor face are level, so zero is correct. Remeasure if moved.
const float CAMERA_TO_STOP_REFERENCE_CM = 0.0f;
const float APPROACH_SPEED_CM_S = 4.0f;
const float MAX_APPROACH_STEP_CM = 3.0f;
const float ALIGN_TOLERANCE_DEG = 3.0f;

// Camera tag visibility was physically observed to end below about 27 cm.
// Acquire aligned target + agreeing ultrasonic data before crossing that limit.
const float CAMERA_MIN_VISIBLE_CM = 27.0f;
const float CAMERA_ULTRASONIC_HANDOFF_CM = 35.0f;
const float HANDOFF_RANGE_TOLERANCE_CM = 6.0f;
const float ULTRASONIC_APPROACH_SPEED_CM_S = 3.0f;
const float ULTRASONIC_APPROACH_STEP_CM = 1.0f;
const float ULTRASONIC_STOP_CONFIRM_CM = 9.0f;  // Confirm inside the 10 cm limit.
const float ULTRASONIC_MAX_APPROACH_TRAVEL_CM = 35.0f;
const float ULTRASONIC_TRAVEL_MARGIN_CM = 3.0f;

// HC-SR04 wiring confirmed 2026-11-16. Face is level with the lens, so the
// ultrasonic reading already measures clearance from the required front face.
const int ULTRASONIC_TRIG_PIN = 22;
const int ULTRASONIC_ECHO_PIN = 23;
const unsigned long ULTRASONIC_PING_INTERVAL_MS = 70UL;
const unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 12000UL;
const unsigned long ULTRASONIC_FRESHNESS_MS = 200UL;
const unsigned long ULTRASONIC_CONFIRM_TIMEOUT_MS = 500UL;
const float ULTRASONIC_MIN_CM = 2.0f;
const float ULTRASONIC_MAX_CM = 200.0f;
const float ULTRASONIC_STABLE_SPREAD_CM = 2.0f;

// Fits from tag-2 measurements on 2026-11-16; see the calibration worksheet.
// Range: three centered lens distances. Heading: centered and lateral samples.
// Validate nearer than 30 cm before autonomous use; these are preliminary fits.
const float RANGE_HEIGHT_SCALE_CM_PX = 3390.0263f;
const float RANGE_OFFSET_CM = 1.038563f;
// Later centered validation: displayed range was about 1.5 cm too large.
// Keep this correction separate from the original fit and raw observations.
const float RANGE_BIAS_CORRECTION_CM = -1.5f;
const float CAMERA_CENTER_X_PX = 171.11111f;
const float CAMERA_FOCAL_LENGTH_PX = 300.7144f;

const unsigned long CAMERA_POLL_INTERVAL_MS = 100UL;
const unsigned long CAMERA_TIMEOUT_MS = 100UL;
const unsigned long CAMERA_CONNECT_TIMEOUT_MS = 10000UL;
const unsigned int CAMERA_RETRY_COUNT = 3;
const unsigned long TAG_PAUSE_MS = 1000UL;

// true = camera + ultrasonic CSV, motors braked; false = autonomous 1 -> 2 -> 3.
const bool CAMERA_CALIBRATION_MODE = true;
const int CALIBRATION_TAG_ID = 2;

#endif
