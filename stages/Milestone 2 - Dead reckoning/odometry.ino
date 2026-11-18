/* odometry.ino
 * Encoder dead reckoning and return to initial pose.
 * X and Y in cm, theta in radians (positive CCW).
 * Pose is the midpoint between the two drive wheels.
 */
const float HOME_POSITION_TOL_CM = 3.0f;
const float HOME_HEADING_TOL_DEG = 3.0f;
const float HOME_SPEED_CM_S = 10.0f;
const int HOME_MAX_ATTEMPTS = 3;
const unsigned long ODOM_REPORT_MS = 500UL;

float robot_x = 0.0f;
float robot_y = 0.0f;
float robot_theta = 0.0f;
long odom_last_l = 0;
long odom_last_r = 0;
unsigned long odom_last_report = 0;
bool odom_ready = false;

// Normalize to [-pi, pi] for shortest-angle turns.
float wrapRadians(float a) {
  const float pi_rad = PI;
  while (a > pi_rad) a -= 2.0f * pi_rad;
  while (a < -pi_rad) a += 2.0f * pi_rad;
  return a;
}

void printOdometry() {
  Serial.print(F("Pose X=")); Serial.print(robot_x);
  Serial.print(F(", Y=")); Serial.print(robot_y);
  Serial.print(F(", theta=")); Serial.println(robot_theta * 180.0f / PI);
}

void initOdometry() {
  robot_x = 0.0f;
  robot_y = 0.0f;
  robot_theta = 0.0f;
  odom_last_l = enc_l.read();
  odom_last_r = enc_r.read();
  odom_last_report = millis();
  odom_ready = true;
  printOdometry();
}

// Integrate measured wheel displacements, not requested distances or angles.
void updateOdometry() {
  if (!odom_ready) return;
  const long current_l = enc_l.read();
  const long current_r = enc_r.read();
  const long delta_l = current_l - odom_last_l;
  const long delta_r = current_r - odom_last_r;
  odom_last_l = current_l;
  odom_last_r = current_r;

  if (delta_l != 0 || delta_r != 0) {
    const float cm_per_tick = PI * WHEEL_DIAMETER_CM / ENCODER_TICKS_PER_WHEEL_REV;
    const float ds_l = delta_l * cm_per_tick;
    const float ds_r = delta_r * cm_per_tick;
    const float ds = 0.5f * (ds_l + ds_r);
    const float dtheta = (ds_r - ds_l) / TRACK_WIDTH_CM;
    const float midpoint = robot_theta + 0.5f * dtheta;
    // Arc-length correction is nearly 1 for frequent small samples.
    float chord_scale = 1.0f;
    if (abs(dtheta) > 0.0001f) {
      chord_scale = sin(0.5f * dtheta) / (0.5f * dtheta);
    }
    robot_x += ds * chord_scale * cos(midpoint);
    robot_y += ds * chord_scale * sin(midpoint);
    robot_theta = wrapRadians(robot_theta + dtheta);
  }

  // Skip a periodic report if the serial buffer is busy, not the pose update.
  if (millis() - odom_last_report >= ODOM_REPORT_MS &&
      Serial.availableForWrite() >= 48) {
    odom_last_report = millis();
    printOdometry();
  }
}

// Capture residual wheel motion while the robot is settling or waiting.
void odometryDelay(unsigned long duration_ms) {
  const unsigned long started = millis();
  while (millis() - started < duration_ms) {
    updateOdometry();
    delay(1);
  }
  updateOdometry();
}

// Recheck encoder-estimated angle after every turn, using bounded attempts.
bool faceHeading(float desired_rad) {
  for (int i = 0; i < HOME_MAX_ATTEMPTS; ++i) {
    updateOdometry();
    float error_deg = wrapRadians(desired_rad - robot_theta) * 180.0f / PI;
    if (abs(error_deg) <= HOME_HEADING_TOL_DEG) return true;
    if (!rotateCCW(error_deg)) return false;
  }
  updateOdometry();
  return abs(wrapRadians(desired_rad - robot_theta) * 180.0f / PI) <= HOME_HEADING_TOL_DEG;
}

// A direct return is suitable for the clear tag-tracking arena.
// The ultrasonic guard stops for reported obstacles, not active path planning.
bool returnToStart() {
  stopDriving();
  odometryDelay(200);
  for (int attempt = 0; attempt < HOME_MAX_ATTEMPTS; ++attempt) {
    updateOdometry();
    float distance = sqrt(robot_x * robot_x + robot_y * robot_y);
    if (distance > HOME_POSITION_TOL_CM) {
      float bearing = atan2(-robot_y, -robot_x);
      Serial.print(F("Home distance: ")); Serial.println(distance);
      Serial.print(F("Home bearing: ")); Serial.println(bearing * 180.0f / PI);
      if (!faceHeading(bearing)) { stopDriving(); return false; }

      // Recalculate distance after rotation and correct any residual bearing.
      distance = sqrt(robot_x * robot_x + robot_y * robot_y);
      if (distance > HOME_POSITION_TOL_CM) {
        bearing = atan2(-robot_y, -robot_x);
        if (!faceHeading(bearing)) { stopDriving(); return false; }
        distance = sqrt(robot_x * robot_x + robot_y * robot_y);
      }
      if (distance > HOME_POSITION_TOL_CM) {
        home_guard_active = true;
        bool ok = moveWheels(distance, distance, HOME_SPEED_CM_S);
        home_guard_active = false;
        if (!ok) { stopDriving(); return false; }
      }
    }
    // Make sure position is within tolerance before restoring starting heading.
    updateOdometry();
    if (sqrt(robot_x * robot_x + robot_y * robot_y) <= HOME_POSITION_TOL_CM) {
      if (!faceHeading(0.0f)) { stopDriving(); return false; }
      stopDriving();
      odometryDelay(200);
      if (sqrt(robot_x * robot_x + robot_y * robot_y) <= HOME_POSITION_TOL_CM &&
          abs(robot_theta * 180.0f / PI) <= HOME_HEADING_TOL_DEG) return true;
    }
  }
  stopDriving();
  return false;
}
