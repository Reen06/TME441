/* TME 441
*  Milestone 2 Solution
*
*  Navigate to a series of waypoints using camera input.
*
*  Reuses driving.ino and motor_control.h from Milestone 1.
*
*  Starter author: Matthew Ford
*/
#include <Encoder.h>
#include "motor_control.h"
#include "HUSKYLENS.h"

HUSKYLENS cam;

// First run with true: print camera measurements without moving the robot.
// After calibration and safe bench testing, change to false for navigation.
const bool CALIBRATION_MODE = false;
const int CALIBRATION_TAG_ID = 1;

/* A struct groups related variables together.
*  TagPose contains the status, range, and heading of a detected tag.
*/
struct TagPose {
  int status;     // 1 = found, 0 = not found, -1 = camera/measurement error
  float range;    // [cm] Estimated range referenced to front sensor
  float heading;  // [deg] Heading relative to camera (+ = CCW)
};

// Explicit declarations avoid Arduino's automatic struct-prototype issue.
TagPose rangeAndHeadingToTag(int tag_id);
int driveToTag(int tag_id);
void stopDriving();
bool driveDistance(float distance_cm);
bool rotateCCW(float angle_degrees);
void initRangeSensor();
float ultrasonicRangeCM();
void initOdometry();
void updateOdometry();
void printOdometry();
void odometryDelay(unsigned long duration_ms);
bool returnToStart();
bool moveWheels(float distance_l_cm, float distance_r_cm, float max_speed_cm_s);

int main() {
  init();

  // Initialize communication interfaces
  Serial.begin(9600);
  Serial3.begin(9600);  // HuskyLens

  initMotors();
  stopDriving();
  initRangeSensor();

  // Configure HuskyLens camera
  unsigned long start_time = millis();
  while (!cam.begin(Serial3)) {
    Serial.println(F("Begin failed! Check HuskyLens Serial 9600 and wiring."));
    if (millis() - start_time >= 10000UL) {
      stopDriving();
      while (true) { delay(100); }
    }
    delay(100);
  }

  Serial.println(F("HuskyLens Connected!"));

  // Set the HuskyLens to tag recognition mode
  if (!cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
    Serial.println(F("Cannot set tag recognition mode."));
    stopDriving();
    while (true) { delay(100); }
  }
  delay(1000);

  // Calibration: record xCenter and height at known distances and angles.
  while (CALIBRATION_MODE) {
    TagPose pose = rangeAndHeadingToTag(CALIBRATION_TAG_ID);
    Serial.print(F("status=")); Serial.println(pose.status);
    Serial.print(F("ultrasonic_cm=")); Serial.println(ultrasonicRangeCM());
    delay(300);
  }

  // Define the initial axle-center position and heading as (0, 0, 0).
  initOdometry();
  bool all_tags_reached = false;

  // Navigate to each April tag in sequence
  for (int current_tag = 1; current_tag <= 3; current_tag++) {
    odometryDelay(1000);

    Serial.print(F("Navigating to tag: "));
    Serial.println(current_tag);

    int result = driveToTag(current_tag);
    stopDriving();
    if (result != 1) {
      Serial.print(F("Navigation failed. Status: "));
      Serial.println(result);
      break;
    }

    Serial.print(F("Reached tag: "));
    Serial.println(current_tag);
    if (current_tag == 3) {
      Serial.println(F("All three tags reached."));
      all_tags_reached = true;
    }
  }

  // Return only after all three tags were successfully reached.
  if (all_tags_reached) {
    Serial.println(F("Returning to initial pose."));
    if (returnToStart()) {
      Serial.println(F("Home pose reached (encoder estimate)."));
    } else {
      Serial.println(F("Return failed. Robot stopped."));
    }
    printOdometry();
  }

  // Stay stopped after the return or any navigation failure.
  stopDriving();
  while (true) { odometryDelay(100); }
}
