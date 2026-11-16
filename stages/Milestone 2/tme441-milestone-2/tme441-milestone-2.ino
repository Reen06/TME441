/* TME 441 Milestone 2: autonomous AprilTag route 1 -> 2 -> 3, then HALT.
 * Camera and motor code adapted from Matthew Ford's supplied starter.
 * Upload THIS sketch; settings and calibration are in navigation_config.h.
 */
#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include "navigation_config.h"
#include "camera.h"
#include "driving.h"
#include "navigation.h"
#include "ultrasonic.h"

HUSKYLENS cam;
int mission_status = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);  // Camera TX -> Mega RX3 D15; camera RX -> TX3 D14.
  initMotors();
  stopDriving();
  initUltrasonic();
  if (!navigationSettingsValid()) {
    Serial.println(F("[fault] invalid navigation/driving settings"));
    mission_status = TAG_NAVIGATION_ERROR;
    return;
  }
  cam.setTimeOutDuration(CAMERA_TIMEOUT_MS);
  const unsigned long connecting_ms = millis();
  while (!cam.begin(Serial3)) {
    if (millis() - connecting_ms >= CAMERA_CONNECT_TIMEOUT_MS) {
      Serial.println(F("[fault] camera connection timeout: check Serial3 and Serial 9600 mode"));
      mission_status = TAG_NAVIGATION_ERROR;
      return;
    }
    delay(200);
  }
  if (!cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
    Serial.println(F("[fault] cannot select Tag Recognition"));
    mission_status = TAG_NAVIGATION_ERROR;
    return;
  }
  delay(1000);
  if (!cam.request()) {
    Serial.println(F("[fault] camera preflight request failed"));
    mission_status = TAG_NAVIGATION_ERROR;
    return;
  }
  Serial.print(F("[camera] learned IDs="));
  Serial.println(cam.countLearnedIDs());
  const int required_id = CAMERA_CALIBRATION_MODE ? CALIBRATION_TAG_ID : LAST_TAG_ID;
  if (cam.countLearnedIDs() < required_id) {
    Serial.println(F("[fault] learn tags 1, 2, and 3 in Tag Recognition before running"));
    mission_status = TAG_NAVIGATION_ERROR;
    return;
  }
  if (CAMERA_CALIBRATION_MODE) {
    Serial.println(F("[calibration] camera + HC-SR04; motors braked; place tag board at measured distances"));
    Serial.println(F("time_ms,tag_id,status,x_px,height_px,range_cm,heading_deg,ultrasonic_status,ultrasonic_cm"));
    return;
  }
  Serial.println(F("[route] autonomous startup: 1 -> 2 -> 3, then HALT"));
  mission_status = runMilestone2();
  stopDriving();
}

void loop() {
  if (CAMERA_CALIBRATION_MODE && mission_status == 0) {
    printCameraCalibrationSample();
    delay(200);
  } else {
    delay(20);  // setup ran the route once; completion/fault holds until reset.
  }
}
