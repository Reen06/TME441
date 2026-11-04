/* TME 441 - HuskyLens tag navigation built on V2 smooth driving.
 * Camera and navigation adapted from Matthew Ford's Milestone 2 solution.
 * Configure search, camera calibration, and stopping in navigation_config.h.
 */
#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include "navigation_config.h"
#include "camera.h"
#include "driving.h"

HUSKYLENS cam;

int main() {
  init();
  Serial.begin(115200);
  Serial3.begin(9600);  // Mega RX3 = 15, TX3 = 14.
  initMotors();
  stopDriving();

  if (!navigationSettingsValid()) {
    Serial.println(F("Invalid navigation settings. Check navigation_config.h."));
    while (true) { delay(100); }
  }

  cam.setTimeOutDuration(CAMERA_TIMEOUT_MS);
  while (!cam.begin(Serial3)) {
    Serial.println(F("HuskyLens not connected. Check wiring and Serial 9600 mode."));
    delay(1000);
  }

  if (!cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
    Serial.println(F("Cannot select Tag Recognition. Reset after checking camera."));
    while (true) {
      stopDriving();
      delay(100);
    }
  }
  delay(1000);
  Serial.println(F("HuskyLens connected. Navigating with V2 smooth driving."));

  int current_tag = FIRST_TAG_ID;
  while (true) {
    Serial.print(F("Navigating to tag: "));
    Serial.println(current_tag);

    const int result = driveToTag(current_tag);
    stopDriving();
    if (result == 1) {
      Serial.println(F("Tag stopping distance reached."));
      if (FIRST_TAG_ID == LAST_TAG_ID) {
        while (true) { delay(100); }  // Single-tag mode holds the final stop.
      }
      current_tag = (current_tag < LAST_TAG_ID) ? current_tag + 1 : FIRST_TAG_ID;
    } else {
      Serial.println(F("Camera or movement fault. Stopped until reset."));
      while (true) {
        stopDriving();
        delay(100);
      }
    }
    delay(TAG_PAUSE_MS);
  }
}
