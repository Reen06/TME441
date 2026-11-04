/* TME 441 - HuskyLens tag navigation with V2 smooth driving.
 * Camera and navigation adapted from Matthew Ford's Milestone 2 solution.
 * driving.ino and motor_control.h are copies of v2_smooth_square.
 */
#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>

HUSKYLENS cam;

int main() {
  init();
  Serial.begin(9600);
  Serial3.begin(9600);  // Mega RX3 = 15, TX3 = 14.
  initMotors();
  stopDriving();

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

  int current_tag = 1;
  while (true) {
    Serial.print(F("Navigating to tag: "));
    Serial.println(current_tag);

    const int result = driveToTag(current_tag);
    stopDriving();
    if (result == 1) {
      current_tag = (current_tag < 3) ? current_tag + 1 : 1;
    } else if (result == -1) {
      Serial.println(F("Tag not found. Retrying the same tag."));
    } else {
      Serial.println(F("Camera or movement fault. Stopped until reset."));
      while (true) {
        stopDriving();
        delay(100);
      }
    }
    delay(1000);
  }
}
