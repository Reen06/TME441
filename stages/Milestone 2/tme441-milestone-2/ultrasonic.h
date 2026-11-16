#ifndef MILESTONE_2_ULTRASONIC_H
#define MILESTONE_2_ULTRASONIC_H

enum UltrasonicStatus {
  ULTRASONIC_NOT_READY = -2, ULTRASONIC_INVALID = -1,
  ULTRASONIC_NO_ECHO = 0, ULTRASONIC_OK = 1
};
struct UltrasonicReading {
  UltrasonicStatus status;
  float distance_cm;
  unsigned long sampled_ms;
  float maximum_cm;  // Upper reading in a stable three-ping cluster.
};
void initUltrasonic();
UltrasonicReading readUltrasonic();
bool ultrasonicReadingFresh(const UltrasonicReading &reading);
// Call while braked. Median of three agreeing fresh pings, at most five attempts.
UltrasonicReading readStableUltrasonic();
#endif
