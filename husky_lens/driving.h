#ifndef HUSKY_DRIVING_H
#define HUSKY_DRIVING_H

// Called during a movement; false requests immediate braking.
typedef bool (*MotionMonitor)();
void setMotionMonitor(MotionMonitor monitor);
bool wasMotionInterrupted();
void stopDriving();
bool driveDistance(float distance_cm);
bool driveDistance(float distance_cm, float speed_cm_s);
bool rotateCCW(float angle_degrees);
bool rotateContinuously(float turn_rate_deg_s);

#endif
