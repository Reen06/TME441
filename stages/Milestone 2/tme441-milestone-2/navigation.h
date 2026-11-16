#ifndef MILESTONE_2_NAVIGATION_H
#define MILESTONE_2_NAVIGATION_H

enum NavigationResult {
  TAG_REACHED = 1, TAG_SEARCH_FAILED = -1, TAG_NAVIGATION_ERROR = -2
};
bool navigationSettingsValid();
int driveToTag(int tag_id);
int runMilestone2();
void printCameraCalibrationSample();
#endif
