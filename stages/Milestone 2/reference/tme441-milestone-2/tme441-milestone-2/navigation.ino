/* navigation.ino
*  Functions for navigating the robot to April tags using the HuskyLens camera
*
*  @author Matthew Ford
*/


/* Find and approach the specified April tag
*
*  @param tag_id ID number of the desired tag
*  @return 
*     1 = Success
*    -1 = Searched unsuccessfully for a tag for 20 sec
*    -2 = Other error, or lost connection with HuskyLens
*/
int driveToTag(int tag_id) {

  int status;
  int range;

  status = pointToTag(tag_id);  // First search
  if (status == -2) { return -2; }
  if (status == -1) { return -1; }

  // Check range to tag
  range = rangeToTag(tag_id);

  // Drive to tag, stopping 10 cm short
  driveDistance(range - 10);

  return 1;
}

/* Rotate robot to point to the specified April tag
*
*  @param tag_id ID number of the desired tag
*  @return
*     1 = Success
*    -1 = Search unsuccessfully for 20 sec
*    -2 = Other error, or lost connection with HuskyLens
*/
int pointToTag(int tag_id) {
  int x;
  long start_time = millis();

  while ((millis() - start_time) < 30000) {
    x = getObjCenterX(tag_id);

    if (x == -2) {  // HuskyLens error
      return -2;
    } else if (x == -1) {  // Tag not found
      rotateCCW(30);
      delay(100);
    } else if (abs(x - 160) < 10) {  // Tag is centered
      return 1;
    } else {  // Tag not centered. Minor adjustment
      if (x < 160) {
        rotateCCW(8);
      } else {
        rotateCCW(-8);
      }
      delay(100);
    }
  }
  
  return -1;  // Timeout
}
