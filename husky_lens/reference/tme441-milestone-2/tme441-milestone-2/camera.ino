/* camera.ino
*  Functions for communicating with the HuskyLens camera
*
*  @author Matthew Ford
*/

/* A struct is a C++ data type that groups related variables together.
*  Here, TagPose represents the status, range, and heading of a detected tag.
*  You may delete this comment and replace it with your own.
*/
struct TagPose {
  int status;   // 1 if tag is found, 0 if not found, -1 if error
  int range;    // [cm] Range to tag
  int heading;  // [deg] Heading to tag relative to camera
};

// Print a HuskyLens result
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

/* Get the X coordinate of a specified object in the HuskyLens view.
*
*  This function requires HUSKLENS cam to be defined as a global variable.
*
*  @param obj_id ID of the target object
*  @return
*     x coordinate (between 0-360) of desired object, if found.
*    -1 if object not found
*    -2 if other communication error
*/
int getObjCenterX(int obj_id) {
  HUSKYLENSResult result;

  if (!cam.request()) {
    return -2;  // Communication error
  }

  while (cam.available()) {
    result = cam.read();  // Check next available block
    if (result.ID == obj_id) {
      return result.xCenter;
    }
  }

  return -1;  // Object not found
}

/* Estimate distance to tag based on apparent size
*
*  @param tag_id ID of the desired tag
*  @return
*     Estimated distance to desired tag (cm), if found
*     -1 if tag not found
*     -2 if other communication error
*/
int rangeToTag(int tag_id) {
  HUSKYLENSResult result;

  if (!cam.request()) {
    return -2;  // Communication error
  }

  while (cam.available()) {
    result = cam.read();  // Check next available block
    if (result.ID == tag_id) {
      return 3489 / result.height - 3;  // Based on experimental data
    }
  }

  return -1;  // Object not found
}
