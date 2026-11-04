# HuskyLens with V2 smooth driving

Open [husky_lens.ino](../husky_lens.ino) to upload this version to the Arduino Mega
2560. It uses the supplied Milestone 2 camera/tag-navigation code with **V2**
smooth driving. Its `driving.ino` and `motor_control.h` start as exact copies from
[`v2_smooth_square`](../../v2_smooth_square/).

The sketch searches for learned tag IDs **1, 2, 3**, aligns with each tag, drives
to an estimated 10 cm away, then repeats the sequence. V2 supplies the movement
functions; the HuskyLens sketch's entry point supplies the tag route instead of
V2's square route.

## Files

File paths in the table below are relative to `husky_lens/`.

| Path | Purpose |
| --- | --- |
| `husky_lens.ino` | Initialize motors and camera; visit tag IDs 1 through 3. |
| `driving.ino` | V2 smooth ramps, encoder targets, stall detection, and movement timeouts. |
| `motor_control.h` | V2 motor wiring, encoders, and wheel-speed PID. |
| `camera.ino` | Query tag center and estimate range from the detected height. |
| `navigation.ino` | Search, align, and approach using V2 movement functions. |
| [reference/HUSKYLENSArduino](../reference/HUSKYLENSArduino/) | Original downloaded library, examples, API/protocol docs, and ZIP. |
| [reference/tme441-milestone-2](../reference/tme441-milestone-2/) | Original supplied sketch, instructions, and printable tag images. |

Both reference packages are preserved with their redundant extraction wrappers
removed. Edit the working files above when developing this version.

## Connect and upload

1. Keep the V2 motor and encoder wiring. Connect camera **TX to Mega RX3 (15)**,
   camera **RX to Mega TX3 (14)**, power to 5V, and ground to ground. This sketch
   uses **Serial3**, as in the supplied Milestone 2 code.
2. Set the HuskyLens protocol to **Serial 9600** and learn the three AprilTags as
   IDs 1, 2, and 3. The sketch selects **Tag Recognition** at startup.
3. Install **Encoder by Paul Stoffregen** in Arduino IDE. Install the supplied
   library ZIP using **Sketch > Include Library > Add .ZIP Library**, selecting
   [`reference/HUSKYLENSArduino/HUSKYLENS.zip`](../reference/HUSKYLENSArduino/HUSKYLENS.zip).
4. Open `husky_lens.ino`, select **Arduino Mega or Mega 2560** and the board's
   port, then verify and upload. Open Serial Monitor at **9600 baud**.

To verify with Arduino CLI from the project root using the supplied libraries:

```powershell
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 --libraries .vscode/libraries --library husky_lens/reference/HUSKYLENSArduino/HUSKYLENS husky_lens
```

Calibrate the wheel dimensions, encoder counts, speed limits, and PID gains in
`husky_lens/` as for V2. The range formula `3489 / height - 3` comes from the
supplied Milestone 2 code and needs calibration for the printed tag size and
camera mounting on your robot.

## Initial behavior and limits

The robot waits stopped until the camera connects. A missing tag triggers a
search, with a 30-second deadline checked between V2 moves. A search timeout
retries the same tag. Camera errors, invalid range data, or a failed movement
stop navigation until reset. A tag already within 10 cm causes no reverse move.

This initial version keeps V2's blocking movements: the camera is read between
moves, and the approach is one encoder-controlled distance. It does not update
the approach from camera frames while moving. Test and refine the supplied
navigation behavior on the robot as required by Milestone 2.
