# HuskyLens with V2 smooth driving

Open [husky_lens.ino](../husky_lens.ino) to upload this version to the Arduino Mega
2560. It uses the supplied Milestone 2 camera/tag-navigation code with **V2**
smooth driving. The movement code builds on
[`v2_smooth_square`](../../v2_smooth_square/), adding a camera monitor and continuous
search rotation. The motor wiring and PID remain in `motor_control.h`.

The sketch searches for learned tag IDs **1, 2, 3**, aligns with each tag, drives
to a configured clearance, then repeats the sequence. Missing tags trigger a
continuous search. Visible tags supply both a turn angle and a range estimate.
The default stopping threshold is **20 cm plus a 5 cm margin**, measured from the
front of the robot after applying the camera mounting offset.

## Files

File paths in the table below are relative to `husky_lens/`.

| Path | Purpose |
| --- | --- |
| `husky_lens.ino` | Initialize motors and camera; visit tag IDs 1 through 3. |
| `driving.ino` | V2 smooth ramps, encoder targets, stall detection, and movement timeouts. |
| `motor_control.h` | V2 motor wiring, encoders, and wheel-speed PID. |
| `navigation_config.h` | Search speed/direction, stopping clearance, approach settings, and camera calibration. |
| `driving.h` | Movement commands and the camera-monitor callback interface. |
| `camera.h` / `camera.ino` | Read tag height and horizontal position from one frame and calculate range/bearing. |
| `navigation.ino` | Search continuously, align by bearing, and approach with camera checks during each move. |
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
   port, then verify and upload. Open Serial Monitor at **115200 baud**.
   The camera connection on Serial3 remains **9600 baud**.

To verify with Arduino CLI from the project root using the supplied libraries:

```powershell
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 --libraries .vscode/libraries --library husky_lens/reference/HUSKYLENSArduino/HUSKYLENS husky_lens
```

## Settings

Edit [`navigation_config.h`](../navigation_config.h):

| Setting | Default | Meaning |
| --- | --- | --- |
| `SEARCH_TURN_SPEED_DEG_S` | `30.0f` | Requested search rotation speed in degrees/second. |
| `SEARCH_DIRECTION` | `1` | `1` = counterclockwise, `-1` = clockwise, viewed from above. |
| `STOP_DISTANCE_CM` | `20.0f` | Desired minimum front-of-robot clearance to the tag plane. |
| `STOP_MARGIN_CM` | `5.0f` | Additional distance for earlier stopping. |
| `CAMERA_TO_FRONT_CM` | `0.0f` | Measure and enter the lens-to-front-bumper distance; positive when the lens is behind the bumper. |
| `APPROACH_SPEED_CM_S` | `6.0f` | Maximum approach wheel speed. |
| `MAX_APPROACH_STEP_CM` | `5.0f` | Maximum encoder distance before stopping to re-plan. |
| `ALIGN_TOLERANCE_DEG` | `3.0f` | Bearing error allowed before forward travel. |
| `CAMERA_POLL_INTERVAL_MS` | `100` | Time between camera queries during motion, excluding query duration. |
| `FIRST_TAG_ID`, `LAST_TAG_ID` | `1`, `3` | Original repeating route. Set both to `1` to approach tag 1 and remain stopped. |

Search ramps up using `SLEW_ACCEL_CM_S2` and the wheel-speed PID. Its wheel speed
is capped by `TURN_SPEED_CM_S` in `driving.ino`. With the current 17.5 cm track,
30 degrees/second is about 4.58 cm/s at each wheel. Requested speeds below
`MIN_MOTION_SPEED_CM_S` are rejected; requests above the turning speed are capped.
The finite alignment and approach moves keep V2's encoder-based speed profiles.
Approach speeds are capped by `DRIVE_SPEED_CM_S` and must meet the minimum speed.

## Range, bearing, and calibration

The camera reads height and horizontal position from the **same frame** of the
selected tag:

```text
range_cm = RANGE_HEIGHT_SCALE_CM_PX / height_px + RANGE_OFFSET_CM
bearing_deg = atan2(CAMERA_CENTER_X_PX - xCenter_px, CAMERA_FOCAL_LENGTH_PX) * 180 / PI
front_clearance_cm = range_cm - CAMERA_TO_FRONT_CM
```

A tag left of center produces a positive CCW angle; a tag right of center
produces a negative clockwise angle. The robot calls `rotateCCW(bearing_deg)`
instead of applying a fixed correction angle, then checks the camera again.

The initial range fit, `3489 / height - 3`, comes from the supplied TME code.
Recalibrate for the actual printed tag size and mounting. Record the apparent
heights `h1`, `h2` at two measured camera-to-tag distances `d1`, `d2`:

```text
scale = (d1 - d2) / (1/h1 - 1/h2)
offset = d1 - scale/h1
```

Use floating-point arithmetic and validate the fit at additional distances,
especially around the stopping threshold. Keep the tag upright and facing the
camera for calibration.

`CAMERA_FOCAL_LENGTH_PX = 280` is an initial estimate, **not a measured camera
specification**. Place a tag at a known bearing and calculate
`focal_length_px = abs(xCenter_px - center_x_px) / tan(abs(bearing_radians))`.
Check several positions on both sides, and adjust `CAMERA_CENTER_X_PX` if needed.
Also calibrate wheel dimensions, encoder counts, speed limits, and PID as for V2.

## Motion and stopping

The robot waits stopped until the camera connects. If the requested tag is
missing, it rotates continuously in the configured direction with no search
deadline. Encoder stall detection remains active.

When the tag appears, it brakes, estimates bearing, and aligns. It then advances
in steps no larger than `MAX_APPROACH_STEP_CM`. During both alignment and forward
travel, the movement monitor queries the camera: a missing tag brakes the move
before searching again; changed bearing brakes forward motion before realigning;
the stopping threshold brakes immediately. Each completed step also brakes and
rechecks range. A tag already inside the threshold causes no reverse motion.

Communication errors, invalid/clipped detections, or encoder movement faults
brake both motors and stop navigation until reset. Stopping bypasses the slew
ramp. The HuskyLens library's serial requests are synchronous and can briefly
delay PID updates, so allow for query time and braking travel in the margin.

Clearance is an estimate to the selected tag plane. Tag tilt, the range fit,
mounting offset, and physical braking affect actual stopping distance; the camera
does not detect unrelated obstacles. Measure these on the robot before reducing
the margin.

Software checks are described in [tests/husky_lens](../../tests/husky_lens/README.md).
