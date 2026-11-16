# Milestone 2 operation and wiring

Upload [the active sketch](../tme441-milestone-2/tme441-milestone-2.ino).
The sketch runs its mission once at startup. A reset restarts at tag 1.
Use Serial Monitor at **115200 baud** to see the current target and stop reason.

## Camera wiring and preparation

| Camera connection | Arduino Mega 2560 |
| --- | --- |
| TX | RX3, digital 15 |
| RX | TX3, digital 14 |
| Power | Regulated 5V |
| Ground | Common GND |

Use the supplied mount and secure the camera. Set its protocol to **Serial 9600**.
Select **Tag Recognition**, learn the three printed AprilTags as IDs **1, 2, 3**,
and check each label on the camera LCD. The program selects the algorithm again
at startup and prints the learned-ID count. An insufficient count stops the robot
before driving. A learned count does not prove that the correct signs received the
correct IDs: check the LCD yourself.

Install **Encoder by Paul Stoffregen** in Arduino IDE and add the supplied
[HUSKYLENS library ZIP](../reference/HUSKYLENSArduino/HUSKYLENS.zip).
Motor/encoder pins and PID remain in `motor_control.h`; the V2 driving code is in
`driving.ino`. The original supplied packages are in `reference/`.

## Route and return values

The route is **1 -> 2 -> 3**, then motors remain braked until reset. The route
advances only when `driveToTag(target)` returns `1`. A missing tag does not cause
it to skip ahead.

| Result | Meaning and behavior |
| --- | --- |
| `1` | Arrived using a valid close camera measurement or the completed bounded final encoder move. Advance to the next ID; halt after ID 3. |
| `-1` | No usable target found within 45 seconds of continuous search. Brake and report the failed ID. |
| `-2` | Persistent camera error, encoder stall, invalid settings, or 120-second navigation deadline. Brake and report the failed ID. |

An isolated camera read error interrupts the movement immediately. Up to three
reads are then attempted while stopped. A partial/clipped detection is distinguished
from communication failure: ordinary movement brakes and searches again until the
tag is fully visible. The timeouts also prevent repeated failed alignment from
running indefinitely.

## Geometry and stopping

`rangeAndHeadingToTag(tag_id)` uses only blocks matching the requested ID. It
calculates both estimates from the same camera response, before any alignment turn:

```text
range_cm = RANGE_HEIGHT_SCALE_CM_PX / height_px + RANGE_OFFSET_CM
heading_deg = atan2(CAMERA_CENTER_X_PX - x_px, CAMERA_FOCAL_LENGTH_PX) * 180 / PI
clearance_cm = range_cm - CAMERA_TO_STOP_REFERENCE_CM
```

Left of center is positive/CCW; right of center is negative/CW. The function
returns `TagPose`, which includes `status`, `range_cm`, and `heading_deg`.
Clipped tag height is never used for range.

The intended stopping point is **8 cm** from the reference front face, inside
the required **10 cm** limit. A centered valid measurement at 10 cm or less also
counts as arrival. The old 25 cm threshold has been removed. Measure the lens
position relative to the front of the ultrasonic sensor, or the robot front until
that sensor is fitted. Enter this positive rearward offset in
`CAMERA_TO_STOP_REFERENCE_CM`; its initial zero is a placeholder.

At close range the full tag may leave the image. When a fresh, fully visible,
centered tag measurement plans **5 cm or less** of remaining travel, the robot may
complete that fixed encoder target even if the image becomes partial or missing.
It never increases the target based on a clipped image. A communication error,
changed valid heading, encoder fault, or navigation deadline still stops this move.
This is a short encoder finish based on camera geometry, not independent obstacle
sensing. Wheel slip and the range fit affect the resulting physical distance.

## Settings

Edit [navigation_config.h](../tme441-milestone-2/navigation_config.h).

| Setting | Initial value | Purpose |
| --- | --- | --- |
| `SEARCH_TURN_SPEED_DEG_S` | `20` | Preserves the slower speed selected during robot testing. |
| `SEARCH_DIRECTION` | `1` | CCW; use `-1` for CW. |
| `STOP_TARGET_CM` | `8` | Intended clearance from the reference front face. |
| `MAX_STOP_CLEARANCE_CM` | `10` | Maximum accepted measured stopping clearance. |
| `CAMERA_TO_STOP_REFERENCE_CM` | `0`, unmeasured | Lens distance behind the reference front face. |
| `APPROACH_SPEED_CM_S` | `4` | Maximum forward speed. |
| `MAX_APPROACH_STEP_CM` | `3` | Ordinary movement before re-planning. |
| `MAX_FINAL_APPROACH_CM` | `5` | Maximum fixed encoder finish after a fresh centered measurement. |
| `ALIGN_TOLERANCE_DEG` | `3` | Heading tolerance before forward travel. |
| `TAG_SEARCH_TIMEOUT_MS` | `45000` | About 2.5 revolutions at 20 degrees/sec, excluding braking/ramp effects. |
| `TAG_NAVIGATION_TIMEOUT_MS` | `120000` | Total limit per target, including alignment and re-acquisition. |
| `CAMERA_CALIBRATION_MODE` | `false` | Set true for camera CSV with motors braked. |

Search wheel speed must meet the minimum speed in `driving.ino`. The 20 degree/sec
setting is just above that minimum with the present wheelbase. Calibrate the
wheelbase/encoder conversion and PID rather than reducing it below a usable speed.

## Diagnosing a stop after tag 1

Capture Serial output from startup through the stop. These messages locate it:

```text
[route] starting target=1
[tag] ID=1 status=1
[route] reached target=1
[route] starting target=2
...
[route] reached target=3
[route] COMPLETE: tags 1 -> 2 -> 3. Motors braked until reset.
```

If `starting target=2` never appears, inspect the preceding tag-1 status. If it
appears and then stops, the failure belongs to tag 2. `status=-1` means target
visibility/search; `status=-2` is accompanied by camera, movement, or deadline logs.
Check learned IDs, lighting, clear sight of the next sign, battery power, and the
camera cable while pivoting. The prior implementation stopped on any clipped
block, and its tests did not exercise the full route. Without the old Serial log,
that is a plausible cause of the reported behavior, not a confirmed diagnosis.

## Ultrasonic later

This stage uses camera and encoders only. For a **standard four-pin HC-SR04**, the
reserved wiring is:

| HC-SR04 label | Mega connection |
| --- | --- |
| VCC | 5V |
| GND | Common GND |
| TRIG | D22 |
| ECHO | D23 |

Follow the sensor's printed labels. These pins are free in this sketch. Other
models may need different wiring. See the [HC-SR04 datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
and [Mega pinout](https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf).
Connecting a sensor alone does not enable ranging; support is reserved for later.

Follow [calibration and arena tests](CALIBRATION_AND_ARENA_TESTS.md) before judging
the physical requirement complete.
