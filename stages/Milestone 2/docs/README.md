# Milestone 2 camera and ultrasonic operation

Upload [the active sketch](../tme441-milestone-2/tme441-milestone-2.ino), Arduino Mega
2560. Serial Monitor is **115200 baud**. A reset starts the selected mode again.

## Confirmed wiring

| Device pin | Arduino Mega connection |
| --- | --- |
| HuskyLens TX | RX3, D15 |
| HuskyLens RX | TX3, D14 |
| HuskyLens power / ground | 5V / common GND |
| HC-SR04 VCC | 5V |
| HC-SR04 GND | Common GND |
| HC-SR04 TRIG | D22 |
| HC-SR04 ECHO | D23 |

The user confirmed the HC-SR04 front face is level with the camera lens. Camera
range therefore uses `CAMERA_TO_STOP_REFERENCE_CM = 0`, and ultrasound measures
directly from the required front face. Re-measure the offset if either mount moves.
These signal pins do not overlap the existing motor, encoder, or camera pins.

Set HuskyLens to **Tag Recognition**, protocol **Serial 9600**, and learn the
three signs as **IDs 1, 2, 3**. Verify each ID on its LCD. Startup prints learned
ID count and halts if too few IDs are learned. Install **Encoder by Paul Stoffregen**
and the supplied [HUSKYLENS ZIP](../reference/HUSKYLENSArduino/HUSKYLENS.zip).

## Calibration mode and CSV

`CAMERA_CALIBRATION_MODE = true` currently keeps motors braked and prints both
sensors for `CALIBRATION_TAG_ID = 2`:

```text
time_ms,tag_id,status,x_px,height_px,range_cm,heading_deg,ultrasonic_status,ultrasonic_cm
```

The original seven camera columns keep their order. Camera status: `1` valid,
`0` missing target, `2` partial/clipped, `-2` communication/invalid-data error.
Ultrasonic status: `1` valid, `0` echo timeout, `-1` invalid/out-of-range, `-2` not
ready. Invalid ultrasonic distance is `nan`, not zero cm. Below the camera's
observed 27 cm limit, camera status may be zero while ultrasonic status is still
one and the final column shows a usable distance.

Measure against the actual board at 30, 20, and 10 cm from the HC-SR04 face.
Compare the last column to a ruler before autonomous trials. The camera's -1.5 cm
bias correction is not applied to ultrasonic measurements.

## Autonomous route and handoff

Set calibration mode **false**, verify, and upload when sensor measurements check
out. The robot runs **1 -> 2 -> 3 -> HALT** once, with no attached cable required.
`driveToTag(int)` returns `1` for arrival, `-1` for unsuccessful search, and `-2`
for other faults. The route advances only on `1`; failures never skip a tag.

The camera supplies the correct target ID and heading. Normal forward steps stop
at the **35 cm handoff**, leaving margin before observed visibility loss at 27 cm.
While braked, the robot requires three fresh sonar pings agreeing within 2 cm,
using at most five attempts. Their median must agree with camera clearance within
6 cm. A disagreement is a fault, since ultrasound might be seeing another object.

After handoff the target identity is latched. The robot advances straight in steps
of **at most 1 cm** at 3 cm/s. During each step, live sonar is checked every 70 ms
or later. A reading at 8 cm or less brakes immediately. Between steps, stationary
confirmation accepts arrival only when three fresh agreeing readings have median
at most **9 cm** and their largest value is at most **10 cm**. Thus a single short
spurious echo can stop a move but cannot mark the tag reached.

Camera communication and visible heading are rechecked while braked between close
steps. Missing/partial tag images are expected below 27 cm and do not start search
pivots after handoff. A visible heading outside tolerance requests realignment of
the same target. A persistent camera communication failure still returns `-2`.

Close movement stops on missing, invalid, or stale sonar data. There is no blind
encoder-only completion fallback. Encoder stalls, an encoder travel budget based
on handoff range, and the 120-second per-tag deadline also stop the approach.
After arrival at tag 3 the motors remain braked until reset.

Ultrasound measures the nearest usable reflection in its beam; it does not identify
AprilTags or guarantee that a reflector is the sign. Correct alignment, camera/range
agreement at handoff, clear approach paths, and physical tests remain necessary.

## Sensor timing and settings

The driver uses a 10 us trigger, `echo_us / 58.0` for cm, an explicit **12 ms** echo
timeout, and **70 ms** or more between pings. It rejects readings outside 2-200 cm.
The bounded echo wait avoids Arduino's long default timeout in the movement loop.
See the [HC-SR04 datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
and [Arduino pulseIn reference](https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/).

Edit [navigation_config.h](../tme441-milestone-2/navigation_config.h):

| Setting | Value | Purpose |
| --- | --- | --- |
| `CAMERA_MIN_VISIBLE_CM` | 27 | User-observed camera limit. |
| `CAMERA_ULTRASONIC_HANDOFF_CM` | 35 | Transfer distance control while tag is still visible/aligned. |
| `HANDOFF_RANGE_TOLERANCE_CM` | 6 | Maximum camera/sonar disagreement before handoff. |
| `ULTRASONIC_TRIG_PIN`, `ULTRASONIC_ECHO_PIN` | 22, 23 | Confirmed HC-SR04 signal pins. |
| `ULTRASONIC_APPROACH_STEP_CM` | 1 | Maximum near-range movement between stationary checks. |
| `ULTRASONIC_APPROACH_SPEED_CM_S` | 3 | Close movement speed. |
| `STOP_TARGET_CM` | 8 | Intended target and immediate moving brake threshold. |
| `ULTRASONIC_STOP_CONFIRM_CM` | 9 | Maximum median for stationary arrival confirmation. |
| `MAX_STOP_CLEARANCE_CM` | 10 | Largest allowed confirmation sample. |
| `ULTRASONIC_MAX_APPROACH_TRAVEL_CM` | 35 | Hard travel cap, further limited by initial sonar range. |
| `ULTRASONIC_TRAVEL_MARGIN_CM` | 3 | Small allowance in the handoff-derived encoder budget. |
| `ULTRASONIC_PING_INTERVAL_MS` | 70 | Minimum delay between pings. |
| `ULTRASONIC_ECHO_TIMEOUT_US` | 12000 | Maximum echo wait. |
| `ULTRASONIC_FRESHNESS_MS` | 200 | Maximum sample age for use. |
| `ULTRASONIC_STABLE_SPREAD_CM` | 2 | Maximum spread in a three-ping confirmation cluster. |
| `TAG_SEARCH_TIMEOUT_MS` | 45000 | Missing target search deadline. |
| `TAG_NAVIGATION_TIMEOUT_MS` | 120000 | Complete per-tag deadline. |

The camera fits, original observations, and independent -1.5 cm camera correction
are preserved in the [calibration worksheet](CALIBRATION_AND_ARENA_TESTS.md).

## Diagnostic messages

```text
[route] starting target=1
[handoff] target=1 camera_cm=... ultrasonic_cm=...
[ultrasonic] target=1 clearance_cm=...
[arrival] confirmed by three ultrasonic pings, motors braked
[tag] ID=1 status=1
[route] reached target=1
[route] starting target=2
...
[route] COMPLETE: tags 1 -> 2 -> 3. Motors braked until reset.
```

A handoff disagreement suggests an obstruction, alignment, sensor reading, or
camera calibration problem. A close-movement echo fault suggests wiring, reflection,
or sensor issues; the robot stops instead of guessing that the path is clear.
Capture the complete target/status messages to diagnose a stop after tag 1.
