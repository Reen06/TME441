# Camera calibration and Milestone 2 acceptance

Initial physical range observations are recorded below. Continue this worksheet
with the actual camera, printed signs, mounting, wheels, and battery-powered robot.

## Initial tag-2 observations (2026-11-16)

User-reported measurements, with timestamps omitted. All have `status=1`, meaning
a full, valid detection. The user confirmed all distances are measured from the
**camera lens**, with the tag centered in front of the camera. No lens-to-front
mounting offset is inferred from these readings.

| Reported distance (cm) | Tag ID | Status | X (px) | Height (px) | Original estimated range (cm) | Original estimated heading (deg) |
| --- | --- | --- | --- | --- | --- | --- |
| 30 | 2 | 1 | 171 | 116 | 27.08 | -2.25 |
| 60 | 2 | 1 | 166 | 58 | 57.16 | -1.23 |
| 90 | 2 | 1 | 160 | 38 | 88.82 | 0.00 |

A preliminary least-squares fit of reported distance versus inverse height is:

```text
reported_distance_cm = 3390.0262927 / height_px + 1.0385627
```

| Reported distance (cm) | Fitted value (cm) | Fit residual (cm) |
| --- | --- | --- |
| 30 | 30.263 | +0.263 |
| 60 | 59.487 | -0.513 |
| 90 | 90.250 | +0.250 |

The range fit RMSE on these three fitting points is **0.363 cm**, with a maximum
absolute fitting residual of **0.513 cm**. This is not an independent validation
error or a bound on physical stopping accuracy. These fitted range constants
are now applied in `navigation_config.h`.

## Lateral-offset observations (2026-11-16)

The user kept the sign perpendicular to the camera axis, translated it left/right,
and measured forward depth from the lens. These are translations, not rotations
of the sign. For rightward lateral offset `L` at forward depth `D`, the ground-truth
heading is `-atan2(L, D)` in degrees. Leftward offset gives positive/CCW heading.
All six observations have tag ID 2 and status 1. Original display values are
preserved below; the physical offsets and raw pixels are the fitting inputs.

| Forward depth (cm) | Side | Lateral offset (cm) | X (px) | Height (px) | Original displayed range (cm) | Original displayed heading (deg) |
| --- | --- | --- | --- | --- | --- | --- |
| 30 | R | 7.25 | 250 | 112 | 28.15 | -17.82 |
| 30 | L | 7.25 | 103 | 112 | 28.15 | 11.51 |
| 60 | R | 19.25 | 271 | 58 | 59.30 | -21.62 |
| 60 | L | 19.25 | 76 | 56 | 59.30 | 16.70 |
| 90 | R | 36.75 | 293 | 38 | 88.82 | -25.41 |
| 90 | L | 36.75 | 50 | 36 | 93.92 | 21.45 |

The 60 cm R row reports height 58 and displayed range 59.30; the previous formula
would give 57.16 at height 58. This apparent transcription mismatch is preserved,
not silently corrected. It does not affect the heading fit, which uses lateral
position, forward depth, and X only.

Least squares of `x_px = center_px + focal_px * right_offset_cm / forward_cm`,
using the three centered observations and six offset observations together, gives:

```text
CAMERA_CENTER_X_PX = 171.11111
CAMERA_FOCAL_LENGTH_PX = 300.7144
heading_deg = atan2(171.11111 - x_px, 300.7144) * 180 / PI
```

| Depth (cm) | Side/offset (cm) | True heading (deg) | Fitted heading (deg) | Residual (deg) |
| --- | --- | --- | --- | --- |
| 30 | Center | 0.000 | 0.021 | +0.021 |
| 60 | Center | 0.000 | 0.974 | +0.974 |
| 90 | Center | 0.000 | 2.116 | +2.116 |
| 30 | R 7.25 | -13.586 | -14.700 | -1.114 |
| 30 | L 7.25 | +13.586 | +12.762 | -0.824 |
| 60 | R 19.25 | -17.788 | -18.375 | -0.587 |
| 60 | L 19.25 | +17.788 | +17.551 | -0.237 |
| 90 | R 36.75 | -22.212 | -22.064 | +0.148 |
| 90 | L 36.75 | +22.212 | +21.937 | -0.275 |

Heading fitting RMSE is **0.934 degrees**, with a maximum absolute residual of
**2.116 degrees** on these nine observations. The model implies roughly **55.8
degrees** of horizontal image coverage, but the actual full-tag detection limits
at each distance remain to be measured. The symmetric L/R pairs imply centers
176.5, 173.5, and 171.5 pixels, while the centered observations range from 160 to
171 pixels. Placement noise, lens distortion, and detection noise have not been
separated by this small data set. Keep the current 3-degree alignment tolerance
until independent checks justify a tighter value.

Only the centered observations fit the range model because the robot aligns and
then re-reads the camera before forward travel. The lateral observations are an
off-axis range check: the 90 cm L sample gives about 95.21 cm with the centered
fit, an error of about 5.21 cm. This reinforces the need to align and acquire a
fresh centered range rather than drive from an old off-axis estimate.

## Centered validation correction (2026-11-16)

After applying the initial range/heading fit, the user reported that centered
physical distances are consistently about **1.5 cm less than the displayed range**.
No exact new distance/height pairs were supplied with this aggregate report. The
original raw observations and their original fitting statistics remain unchanged.

The calculation was checked for integer division, repeated offsets, and mounting
reference subtraction. Division uses floating point, the fitted offset is applied
once, and the mounting offset is zero as confirmed by the user. No calculation
error explaining this constant discrepancy was found. The aggregate observations
do not establish a specific physical cause.

A separate empirical correction is now applied in the camera range function:

```text
RANGE_BIAS_CORRECTION_CM = -1.5
range_cm = 3390.0263 / height_px + 1.038563 - 1.5
         = 3390.0263 / height_px - 0.461437
```

A former display of 21.50 cm now becomes 20.00 cm for the same camera data.
The correction affects both calibration output and navigation exactly once;
it does not change the heading fit, range scale, physical mounting offset,
or the 8 cm stopping target. The end-to-end final-movement test checks that the
planned travel is shortened by 1.5 cm, rather than applying the correction twice.

The 0.363 cm range RMSE reported above describes the **original three-point fit
before this later bias correction**, not the corrected model's current accuracy.
A new validation RMSE cannot be calculated from an approximate aggregate report.
Keep calibration mode enabled and check new centered measurements, especially
20 cm and the nearest distance with a full, valid tag detection. Record exact
physical distances, raw heights, and corrected readings to verify that the error
is actually constant across distances. Do not fit a new scale or alter the raw
historical observations without that evidence.

## Observed camera visibility and HC-SR04 integration (2026-11-16)

The user reports the tag cannot be seen closer than **27 cm**. This physical
observation supersedes the earlier extrapolation from tag height. The previous
camera-only 10 cm encoder finish could begin too late to bridge this visibility
gap. It has now been removed and replaced by ultrasonic distance control.

The user confirmed a **standard HC-SR04**, wired TRIG D22, ECHO D23, VCC 5V,
GND common, with its front face **level with the camera lens**. The stopping
reference is the ultrasonic face, so `CAMERA_TO_STOP_REFERENCE_CM = 0` remains
correct for this installation.

Handoff occurs while the correct tag is fully visible and aligned, at about
35 cm. Three fresh, agreeing sonar pings must agree with camera range within
6 cm. Close travel uses live sonar and <=1 cm encoder-bounded moves. The 8 cm
moving brake threshold and a stationary median <=9 cm, with all three confirming
readings <=10 cm, control arrival. Echo timeout/invalid/stale data cannot authorize
forward close travel or count as a zero-distance arrival. An encoder budget and
per-target deadline bound failure cases with a stuck numeric reading.

Calibration mode remains enabled for tag 2 and now appends `ultrasonic_status`
and `ultrasonic_cm` to the original seven-column CSV. Measure a flat sign board
at known **30, 20, and 10 cm from the ultrasonic face**, including the region where
camera status becomes missing. Record actual values below; none are fabricated.

| Physical face-to-board distance (cm) | Camera status/range | Ultrasonic status/range | Error / notes |
| --- | --- | --- | --- |
| 30 | | | |
| 20 | | | |
| 10 | | | |

The camera's -1.5 cm correction applies only to camera range, not to sonar.
Check several pings for stability and test timeout behavior with the wheels raised.
Only after these checks set calibration mode false for a supervised floor trial,
measure actual stopping clearance, then perform the full three-tag arena trials.
See the [operation guide](README.md) for timing, statuses, handoff, and fault logs.

## Camera measurements

1. Set `CAMERA_CALIBRATION_MODE = true` in the active `navigation_config.h`, verify,
   and upload. Motors stay braked. Serial Monitor at 115200 prints:
   `time_ms,tag_id,status,x_px,height_px,range_cm,heading_deg,ultrasonic_status,ultrasonic_cm`.
2. Use the actual printed tag. At known lens-to-tag distances (for example 20,
   30, 50, 80, and 120 cm), record repeated heights and pixel centers. Keep the
   board upright and facing the camera. Use `status=1` full detections only.
3. Fit `range = scale / height + offset`. Two measured points give:
   `scale = (d1 - d2) / (1/h1 - 1/h2)` and `offset = d1 - scale/h1`.
   Use floating-point arithmetic, then validate at additional distances.
4. Hold the tag at measured headings on both sides (for example -15, -10, -5,
   0, 5, 10, 15 degrees). Fit/check `CAMERA_CENTER_X_PX` and
   `CAMERA_FOCAL_LENGTH_PX`. One off-center point gives
   `focal_px = abs(x_px - center_px) / tan(abs(known_heading_radians))`.
5. Check the full field of view and the closest distance at which the whole tag
   remains visible. Measure the camera lens distance behind the stopping reference
   face, and enter `CAMERA_TO_STOP_REFERENCE_CM`.
6. Record the fitted parameters and errors below, then set calibration mode back
   to **false** and upload the autonomous version.

| Known lens distance (cm) | Known heading (deg) | Height (px) | Center X (px) | Estimated range (cm) | Estimated heading (deg) |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
| | | | | | |
| | | | | | |
| | | | | | |

| Parameter | Measured/fitted value | Validation error or notes |
| --- | --- | --- |
| `RANGE_HEIGHT_SCALE_CM_PX` | 3390.0263 | Three centered lens-distance samples. |
| `RANGE_OFFSET_CM` | 1.038563 | Original centered fit, before the separate validation correction. |
| `RANGE_BIAS_CORRECTION_CM` | -1.5 | User reports a roughly constant 1.5 cm range overestimate; new validation pending. |
| `CAMERA_CENTER_X_PX` | 171.11111 | Joint fit of nine centered/lateral observations. |
| `CAMERA_FOCAL_LENGTH_PX` | 300.7144 | Heading fitting RMSE 0.934 degrees; independent checks pending. |
| `CAMERA_TO_STOP_REFERENCE_CM` | 0 | User confirms lens at robot front edge. |
| Full horizontal field of view | About 55.8 degrees from model | Actual full-tag detection limits pending. |
| Closest usable tag visibility | About 27 cm | User-observed; ultrasonic handoff is set to 35 cm. |

The initial `3489`, `-3`, and focal length `280` were inherited/estimated values;
they have now been replaced by the fits above. The 8 cm target gives an intended 2 cm allowance inside
the 10 cm limit; it does not substitute for measuring error and braking travel.
If that allowance is smaller than the observed uncertainty, improve calibration
or reduce approach speed and validate again before a full trial.

## Motor and route checks

1. Confirm encoder direction, straight travel, CW/CCW signs, wheel diameter,
   wheelbase, and counts/revolution. Check for wheel slip during the final move.
2. With the wheels raised, confirm motor braking, all three learned tag IDs, and
   that startup says `1 -> 2 -> 3`. Use the Serial target/status logs to diagnose.
3. On the floor, verify tag 1 alone is approached without collision, then present
   tags 2 and 3 in the specified relative visibility arrangement. Confirm pivoting
   finds the next tag and that each ID gets its own arrival message.
4. Measure each final distance from the **front of the mounted ultrasonic sensor**.
   Verify camera-to-sonar handoff before the 27 cm visibility loss and a confirmed
   stop inside 10 cm without collision.
5. Remove/cover the next tag: the robot must brake on visibility loss and search,
   then return `-1` and halt after the search deadline if it remains absent.
6. Cover/disconnect the ultrasonic sensor with wheels raised: close movement must
   brake and return `-2`, with no blind encoder finish.
7. Disconnect camera communication with wheels raised: movement must brake and
   persistent failure must produce `-2`. Block wheel motion with wheels raised
   to confirm stall stopping. The route must not skip a failed tag.

## Arena trials and team readiness

Use the actual 150 x 150 cm arena, three letter-sized sign boards, battery power,
no cable/tether, and several random starting positions and orientations. Each
next tag must be visible from the previous one. Confirm all of the following:

- Correct order **1, 2, 3**, with no collision and measured clearance at most 10 cm.
- Motors remain braked after tag 3; it never restarts at tag 1 without a reset.
- Secure mounting, organized color-coded wiring, and no tangled/excess wiring.
- Every teammate can explain same-frame range/heading geometry, calibration,
  encoder driving, target transitions, error statuses, and test evidence.

| Trial | Start position/orientation | Tag 1 clearance (cm) | Tag 2 clearance (cm) | Tag 3 clearance (cm) | Order/halts correctly | Collision? | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |

Save the measured results and Serial logs with this stage. Software checks are
listed in [tests/README.md](../tests/README.md); they simulate I/O and cannot verify
assembly, actual camera error, battery behavior, or physical stopping distances.
