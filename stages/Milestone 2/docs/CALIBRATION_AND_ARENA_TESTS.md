# Camera calibration and Milestone 2 acceptance

No physical measurements are supplied here. Fill this worksheet with real data
from the actual camera, printed signs, mounting, wheels, and battery-powered robot.

## Camera measurements

1. Set `CAMERA_CALIBRATION_MODE = true` in the active `navigation_config.h`, verify,
   and upload. Motors stay braked. Serial Monitor at 115200 prints:
   `time_ms,tag_id,status,x_px,height_px,range_cm,heading_deg`.
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
| `RANGE_HEIGHT_SCALE_CM_PX` | | |
| `RANGE_OFFSET_CM` | | |
| `CAMERA_CENTER_X_PX` | | |
| `CAMERA_FOCAL_LENGTH_PX` | | |
| `CAMERA_TO_STOP_REFERENCE_CM` | | |
| Full horizontal field of view | | |
| Closest full-tag visibility | | |

The initial `3489`, `-3`, and focal length `280` are inherited/estimated values,
not experimental results. The 8 cm target gives an intended 2 cm allowance inside
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
4. Measure each final distance from the **front of the ultrasonic sensor**, once
   mounted. Until then use a marked robot-front reference; that temporary test
   does not establish the exact final sensor-reference requirement.
5. Remove/cover the next tag: the robot must brake on visibility loss and search,
   then return `-1` and halt after the search deadline if it remains absent.
6. Disconnect camera communication with wheels raised: movement must brake and
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
