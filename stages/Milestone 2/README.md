# Milestone 2: Tag Tracking

Active implementation: **camera identifies/aligns tags, HC-SR04 measures the final
clearance, route 1 -> 2 -> 3 -> HALT**.

Open [tme441-milestone-2.ino](tme441-milestone-2/tme441-milestone-2.ino) in Arduino IDE.
Select **Arduino Mega 2560**. Serial Monitor is **115200 baud** and the camera UART
is **9600**. Calibration mode currently remains enabled for tag 2, with motors braked.

| Folder | Contents |
| --- | --- |
| `tme441-milestone-2/` | Active sketch, driving, camera, ultrasonic, navigation, and settings. |
| [docs/](docs/README.md) | Wiring, camera/ultrasonic handoff, statuses, and settings. |
| [Calibration and arena tests](docs/CALIBRATION_AND_ARENA_TESTS.md) | Raw measurements, fitted camera model, observed visibility limit, and physical checks. |
| [tests/](tests/README.md) | Actual camera, HC-SR04, driving, and full-route code exercised with simulated I/O. |
| `reference/` | Original supplied packages, preserved unchanged. |

## Current behavior

- Check the camera and learned IDs at startup, then visit IDs 1, 2, 3 once.
- `rangeAndHeadingToTag(int)` estimates range and heading together before alignment.
- Search and align by camera; approach to about **35 cm** while the tag is visible.
- Require three fresh, agreeing HC-SR04 pings whose range agrees with the locked,
  aligned camera target. The reported camera visibility cutoff is about **27 cm**.
- Approach straight in encoder-bounded steps of at most **1 cm**, with live
  ultrasonic monitoring. Losing the close-range tag image does not start a pivot.
- Aim for 8 cm. Confirm arrival while braked using three agreeing pings with median
  at most **9 cm**, and all three readings at most **10 cm**, from the sensor face.
- Invalid/timeout echo during close movement brakes and returns `-2`. A travel
  budget and deadline prevent an unchanging far reading from driving indefinitely.
- A failed tag is never skipped. After tag 3, hold the brakes until reset.

The earlier camera-only encoder finish has been replaced. Initial camera fits and
a separate -1.5 cm correction are recorded; physical sonar and stopping checks
remain necessary. Future stages have separate folders beside this one.
