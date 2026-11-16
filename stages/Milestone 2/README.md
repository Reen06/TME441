# Milestone 2: Tag Tracking

Active camera-only implementation of the assignment route: **1 -> 2 -> 3 -> HALT**.

Open [tme441-milestone-2.ino](tme441-milestone-2/tme441-milestone-2.ino) in Arduino IDE.
Select **Arduino Mega 2560**. All the other files in that sketch folder are tabs
of the same program. Serial Monitor is **115200 baud**; the camera UART is **9600**.
The former `husky_lens/` sketch and its tests now live in this stage.

| Folder | Contents |
| --- | --- |
| `tme441-milestone-2/` | Active sketch, `driving.ino`, `camera.ino`, `navigation.ino`, and settings. |
| [docs/](docs/README.md) | Wiring, behavior, status messages, and settings. |
| [docs/CALIBRATION_AND_ARENA_TESTS.md](docs/CALIBRATION_AND_ARENA_TESTS.md) | Required camera measurements and physical acceptance tests. |
| [tests/](tests/README.md) | Simulated checks of the actual code, including the complete three-tag route. |
| `reference/` | Original supplied camera library and Milestone 2 package, preserved unchanged. |

The camera fit and mounting offset still need measurements on the actual robot.
Compilation and simulated checks cannot establish physical stopping accuracy.

## Software behavior

- Powers on autonomously and checks camera communication and learned tag count.
- `rangeAndHeadingToTag(int)` returns a struct with range in cm and heading in degrees from one response.
- `driveToTag(int)` returns `1` on arrival, `-1` on unsuccessful search, and `-2` on other faults.
- Visits each target exactly once, never skips a failed tag, and holds the brakes after tag 3.
- Searches by pivoting in place; normal approach steps retain camera monitoring.
- Plans only the last 5 cm or less from a fresh, centered camera pose if the tag leaves view near the robot.

Future dead-reckoning, range/heading experiments, and Milestone 3 each have their
own placeholder folder beside this one. They are not additional upload choices yet.
