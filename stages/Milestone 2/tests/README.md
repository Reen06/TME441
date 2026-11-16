# Milestone 2 software checks

`test_navigation.cpp` includes the active `camera.ino`, `navigation.ino`, and
`driving.ino`. Camera responses, encoder motion, time, Serial logs, and PID commands
are simulated. Checks cover geometry, signs, clipping recovery, the bounded final
move, braking, camera retries, search/deadline failures, and the complete route.

From the workspace root with a native C++ compiler:

```powershell
c++ -std=c++11 -O2 "stages/Milestone 2/tests/test_navigation.cpp" -o "$env:TEMP/milestone2_tests.exe"
& "$env:TEMP/milestone2_tests.exe"
```

Or use `./stages/Milestone 2/tests/run_tests.ps1` with `-Compiler` set to a native
C++ compiler or a Zig executable. The runner recognizes Zig and uses its C++ driver.

Verify the actual Arduino entry point, preprocessing, libraries, and Mega target:

```powershell
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 --library "stages/Milestone 2/reference/HUSKYLENSArduino/HUSKYLENS" "stages/Milestone 2/tme441-milestone-2"
```

If `arduino-cli` is not on PATH, Arduino IDE installs it at
`C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe`.
Install **Encoder by Paul Stoffregen** in Arduino IDE's Library Manager first.
In the original workspace, if you use its local Encoder copy instead, append
`--libraries .vscode/libraries`. That local folder is not part of this stage.
Build output should go in a fresh temporary directory, not the source tree.

Regression scenarios include successful approaches for **all three IDs**, searching
for tag 2 after tag 1, no wraparound after tag 3, and stopping rather than skipping
when tag 2 is missing or its camera requests fail. These checks address the gap in
the earlier tests, which exercised only individual `driveToTag(1)` calls.

Physical calibration and acceptance trials are in
[CALIBRATION_AND_ARENA_TESTS.md](../docs/CALIBRATION_AND_ARENA_TESTS.md).
