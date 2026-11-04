# HuskyLens navigation checks

`test_navigation.cpp` includes the actual `camera.ino`, `navigation.ino`, and
`driving.ino` from `husky_lens/`. It supplies a queued camera, simulated encoder
movement, a clock, and PID command recording. The existing motor PID is replaced
with a command recorder for these navigation tests.

From the project root, with a native C++ compiler:

```powershell
c++ -std=c++11 -O2 tests/husky_lens/test_navigation.cpp -o "$env:TEMP/husky_navigation_tests.exe"
& "$env:TEMP/husky_navigation_tests.exe"
```

The checks cover same-frame height/bearing geometry, left/right turn signs,
invalid and clipped detections, continuous search past 60 seconds, slew, stall
stopping, V2 encoder targets, interruption at the stopping distance, tag loss and
reacquisition, camera failure, and alignment changes during approach.

Verify the full sketch with the installed Arduino libraries:

```powershell
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 --libraries .vscode/libraries --library husky_lens/reference/HUSKYLENSArduino/HUSKYLENS husky_lens
```

The [navigation guide](../../husky_lens/docs/README.md) explains camera and
clearance calibration. Simulated tests do not establish physical stopping accuracy.
