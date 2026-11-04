// Executes the real camera, navigation, and driving tabs with simulated I/O.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using std::isfinite;
#define PI 3.14159265358979323846
#define F(text) text
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(a) ((a) < 0 ? -(a) : (a))
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#include "../../husky_lens/navigation_config.h"
#include "../../husky_lens/camera.h"
#include "../../husky_lens/driving.h"

unsigned long clock_ms;
long command_l, command_r;
bool encoders_moving = true;
int brake_transitions, reset_count;
struct Command { long l, r; };
std::vector<Command> commands;
struct FakeEncoder {
  double position = 0;
  long read() { return std::lround(position); }
} enc_l, enc_r;

unsigned long millis() { return clock_ms; }
void delay(unsigned long ms) {
  clock_ms += ms;
  if (encoders_moving) {
    enc_l.position += double(command_l) * ms / 1000.0;
    enc_r.position += double(command_r) * ms / 1000.0;
  }
}
void updatePID(long l, long r, bool) {
  if ((command_l || command_r) && !l && !r) ++brake_transitions;
  command_l = l;
  command_r = r;
  commands.push_back({l, r});
}
void resetPID() { ++reset_count; }
struct FakeSerial {
  template <typename T> void print(T) {}
  template <typename T> void println(T) {}
} Serial;

const int COMMAND_RETURN_BLOCK = 1;
struct HUSKYLENSResult {
  int command, ID, xCenter, yCenter, width, height;
};
struct Frame { bool ok; std::vector<HUSKYLENSResult> blocks; };
struct FakeCamera {
  std::vector<Frame> frames;
  Frame current = {true, {}};
  unsigned index = 0, block_index = 0, requests = 0;
  bool requestBlocks(int) {
    ++requests;
    if (index < frames.size()) current = frames[index++];
    block_index = 0;
    return current.ok;
  }
  bool available() { return block_index < current.blocks.size(); }
  HUSKYLENSResult read() { return current.blocks[block_index++]; }
} cam;

#include "../../husky_lens/driving.ino"
#include "../../husky_lens/camera.ino"
#include "../../husky_lens/navigation.ino"

int checks = 0;
void check(bool condition, const char *message) {
  ++checks;
  if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
Frame tag(int x, int height, int id = 1) {
  return {true, {{COMMAND_RETURN_BLOCK, id, x, 120, 40, height}}};
}
Frame missing() { return {true, {}}; }
void reset() {
  clock_ms = 0;
  command_l = command_r = 0;
  enc_l.position = enc_r.position = 0;
  encoders_moving = true;
  brake_transitions = reset_count = 0;
  commands.clear();
  cam = FakeCamera();
  setMotionMonitor(0);
  movement_interrupted = false;
  last_camera_ms = 0;
  watched_tag_id = 1;
  watched_pose = {};
}
bool timed_monitor() { return millis() < 65000UL; }

void test_camera_geometry() {
  reset();
  cam.frames = {tag(160, 70), tag(120, 70), tag(200, 140)};
  TagPose center = readTagPose(1), left = readTagPose(1), right = readTagPose(1);
  check(center.status == TAG_VISIBLE && abs(center.bearing_deg) < 0.001f,
        "centered tag has zero bearing");
  check(abs(center.range_cm - 46.842857f) < 0.001f, "height determines range");
  check(abs(left.bearing_deg - 8.130102f) < 0.001f, "left pixel offset gives CCW angle");
  check(abs(right.bearing_deg + 8.130102f) < 0.001f, "right pixel offset gives CW angle");
  check(right.range_cm < center.range_cm, "larger tag is closer");
  check(cam.requests == 3, "range and bearing share one camera frame");

  cam.frames = {tag(160, 70, 2), tag(160, 0), tag(10, 70), {false, {}}};
  cam.index = 0;
  check(readTagPose(1).status == TAG_MISSING, "ignore other tag IDs");
  check(readTagPose(1).status == TAG_ERROR, "reject zero-height detections");
  check(readTagPose(1).status == TAG_ERROR, "reject clipped tag range estimates");
  check(readTagPose(1).status == TAG_ERROR, "propagate communication errors");
}

void test_continuous_search_and_stalls() {
  reset();
  setMotionMonitor(timed_monitor);
  check(!rotateContinuously(30), "monitor can interrupt continuous search");
  check(wasMotionInterrupted(), "search distinguishes interruption from fault");
  check(clock_ms >= 65000, "missing tag search continues past old movement timeout");
  check(enc_l.read() < 0 && enc_r.read() > 0, "positive search rate turns CCW");
  check(brake_transitions == 1 && reset_count == 1,
        "search does not brake or reset PID between camera checks");
  check(command_l == 0 && command_r == 0, "interruption brakes both wheels");
  long previous = 0;
  bool ramp_ok = true;
  for (const Command &command : commands) {
    if (!command.r) continue;
    if (command.r < previous || command.r - previous > 1 || command.r > 298) ramp_ok = false;
    previous = command.r;
  }
  check(ramp_ok, "continuous search slews up without exceeding configured speed");

  reset();
  setMotionMonitor(timed_monitor);
  rotateContinuously(-30);
  check(enc_l.read() > 0 && enc_r.read() < 0, "negative search rate turns clockwise");

  reset();
  encoders_moving = false;
  check(!rotateContinuously(30) && !wasMotionInterrupted(), "stalled search reports fault");
  check(clock_ms >= NO_PROGRESS_TIMEOUT_MS && command_l == 0 && command_r == 0,
        "stall timeout brakes both wheels");
}

void test_finite_moves() {
  reset();
  check(driveDistance(5, 6), "V2 finite approach completes at encoder target");
  check(enc_l.read() >= 324 && enc_r.read() >= 324, "both approach encoders reach target");
  check(command_l == 0 && command_r == 0, "finite approach ends braked");
  reset();
  check(rotateCCW(-8), "V2 finite clockwise turn completes");
  check(enc_l.read() > 0 && enc_r.read() < 0, "finite turn uses correct wheel signs");
  reset();
  check(!driveDistance(5, NAN), "reject non-finite approach speed");
  check(!rotateContinuously(INFINITY), "reject non-finite search speed");
}

void test_navigation_stops_and_reacquires() {
  reset();
  check(navigationSettingsValid(), "default navigation settings are valid");
  cam.frames = {tag(160, 140)};
  check(driveToTag(1) == 1, "already-close tag needs no move");
  check(enc_l.read() == 0 && enc_r.read() == 0, "already-close tag never drives backward");

  reset();
  cam.frames = {tag(160, 60), tag(160, 140)};
  check(driveToTag(1) == 1, "approach stops on a close camera frame during the move");
  check(enc_l.read() > 0 && enc_l.read() < 65, "close frame interrupts before full five-cm step");
  check(command_l == 0 && command_r == 0, "arrival holds both motors braked");

  reset();
  cam.frames = {tag(160, 60), missing(), missing(), missing(), tag(160, 140)};
  check(driveToTag(1) == 1, "lost tag is searched for again and reacquired");
  bool drove = false, searched = false;
  for (const Command &command : commands) {
    drove |= command.l > 0 && command.r > 0;
    searched |= command.l < 0 && command.r > 0;
  }
  check(drove && searched && brake_transitions >= 2, "tag loss brakes approach before search");

  reset();
  cam.frames = {tag(160, 60), {false, {}}};
  check(driveToTag(1) == -2, "communication loss aborts active approach");
  check(command_l == 0 && command_r == 0, "camera fault leaves both wheels braked");

  reset();
  cam.frames = {tag(160, 60), tag(110, 60), tag(110, 60), tag(160, 60),
                tag(160, 60), tag(160, 140)};
  check(driveToTag(1) == 1, "approach realigns when tag bearing drifts");
  bool aligned = false;
  for (const Command &command : commands) aligned |= command.l < 0 && command.r > 0;
  check(aligned, "bearing drift produces a corrective CCW turn");
}

int main() {
  test_camera_geometry();
  test_continuous_search_and_stalls();
  test_finite_moves();
  test_navigation_stops_and_reacquires();
  std::printf("PASS: %d HuskyLens camera/movement/navigation checks.\n", checks);
}
