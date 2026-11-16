// Runs the actual Milestone 2 camera/navigation/driving tabs with simulated I/O.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using std::isfinite;
#define PI 3.14159265358979323846
#define F(text) text
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(a) ((a) < 0 ? -(a) : (a))
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#include "../tme441-milestone-2/navigation_config.h"
#include "../tme441-milestone-2/camera.h"
#include "../tme441-milestone-2/driving.h"
#include "../tme441-milestone-2/navigation.h"

unsigned long clock_ms;
long command_l, command_r;
bool encoders_moving = true;
int brake_transitions, reset_count, requested_id;
struct Command { long l, r; int target; };
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
  command_l = l; command_r = r;
  commands.push_back({l, r, requested_id});
}
void resetPID() { ++reset_count; }
struct FakeSerial {
  std::ostringstream log;
  template <typename T> void print(T v) { log << v; }
  template <typename T> void println(T v) { log << v << '\n'; }
} Serial;
const int COMMAND_RETURN_BLOCK = 1;
struct HUSKYLENSResult { int command, ID, xCenter, yCenter, width, height; };
struct Frame { bool ok; std::vector<HUSKYLENSResult> blocks; };
struct FakeCamera {
  std::vector<Frame> frames;
  std::map<int, std::vector<Frame>> per_tag;
  std::map<int, unsigned> per_tag_index;
  std::vector<int> requested_ids;
  Frame current = {true, {}};
  unsigned index = 0, block_index = 0, requests = 0;
  bool requestBlocks(int id) {
    requested_id = id;
    requested_ids.push_back(id);
    ++requests;
    if (per_tag.count(id)) {
      const auto &script = per_tag[id];
      unsigned &i = per_tag_index[id];
      if (i < script.size()) current = script[i++];
      else if (!script.empty()) current = script.back();
    } else if (index < frames.size()) current = frames[index++];
    block_index = 0;
    return current.ok;
  }
  bool available() { return block_index < current.blocks.size(); }
  HUSKYLENSResult read() { return current.blocks[block_index++]; }
} cam;

#include "../tme441-milestone-2/driving.ino"
#include "../tme441-milestone-2/camera.ino"
#include "../tme441-milestone-2/navigation.ino"

int checks = 0;
void check(bool condition, const char *message) {
  ++checks;
  if (!condition) { std::fprintf(stderr, "FAIL: %s\n%s", message, Serial.log.str().c_str()); std::exit(1); }
}
Frame tag(int x, int height, int id = 1) {
  return {true, {{COMMAND_RETURN_BLOCK, id, x, 120, 40, height}}};
}
Frame missing() { return {true, {}}; }
Frame partial(int id = 1) { return tag(160, 240, id); }
void reset() {
  clock_ms = 0; command_l = command_r = 0;
  enc_l.position = enc_r.position = 0;
  encoders_moving = true; brake_transitions = reset_count = requested_id = 0;
  commands.clear(); cam = FakeCamera();
  Serial.log.str(""); Serial.log.clear();
  setMotionMonitor(0); movement_interrupted = false;
  last_camera_ms = 0; watched_tag_id = 1; watched_pose = {};
  searching = false; monitor_result = 0;
}
bool timed_monitor() { return millis() < 65000UL; }
void check_braked(const char *message) { check(command_l == 0 && command_r == 0, message); }

void test_camera_geometry() {
  reset();
  cam.frames = {tag(160, 70), tag(120, 70), tag(200, 140)};
  TagPose center = rangeAndHeadingToTag(1), left = rangeAndHeadingToTag(1), right = rangeAndHeadingToTag(1);
  check(center.status == TAG_VISIBLE && abs(center.heading_deg) < 0.001f, "centered tag has zero heading");
  check(abs(center.range_cm - 46.842857f) < 0.001f, "tag height determines range");
  check(abs(left.heading_deg - 8.130102f) < 0.001f, "left tag requires CCW turn");
  check(abs(right.heading_deg + 8.130102f) < 0.001f, "right tag requires CW turn");
  check(right.range_cm < center.range_cm, "larger tag is closer");
  check(cam.requests == 3, "range and heading use one response per tag");
  cam.frames = {tag(160, 70, 2), tag(160, 0), tag(10, 70), {false, {}}}; cam.index = 0;
  check(rangeAndHeadingToTag(1).status == TAG_MISSING, "other ID cannot satisfy requested tag");
  check(rangeAndHeadingToTag(1).status == TAG_ERROR, "invalid dimensions are an error");
  check(rangeAndHeadingToTag(1).status == TAG_PARTIAL, "clipped tag is recoverable, not a link error");
  check(rangeAndHeadingToTag(1).status == TAG_ERROR, "communication failure is an error");
  const unsigned before = cam.requests;
  check(rangeAndHeadingToTag(0).status == TAG_ERROR && cam.requests == before, "reject invalid ID before camera request");
}

void test_driving() {
  reset(); setMotionMonitor(timed_monitor);
  check(!rotateContinuously(20) && wasMotionInterrupted(), "slower user-selected search can be interrupted");
  check(clock_ms >= 65000, "driving primitive leaves search deadline to navigation");
  check(enc_l.read() < 0 && enc_r.read() > 0, "positive search pivots CCW");
  check(brake_transitions == 1 && reset_count == 1, "search keeps running between camera checks");
  check_braked("interrupted search brakes both wheels");
  reset(); setMotionMonitor(timed_monitor); rotateContinuously(-20);
  check(enc_l.read() > 0 && enc_r.read() < 0, "negative search pivots CW");
  reset(); encoders_moving = false;
  check(!rotateContinuously(20) && !wasMotionInterrupted(), "stall is a movement fault");
  check_braked("search stall brakes both wheels");
  reset(); check(driveDistance(3, APPROACH_SPEED_CM_S), "finite approach reaches encoder target");
  check(enc_l.read() >= 194 && enc_r.read() >= 194, "both wheels reach their finite targets");
  check_braked("finite approach brakes on completion");
  reset(); check(rotateCCW(-8), "finite clockwise alignment completes");
  check(enc_l.read() > 0 && enc_r.read() < 0, "alignment has correct wheel signs");
  check(!driveDistance(5, NAN), "reject NaN speed");
  check(!rotateContinuously(INFINITY), "reject infinite turn rate");
}

void test_arrival_and_visibility() {
  reset(); check(navigationSettingsValid(), "default stage settings are valid");
  check(STOP_TARGET_CM < 10 && MAX_STOP_CLEARANCE_CM <= 10, "arrival threshold meets within-10-cm requirement");
  watched_pose.status = TAG_VISIBLE; watched_pose.range_cm = 25; watched_pose.heading_deg = 0;
  check(!tagIsCloseEnough(), "25 cm is no longer accepted as arrival");
  watched_pose.range_cm = 9;
  check(tagIsCloseEnough(), "centered 9 cm pose is within specification");
  watched_pose.heading_deg = 20;
  check(!tagIsCloseEnough(), "close but off-center tag still requires alignment");

  reset(); cam.frames = {tag(160, 225), partial()};
  check(driveToTag(1) == TAG_REACHED, "bounded final step finishes when close tag leaves field of view");
  const double distance = enc_l.position * PI * WHEEL_DIAMETER_CM / ENCODER_TICKS_PER_WHEEL_REV;
  check(distance > 4.4 && distance <= MAX_FINAL_APPROACH_CM + 0.02, "final encoder distance stays within frozen five-cm bound");
  check_braked("final arrival leaves both wheels braked");
  check(motion_monitor == 0, "arrival clears the old tag's monitor");

  reset(); cam.frames = {tag(160, 100), partial(), partial(), tag(160, 225), tag(160, 225), partial()};
  check(driveToTag(1) == TAG_REACHED, "partial detection during normal approach brakes and reacquires");
  bool searched = false;
  for (const auto &c : commands) searched |= c.l < 0 && c.r > 0;
  check(searched && brake_transitions >= 2, "visibility loss causes search after braking");

  reset(); cam.frames = {tag(160, 100), {false, {}}, tag(160, 225), partial()};
  check(driveToTag(1) == TAG_REACHED, "one camera failure brakes then recovers from a new valid pose");
  check(brake_transitions >= 2, "camera interruption stops movement before retrying");

  reset(); cam.frames = {tag(110, 100), tag(160, 100), tag(160, 225), partial()};
  check(driveToTag(1) == TAG_REACHED, "same-frame heading is used to align before approaching");
  bool aligned = false;
  for (const auto &c : commands) aligned |= c.l < 0 && c.r > 0;
  check(aligned, "left-of-center tag triggers a CCW correction");
}

void test_timeouts_and_faults() {
  reset(); cam.frames = {missing()};
  check(driveToTag(1) == TAG_SEARCH_FAILED, "missing tag returns required -1 status");
  check(clock_ms >= TAG_SEARCH_TIMEOUT_MS && clock_ms < TAG_SEARCH_TIMEOUT_MS + 1000, "search stops after configured deadline");
  check_braked("search timeout brakes both wheels");
  reset(); cam.frames = {tag(160, 225, 2)};
  check(driveToTag(1) == TAG_SEARCH_FAILED, "wrong visible ID never advances the route");
  reset(); cam.frames = {partial()};
  check(driveToTag(1) == TAG_SEARCH_FAILED, "permanently clipped tag times out instead of falsely arriving");
  reset(); cam.frames = {{false, {}}};
  check(driveToTag(1) == TAG_NAVIGATION_ERROR, "persistent camera failure returns -2");
  check(cam.requests == CAMERA_RETRY_COUNT, "persistent failure uses bounded retries while braked");
  check_braked("persistent camera fault leaves motors braked");
  reset(); encoders_moving = false; cam.frames = {tag(160, 225)};
  check(driveToTag(1) == TAG_NAVIGATION_ERROR, "stalled final approach does not count as arrival");
  reset(); cam.frames = {tag(110, 100)};
  check(driveToTag(1) == TAG_NAVIGATION_ERROR && clock_ms >= TAG_NAVIGATION_TIMEOUT_MS,
        "repeated unsuccessful alignment cannot run forever");
  reset(); check(driveToTag(-1) == TAG_NAVIGATION_ERROR, "invalid navigation target returns -2");
}

void test_complete_route() {
  reset();
  cam.per_tag[1] = {tag(160, 225, 1), partial(1)};
  cam.per_tag[2] = {missing(), partial(2), missing(), tag(160, 225, 2), tag(160, 225, 2), partial(2)};
  cam.per_tag[3] = {missing(), tag(160, 225, 3), tag(160, 225, 3), partial(3)};
  check(runMilestone2() == TAG_REACHED, "full mission reaches all three tags");
  std::vector<int> sequence;
  for (int id : cam.requested_ids) if (sequence.empty() || sequence.back() != id) sequence.push_back(id);
  check(sequence == std::vector<int>({1, 2, 3}), "route is exactly 1 -> 2 -> 3 with no wraparound");
  for (int id = 1; id <= 3; ++id) {
    bool forward = false;
    for (const auto &c : commands) forward |= c.target == id && c.l > 0 && c.r > 0;
    check(forward, "each route target receives its own forward approach");
  }
  const std::string log = Serial.log.str();
  check(log.find("reached target=1") < log.find("starting target=2"), "tag 2 starts only after tag 1 arrival");
  check(log.find("reached target=2") < log.find("starting target=3"), "tag 3 starts only after tag 2 arrival");
  check(log.find("[route] COMPLETE") != std::string::npos, "completion is logged after tag 3");
  check_braked("completed route holds all motors braked");
  check(motion_monitor == 0, "route leaves no stale target callback");

  reset(); cam.per_tag[1] = {tag(160, 225, 1), partial(1)}; cam.per_tag[2] = {missing()};
  check(runMilestone2() == TAG_SEARCH_FAILED, "tag 2 search failure is reported after successful tag 1");
  bool requested_three = false;
  for (int id : cam.requested_ids) requested_three |= id == 3;
  check(!requested_three, "failed tag 2 is never skipped");
  check_braked("failed route remains braked");

  reset(); cam.per_tag[1] = {tag(160, 225, 1), partial(1)}; cam.per_tag[2] = {{false, {}}};
  check(runMilestone2() == TAG_NAVIGATION_ERROR, "camera failure on tag 2 is diagnosed instead of silently ending route");
  check(Serial.log.str().find("HALTED at target=2 status=-2") != std::string::npos,
        "serial log identifies the exact target and failure status");
}

int main() {
  test_camera_geometry(); test_driving(); test_arrival_and_visibility();
  test_timeouts_and_faults(); test_complete_route();
  std::printf("PASS: %d Milestone 2 camera/movement/three-tag-route checks.\n", checks);
}
