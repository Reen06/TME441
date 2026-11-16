// Actual camera, motor-motion, HC-SR04 driver, and navigation with simulated I/O.
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
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(a) ((a)<0?-(a):(a))
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#include "../tme441-milestone-2/navigation_config.h"
#include "../tme441-milestone-2/camera.h"
#include "../tme441-milestone-2/driving.h"
#include "../tme441-milestone-2/navigation.h"
#include "../tme441-milestone-2/ultrasonic.h"
unsigned long clock_ms;
long command_l,command_r;
bool encoders_moving=true;
int brake_transitions,reset_count,requested_id;
struct Command {long l,r; int target;};
std::vector<Command> commands;
struct FakeEncoder {double position=0; long read(){return std::lround(position);}} enc_l,enc_r;
unsigned long millis(){return clock_ms;}
void delay(unsigned long ms){
  clock_ms+=ms;
  if(encoders_moving){enc_l.position+=double(command_l)*ms/1000.;enc_r.position+=double(command_r)*ms/1000.;}
}
void delayMicroseconds(unsigned int){}
void pinMode(int,int){}
void digitalWrite(int,int){}
unsigned long pulseIn(int,int,unsigned long);
void updatePID(long l,long r,bool){
  if((command_l||command_r)&&!l&&!r)++brake_transitions;
  command_l=l;command_r=r;commands.push_back({l,r,requested_id});
}
void resetPID(){++reset_count;}
struct FakeSerial{
  std::ostringstream log;
  template<class T>void print(T v){log<<v;}
  template<class T>void println(T v){log<<v<<'\n';}
}Serial;
float scene_initial_cm=30;
double scene_origin_ticks=0;
bool sonar_no_echo=false,sonar_tracks_motion=true;
float sonar_forced_cm=NAN,sonar_fail_after_cm=INFINITY;
std::vector<float> echo_script;
unsigned echo_index=0;
std::vector<unsigned long> ping_times;
std::map<int,float> scene_distances;
float sceneTrueClearance();
float sceneTravelled();
void beginScene(int id);
const int COMMAND_RETURN_BLOCK=1;
struct HUSKYLENSResult{int command,ID,xCenter,yCenter,width,height;};
struct Frame{bool ok;std::vector<HUSKYLENSResult>blocks;};
struct FakeCamera{
  std::vector<Frame>frames;
  std::map<int,std::vector<Frame>>per_tag;
  std::map<int,unsigned>per_tag_index;
  std::vector<int>requested_ids;
  Frame current={true,{}};
  unsigned index=0,block_index=0,requests=0,hidden_requests=0;
  bool auto_scene=false;
  void physicalFrame(int id){
    const float d=sceneTrueClearance();
    if(d<CAMERA_MIN_VISIBLE_CM){current={true,{}};++hidden_requests;return;}
    const int height=std::lround(RANGE_HEIGHT_SCALE_CM_PX/(d-RANGE_OFFSET_CM-RANGE_BIAS_CORRECTION_CM));
    current={true,{{COMMAND_RETURN_BLOCK,id,171,120,40,height}}};
  }
  bool requestBlocks(int id){
    if(requested_id!=id)beginScene(id);
    requested_id=id;requested_ids.push_back(id);++requests;
    bool scripted=false;
    if(per_tag.count(id)){
      const auto&s=per_tag[id];unsigned&i=per_tag_index[id];
      if(i<s.size()){current=s[i++];scripted=true;}
      else if(!auto_scene&&!s.empty()){current=s.back();scripted=true;}
    }else if(index<frames.size()){current=frames[index++];scripted=true;}
    if(!scripted&&auto_scene)physicalFrame(id);
    block_index=0;return current.ok;
  }
  bool available(){return block_index<current.blocks.size();}
  HUSKYLENSResult read(){return current.blocks[block_index++];}
}cam;
#include "../tme441-milestone-2/driving.ino"
#include "../tme441-milestone-2/ultrasonic.ino"
#include "../tme441-milestone-2/camera.ino"
#include "../tme441-milestone-2/navigation.ino"
float sceneTravelled(){
  return ((enc_l.position+enc_r.position)*0.5-scene_origin_ticks)*PI*WHEEL_DIAMETER_CM/ENCODER_TICKS_PER_WHEEL_REV;
}
float sceneTrueClearance(){return scene_initial_cm-sceneTravelled();}
void beginScene(int id){
  scene_origin_ticks=(enc_l.position+enc_r.position)*0.5;
  scene_initial_cm=scene_distances.count(id)?scene_distances[id]:30.0f;
}
unsigned long pulseIn(int,int,unsigned long timeout){
  ping_times.push_back(clock_ms);
  float cm=sonar_tracks_motion?sceneTrueClearance():scene_initial_cm;
  if(isfinite(sonar_forced_cm))cm=sonar_forced_cm;
  if(echo_index<echo_script.size())cm=echo_script[echo_index++];
  if(sonar_no_echo||sceneTravelled()>=sonar_fail_after_cm||!isfinite(cm)){
    delay((timeout+999)/1000);return 0;
  }
  const unsigned long us=std::lround(max(0.0f,cm)*58.0f);
  delay(max(1UL,(us+999)/1000));return us;
}
int checks=0;
void check(bool c,const char*m){
  ++checks;if(!c){std::fprintf(stderr,"FAIL: %s\n%s",m,Serial.log.str().c_str());std::exit(1);}
}
Frame tag(int x,int height,int id=1){return{true,{{COMMAND_RETURN_BLOCK,id,x,120,40,height}}};}
Frame missing(){return{true,{}};}
Frame partial(int id=1){return tag(160,240,id);}
void reset(){
  clock_ms=0;command_l=command_r=0;enc_l.position=enc_r.position=0;
  encoders_moving=true;brake_transitions=reset_count=requested_id=0;
  commands.clear();cam=FakeCamera();Serial.log.str("");Serial.log.clear();
  scene_initial_cm=30;scene_origin_ticks=0;scene_distances.clear();
  sonar_no_echo=false;sonar_tracks_motion=true;sonar_forced_cm=NAN;sonar_fail_after_cm=INFINITY;
  echo_script.clear();echo_index=0;ping_times.clear();initUltrasonic();
  setMotionMonitor(0);movement_interrupted=false;
  last_camera_ms=0;watched_tag_id=1;watched_pose={};searching=false;monitor_result=0;
}
bool timed_monitor(){return millis()<65000UL;}
void check_braked(const char*m){check(command_l==0&&command_r==0,m);}
void test_camera_geometry() {
  reset();
  cam.frames = {tag(171, 70), tag(103, 112), tag(250, 112)};
  TagPose center = rangeAndHeadingToTag(1), left = rangeAndHeadingToTag(1), right = rangeAndHeadingToTag(1);
  check(center.status == TAG_VISIBLE && abs(center.heading_deg) < 0.03f, "calibrated image center has near-zero heading");
  check(abs(center.range_cm - 47.96751f) < 0.001f, "camera range includes the centered validation correction");
  check(abs(left.heading_deg - 13.58599f) < 1.2f, "measured left offset produces a CCW heading");
  check(abs(right.heading_deg + 13.58599f) < 1.2f, "measured right offset produces a CW heading");
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

void test_recorded_calibration() {
  // Physical input/ground-truth observations, not precomputed camera outputs.
  struct Observation { float forward_cm, right_cm; int x, height; };
  const Observation samples[] = {
    {30, 0, 171, 116}, {60, 0, 166, 58}, {90, 0, 160, 38},
    {30, 7.25f, 250, 112}, {30, -7.25f, 103, 112},
    {60, 19.25f, 271, 58}, {60, -19.25f, 76, 56},
    {90, 36.75f, 293, 38}, {90, -36.75f, 50, 36}
  };
  for (unsigned int i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
    reset();
    const Observation &s = samples[i];
    cam.frames = {tag(s.x, s.height, 2)};
    const TagPose pose = rangeAndHeadingToTag(2);
    const float true_heading = -atan2(s.right_cm, s.forward_cm) * 180.0f / PI;
    check(pose.status == TAG_VISIBLE && abs(pose.heading_deg - true_heading) < 2.2f,
          "recorded lateral/centered observation matches the fitted heading tolerance");
    if (i < 3) {
      // Preserve the original raw measurements and test their original fit.
      // A later aggregate validation report adds a separate correction; it
      // does not replace those historical measured distances.
      const float original_fit_range = pose.range_cm - RANGE_BIAS_CORRECTION_CM;
      check(abs(original_fit_range - s.forward_cm) < 0.6f,
            "original centered observations retain their original fitting tolerance");
    }
  }
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


void test_ultrasonic_driver(){
  reset();
  auto r=readUltrasonic();
  check(r.status==ULTRASONIC_OK&&abs(r.distance_cm-30)<0.03f,"echo pulse converts to sensor-face centimeters");
  const auto count=ping_times.size();readUltrasonic();
  check(ping_times.size()==count,"cached polling does not retrigger too soon");
  delay(ULTRASONIC_PING_INTERVAL_MS);readUltrasonic();
  check(ping_times.size()==count+1&&ping_times.back()-ping_times.front()>60,"ping spacing obeys >60 ms datasheet interval");
  delay(ULTRASONIC_FRESHNESS_MS+1);
  check(!ultrasonicReadingFresh(r),"stale cached distance cannot authorize movement");
  reset();sonar_no_echo=true;r=readUltrasonic();
  check(r.status==ULTRASONIC_NO_ECHO&&!isfinite(r.distance_cm),"no echo is invalid data rather than zero clearance");
  check(clock_ms<=ULTRASONIC_ECHO_TIMEOUT_US/1000+1,"missing echo is bounded by explicit short timeout");
  reset();sonar_forced_cm=0.5f;r=readUltrasonic();
  check(r.status==ULTRASONIC_INVALID,"below-minimum echo is rejected");
  reset();echo_script={8,30,30};r=readStableUltrasonic();
  check(ultrasonicReadingFresh(r)&&abs(r.distance_cm-30)<0.03f,"three fresh pings reject one false near reading");
  check(ping_times.size()>=3&&ping_times.size()<=5,"stable confirmation uses distinct agreeing pings, not cached copies");
  reset();echo_script={5,15,25,35,45};r=readStableUltrasonic();
  check(r.status!=ULTRASONIC_OK,"wildly inconsistent echoes cannot authorize handoff or arrival");
  reset();echo_script={30,30,NAN,NAN,30};r=readStableUltrasonic();
  check(r.status!=ULTRASONIC_OK,"old valid pings cannot form a fresh confirmation after echo dropouts");
  reset();sonar_no_echo=true;r=readStableUltrasonic();
  check(r.status!=ULTRASONIC_OK&&ping_times.size()==5,"stationary confirmation has bounded retry attempts");
}

void test_handoff_and_arrival(){
  reset();check(navigationSettingsValid(),"confirmed sensor and handoff settings are valid");
  check(CAMERA_ULTRASONIC_HANDOFF_CM>27,"handoff is above observed camera visibility loss");
  cam.auto_scene=true;scene_distances[1]=50;
  check(driveToTag(1)==TAG_REACHED,"camera approach hands off and reaches tag with live ultrasonic control");
  check(cam.hidden_requests>0,"route tolerates actual simulated loss of tag below 27 cm");
  check(sceneTrueClearance()<=10.0f&&sceneTrueClearance()>ULTRASONIC_MIN_CM,"arrival uses physical simulated sensor-face clearance within 10 cm");
  check(Serial.log.str().find("[handoff] target=1")!=std::string::npos,"handoff identifies locked target");
  check(Serial.log.str().find("[arrival] confirmed by three ultrasonic pings")!=std::string::npos,"arrival is independently confirmed after braking");
  check_braked("ultrasonic arrival leaves wheels braked");
  check(motion_monitor==0,"arrival removes old motion callback");
  bool reversed=false;for(const auto&c:commands)reversed|=c.l<0||c.r<0;
  check(!reversed,"lost close-range image does not trigger search pivots after handoff");

  reset();cam.auto_scene=true;sonar_forced_cm=10;
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"unrelated close obstacle cannot satisfy a camera tag 30 cm away");
  check(sceneTravelled()==0,"distance disagreement stops before forward travel");
  check_braked("handoff disagreement leaves motors braked");

  reset();cam.auto_scene=true;sonar_no_echo=true;
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"no stable handoff echo stops instead of blind final drive");
  check(sceneTravelled()==0,"no-echo handoff causes no forward motion");

  reset();cam.auto_scene=true;sonar_fail_after_cm=2;
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"echo loss during close movement aborts the route");
  check(sceneTravelled()<3.1f,"echo loss brakes within a bounded one-cm movement");
  check_braked("echo timeout while moving leaves both wheels braked");

  reset();cam.auto_scene=true;sonar_tracks_motion=false;
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"unchanging far echo cannot drive indefinitely");
  check(sceneTravelled()<=26,"encoder travel budget bounds a stuck numeric echo");

  reset();cam.frames={tag(171,111),{false,{}}};
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"persistent camera link failure after handoff stops the approach");
  check_braked("camera link failure remains braked");

  reset();cam.frames={tag(171,111),tag(110,111),tag(110,111),tag(171,111),tag(171,111),missing()};
  check(driveToTag(1)==TAG_REACHED,"visible heading drift requires re-alignment of the same target");
  bool aligned=false;for(const auto&c:commands)aligned|=c.l<0&&c.r>0;
  check(aligned,"heading drift causes an alignment turn before resuming ultrasonic travel");
}

void test_faults_and_route(){
  reset();cam.frames={missing()};sonar_forced_cm=8;
  check(driveToTag(1)==TAG_SEARCH_FAILED,"near echo alone never establishes a tag identity");
  check(clock_ms>=TAG_SEARCH_TIMEOUT_MS,"missing tag obeys search deadline");
  check_braked("tag search failure holds brakes");
  reset();cam.frames={tag(171,111,2)};
  check(driveToTag(1)==TAG_SEARCH_FAILED,"wrong camera ID cannot authorize ultrasonic handoff");
  reset();cam.frames={partial()};
  check(driveToTag(1)==TAG_SEARCH_FAILED,"partial tag without a prior lock cannot authorize approach");
  reset();cam.frames={{false,{}}};
  check(driveToTag(1)==TAG_NAVIGATION_ERROR&&cam.requests==CAMERA_RETRY_COUNT,"persistent camera errors have bounded retries");
  reset();cam.auto_scene=true;encoders_moving=false;
  check(driveToTag(1)==TAG_NAVIGATION_ERROR,"encoder stall cannot become an ultrasonic arrival");
  reset();check(driveToTag(-1)==TAG_NAVIGATION_ERROR,"invalid navigation target is rejected");

  reset();cam.auto_scene=true;scene_distances[1]=50;scene_distances[2]=45;scene_distances[3]=40;
  cam.per_tag[2]={missing(),missing()};
  check(runMilestone2()==TAG_REACHED,"full camera/ultrasonic mission reaches all three tags");
  std::vector<int>sequence;
  for(int id:cam.requested_ids)if(sequence.empty()||sequence.back()!=id)sequence.push_back(id);
  check(sequence==std::vector<int>({1,2,3}),"route is exactly 1 -> 2 -> 3 without restarting");
  for(int id=1;id<=3;++id){
    bool forward=false;for(const auto&c:commands)forward|=c.target==id&&c.l>0&&c.r>0;
    check(forward,"each tag has its own verified approach");
  }
  const auto log=Serial.log.str();
  check(log.find("reached target=1")<log.find("starting target=2"),"tag 2 starts after tag 1 arrival");
  check(log.find("reached target=2")<log.find("starting target=3"),"tag 3 starts after tag 2 arrival");
  check(log.find("[route] COMPLETE")!=std::string::npos,"third tag logs mission completion");
  check_braked("completed three-tag route remains braked");
  check(cam.hidden_requests>3,"all-target test includes close-range camera loss");

  reset();cam.per_tag[1]={tag(171,111),missing()};cam.per_tag[2]={missing()};
  check(runMilestone2()==TAG_SEARCH_FAILED,"missing tag 2 is reported after a successful tag 1");
  bool skipped=false;for(int id:cam.requested_ids)skipped|=id==3;
  check(!skipped,"tag 2 failure never skips ahead to tag 3");
  check_braked("failed three-tag route remains braked");
}
int main(){
  test_camera_geometry();test_recorded_calibration();test_driving();test_ultrasonic_driver();
  test_handoff_and_arrival();test_faults_and_route();
  std::printf("PASS: %d Milestone 2 camera/ultrasonic/movement/route checks.\n",checks);
}
