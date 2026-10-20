// diag_test.cpp — host-side tests for the Serial menu in src/rescue_robot/diag.ino.
// The menu is delivery-only, so every module it may touch is linked against a
// recorder that captures the call and returns a scripted value. Each test can
// then assert BOTH that a command reached the right module AND that its output
// carries the right field, with no AVR toolchain. Parsing lives in
// diag_parse_test.cpp. Run: ./test/run_tests.sh.

#include <cstdio>
#include <cstring>
#include <string>

#include "arduino_stub/Arduino.h"
#include "config.h"
#include "line.h"
#include "link.h"
#include "motors.h"
#include "sensors.h"

// Recorder standing in for the sibling delivery layers.

namespace recorder {

struct MotorCall {
  int left;
  int right;
};
struct SideMotorCall {
  rescue::MotorSide side;
  int speed;
};

int setMotorBothCalls = 0;
int setMotorSideCalls = 0;
int motorStopCalls = 0;
int lineUpdates = 0;
int probeClassifyCalls = 0;
MotorCall lastBoth{0, 0};
SideMotorCall lastSide{rescue::MotorSide::Left, 0};

int gReflectivity = 700;   // bright -> silver
bool gConductive = true;   // silver + conductive -> live
rescue::LineReading gLineReading{3500, 0, true, rescue::LineClass::ON_LINE};
rescue::Liveness gLiveness = rescue::Liveness::Fresh;
rescue::LinkPolicy gPolicy = rescue::LinkPolicy::LinkFresh;
uint16_t gTxOverflows = 3;
uint8_t gTxQueued = 2;

void reset() {
  setMotorBothCalls = 0;
  setMotorSideCalls = 0;
  motorStopCalls = 0;
  lineUpdates = 0;
  probeClassifyCalls = 0;
  lastBoth = MotorCall{0, 0};
  lastSide = SideMotorCall{rescue::MotorSide::Left, 0};
}

}  // namespace recorder

// The entry points the menu calls, mirroring the sibling .ino delivery layers.
void setMotor(int leftSpeed, int rightSpeed) {
  ++recorder::setMotorBothCalls;
  recorder::lastBoth = recorder::MotorCall{leftSpeed, rightSpeed};
}
void setMotor(rescue::MotorSide side, int speed) {
  ++recorder::setMotorSideCalls;
  recorder::lastSide = recorder::SideMotorCall{side, speed};
}
void motorsStop() { ++recorder::motorStopCalls; }

rescue::LineReading updateLine() {
  ++recorder::lineUpdates;
  return recorder::gLineReading;
}

rescue::VictimClass classifyVictim() {
  ++recorder::probeClassifyCalls;
  return rescue::classifyFromReadings(
      rescue::applyReflectivity(rescue::SurfaceKind::Black,
                                recorder::gReflectivity),
      recorder::gConductive ? rescue::Conductivity::Conductive
                            : rescue::Conductivity::Insulating);
}
int readReflectivity() { return recorder::gReflectivity; }
bool readConductivity() { return recorder::gConductive; }
rescue::SurfaceKind probeSurface() {
  return rescue::applyReflectivity(rescue::SurfaceKind::Black,
                                   recorder::gReflectivity);
}

rescue::Liveness linkLiveness() { return recorder::gLiveness; }
rescue::LinkPolicy linkPolicy() { return recorder::gPolicy; }
uint16_t linkTxOverflows() { return recorder::gTxOverflows; }
uint8_t linkTxQueued() { return recorder::gTxQueued; }
bool linkReceiveFresh(uint8_t) { return true; }

// The menu under test.
#include "diag.ino"

namespace {

int gFailures = 0;
int gChecks = 0;

void report(bool ok, const std::string& what) {
  ++gChecks;
  if (!ok) {
    ++gFailures;
    std::printf("FAIL diag: %s\n", what.c_str());
  }
}

bool contains(const char* haystack, const char* needle) {
  return std::strstr(haystack, needle) != nullptr;
}

// Run one command line; return everything it printed.
std::string run(const char* line) {
  serialClearOut();
  gSerialIn.clear();
  recorder::reset();
  serialFeed(line);
  diagTick();
  return gSerialOut;
}

}

int main() {
  // --- help
  {
    const std::string out = run("h");
    // Help must advertise exactly the four permitted groups, nothing more.
    for (const char* group : {"motor", "line", "link", "probe"}) {
      report(contains(out.c_str(), group), "help omits a command group");
    }
    // Help advertises '?', so it has to work.
    report(contains(run("?").c_str(), "probe"), "'?' does not print help");
    // No module call may happen for help.
    report(recorder::setMotorBothCalls == 0 && recorder::setMotorSideCalls == 0 &&
               recorder::motorStopCalls == 0 && recorder::lineUpdates == 0 &&
               recorder::probeClassifyCalls == 0,
           "help invoked a hardware module");
  }
  // --- constants
  {
    const std::string out = run("c");
    report(contains(out.c_str(), "LEFT"), "constants do not print LEFT");
    report(contains(out.c_str(), "RIGHT"), "constants do not print RIGHT");
    // The printed pins must be the remapped ones, so a bench engineer can
    // confirm the flashed sketch against config.h over Serial.
    report(contains(out.c_str(), "4") && contains(out.c_str(), "6"),
           "constants do not print the channel B pin set {4,6}");
    report(contains(out.c_str(), "3") && contains(out.c_str(), "7"),
           "constants do not print the channel A pin set {3,7}");
    report(recorder::setMotorBothCalls == 0 && recorder::lineUpdates == 0,
           "constants command drove hardware");
  }
  // --- motor: drive both sides
  {
    const std::string out = run("m 150 -150");
    report(recorder::setMotorBothCalls == 1,
           "'m 150 -150' did not call setMotor(left,right) exactly once");
    report(recorder::lastBoth.left == 150 && recorder::lastBoth.right == -150,
           "'m 150 -150' forwarded the wrong speeds");
    report(contains(out.c_str(), "150"), "motor command did not echo the speed");
  }

  // Section 7 checklist commands route to the drive layer.
  {
    run("m 150 150");
    report(recorder::lastBoth.left == 150 && recorder::lastBoth.right == 150,
           "'m 150 150' did not reach setMotor as a straight command");
    run("m -150 150");
    report(recorder::lastBoth.left == -150 && recorder::lastBoth.right == 150,
           "'m -150 150' did not reach setMotor as a pivot command");
    run("m 150 50");
    report(recorder::lastBoth.left == 150 && recorder::lastBoth.right == 50,
           "'m 150 50' did not reach setMotor as an arc command");
  }

  // A single-side command must target one side, not both.
  {
    run("f 150");
    report(recorder::setMotorSideCalls == 1,
           "'f 150' did not call setMotor(side, speed) exactly once");
    report(recorder::lastSide.side == rescue::MotorSide::Left &&
               recorder::lastSide.speed == 150,
           "'f 150' did not target the LEFT side");
    report(recorder::setMotorBothCalls == 0,
           "'f 150' also drove both sides");
    run("r -150");
    report(recorder::lastSide.side == rescue::MotorSide::Right &&
               recorder::lastSide.speed == -150,
           "'r -150' did not target the RIGHT side");
  }

  // stop must halt, not command zero speeds.
  {
    run("x");
    report(recorder::motorStopCalls == 1, "'x' did not call motorsStop()");
    report(recorder::setMotorBothCalls == 0,
           "'x' commanded a speed instead of stopping");
  }
  // --- line
  {
    recorder::gLineReading =
        rescue::LineReading{5250, 1750, true, rescue::LineClass::ON_LINE};
    const std::string out = run("l");
    report(recorder::lineUpdates == 1, "'l' did not read the line bar once");
    report(contains(out.c_str(), "5250"), "line dump omits the centroid");
    report(contains(out.c_str(), "ON_LINE"), "line dump omits the class name");
    report(contains(out.c_str(), "1750"), "line dump omits the error");
  }

  // A lost line must read distinctly from a seen one.
  {
    recorder::gLineReading =
        rescue::LineReading{3500, 0, false, rescue::LineClass::ALL_WHITE};
    const std::string out = run("l");
    report(contains(out.c_str(), "ALL_WHITE"),
           "line dump omits the ALL_WHITE class");
    report(contains(out.c_str(), "LOST"),
           "line dump does not flag a lost line");
  }
  // --- link
  {
    recorder::gLiveness = rescue::Liveness::Stale;
    recorder::gPolicy = rescue::LinkPolicy::DegradeToLineOnly;
    recorder::gTxQueued = 5;
    recorder::gTxOverflows = 7;
    const std::string out = run("k");
    report(contains(out.c_str(), "Stale") || contains(out.c_str(), "STALE"),
           "link dump omits liveness");
    report(contains(out.c_str(), "5"), "link dump omits the queue depth");
    report(contains(out.c_str(), "7"), "link dump omits the overflow count");
  }
  // --- probe
  {
    recorder::gReflectivity = 700;
    recorder::gConductive = true;
    const std::string out = run("p");
    report(recorder::probeClassifyCalls == 1,
           "'p' did not classify the victim exactly once");
    report(contains(out.c_str(), "Live") || contains(out.c_str(), "LIVE"),
           "probe dump does not report a live victim");
    report(contains(out.c_str(), "700"), "probe dump omits the raw reading");
  }
  {
    recorder::gReflectivity = 100;
    recorder::gConductive = false;
    const std::string out = run("p");
    report(contains(out.c_str(), "Dead") || contains(out.c_str(), "DEAD"),
           "probe dump does not report a dead victim");
  }
  // Anything off the truth table must surface as fake, never as a deposit.
  {
    recorder::gReflectivity = 700;
    recorder::gConductive = false;
    const std::string out = run("p");
    report(contains(out.c_str(), "Fake") || contains(out.c_str(), "FAKE"),
           "probe dump does not report an unclassifiable victim as fake");
  }
  // --- unknown / malformed input
  {
    const std::string out = run("zzz");
    report(contains(out.c_str(), "?") || contains(out.c_str(), "help") ||
               contains(out.c_str(), "ERR"),
           "unknown command gives no guidance");
    report(recorder::setMotorBothCalls == 0 && recorder::lineUpdates == 0 &&
               recorder::probeClassifyCalls == 0,
           "unknown command drove hardware");
  }
  {
    // A motor command missing its argument must not drive anything.
    const std::string out = run("m 150");
    report(recorder::setMotorBothCalls == 0,
           "'m 150' drove motors despite a missing argument");
    report(contains(out.c_str(), "ERR") || contains(out.c_str(), "?"),
           "'m 150' gave no guidance");
  }
  // --- an empty line is a no-op
  {
    const std::string out = run("");
    report(recorder::setMotorBothCalls == 0 && recorder::lineUpdates == 0,
           "an empty line drove hardware");
  }

  std::printf("[diag] checks: %d  failures: %d\n", gChecks, gFailures);
  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  return 1;
}
