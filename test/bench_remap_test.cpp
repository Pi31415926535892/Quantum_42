// bench_remap_test.cpp — host-side assertion that the two legacy bench
// sketches carry the SAME drivetrain pin map as src/rescue_robot/config.h.
//
// The legacy sketches ("PWM Test" and "Robocup.ino") must stay self-contained —
// they may not include the src/rescue_robot headers, because they are the
// standalone polarity reference flashed on a bare Mega. The cost of that
// freedom is three copies of the pin table that can silently drift apart, which
// is exactly the defect HARDWARE.md section 6 documents. This suite is the
// drift alarm: it parses each sketch's constants and compares the resulting pin
// sets against config.h, which is the source of truth.
//
// Both sketches are also checked for the polarity half of the section 6 fix:
// positive must mean forward on BOTH channels, so names alone are not enough.
// The two sketches express polarity differently — "PWM Test" selects it per
// signed speed, the line follower is forward-only — so each is checked against
// the shape it actually uses.
//
// Build/run: ./test/run_tests.sh (picked up automatically).

#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "config.h"
#include "motors.h"

namespace {

int gFailures = 0;
int gChecks = 0;

void report(bool ok, const std::string& what) {
  ++gChecks;
  if (!ok) {
    ++gFailures;
    std::printf("FAIL bench_remap: %s\n", what.c_str());
  }
}

std::string readFile(const char* path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return std::string();
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

// Pull `const uint8_t <NAME> <spaces>= <digits>;` out of a sketch body.
bool constantValue(const std::string& body, const std::string& name, int& out) {
  const std::string needle = "const uint8_t " + name;
  size_t cursor = body.find(needle);
  if (cursor == std::string::npos) return false;
  cursor += needle.size();
  size_t assign = body.find('=', cursor);
  if (assign == std::string::npos) return false;
  // Guard against matching a longer name that merely starts with `name`.
  for (size_t i = cursor; i < assign; ++i) {
    if (!std::isspace(static_cast<unsigned char>(body[i]))) return false;
  }
  ++assign;
  while (assign < body.size() && std::isspace(static_cast<unsigned char>(body[assign]))) {
    ++assign;
  }
  if (assign >= body.size() || body[assign] < '0' || body[assign] > '9') {
    return false;
  }
  int value = 0;
  while (assign < body.size() && body[assign] >= '0' && body[assign] <= '9') {
    value = value * 10 + (body[assign] - '0');
    ++assign;
  }
  out = value;
  return true;
}

// The three constants each legacy sketch defines per motor channel, in the
// order (in1, in2, pwm) that both the L298N section 2 table and config.h use.
const char* const kLeftNames[3] = {"LEFT_IN1", "LEFT_IN2", "LEFT_PWM"};
const char* const kRightNames[3] = {"RIGHT_IN1", "RIGHT_IN2", "RIGHT_PWM"};

// Assert one channel's three pin constants equal the config.h values.
void checkSide(const std::string& label, const std::string& body,
               const char* const names[3], int in1, int in2, int pwm) {
  const int expected[3] = {in1, in2, pwm};
  for (int i = 0; i < 3; ++i) {
    int found = -1;
    const std::string name = names[i];
    if (!constantValue(body, name, found)) {
      report(false, label + ": missing constant " + name);
      continue;
    }
    report(found == expected[i],
           label + ": " + name + " = " + std::to_string(found) +
               ", config.h says " + std::to_string(expected[i]));
  }
}

// A signed drive must raise in1 and lower in2 for a positive speed. The sign
// test is looked up by the speed PARAMETER it branches on ("speed" in the
// shared driveChannel helper), and the pin writes by whatever the direction
// inputs are named, so the check follows the sketch's real shape rather than a
// hardcoded copy of it.
void checkSignedForwardBranch(const std::string& label, const std::string& body,
                              const std::string& speedVar,
                              const std::string& in1, const std::string& in2) {
  const size_t branch = body.find(speedVar + " > 0");
  if (branch == std::string::npos) {
    report(false, label + ": no forward branch for " + speedVar);
    return;
  }
  const size_t branchEnd = body.find("else if", branch);
  if (branchEnd == std::string::npos) {
    report(false, label + ": forward branch for " + speedVar + " is unterminated");
    return;
  }
  const size_t highIn1 = body.find("digitalWrite(" + in1 + ", HIGH)", branch);
  report(highIn1 != std::string::npos && highIn1 < branchEnd,
         label + ": forward does not drive " + in1 + " HIGH");
  const size_t lowIn2 = body.find("digitalWrite(" + in2 + ", LOW)", branch);
  report(lowIn2 != std::string::npos && lowIn2 < branchEnd,
         label + ": forward does not drive " + in2 + " LOW");
}

// Forward-only sketches park the direction pins once, so the requirement is
// simply that in1 is HIGH and in2 LOW somewhere in the drive path, and that
// in2 is never driven HIGH anywhere (that inversion is the section 6 bug).
void checkForwardOnlyPolarity(const std::string& label, const std::string& body,
                              const std::string& in1, const std::string& in2) {
  report(body.find("digitalWrite(" + in1 + ", HIGH)") != std::string::npos,
         label + ": forward never drives " + in1 + " HIGH");
  report(body.find("digitalWrite(" + in2 + ", LOW)") != std::string::npos,
         label + ": forward never drives " + in2 + " LOW");
  report(body.find("digitalWrite(" + in2 + ", HIGH)") == std::string::npos,
         label + ": " + in2 + " is driven HIGH, which reverses the channel");
}

void checkSketchPins(const char* path) {
  const std::string body = readFile(path);
  const std::string label = path;
  if (body.empty()) {
    report(false, label + ": cannot read sketch");
    return;
  }
  checkSide(label, body, kLeftNames, rescue::MOTOR_LEFT_IN1,
            rescue::MOTOR_LEFT_IN2, rescue::MOTOR_LEFT_PWM);
  checkSide(label, body, kRightNames, rescue::MOTOR_RIGHT_IN1,
            rescue::MOTOR_RIGHT_IN2, rescue::MOTOR_RIGHT_PWM);
}

// The line follower needs a POSITIVE proportional gain: with the section 6
// remap a line on the right has to speed the LEFT wheel up, and Kd must stay
// zero until the P term tracks (HARDWARE.md section 8).
void checkRobocupGains(const std::string& body) {
  const size_t kp = body.find("Kp =");
  if (kp == std::string::npos) {
    report(false, "Robocup.ino: no Kp constant found");
  } else {
    size_t cursor = kp + 4;
    while (cursor < body.size() && std::isspace(static_cast<unsigned char>(body[cursor]))) {
      ++cursor;
    }
    report(cursor < body.size() && body[cursor] != '-',
           "Robocup.ino: Kp is negative; the remapped wiring needs a positive Kp "
           "so a right-hand line speeds the left wheel up");
  }

  const size_t kd = body.find("Kd =");
  if (kd == std::string::npos) {
    report(false, "Robocup.ino: no Kd constant found");
    return;
  }
  const std::string kdLine = body.substr(kd, body.find('\n', kd) - kd);
  report(kdLine.find("0.0") != std::string::npos,
         "Robocup.ino: Kd must be 0.0 until the P term tracks, found: " + kdLine);
}

// config.h must still BE the section 6 remap, otherwise a config edit would
// silently redefine the ground truth both legacy sketches are compared against.
void checkConfigIsRemap() {
  report(rescue::MOTOR_LEFT_PWM == 6 && rescue::MOTOR_LEFT_IN1 == 4 &&
             rescue::MOTOR_LEFT_IN2 == 2,
         "config.h LEFT is not the section 6 channel B set {6,4,2}");
  report(rescue::MOTOR_RIGHT_PWM == 7 && rescue::MOTOR_RIGHT_IN1 == 3 &&
             rescue::MOTOR_RIGHT_IN2 == 5,
         "config.h RIGHT is not the section 6 channel A set {7,3,5}");
  report(rescue::LEFT_CHANNEL_PINS.pwm == rescue::MOTOR_LEFT_PWM &&
             rescue::LEFT_CHANNEL_PINS.in1 == rescue::MOTOR_LEFT_IN1 &&
             rescue::LEFT_CHANNEL_PINS.in2 == rescue::MOTOR_LEFT_IN2,
         "config.h LEFT_CHANNEL_PINS does not mirror the LEFT constants");
  report(rescue::RIGHT_CHANNEL_PINS.pwm == rescue::MOTOR_RIGHT_PWM &&
             rescue::RIGHT_CHANNEL_PINS.in1 == rescue::MOTOR_RIGHT_IN1 &&
             rescue::RIGHT_CHANNEL_PINS.in2 == rescue::MOTOR_RIGHT_IN2,
         "config.h RIGHT_CHANNEL_PINS does not mirror the RIGHT constants");
}

}  // namespace

int main() {
  // Print both sides so a bench engineer can diff this output against the
  // HARDWARE.md section 6 table by eye as well as by assertion.
  std::printf("config.h LEFT  = {pwm %d, in1 %d, in2 %d}\n",
              rescue::MOTOR_LEFT_PWM, rescue::MOTOR_LEFT_IN1,
              rescue::MOTOR_LEFT_IN2);
  std::printf("config.h RIGHT = {pwm %d, in1 %d, in2 %d}\n",
              rescue::MOTOR_RIGHT_PWM, rescue::MOTOR_RIGHT_IN1,
              rescue::MOTOR_RIGHT_IN2);
  std::printf("expecting LEFT = {pwm 6, in1 4, in2 2} (L298N channel B), "
              "RIGHT = {pwm 7, in1 3, in2 5} (L298N channel A)\n");

  checkConfigIsRemap();
  checkSketchPins("PWM Test");
  checkSketchPins("Robocup.ino");

  // Polarity, per the shape each sketch uses. "PWM Test" selects direction
  // from the signed speed inside one shared helper; the line follower is
  // forward-only, so it is checked separately below.
  const std::string pwmTest = readFile("PWM Test");
  checkSignedForwardBranch("PWM Test", pwmTest, "speed", "in1", "in2");

  const std::string robocup = readFile("Robocup.ino");
  checkForwardOnlyPolarity("Robocup.ino", robocup, "LEFT_IN1", "LEFT_IN2");
  checkForwardOnlyPolarity("Robocup.ino", robocup, "RIGHT_IN1", "RIGHT_IN2");
  checkRobocupGains(robocup);

  std::printf("[bench_remap] checks: %d  failures: %d\n", gChecks, gFailures);
  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  return 1;
}
