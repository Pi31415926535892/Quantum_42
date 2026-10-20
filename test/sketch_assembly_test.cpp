// sketch_assembly_test.cpp — the full sketch, assembled and proven.
//
// Two jobs. First it builds every delivery .ino into ONE translation unit,
// which is what the Arduino builder does: if the sketch only compiles as
// separate files, the sketch does not compile. That alone catches a duplicate
// definition, a mismatched forward declaration and a missing stub.
//
// Second it proves the things assembly is for: one boot order, a calibration
// gate the mission cannot walk through, a status latch that survives, and a pin
// table with a single owner.
//
// The boot-order behaviour is exercised with locally-named mock steps rather
// than the delivery steps themselves: this TU already contains the real
// definition of every module's Begin(), and no test wants to power a motor to
// prove an order. The text checks read main.ino itself for the one boot table
// and the whole-sketch properties.
//
// Build/run: ./test/run_tests.sh (picked up automatically).

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "arduino_stub/Arduino.h"

#include "sketch.h"
#include "status.h"

// --- Part 1: the sketch, concatenated the way the Arduino builder does -----
//
// Order is alphabetical by file, main.ino last because it is the entry point.
// Any order works: that is the property under test.

#include "../src/rescue_robot/calibration.ino"
#include "../src/rescue_robot/diag.ino"
#include "../src/rescue_robot/eeprom_store.ino"
#include "../src/rescue_robot/fsm.ino"
#include "../src/rescue_robot/line.ino"
#include "../src/rescue_robot/link.ino"
#include "../src/rescue_robot/markers.ino"
#include "../src/rescue_robot/mission.ino"
#include "../src/rescue_robot/motors.ino"
#include "../src/rescue_robot/sensors.ino"
#include "../src/rescue_robot/main.ino"

#include "dryframe.h"

namespace {

std::string readFile(const char* path) {
  std::ifstream in(path);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// --- Part 2: the boot order as behaviour -----------------------------------
//
// The mocks carry the contract's step names so the recorded order is exactly
// the order the contract names. They are file-local, so the real delivery
// definitions in Part 1 never compete with them.

std::vector<std::string> gCalls;

void record(const char* name) { gCalls.push_back(name); }

#define MOCK_STEP(name) static void mock_##name() { record(#name); }

MOCK_STEP(statusBegin)
MOCK_STEP(eepromStoreBegin)
MOCK_STEP(motorsBegin)
MOCK_STEP(sensorsBegin)
MOCK_STEP(lineBegin)
MOCK_STEP(markersBegin)
MOCK_STEP(linkBegin)
MOCK_STEP(fsmBegin)
MOCK_STEP(beginCalRun)
MOCK_STEP(missionBegin)

#undef MOCK_STEP

// The order the contract forces, in one place. A step that moves is a step that
// has been reasoned about.
const char* const kExpectedBoot[] = {
    "statusBegin", "eepromStoreBegin", "motorsBegin",   "sensorsBegin",
    "lineBegin",   "markersBegin",    "linkBegin",     "fsmBegin",
    "beginCalRun", "missionBegin",
};
const size_t kBootStepCount =
    sizeof(kExpectedBoot) / sizeof(kExpectedBoot[0]);

void testBootOrderWalksEveryStepOnce() {
  gCalls.clear();
  const rescue::BootFn steps[] = {
      mock_statusBegin,   mock_eepromStoreBegin, mock_motorsBegin,
      mock_sensorsBegin,  mock_lineBegin,        mock_markersBegin,
      mock_linkBegin,     mock_fsmBegin,         mock_beginCalRun,
      mock_missionBegin,
  };
  rescue::runBootOrder(steps, kBootStepCount, nullptr);

  CHECK_EQ(static_cast<int>(gCalls.size()), static_cast<int>(kBootStepCount));
  for (size_t index = 0; index < gCalls.size(); ++index) {
    CHECK_EQ(gCalls[index], std::string(kExpectedBoot[index]));
  }
}

void testLoopPlan() {
  // While the sweep owns the sensors the mission must not sample them.
  CHECK(rescue::planLoopPass(true) == rescue::LoopPass::CalibrationSweep,
        "a running sweep owns the loop");
  CHECK(rescue::planLoopPass(false) == rescue::LoopPass::Mission,
        "with no sweep the mission owns the loop");
}

void testStatusLatch() {
  rescue::statusReset();
  CHECK(rescue::statusSignal() == rescue::StatusSignal::Boot,
        "a reset indicator starts on the boot signal");

  rescue::statusSet(rescue::StatusSignal::Ready);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::Ready,
        "ready clears the transient boot signal");

  rescue::statusSet(rescue::StatusSignal::NiclaMissing);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::NiclaMissing,
        "a closed READY window latches");

  // A fault is not a status: the next healthy pass must not erase why the robot
  // is running blind.
  rescue::statusSet(rescue::StatusSignal::Ready);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::NiclaMissing,
        "a healthy pass does not clear a latched fault");

  // A later fault replaces an earlier one: the newest reason is the useful one.
  rescue::statusSet(rescue::StatusSignal::CalibrationAborted);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::CalibrationAborted,
        "a later fault replaces the latched one");

  // A restart is not a fault either, but it is transient, so ready clears it.
  rescue::statusReset();
  rescue::statusSet(rescue::StatusSignal::LossOfPower);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::LossOfPower,
        "a restart latches too");
  rescue::statusSet(rescue::StatusSignal::Ready);
  CHECK(rescue::statusSignal() == rescue::StatusSignal::Ready,
        "ready clears a restart, which the next pass proves is over");
}

void testStatusTickIsNonBlocking() {
  rescue::statusReset();
  rescue::statusSet(rescue::StatusSignal::NiclaMissing);
  const rescue::StatusPattern pattern = rescue::statusPattern();
  CHECK(pattern.onMs > 0 && pattern.offMs > 0,
        "a fault blinks rather than sitting steady");

  // The cadence is decided by elapsed time handed in, never by a wait inside.
  rescue::statusTick(0);
  const bool first = rescue::statusLampOn();
  rescue::statusTick(pattern.onMs);
  const bool second = rescue::statusLampOn();
  CHECK(first != second, "the lamp changes with elapsed time, not on a counter");
}

void testSketchSourceIsWhole() {
  const std::string main_ino = readFile("src/rescue_robot/main.ino");
  CHECK(!main_ino.empty(), "main.ino exists at the sketch root");

  // One owner per header: a second include is a sign the author was unsure.
  // Arduino.h is a vendor header, so the sketch brings it in by name; the rest
  // are the project's own, included by path.
  struct HeaderNeedle {
    const char* header;
    bool angled;
  };
  const HeaderNeedle headers[] = {
      {"Arduino.h", true},  {"config.h", false},     {"calibration.h", false},
      {"diag_parse.h", false},    {"eeprom_store.h", false},
      {"fsm.h", false},     {"line.h", false},       {"link.h", false},
      {"markers.h", false}, {"mission.h", false},    {"motors.h", false},
      {"sensors.h", false}, {"sketch.h", false},     {"status.h", false}};
  for (const HeaderNeedle header : headers) {
    const std::string needle = header.angled
                                   ? std::string("#include <") + header.header +
                                         ">"
                                   : std::string("#include \"") +
                                         header.header + "\"";
    size_t count = 0;
    for (size_t at = main_ino.find(needle); at != std::string::npos;
         at = main_ino.find(needle, at + needle.size())) {
      ++count;
    }
    CHECK_EQ(static_cast<int>(count), 1);
  }
}

void testBootTableMatchesTheOrder() {
  const std::string main_ino = readFile("src/rescue_robot/main.ino");
  // The declaration is the one anchored, comment-free occurrence: comments
  // mention the boot order but never the array's qualified spelling.
  const size_t open = main_ino.find("BootFn kBootOrder[]");
  CHECK(open != std::string::npos, "main.ino installs one boot table");
  if (open == std::string::npos) return;

  size_t brace = main_ino.find('{', open);
  size_t close = main_ino.find('}', brace);
  CHECK(brace != std::string::npos && close != std::string::npos,
        "the boot table is an initialiser list");
  if (brace == std::string::npos || close == std::string::npos) return;

  // Each step is one entry, name followed by its comma, so the needle is the
  // entry as it will actually appear in a function-pointer initialiser list.
  std::string body = main_ino.substr(brace + 1, close - brace - 1);
  for (const char* step : kExpectedBoot) {
    const std::string needle = std::string(step) + ",";
    const size_t at = body.find(needle);
    CHECK(at != std::string::npos, step);
    body = (at == std::string::npos) ? body : body.substr(at + needle.size());
  }
}

void testNoBlockingInTheLoop() {
  const std::string main_ino = readFile("src/rescue_robot/main.ino");
  CHECK(main_ino.find("delay(") == std::string::npos,
        "the assembled loop has no delay()");
}

void testLinePinsLiveInConfig() {
  const std::string config = readFile("src/rescue_robot/config.h");
  const std::string line_ino = readFile("src/rescue_robot/line.ino");
  CHECK(config.find("LINE_SENSOR_PINS") != std::string::npos,
        "config.h owns the line sensor pin table");
  CHECK(line_ino.find("LINE_SENSOR_PINS[") != std::string::npos,
        "line.ino reads the table");
  CHECK(line_ino.find("constexpr uint8_t LINE_SENSOR_PINS") ==
            std::string::npos,
        "line.ino no longer declares its own pin table");
}

}  // namespace

int main() {
  testBootOrderWalksEveryStepOnce();
  testLoopPlan();
  testStatusLatch();
  testStatusTickIsNonBlocking();
  testSketchSourceIsWhole();
  testBootTableMatchesTheOrder();
  testNoBlockingInTheLoop();
  testLinePinsLiveInConfig();
  return DF_SUMMARIZE("sketch_assembly");
}