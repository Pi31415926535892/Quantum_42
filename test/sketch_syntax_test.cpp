// sketch_syntax_test.cpp — host-side compile check for the .ino files.
//
// The AVR toolchain is not available on this machine, so the .ino files are
// instead type-checked as C++14 translation units against the inert stand-ins
// in test/arduino_stub (Arduino.h, QTRSensors.h). A .ino has no extension the
// compiler recognises, so each is pulled in with an #include instead.
//
// Each sketch is wrapped in its own namespace: the two legacy sketches are
// separate flash targets that both define LEFT_IN1 and friends at global scope,
// so sharing one translation unit would collide. The namespaces are a harness
// device only — on the Mega each sketch is compiled alone.
//
// What this proves: the sketches parse, the names they use exist, and the
// types line up. What it does NOT prove: that the real Arduino core and the
// real QTR-8A library have the same signatures, nor any AVR runtime behaviour.
// Both remain bench questions (HARDWARE.md section 7).
//
// Build/run: ./test/run_tests.sh (picked up automatically).

#include <cstdio>

#include "arduino_stub/Arduino.h"
#include "arduino_stub/QTRSensors.h"

// The pure module headers, for the types diag.ino's stand-ins below return. Each
// one carries its own names, so this order carries no meaning and a later header
// can be inserted at either end of it.
#include "calibration.h"
#include "line.h"
#include "link.h"
#include "markers.h"
#include "motors.h"
#include "sensors.h"

namespace pwm_test_sketch {
#include "../PWM Test"
}  // namespace pwm_test_sketch

namespace robocup_sketch {
#include "../Robocup.ino"
}  // namespace robocup_sketch

// diag.ino is delivery-only and declares the module entry points it calls, so
// it needs the same stand-ins the real sketch would supply.
void setMotor(rescue::MotorSide, int) {}
void setMotor(int, int) {}
void motorsStop() {}
rescue::LineReading updateLine() { return rescue::LineReading{}; }
rescue::VictimClass classifyVictim() { return rescue::VictimClass::Fake; }
int readReflectivity() { return 0; }
bool readConductivity() { return false; }
rescue::SurfaceKind probeSurface() { return rescue::SurfaceKind::Black; }
rescue::Liveness linkLiveness() { return rescue::Liveness::Lost; }
rescue::LinkPolicy linkPolicy() { return rescue::LinkPolicy::LinkFresh; }
uint16_t linkTxOverflows() { return 0; }
uint8_t linkTxQueued() { return 0; }

#include "diag.ino"

// The mission sketch (fsm.ino plus mission.ino) is a real delivery layer: it
// brings the modules up and drives them, so it gets the same treatment as
// diag.ino — the module entry points it calls are stood in for, and the two
// files are compiled together the way the Arduino builder concatenates them.
void motorsBegin() {}
void sensorsBegin() {}
void updateBumper() {}
bool bumperPressed(rescue::BumperZone) { return false; }
bool consumeAnyBumperEvent(rescue::BumperZone& zone) {
  zone = rescue::BumperZone::Left;
  return false;
}
void lineBegin() {}
void markersBegin() {}
void setMarkerWhiteBaseline(uint16_t) {}
rescue::MarkerPoll pollMarkerSensors() { return rescue::MarkerPoll{}; }
void linkBegin() {}
void linkTick() {}
const rescue::CachedFrame* linkLastGood(uint8_t) { return nullptr; }
bool linkReceiveFresh(uint8_t) { return false; }
// The restart asks for one concurrent radar sweep. Stand-in: the request is
// accepted and reported sent, so the sweep ledger spends its budget exactly as
// it does on the wire.
bool linkSend(uint8_t, const uint8_t*, uint8_t) { return true; }
void eepromStoreBegin() {}
bool loadStoredCalibration(rescue::CalibrationRecord& out) {
  out = rescue::CalibrationRecord{};
  return false;
}
// The persisted ledger the restart stages from, and the one write it makes when
// the resume publishes. A blank device reports fresh — a clean start — and the
// write is accepted and dropped. SaveResult is the store's own verdict type,
// reached through eeprom_store.h (which calibration.h above already includes),
// so the stub names it in the store's namespace and the mission sketch's
// using-directive stays unambiguous. The zero value is the store's "Saved" — a
// verdict nothing in this suite reads.
bool loadStoredAccounting(rescue::AccountingRecord& out) {
  out = rescue::freshAccounting();
  return false;
}
rescue::SaveResult storeAccounting(const rescue::AccountingRecord&) {
  return rescue::SaveResult{};
}

#include "fsm.ino"
#include "mission.ino"

namespace {

int gFailures = 0;

void report(bool ok, const char* what) {
  if (!ok) {
    ++gFailures;
    std::printf("FAIL sketch_syntax: %s\n", what);
  }
}

}  // namespace

int main() {
  // Referencing one symbol from each translation unit proves the definitions
  // were actually emitted, not merely parsed.
  std::printf("PWM Test    : LEFT {in1=%d in2=%d pwm=%d}  "
              "RIGHT {in1=%d in2=%d pwm=%d}  max=%d\n",
              pwm_test_sketch::LEFT_IN1, pwm_test_sketch::LEFT_IN2,
              pwm_test_sketch::LEFT_PWM, pwm_test_sketch::RIGHT_IN1,
              pwm_test_sketch::RIGHT_IN2, pwm_test_sketch::RIGHT_PWM,
              pwm_test_sketch::MAX_SPEED);
  std::printf("Robocup.ino : LEFT {in1=%d in2=%d pwm=%d}  "
              "RIGHT {in1=%d in2=%d pwm=%d}  Kp=%.3f Kd=%.3f\n",
              robocup_sketch::LEFT_IN1, robocup_sketch::LEFT_IN2,
              robocup_sketch::LEFT_PWM, robocup_sketch::RIGHT_IN1,
              robocup_sketch::RIGHT_IN2, robocup_sketch::RIGHT_PWM,
              robocup_sketch::Kp, robocup_sketch::Kd);

  // The two sketches are separate flash targets, so they must agree on the
  // pin table. The real assertion for this lives in bench_remap_test.cpp,
  // which compares both against config.h; this is a cheap cross-check that
  // they also agree with each other after being pulled into one build.
  report(pwm_test_sketch::LEFT_IN1 == robocup_sketch::LEFT_IN1 &&
             pwm_test_sketch::LEFT_IN2 == robocup_sketch::LEFT_IN2 &&
             pwm_test_sketch::LEFT_PWM == robocup_sketch::LEFT_PWM &&
             pwm_test_sketch::RIGHT_IN1 == robocup_sketch::RIGHT_IN1 &&
             pwm_test_sketch::RIGHT_IN2 == robocup_sketch::RIGHT_IN2 &&
             pwm_test_sketch::RIGHT_PWM == robocup_sketch::RIGHT_PWM,
         "the two legacy sketches disagree on the pin table");

  // The corrected gains: positive Kp for the remapped wiring, Kd off until
  // the P term tracks.
  report(robocup_sketch::Kp > 0.0f,
         "Robocup.ino Kp must be positive under the section 6 remap");
  report(robocup_sketch::Kd == 0.0f,
         "Robocup.ino Kd must be 0 until the P term tracks");

  // The line-lost predicate the archive now uses, on both readings. The
  // reading must land in the sketch's own sensorValues buffer, because that
  // is what lineIsPresent() inspects.
  robocup_sketch::qtr.qtrSeedDarkChannels(8);
  robocup_sketch::qtr.readLineBlack(robocup_sketch::sensorValues);
  const bool onTheLine = robocup_sketch::lineIsPresent();

  robocup_sketch::qtr.qtrSeedDarkChannels(0);
  robocup_sketch::qtr.readLineBlack(robocup_sketch::sensorValues);
  const bool offTheLine = robocup_sketch::lineIsPresent();

  std::printf("lineIsPresent: on the line = %s, off the line = %s\n",
              onTheLine ? "true" : "false", offTheLine ? "true" : "false");
  report(onTheLine, "lineIsPresent() is false while sitting on the line");
  report(!offTheLine, "lineIsPresent() is true when the line is gone");

  // A single dark channel is a shadow, not a line.
  robocup_sketch::qtr.qtrSeedDarkChannels(1);
  robocup_sketch::qtr.readLineBlack(robocup_sketch::sensorValues);
  report(!robocup_sketch::lineIsPresent(),
         "lineIsPresent() accepts a single dark channel as the line");

  // diagTick() must be callable and must be a no-op with no pending input.
  gSerialIn.clear();
  serialClearOut();
  diagTick();
  report(gSerialOut.empty(), "diagTick printed with no input");

  std::printf("[sketch_syntax] failures: %d\n", gFailures);
  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  return 1;
}
