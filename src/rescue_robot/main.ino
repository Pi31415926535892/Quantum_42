// main.ino — the one sketch: boot order, loop pass plan, status lamp.
//
// This is the only file with a setup()/loop(). Everything else is a module:
// fsm.ino owns the mission engine and its table, mission.ino owns the per-tick
// sensing pass, and neither knows that setup() or loop() exist. The boot order
// is one array walked by sketch.h's runBootOrder(), so a module that joins or
// leaves is a one-line edit to that array, not another call
// buried in a setup(). The pass the loop runs belongs to sketch.h's planLoopPass:
// while the venue sweep has a window open it owns the bar the mission would
// otherwise sample, so the mission waits in Boot until the sweep closes.
//
// The status lamp (status.h + the fused LED) is the only thing this file
// touches that no module claims: it is read once per loop and driven with the
// hesitations the modules report — a boot, a running mission, a lost link or a
// refused calibration are all visible on one diode.
//
// Boot steps follow the contract's order: the lamp starts first, the mission
// brings the drivetrain, bar, markers, link and store up before its own state,
// and the calibration run opens its first sweep window last — so at power-on
// the robot waits on the operator's bar swipe before the mission ever ticks.
// missionBegin() repeats some of the module begins it was self-sufficient with
// before the boot table existed; each begin is idempotent initialization, so
// the redundancy is harmless and its self-sufficiency is what the module tests
// still rely on.

#include <Arduino.h>

#include "calibration.h"
#include "config.h"
#include "diag_parse.h"
#include "eeprom_store.h"
#include "fsm.h"
#include "line.h"
#include "link.h"
#include "markers.h"
#include "mission.h"
#include "motors.h"
#include "sensors.h"
#include "sketch.h"
#include "status.h"

// The builder concatenates the delivery files into this one translation unit;
// these names are declared here so the boot table reads top to bottom instead
// of trusting the IDE's generated prototypes.
void eepromStoreBegin();
void motorsBegin();
void sensorsBegin();
void lineBegin();
void markersBegin();
void linkBegin();
void fsmBegin();
void beginCalRun();
void missionBegin();
void diagTick();
void tickCalSweep();
CalDecision finishCalSweep(CalFallback fallback);
bool calSweepRunning();
rescue::Liveness linkLiveness();

// The lamp is this file's instrument: reset to Boot, its pin is ours.
static void statusBegin() {
  rescue::statusReset();
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, rescue::statusLampOn() ? HIGH : LOW);
}

// The contract's boot order, one per module, in the order reason demands.
static const rescue::BootFn kBootOrder[] = {
    statusBegin,
    eepromStoreBegin,
    motorsBegin,
    sensorsBegin,
    lineBegin,
    markersBegin,
    linkBegin,
    fsmBegin,
    beginCalRun,
    missionBegin,
};
static const size_t kBootStepCount =
    sizeof(kBootOrder) / sizeof(kBootOrder[0]);

namespace {

uint32_t g_lampPrevMs = 0;

// Advance the lamp's cadence and push it to the LED; nothing blocks.
void lampTick() {
  const uint32_t now = millis();
  rescue::statusTick(static_cast<uint32_t>(now - g_lampPrevMs));
  g_lampPrevMs = now;
  digitalWrite(STATUS_LED_PIN, rescue::statusLampOn() ? HIGH : LOW);
}

// Once the sweep's window has closed, settle the run exactly once and let the
// lamp say what the venue decided. A sweep the run may not start on is a
// latched CalibrationAborted; a standing basis just lets the mission proceed.
bool g_calSettled = false;
void settleCalibration() {
  if (calSweepRunning() || g_calSettled) return;
  g_calSettled = true;
  const CalDecision decision =
      finishCalSweep(CalFallback::LastGoodThenDegrade);
  if (decision == CalDecision::Abort) {
    rescue::statusSet(rescue::StatusSignal::CalibrationAborted);
  }
}

// While the mission owns the pass, the lamp reports the host link: fresh is
// Ready, lost latches NiclaMissing (which only a reset clears, by design).
void sampleHealth() {
  rescue::statusSet(linkLiveness() == rescue::Liveness::Lost
                        ? rescue::StatusSignal::NiclaMissing
                        : rescue::StatusSignal::Ready);
}

}  // namespace

void setup() {
  rescue::runBootOrder(kBootOrder, kBootStepCount, nullptr);
}

void loop() {
  lampTick();
  diagTick();

  if (rescue::planLoopPass(calSweepRunning()) ==
      rescue::LoopPass::CalibrationSweep) {
    tickCalSweep();
    settleCalibration();
  } else {
    missionTick();
    sampleHealth();
  }
}