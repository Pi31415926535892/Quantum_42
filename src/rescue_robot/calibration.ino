// calibration.ino — AVR delivery layer for the venue sweep.
//
// Owns every Arduino-side touch on behalf of calibration.h: the QTR-8A pin
// list, the millis clock that times the window, the per-tick sample, the
// console diagnostic dump and the store commit. It contains no sweep logic —
// the window arithmetic, the per-channel midpoints, the contrast-health gate
// and the retry state machine all live in calibration.h.
//
// Non-blocking by construction: this file never calls delay() and never spins.
// The caller drives it with beginCalSweep() once, tickCalSweep() every main
// loop, and finishCalSweep() when the window is due to have closed. A sweep
// lasts ten seconds of wall clock, so the caller stays responsive to the start
// button throughout.
//
// Wiring (HARDWARE.md §4): the bar's connector was flipped on 2026-10-20, so
// OUT1 (leftmost) lands on A14 and OUT8 (rightmost) on A0. The table keeps
// logical order, so the sweep still walks left to right; the black-to-white
// spread at the locked 11.2 mm ride height is exactly what the health gate
// judges.
//
// The store calls below (loadStoredCalibration, storeCalibration, SaveResult)
// live in eeprom_store.ino, which the Arduino build concatenates into this same
// translation unit. A venue sweep is a single-record write, not a release: the
// write-before-release ordering exists to pair a calibration commit with a
// victim release, and a sweep never carries accounting.

#include <Arduino.h>

#include "calibration.h"
#include "config.h"
#include "eeprom_store.h"
#include "line.h"

using namespace rescue;

// The store's entry points live in eeprom_store.ino, which the Arduino build
// concatenates into this same translation unit. Only the names are repeated
// here, because the host test builds this file on its own — and because the
// Arduino IDE's automatic prototype generation is not something to lean on for
// a file this project's own assembly test compiles. SaveResult is not repeated:
// it is a decision type, so it lives in eeprom_store.h with the rest of the
// store's vocabulary.
bool loadStoredCalibration(CalibrationRecord& out);
SaveResult storeCalibration(const CalibrationRecord& record);

namespace {

// The bar's pins are the line module's pins: same table, same eight sensors,
// read in the same order. Sweeping a different bar than the one the robot
// follows would calibrate the wrong readings, so the sweep never declares its
// own wiring.
constexpr uint8_t CAL_SENSOR_PINS[CAL_CHANNEL_COUNT] = {
    LINE_SENSOR_PINS[0], LINE_SENSOR_PINS[1], LINE_SENSOR_PINS[2],
    LINE_SENSOR_PINS[3], LINE_SENSOR_PINS[4], LINE_SENSOR_PINS[5],
    LINE_SENSOR_PINS[6], LINE_SENSOR_PINS[7]};

SweepAccumulator g_accumulator;
CalAttempt g_run;
uint32_t g_startedMs = 0;
uint32_t g_lastSampleMs = 0;
bool g_sweeping = false;

// One pass over the bar, in channel order. The QTR-8A analog variant runs its
// emitters continuously, so there is nothing to switch on around the read.
CalRawVector readBar() {
  CalRawVector samples{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    samples.reading[channel] = analogRead(CAL_SENSOR_PINS[channel]);
  }
  return samples;
}

// Arduino's clock accessor returns unsigned long; the sweep's window arithmetic
// wants a fixed-width return type.
uint32_t nowMs() { return static_cast<uint32_t>(millis()); }

// Take a sample if the cadence is due and the window still has time in it.
// Once the window has closed the totals are left exactly as they stand, so a
// slow loop costs coverage rather than overshooting into the next half.
void sampleDue(uint32_t elapsedMs) {
  const uint32_t now = nowMs();
  if (sweepWindowElapsed(elapsedMs)) return;
  if (now - g_lastSampleMs < CAL_SAMPLE_INTERVAL_MS) return;
  sweepAccumulate(g_accumulator, sweepPhaseAt(elapsedMs), readBar());
  g_lastSampleMs = now;
}

// The per-channel numbers a rejected sweep has to be judged on. Printed for
// every closed window, accepted or not, so a refusal can be diagnosed from the
// log alone.
void printSweep(const SweepResult& result) {
  Serial.print("cal: healthy ");
  Serial.print(result.healthy_count);
  Serial.print('/');
  Serial.print(CAL_CHANNEL_COUNT);
  Serial.print(" weak mask ");
  Serial.println(result.weak_channel_mask);
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const SweepChannel& entry = result.channel[channel];
    Serial.print("  ch");
    Serial.print(channel);
    Serial.print(" black ");
    Serial.print(entry.black_level);
    Serial.print(" white ");
    Serial.print(entry.white_level);
    Serial.print(" mid ");
    Serial.print(entry.midpoint());
    Serial.print(" contrast ");
    Serial.print(entry.contrast());
    Serial.print(entry.healthy ? " ok" : " REJECTED");
    Serial.print(" weight ");
    Serial.println(entry.weight);
  }
}

void logDegradation(CalDecision decision, const SweepResult& result) {
  Serial.print("cal: DEGRADED - running on ");
  Serial.print(calDecisionName(decision));
  Serial.print(" after ");
  Serial.print(g_run.attempt);
  Serial.print(" sweeps; healthy ");
  Serial.print(result.healthy_count);
  Serial.print(", weak mask ");
  Serial.println(result.weak_channel_mask);
}

// Act on the settled decision: install the basis the run will steer on, and
// persist the venue basis only when this sweep is the one that earned it.
void applyDecision(CalDecision decision, const SweepResult& result,
                   const CalibrationRecord& lastGood) {
  switch (decision) {
    case CalDecision::Persisted:
      setCalibration(lineCalibrationFrom(result));
      Serial.print("cal: store ");
      Serial.println(storeCalibration(storedRecordFrom(result)) == SaveResult::Saved
                         ? "saved"
                         : "REFUSED");
      break;
    case CalDecision::LastGoodPersisted:
      setCalibration(lineCalibrationFrom(lastGood));
      logDegradation(decision, result);
      break;
    case CalDecision::Degraded:
      // Deliberately not persisted: a sweep the gate refused is not a venue
      // basis, so it may not overwrite the last good one in the store.
      setCalibration(lineCalibrationFrom(result));
      logDegradation(decision, result);
      break;
    case CalDecision::Abort:
      // Two roads lead here: the venue forbade degrading, or there was nothing
      // left to degrade to. Either way there is no basis to steer on.
      Serial.println("cal: ABORT - no calibration basis to steer on");
      break;
    case CalDecision::Pending:
      Serial.println("cal: sweep refused, sweeping again");
      break;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Public surface
// ---------------------------------------------------------------------------

// Open a window: clear the totals, start the clock, and tell the operator to
// hold the bar over the black line for the first half and the white floor for
// the second. The retry allowance is deliberately untouched — a retry must cost
// one of the two retries, not refill them.
void beginCalSweep() {
  sweepReset(g_accumulator);
  g_startedMs = nowMs();
  g_lastSampleMs = g_startedMs;
  g_sweeping = true;
  Serial.println("cal: sweep open - hold the bar over BLACK, then WHITE");
}

// Open the first window of a run: refill the retry allowance and sweep. The
// caller does this once, when calibration starts, not once per sweep.
void beginCalRun() {
  calRunBegin(g_run);
  beginCalSweep();
}

// One sweep slice. Sample when the cadence is due; the window decides when the
// sweep is over, not this call.
void tickCalSweep() {
  if (!g_sweeping) return;
  sampleDue(nowMs() - g_startedMs);
}

// Close the window and settle the run. Returns Pending while the ten seconds
// have not yet run out — the caller keeps ticking and asks again. A run that
// owes a retry also returns Pending, and the caller re-opens the window with
// beginCalSweep().
CalDecision finishCalSweep(CalFallback fallback) {
  if (!g_sweeping) return g_run.decision;
  if (!sweepWindowElapsed(nowMs() - g_startedMs)) return CalDecision::Pending;
  g_sweeping = false;

  const SweepResult result = sweepEvaluate(g_accumulator);
  CalibrationRecord lastGood{};
  // Only a stored record the sweep itself would have accepted may stand in for
  // a fresh venue sweep; a stored weak bar is not a basis either.
  const bool lastGoodStored = loadStoredCalibration(lastGood) &&
                              lastGood.contrast == ContrastVerdict::Healthy;
  const CalVenuePolicy policy{fallback, lastGoodStored};
  calConcludeAttempt(g_run, result, policy);

  printSweep(result);
  applyDecision(g_run.decision, result, lastGood);
  Serial.print("cal: decision ");
  Serial.println(calDecisionName(g_run.decision));
  return g_run.decision;
}

// True while a window is open and the caller still owes samples to it.
bool calSweepRunning() { return g_sweeping; }

// The run's standing after the last closed window: attempts spent, healthy
// channels, weak mask and decision. The diagnostics menu reads this.
CalAttempt calAttemptState() { return g_run; }
