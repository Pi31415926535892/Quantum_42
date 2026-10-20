// sensors_test.cpp — host-side tests for src/rescue_robot/sensors.h (pure C++14).
// Build/run: ./test/run_tests.sh  (or see that script for the g++ line).

#include "sensors.h"

#include "dryframe.h"

using rescue::BumperPlate;
using rescue::BumperZone;
using rescue::DebouncedButton;
using rescue::ProbeClassifier;
using rescue::SurfaceKind;
using rescue::VictimClass;

namespace {

using rescue::BUTTON_DEBOUNCE_MS;
using rescue::REFLECTIVITY_HYSTERESIS;
using rescue::REFLECTIVITY_SILVER_THRESHOLD;
namespace Table = rescue;  // classifyFromReadings / applyReflectivity

// Reading levels used across the probe tests, expressed against the shipped
// thresholds so the suite keeps working if the constants are re-tuned.
constexpr int kSilverStrong = REFLECTIVITY_SILVER_THRESHOLD + REFLECTIVITY_HYSTERESIS + 50;
constexpr int kBlackStrong = REFLECTIVITY_SILVER_THRESHOLD - REFLECTIVITY_HYSTERESIS - 50;
constexpr int kEnterSilver = REFLECTIVITY_SILVER_THRESHOLD + REFLECTIVITY_HYSTERESIS;
constexpr int kLeaveSilver = REFLECTIVITY_SILVER_THRESHOLD - REFLECTIVITY_HYSTERESIS;
constexpr int kJustBelowThreshold = REFLECTIVITY_SILVER_THRESHOLD - 1;
constexpr int kJustAboveThreshold = REFLECTIVITY_SILVER_THRESHOLD + 1;

// A debounce window distinct from the shipped default, to prove the button
// takes its window as configuration rather than hard-coding it.
constexpr uint16_t kLongDebounceMs = 60;
constexpr uint32_t kPressStartMs = 100;
void pressZone(BumperPlate& plate, bool left, bool right, uint32_t startMs) {
  plate.update(left, right, startMs);
  plate.update(left, right, startMs + BUTTON_DEBOUNCE_MS);
}

// Debounced button

void buttonStartsReleased() {
  DebouncedButton button;
  CHECK(!button.isPressed(), "a fresh button reads released");
  CHECK(!button.hasPendingEvent(), "a fresh button has no pending press");
  CHECK(!button.consumeEvent(), "consuming a fresh button yields nothing");
}
void bounceInsideWindowIsIgnored() {
  DebouncedButton button;
  button.update(false, 0);
  button.update(true, 10);   // candidate press opens a window at t=10
  button.update(false, 20);  // contact opens: window restarts, still inside
  button.update(false, 40);
  CHECK(!button.isPressed(), "contact bounce never reaches the stable state");
  CHECK(!button.hasPendingEvent(), "contact bounce latches no press event");
}
void cleanPressLatchesExactlyOnce() {
  DebouncedButton button;
  button.update(false, 0);
  button.update(true, 10);
  button.update(true, 10 + BUTTON_DEBOUNCE_MS - 1);
  CHECK(!button.hasPendingEvent(), "press is not latched before the window closes");
  CHECK(!button.isPressed(), "press is not stable before the window closes");
  button.update(true, 10 + BUTTON_DEBOUNCE_MS);
  CHECK(button.isPressed(), "press commits once the window closes");
  CHECK(button.hasPendingEvent(), "a committed press is latched");
  CHECK(button.consumeEvent(), "consume returns the latched press");
  CHECK(!button.consumeEvent(), "a latched press fires exactly once");
}
void releaseDoesNotClearTheLatch() {
  DebouncedButton button;
  button.update(true, 0);  // first sample already held
  button.update(true, BUTTON_DEBOUNCE_MS);
  CHECK(button.hasPendingEvent(), "a held-at-boot press latches");
  button.update(false, 2 * BUTTON_DEBOUNCE_MS);
  button.update(false, 3 * BUTTON_DEBOUNCE_MS);
  CHECK(!button.isPressed(), "a stable release settles the button to released");
  CHECK(button.hasPendingEvent(), "release does not consume a pending press");
  CHECK(button.consumeEvent(), "the pending press survives until it is consumed");
}
void customDebounceWindowIsHonoured() {
  DebouncedButton button(kLongDebounceMs);
  button.update(false, 0);
  button.update(true, kPressStartMs);
  button.update(true, kPressStartMs + BUTTON_DEBOUNCE_MS);
  CHECK(!button.hasPendingEvent(), "the default window does not apply to a custom one");
  button.update(true, kPressStartMs + kLongDebounceMs);
  CHECK(button.isPressed(), "a custom window commits at its own length");
}
void longIdleCommitsInOneCall() {
  DebouncedButton button;
  button.update(false, 0);
  button.update(true, 5);
  button.update(true, 100000);  // a long stall must not lose the press
  CHECK(button.isPressed(), "a press that outlives a long stall still commits");
  CHECK(button.consumeEvent(), "the event survives a long stall");
}
void millisWraparoundStillCommits() {
  constexpr uint32_t kNearOverflow = 0xFFFFFF00UL;
  DebouncedButton button;
  button.update(false, kNearOverflow);
  button.update(true, kNearOverflow + 8);
  button.update(true, 16);  // clock wrapped past zero mid-window
  CHECK(button.isPressed(), "unsigned millis arithmetic survives the rollover");
  CHECK(button.consumeEvent(), "a press spanning the rollover still latches");
}
void resetReturnsToTheIdleState() {
  DebouncedButton button;
  button.update(true, 0);
  button.update(true, BUTTON_DEBOUNCE_MS);
  button.reset();
  CHECK(!button.isPressed(), "reset clears the stable state");
  CHECK(!button.hasPendingEvent(), "reset drops any latched press");
}

// Bumper plate: two independent zones on one never-miss contact

void zonesAreIndependent() {
  BumperPlate plate;
  plate.update(false, false, 0);
  pressZone(plate, true, false, 10);
  CHECK(plate.hasEvent(BumperZone::Left), "the left zone latched its own press");
  CHECK(!plate.hasEvent(BumperZone::Right), "the right zone stays quiet");
  CHECK(plate.consumeEvent(BumperZone::Left), "the left zone press is consumable");
  CHECK(!plate.consumeEvent(BumperZone::Left), "the left latch is spent");
  CHECK(!plate.hasEvent(BumperZone::Right), "the right zone was never involved");
  pressZone(plate, true, true, 60);
  CHECK(!plate.hasEvent(BumperZone::Left), "a held left button does not re-latch");
  CHECK(plate.hasEvent(BumperZone::Right), "the right zone latched independently");
}
void anyZoneReportsTheStruckSide() {
  BumperZone struck = BumperZone::Left;
  BumperPlate plate;
  plate.update(false, false, 0);
  pressZone(plate, false, true, 10);
  CHECK(plate.consumeAnyZone(struck), "a right-zone hit is reported once");
  CHECK(struck == BumperZone::Right, "the reported zone is the side actually struck");
  CHECK(!plate.consumeAnyZone(struck), "the event is spent after being reported");
  pressZone(plate, false, false, 100);
  CHECK(!plate.consumeAnyZone(struck), "a settled plate reports nothing further");
}
void bothZonesCanBeStruckTogether() {
  BumperPlate plate;
  plate.update(false, false, 0);
  pressZone(plate, true, true, 10);
  CHECK(plate.hasEvent(BumperZone::Left), "a square hit latches the left zone");
  CHECK(plate.hasEvent(BumperZone::Right), "a square hit latches the right zone");
  CHECK(plate.consumeEvent(BumperZone::Left), "left drains independently");
  CHECK(plate.hasEvent(BumperZone::Right), "draining left leaves right pending");
}
void plateReportsDebouncedLevels() {
  BumperPlate plate;
  plate.update(false, false, 0);
  pressZone(plate, false, true, 10);
  CHECK(plate.isPressed(BumperZone::Right), "the plate reports the held zone as pressed");
  CHECK(!plate.isPressed(BumperZone::Left), "the untouched zone reads released");
}
void plateResetDropsPendingStrikes() {
  BumperPlate plate;
  plate.update(false, false, 0);
  pressZone(plate, true, true, 10);
  plate.reset();
  CHECK(!plate.isPressed(BumperZone::Left), "reset releases every zone");
  CHECK(!plate.isPressed(BumperZone::Right), "reset releases every zone");
  CHECK(!plate.hasEvent(BumperZone::Left), "reset drops the left latch");
  CHECK(!plate.hasEvent(BumperZone::Right), "reset drops the right latch");
}

// Truth table and hysteresis, exercised directly

void truthTableHasNoFifthOutcome() {
  const SurfaceKind silver = SurfaceKind::Silver;
  const SurfaceKind black = SurfaceKind::Black;
  const rescue::Conductivity yes = rescue::Conductivity::Conductive;
  const rescue::Conductivity no = rescue::Conductivity::Insulating;
  CHECK(Table::classifyFromReadings(silver, yes) == VictimClass::Live,
        "silver + conductive = live");
  CHECK(Table::classifyFromReadings(silver, no) == VictimClass::Fake,
        "silver + insulating = fake");
  CHECK(Table::classifyFromReadings(black, no) == VictimClass::Dead,
        "black + insulating = dead");
  CHECK(Table::classifyFromReadings(black, yes) == VictimClass::Fake,
        "black + conductive = fake");
}
void hysteresisBandIsHonouredByBandEdge() {
  CHECK(Table::applyReflectivity(SurfaceKind::Black, kEnterSilver - 1) == SurfaceKind::Black,
        "one count under the upper band edge stays black");
  CHECK(Table::applyReflectivity(SurfaceKind::Black, kEnterSilver) == SurfaceKind::Silver,
        "the upper band edge itself turns black to silver");
  CHECK(Table::applyReflectivity(SurfaceKind::Silver, kLeaveSilver) == SurfaceKind::Silver,
        "the lower band edge itself keeps silver");
  CHECK(Table::applyReflectivity(SurfaceKind::Silver, kLeaveSilver - 1) == SurfaceKind::Black,
        "one count under the lower band edge turns silver to black");
}

// Surface reading (reflectivity) with hysteresis

void silverNeedsToClearTheUpperBand() {
  ProbeClassifier classifier;
  CHECK(classifier.surface() == SurfaceKind::Black, "an unproven surface starts black");
  CHECK(classifier.classify(kJustAboveThreshold, true) == VictimClass::Fake,
        "a reading inside the hysteresis band is not yet silver");
  CHECK(classifier.surface() == SurfaceKind::Black, "the ambiguous band does not flip it");
  CHECK(classifier.classify(kSilverStrong, true) == VictimClass::Live,
        "a decisively reflective reading is silver and conductive = live");
  CHECK(classifier.surface() == SurfaceKind::Silver, "the surface is now silver");
}
void silverMustFallBelowTheLowerBand() {
  ProbeClassifier classifier;
  CHECK(classifier.classify(kSilverStrong, true) == VictimClass::Live, "silver latch-up");
  CHECK(classifier.classify(kJustBelowThreshold, false) == VictimClass::Fake,
        "a drop inside the hysteresis band keeps silver, and silver + "
        "non-conductive = fake");
  CHECK(classifier.surface() == SurfaceKind::Silver, "the band holds silver stable");
  CHECK(classifier.classify(kBlackStrong, false) == VictimClass::Dead,
        "a decisively dark reading is black and non-conductive = dead");
  CHECK(classifier.surface() == SurfaceKind::Black, "the surface is now black");
}
void boundaryReadingsResolveByBandMembership() {
  ProbeClassifier fromBlack;
  CHECK(fromBlack.classify(REFLECTIVITY_SILVER_THRESHOLD, true) == VictimClass::Fake,
        "exactly at the threshold from black is not yet silver, so conductive reads fake");
  ProbeClassifier fromSilver;
  CHECK(fromSilver.classify(kSilverStrong, true) == VictimClass::Live, "silver latch-up");
  CHECK(fromSilver.classify(REFLECTIVITY_SILVER_THRESHOLD, true) == VictimClass::Live,
        "exactly at the threshold from silver stays silver, so conductive reads live");
}
void classifierResetForgetsTheSurface() {
  ProbeClassifier classifier;
  classifier.classify(kSilverStrong, true);
  classifier.reset();
  CHECK(classifier.surface() == SurfaceKind::Black, "reset returns the surface to black");
  CHECK(classifier.classify(kSilverStrong, true) == VictimClass::Live,
        "a fresh classifier re-proves silver from scratch");
}

// Victim truth table — anything off the table is a fake

void truthTableClassifiesVictims() {
  ProbeClassifier classifier;
  CHECK(classifier.classify(kSilverStrong, true) == VictimClass::Live,
        "silver and conductive = live");
  CHECK(classifier.classify(kBlackStrong, false) == VictimClass::Dead,
        "black and non-conductive = dead");
  CHECK(classifier.classify(kSilverStrong, false) == VictimClass::Fake,
        "silver and non-conductive = fake");
  CHECK(classifier.classify(kBlackStrong, true) == VictimClass::Fake,
        "black and conductive = fake");
  CHECK(classifier.classify(kJustBelowThreshold, true) == VictimClass::Fake,
        "a sub-threshold reading with conductivity falls to the else branch");
}
void weirdSignaturesAreNeverBanked() {
  ProbeClassifier classifier;
  CHECK(classifier.classify(0, false) == VictimClass::Dead,
        "a fully dark reading with no conductivity is dead");
  CHECK(classifier.classify(1023, false) == VictimClass::Fake,
        "a saturated reading with no conductivity is a fake, not a deposit");
  CHECK(classifier.classify(0, true) == VictimClass::Fake,
        "a fully dark reading with conductivity is a fake");
  CHECK(classifier.classify(1023, true) == VictimClass::Live,
        "a saturated reading with conductivity is silver and live");
}
void repeatedClassificationIsStable() {
  ProbeClassifier classifier;
  for (int sample = 0; sample < 5; ++sample) {
    CHECK(classifier.classify(kSilverStrong, true) == VictimClass::Live,
          "repeated live samples stay live");
    CHECK(classifier.classify(kSilverStrong, false) == VictimClass::Fake,
          "repeated fake samples stay fake");
  }
}

}  // namespace

int main() {
  buttonStartsReleased();
  bounceInsideWindowIsIgnored();
  cleanPressLatchesExactlyOnce();
  releaseDoesNotClearTheLatch();
  customDebounceWindowIsHonoured();
  longIdleCommitsInOneCall();
  millisWraparoundStillCommits();
  resetReturnsToTheIdleState();
  zonesAreIndependent();
  anyZoneReportsTheStruckSide();
  bothZonesCanBeStruckTogether();
  plateReportsDebouncedLevels();
  plateResetDropsPendingStrikes();
  truthTableHasNoFifthOutcome();
  hysteresisBandIsHonouredByBandEdge();
  silverNeedsToClearTheUpperBand();
  silverMustFallBelowTheLowerBand();
  boundaryReadingsResolveByBandMembership();
  classifierResetForgetsTheSurface();
  truthTableClassifiesVictims();
  weirdSignaturesAreNeverBanked();
  repeatedClassificationIsStable();
  return DF_SUMMARIZE("sensors_test");
}
