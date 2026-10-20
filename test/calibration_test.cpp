// calibration_test.cpp — host-side tests for src/rescue_robot/calibration.h.
//
// The sweep is driven with the same synthetic streams the AVR delivery layer
// produces — one sample every CAL_SAMPLE_INTERVAL_MS, black half then white
// half — so the window arithmetic, the per-channel midpoints, the
// contrast-health gate, the retry state machine and both output shapes are all
// exercised on the host. Building this file is also the proof that
// calibration.h carries no AVR identifiers and that its output lands in the
// exact structs line.h's setCalibration() and the store's serialiser take.

#include "calibration.h"
#include "dryframe.h"

#include <cstdint>
#include <string>

namespace {

using namespace rescue;

// QTR-8A analog readings at the locked 11.2 mm ride height: the black tape sits
// above the white floor on every channel and the spread is the contrast the
// health gate judges. The per-channel shift keeps the midpoints distinct.
constexpr int32_t VENUE_BLACK_LEVEL = 520;
constexpr int32_t VENUE_WHITE_LEVEL = 380;
constexpr int32_t VENUE_CONTRAST = VENUE_BLACK_LEVEL - VENUE_WHITE_LEVEL;
constexpr int32_t CHANNEL_LEVEL_OFFSET = 7;
constexpr int32_t BLIND_SPREAD = 10;  // this far below black reads back blind
constexpr int32_t GATE_FLOOR_SPREAD = CAL_MIN_CHANNEL_CONTRAST;

struct VenueLevels {
  int32_t black[CAL_CHANNEL_COUNT];
  int32_t white[CAL_CHANNEL_COUNT];
};

VenueLevels venueLevels() {
  VenueLevels levels{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const int32_t offset = static_cast<int32_t>(channel) * CHANNEL_LEVEL_OFFSET;
    levels.black[channel] = VENUE_BLACK_LEVEL + offset;
    levels.white[channel] = VENUE_WHITE_LEVEL + offset;
  }
  return levels;
}

// Pull a channel's white level down until its spread is `spread`.
void squeezeChannel(VenueLevels& levels, uint8_t channel, int32_t spread) {
  levels.white[channel] = levels.black[channel] - spread;
}

// Blind the leftmost `count` channels, the ones the healthy floor is judged on.
void blindChannels(VenueLevels& levels, uint8_t count) {
  for (uint8_t channel = 0; channel < count; ++channel) {
    squeezeChannel(levels, channel, BLIND_SPREAD);
  }
}

void invertBar(VenueLevels& levels) {
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const int32_t level = levels.black[channel];
    levels.black[channel] = levels.white[channel];
    levels.white[channel] = level;
  }
}

// Drive one attempt exactly as the delivery layer does.
SweepAccumulator runSweep(const VenueLevels& levels) {
  SweepAccumulator accumulator;
  sweepReset(accumulator);
  for (uint32_t at = 0; at < CAL_SWEEP_WINDOW_MS; at += CAL_SAMPLE_INTERVAL_MS) {
    const SweepPhase phase = sweepPhaseAt(at);
    const int32_t* level =
        phase == SweepPhase::Black ? levels.black : levels.white;
    CalRawVector raw{};
    for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
      raw.reading[channel] = level[channel];
    }
    sweepAccumulate(accumulator, phase, raw);
  }
  return accumulator;
}

SweepResult sweepVenue(const VenueLevels& levels) {
  return sweepEvaluate(runSweep(levels));
}

// What the line layer hands a channel that saw this much spread.
int32_t expectedWeight(int32_t contrast) {
  return LINE_WEIGHT_MAX * (contrast - LINE_MIN_CONTRAST) /
         (LINE_FULL_CONTRAST - LINE_MIN_CONTRAST);
}

uint32_t weightTotal(const SweepResult& result) {
  uint32_t total = 0;
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    total += result.channel[channel].weight;
  }
  return total;
}

// Conclude one attempt of a run, as finishCalSweep does on the robot.
CalDecision conclude(CalAttempt& run, const SweepResult& result,
                     CalFallback fallback, bool last_good_stored) {
  calConcludeAttempt(run, result, CalVenuePolicy{fallback, last_good_stored});
  return run.decision;
}

// Spend the whole retry allowance on one sweep and report the terminal call.
CalDecision exhaust(CalAttempt& run, const SweepResult& result,
                    CalFallback fallback, bool last_good_stored) {
  CalDecision decision = CalDecision::Pending;
  for (uint8_t attempt = 0; attempt < CAL_MAX_ATTEMPTS; ++attempt) {
    decision = conclude(run, result, fallback, last_good_stored);
  }
  return decision;
}

// A bar blind in every channel but the two the degraded floor requires.
SweepResult marginalSweepResult() {
  VenueLevels levels = venueLevels();
  blindChannels(levels, CAL_CHANNEL_COUNT - CAL_MIN_DEGRADED_CHANNELS);
  return sweepVenue(levels);
}

void testWindowIsTwoHalvesOnTheClock() {
  // The caller holds the bar still and then moves it, so the split is
  // wall-clock, not a sample count.
  CHECK(sweepPhaseAt(0) == SweepPhase::Black, "window opens on black");
  CHECK(sweepPhaseAt(CAL_SWEEP_HALF_MS - 1) == SweepPhase::Black,
        "black holds for the first half");
  CHECK(sweepPhaseAt(CAL_SWEEP_HALF_MS) == SweepPhase::White,
        "white takes the second half");
  CHECK(!sweepWindowElapsed(CAL_SWEEP_WINDOW_MS - 1), "window still open at 9999 ms");
  CHECK(sweepWindowElapsed(CAL_SWEEP_WINDOW_MS), "window closes at 10 s");
}

void testStreamedSamplesAccumulate() {
  // An untouched accumulator has no evidence in it, and a phase with no samples
  // reports no level rather than dividing by zero.
  const SweepAccumulator empty{};
  CHECK_EQ(empty.complete(), false);
  CHECK_EQ(sweepMeanLevel(empty.black_sum[0], empty.black_count), 0);
  const SweepResult noEvidence = sweepEvaluate(empty);
  CHECK_EQ(noEvidence.healthy_count, 0);
  CHECK(!noEvidence.accepted, "an empty sweep is never accepted");

  const SweepAccumulator swept = runSweep(venueLevels());
  CHECK(swept.complete(), "a full window leaves both phases sampled");
  CHECK_EQ(swept.black_count, swept.white_count);
  CHECK_EQ(swept.black_count, CAL_SWEEP_WINDOW_MS / CAL_SAMPLE_INTERVAL_MS / 2);

  SweepAccumulator accumulator;
  sweepReset(accumulator);
  CalRawVector raw{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    raw.reading[channel] = VENUE_BLACK_LEVEL;
  }
  sweepAccumulate(accumulator, SweepPhase::Black, raw);
  CHECK_EQ(accumulator.black_count, 1);
  CHECK_EQ(accumulator.white_count, 0);
  CHECK_EQ(accumulator.black_sum[0], static_cast<uint32_t>(VENUE_BLACK_LEVEL));
  sweepReset(accumulator);
  CHECK_EQ(accumulator.black_count, 0);
  CHECK_EQ(accumulator.black_sum[0], static_cast<uint32_t>(0));

  // A reading from outside the converter's span is clamped, not trusted.
  CalRawVector wild{};
  wild.reading[0] = CAL_ANALOG_MAX + 400;
  wild.reading[1] = CAL_ANALOG_MIN - 400;
  sweepAccumulate(accumulator, SweepPhase::White, wild);
  CHECK_EQ(accumulator.white_sum[0], static_cast<uint32_t>(CAL_ANALOG_MAX));
  CHECK_EQ(accumulator.white_sum[1], static_cast<uint32_t>(CAL_ANALOG_MIN));
}

void testEachChannelKeepsItsOwnThreshold() {
  // The midpoint is the mean of the channel's own pair, so a channel sitting
  // differently on the bar keeps its own threshold.
  const VenueLevels levels = venueLevels();
  const SweepResult result = sweepVenue(levels);
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    CHECK_EQ(result.channel[channel].black_level,
             static_cast<uint16_t>(levels.black[channel]));
    CHECK_EQ(result.channel[channel].white_level,
             static_cast<uint16_t>(levels.white[channel]));
    CHECK_EQ(result.channel[channel].midpoint(),
             (levels.black[channel] + levels.white[channel]) / 2);
    CHECK_EQ(result.channel[channel].contrast(), VENUE_CONTRAST);
  }
  CHECK_EQ(result.healthy_count, CAL_CHANNEL_COUNT);
  CHECK(result.accepted, "a full-contrast venue sweep clears the gate");
  CHECK_EQ(result.weak_channel_mask, 0);
  CHECK(result.contrast == ContrastVerdict::Healthy, "verdict recorded as healthy");
}

void testContrastIsPolarityBlind() {
  // The sweep takes the levels as measured and leaves direction to the line
  // layer, so an inverted bar still calibrates.
  VenueLevels levels = venueLevels();
  invertBar(levels);
  const SweepResult result = sweepVenue(levels);
  CHECK_EQ(result.channel[0].contrast(), VENUE_CONTRAST);
  CHECK(result.channel[0].white_level > result.channel[0].black_level,
        "inverted channel keeps its measured polarity");
  CHECK(result.accepted, "an inverted bar still calibrates");
}

void testWeakChannelIsRejectedAndWeightRedistributed() {
  VenueLevels levels = venueLevels();
  squeezeChannel(levels, 3, BLIND_SPREAD);
  const SweepResult result = sweepVenue(levels);
  CHECK(!result.channel[3].healthy, "the blind channel fails the gate");
  CHECK_EQ(result.weak_channel_mask, 1u << 3);
  CHECK_EQ(result.channel[3].weight, CAL_REJECTED_CHANNEL_WEIGHT);
  CHECK_EQ(result.healthy_count, CAL_CHANNEL_COUNT - 1);
  CHECK(result.accepted, "one blind channel still clears the gate");
  CHECK(result.contrast == ContrastVerdict::Weak, "verdict records the weak bar");
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    if (channel == 3) continue;
    CHECK(result.channel[channel].weight > 0, "healthy channel carries weight");
  }
  CHECK_EQ(weightTotal(result), CAL_WEIGHT_TOTAL);
}

void testGateFloorIsExactlyTwoSided() {
  // On the contrast floor is trusted, one count under is not, and there is no
  // separate untrusted-but-kept band.
  VenueLevels levels = venueLevels();
  squeezeChannel(levels, 0, GATE_FLOOR_SPREAD);
  squeezeChannel(levels, 1, GATE_FLOOR_SPREAD - 1);
  const SweepResult result = sweepVenue(levels);
  CHECK(result.channel[0].healthy, "a channel on the contrast floor is healthy");
  CHECK(!result.channel[1].healthy, "one count under the floor is not");
  CHECK_EQ(result.weak_channel_mask, 1u << 1);
}

void testHealthyChannelFloorIsACount() {
  // The gate is a count, not just a per-channel test: too few channels with
  // usable contrast and the bar cannot place a line at all.
  VenueLevels exact = venueLevels();
  blindChannels(exact, CAL_CHANNEL_COUNT - CAL_MIN_HEALTHY_CHANNELS);
  const SweepResult onFloor = sweepVenue(exact);
  CHECK_EQ(onFloor.healthy_count, CAL_MIN_HEALTHY_CHANNELS);
  CHECK(onFloor.accepted, "exactly the minimum healthy count is accepted");

  VenueLevels shortBar = venueLevels();
  blindChannels(shortBar, CAL_CHANNEL_COUNT - CAL_MIN_HEALTHY_CHANNELS + 1);
  const SweepResult underFloor = sweepVenue(shortBar);
  CHECK_EQ(underFloor.healthy_count, CAL_MIN_HEALTHY_CHANNELS - 1);
  CHECK(!underFloor.accepted, "one short of the minimum is refused");
}

void testAllWeakBarCarriesNoWeight() {
  VenueLevels levels = venueLevels();
  blindChannels(levels, CAL_CHANNEL_COUNT);
  const SweepResult result = sweepVenue(levels);
  CHECK_EQ(result.healthy_count, 0);
  CHECK_EQ(result.weak_channel_mask, 0xFF);
  CHECK(!result.accepted, "an all-blind bar is refused");
  CHECK_EQ(weightTotal(result), static_cast<uint32_t>(0));
}

void testRefusedSweepOwesARetryAndAGoodOnePersists() {
  VenueLevels bad = venueLevels();
  blindChannels(bad, CAL_CHANNEL_COUNT);
  const SweepResult refused = sweepVenue(bad);

  CalAttempt run;
  calRunBegin(run);
  CHECK_EQ(run.attempt, 1);
  CHECK(run.decision == CalDecision::Pending, "a run starts pending");
  CHECK(!calRetryOwed(run), "no retry is owed before the first sweep");

  CHECK(conclude(run, refused, CalFallback::LastGoodThenDegrade, true) ==
            CalDecision::Pending,
        "a refused sweep stays pending");
  CHECK(calRetryOwed(run), "the first refusal owes a retry");
  CHECK_EQ(run.attempt, 2);

  const SweepResult accepted = sweepVenue(venueLevels());
  CHECK(conclude(run, accepted, CalFallback::LastGoodThenDegrade, true) ==
            CalDecision::Persisted,
        "the good retry is persisted");
  CHECK(!calRetryOwed(run), "an accepted sweep owes nothing");
  CHECK_EQ(run.attempt, 2);
  CHECK_EQ(run.healthy_count, CAL_CHANNEL_COUNT);
  CHECK_EQ(run.weak_channel_mask, 0);
  CHECK(calDecisionAcceptsRun(run.decision), "persisted runs may start");
}

void testRetriesSpentPrefersStoredBasis() {
  // Three bad sweeps and the retries are spent. A stored venue basis is then
  // preferred over a run on the bad sweep, which is the declared carve-out.
  VenueLevels bad = venueLevels();
  blindChannels(bad, CAL_CHANNEL_COUNT);
  const SweepResult refused = sweepVenue(bad);
  CalAttempt run;
  calRunBegin(run);
  const CalDecision decision =
      exhaust(run, refused, CalFallback::LastGoodThenDegrade, true);
  CHECK(decision == CalDecision::LastGoodPersisted,
        "the stored basis is preferred once the retries are spent");
  CHECK(!calRetryOwed(run), "the retry allowance is exactly two");
  CHECK_EQ(run.attempt, CAL_MAX_ATTEMPTS);
  CHECK(calDecisionAcceptsRun(decision), "a degraded run may still start");
}

void testRetriesSpentWithNothingStored() {
  // With no stored basis, a sweep that still saw enough channels to say
  // something runs degraded; one that saw fewer refuses to start.
  const SweepResult marginal = marginalSweepResult();
  CHECK_EQ(marginal.healthy_count, CAL_MIN_DEGRADED_CHANNELS);
  CalAttempt run;
  calRunBegin(run);
  CHECK(exhaust(run, marginal, CalFallback::LastGoodThenDegrade, false) ==
            CalDecision::Degraded,
        "a still-informative sweep degrades instead of aborting");

  VenueLevels hopeless = venueLevels();
  blindChannels(hopeless, CAL_CHANNEL_COUNT);
  const SweepResult blind = sweepVenue(hopeless);
  calRunBegin(run);
  const CalDecision decision =
      exhaust(run, blind, CalFallback::LastGoodThenDegrade, false);
  CHECK(decision == CalDecision::Abort, "no basis at all aborts the run");
  CHECK(!calDecisionAcceptsRun(decision), "an abort never lets the run start");
}

void testVenueMayForbidTheDegradePath() {
  const SweepResult marginal = marginalSweepResult();
  CalAttempt run;
  calRunBegin(run);
  CHECK(exhaust(run, marginal, CalFallback::AbortRun, true) == CalDecision::Abort,
        "a stored basis is not used when the venue forbids degrading");
}

void testLineLayerConsumesTheSweep() {
  // A black reading on a healthy channel reads as line, a white reading as
  // floor, and the centroid follows a line under the centred pair of the bar.
  const VenueLevels levels = venueLevels();
  setCalibration(lineCalibrationFrom(sweepVenue(levels)));
  CHECK_EQ(rescue::channelWeight(0, levels.black[0]),
           expectedWeight(VENUE_CONTRAST));
  CHECK_EQ(rescue::channelWeight(0, levels.white[0]), LINE_WEIGHT_MIN);
  CHECK_EQ(rescue::channelWeight(0, levels.white[0] - 50), LINE_WEIGHT_MIN);

  LineVector readings{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const bool on_line = channel == 3 || channel == 4;
    readings.reading[channel] =
        on_line ? levels.black[channel] : levels.white[channel];
  }
  const LineReading reading = analyze(readings);
  CHECK(reading.lineDetected, "the calibrated bar sees its line");
  CHECK_EQ(reading.position, LINE_POSITION_CENTER);
  CHECK_EQ(reading.error, 0);
  CHECK(reading.qualifier == LineClass::ON_LINE, "a centred pair is on-line");
}

void testInvertedChannelCalibratesItselfAndRejectedOneContributesNothing() {
  // Same spread as the upright bar, so the same trust: polarity is the line
  // layer's business, not the sweep's.
  VenueLevels inverted = venueLevels();
  invertBar(inverted);
  setCalibration(lineCalibrationFrom(sweepVenue(inverted)));
  CHECK_EQ(rescue::channelWeight(2, inverted.black[2]),
           expectedWeight(VENUE_CONTRAST));
  CHECK_EQ(rescue::channelWeight(2, inverted.white[2]), LINE_WEIGHT_MIN);

  VenueLevels levels = venueLevels();
  squeezeChannel(levels, 5, BLIND_SPREAD);
  setCalibration(lineCalibrationFrom(sweepVenue(levels)));
  CHECK_EQ(rescue::channelWeight(5, levels.black[5]), LINE_WEIGHT_MIN);
  CHECK(rescue::channelWeight(6, levels.black[6]) > LINE_PRESENT_THRESHOLD,
        "the healthy channels still carry the line");
}

void testStoreRoundTripsTheSweep() {
  // What the sweep hands the store must survive the store's own serialiser,
  // stamp included, with the weak mask and verdict still readable afterwards.
  VenueLevels levels = venueLevels();
  squeezeChannel(levels, 2, BLIND_SPREAD);
  const SweepResult result = sweepVenue(levels);

  uint8_t slot[STORE_CALIBRATION_SLOT_BYTES] = {0};
  const uint8_t revision = 7;
  const uint16_t written =
      writeCalibrationSlot(slot, sizeof(slot), storedRecordFrom(result), revision);
  CHECK_EQ(written, STORE_CALIBRATION_SLOT_BYTES);

  CalibrationRecord loaded;
  const LoadResult outcome = loadCalibration(slot, written, loaded);
  CHECK(outcome.outcome == LoadOutcome::Loaded, "the slot loads back");
  CHECK_EQ(outcome.revision, revision);
  CHECK(loaded.contrast == ContrastVerdict::Weak, "verdict survives the store");
  CHECK_EQ(loaded.weak_channel_mask, 1u << 2);
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    CHECK_EQ(loaded.channels[channel].black_level,
             result.channel[channel].black_level);
    CHECK_EQ(loaded.channels[channel].white_level,
             result.channel[channel].white_level);
    CHECK_EQ(loaded.channels[channel].weight, result.channel[channel].weight);
    // The store's own midpoint agrees with the sweep's.
    CHECK_EQ(channelMidpoint(loaded.channels[channel]),
             result.channel[channel].midpoint());
  }

  // The green baseline belongs to the marker layer, so the sweep leaves the
  // store's defaults rather than stamping zeros over them.
  for (uint8_t sensor = 0; sensor < GREEN_BASELINE_SENSOR_COUNT; ++sensor) {
    CHECK_EQ(loaded.green_baseline[sensor].black_level, DEFAULT_BLACK_LEVEL);
    CHECK_EQ(loaded.green_baseline[sensor].white_level, DEFAULT_WHITE_LEVEL);
  }

  // A disturbed payload is refused, so a half-written sweep can never be
  // mistaken for a venue basis.
  slot[ENVELOPE_HEADER_BYTES] =
      static_cast<uint8_t>(slot[ENVELOPE_HEADER_BYTES] + 1);
  CalibrationRecord damaged;
  CHECK(loadCalibration(slot, written, damaged).outcome ==
            LoadOutcome::ChecksumMismatch,
        "a disturbed slot is refused");
}

void testStoredBasisReinstallsIntoTheLineLayer() {
  // The degrade path loads the record out of the store and hands the line layer
  // the same shape the sweep produces.
  const VenueLevels levels = venueLevels();
  uint8_t slot[STORE_CALIBRATION_SLOT_BYTES] = {0};
  const uint16_t written =
      writeCalibrationSlot(slot, sizeof(slot), storedRecordFrom(sweepVenue(levels)), 1);
  CalibrationRecord loaded;
  CHECK(loadCalibration(slot, written, loaded).outcome == LoadOutcome::Loaded,
        "the venue basis loads back");
  setCalibration(lineCalibrationFrom(loaded));
  CHECK_EQ(rescue::channelWeight(4, levels.black[4]),
           expectedWeight(VENUE_CONTRAST));
  CHECK_EQ(rescue::channelWeight(4, levels.white[4]), LINE_WEIGHT_MIN);
}

void testDecisionsNameThemselves() {
  CHECK(std::string(calDecisionName(CalDecision::Persisted)) == "persisted",
        "an accepted sweep logs as persisted");
  CHECK(std::string(calDecisionName(CalDecision::LastGoodPersisted)) ==
            "last-good-persisted",
        "the carve-out says so in the log");
  CHECK(std::string(calDecisionName(CalDecision::Degraded)) == "degraded",
        "a degraded run says so in the log");
  CHECK(std::string(calDecisionName(CalDecision::Abort)) == "abort",
        "an abort says so in the log");
  CHECK(!calDecisionAcceptsRun(CalDecision::Pending), "a pending run may not start");
}

}  // namespace

int main() {
  testWindowIsTwoHalvesOnTheClock();
  testStreamedSamplesAccumulate();
  testEachChannelKeepsItsOwnThreshold();
  testContrastIsPolarityBlind();
  testWeakChannelIsRejectedAndWeightRedistributed();
  testGateFloorIsExactlyTwoSided();
  testHealthyChannelFloorIsACount();
  testAllWeakBarCarriesNoWeight();
  testRefusedSweepOwesARetryAndAGoodOnePersists();
  testRetriesSpentPrefersStoredBasis();
  testRetriesSpentWithNothingStored();
  testVenueMayForbidTheDegradePath();
  testLineLayerConsumesTheSweep();
  testInvertedChannelCalibratesItselfAndRejectedOneContributesNothing();
  testStoreRoundTripsTheSweep();
  testStoredBasisReinstallsIntoTheLineLayer();
  testDecisionsNameThemselves();
  return DF_SUMMARIZE("calibration_test");
}
