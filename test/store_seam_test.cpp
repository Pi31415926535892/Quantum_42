// store_seam_test.cpp — host-side proof that the two per-channel calibration
// types can share one translation unit with no rename seam.
//
// The line module's {black, white} and the store's {black_level, white_level,
// weight} are genuinely different shapes, and for a while both were called
// rescue::ChannelCalibration. One header papered over it with a
// `#define ChannelCalibration StoredChannelCalibration` around the store's
// include, which meant the two headers could only be used together if the
// reader happened to include the right one first. This suite is the alarm for
// exactly that: the includes below are deliberately hostile (mission first,
// store second, sweep third), the store's type is used under its own name, and
// the sweep's ledger travels as the store's own record instead of a private
// copy with the same four facts.
//
// It also reads calibration.h as text and fails if a macro seam ever comes
// back, because a seam that compiles is indistinguishable from a seam that does
// not until a reader gets the order wrong.
//
// Run: ./test/run_tests.sh

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "dryframe.h"

// Hostile order on purpose: the mission header drags in the line module, the
// store comes after it, and the sweep consumes both. Nothing below may depend
// on which one a reader includes first.
#include "mission.h"
#include "eeprom_store.h"
#include "calibration.h"

using namespace rescue;

namespace {

// Two overloads that differ only by the per-channel type they accept. Calling
// each with its own type compiles only while the two types are DISTINCT; were
// the rename undone, both calls would be ambiguous and this file would not
// build. That is the One Definition Rule proof, at compile time.
void acceptsStored(StoredChannelCalibration& channel) {
  channel.weight = channel.weight;
}
void acceptsLine(ChannelCalibration& channel) { channel.black = channel.black; }

// dryframe formats through operator<<, which the fixed-width integers do not
// have on this platform; every assertion compares as int.
int asInt(long value) { return static_cast<int>(value); }

std::string readFile(const char* path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

// A sweep that saw the bar over black for the first half and over the white
// floor for the second, on every channel with a healthy spread.
SweepResult healthySweep() {
  SweepAccumulator accumulator;
  sweepReset(accumulator);
  CalRawVector raw{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    raw.reading[channel] = 120;
  }
  for (uint16_t sample = 0; sample < 10; ++sample) {
    sweepAccumulate(accumulator, SweepPhase::Black, raw);
  }
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    raw.reading[channel] = 940;
  }
  for (uint16_t sample = 0; sample < 10; ++sample) {
    sweepAccumulate(accumulator, SweepPhase::White, raw);
  }
  return sweepEvaluate(accumulator);
}

void testStoreChannelKeepsItsOwnName() {
  StoredChannelCalibration channel;
  channel.black_level = 200;
  channel.white_level = 900;
  channel.weight = 42;
  acceptsStored(channel);
  // The persisted midpoint is the black/white average, and the weight rides
  // along with it — this is the shape the line module does NOT have.
  CHECK_EQ(asInt(channelMidpoint(channel)), 550);
  CHECK_EQ(asInt(channel.weight), 42);

  LineCalibration line = defaultCalibration(200, 900);
  ChannelCalibration& lineChannel = line.channel[0];
  acceptsLine(lineChannel);
  CHECK_EQ(asInt(lineChannel.black), 200);
  CHECK_EQ(asInt(lineChannel.white), 900);
}

void testSweepFeedsBothShapesWithoutTranslation() {
  const SweepResult result = healthySweep();
  const CalibrationRecord record = storedRecordFrom(result);
  const LineCalibration line = lineCalibrationFrom(record);

  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const StoredChannelCalibration& stored = record.channels[channel];
    CHECK_EQ(asInt(stored.black_level), asInt(result.channel[channel].black_level));
    CHECK_EQ(asInt(stored.white_level), asInt(result.channel[channel].white_level));
    CHECK_EQ(asInt(stored.weight), asInt(result.channel[channel].weight));
    // The two shapes agree on the readings, which is what lets a stored basis
    // be re-derived into a steerable one without a hand-written translation.
    CHECK_EQ(asInt(line.channel[channel].black), asInt(stored.black_level));
    CHECK_EQ(asInt(line.channel[channel].white), asInt(stored.white_level));
  }
  CHECK(record.contrast == ContrastVerdict::Healthy, "a fully healthy sweep stores a healthy verdict");
  CHECK_EQ(asInt(record.weak_channel_mask), 0);

  // The reverse direction: a stored basis re-reads as the same thresholds.
  const LineCalibration fromRecord = lineCalibrationFrom(record);
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    CHECK_EQ(asInt(fromRecord.channel[channel].black),
             asInt(record.channels[channel].black_level));
  }
}

void testRestartStagesTheStoreRecordItself() {
  beginMission();

  AccountingRecord stored = freshAccounting();
  stored.victims_counted = 3;
  stored.deposits_placed = 2;
  stored.landmark_counter = 5;
  stored.loop_pending = true;

  // The store's own record, handed over whole. No private copy with the same
  // four facts, so nothing can disagree about what was persisted.
  beginLopRestage(stored);
  CHECK(lopResumeRequired(), "a cut-short record arms the restart");
  CHECK_EQ(asInt(lopExpectedLandmarks()), 5);

  // A bar over bare floor proves no landmark at all, so the count the restart
  // started from stands: the record is the baseline, not a number to overwrite.
  MissionInputs bare{};
  bare.line.lineDetected = false;
  bare.line.qualifier = LineClass::ALL_WHITE;
  for (uint8_t pass = 0; pass < 3; ++pass) missionLopTrackLandmark(bare);
  CHECK_EQ(asInt(lopReDerivedLandmarks()), 5);
  CHECK_EQ(asInt(lopLandmarkError()), 0);

  const AccountingRecord published = lopReDerivedAccounting();
  CHECK_EQ(asInt(published.victims_counted), 3);
  CHECK_EQ(asInt(published.deposits_placed), 2);
  // Never narrower than the record it came from: the landmarks it had counted
  // survive a restart that proves none.
  CHECK_EQ(asInt(published.landmark_counter), 5);
  // The published ledger is never pending — the restart is over.
  CHECK(!published.loop_pending, "a published ledger is never pending");
}

void testNoMacroSeamRemains() {
  const std::string header = readFile("src/rescue_robot/calibration.h");
  CHECK(!header.empty(), "calibration.h was not readable at the repo root");
  CHECK(header.find("#define ChannelCalibration") == std::string::npos,
        "calibration.h renames the store struct through a macro again");
  CHECK(header.find("#undef ChannelCalibration") == std::string::npos,
        "calibration.h undoes a macro rename again");
  // Both headers are included plainly, in the order they are declared.
  const size_t store = header.find("#include \"eeprom_store.h\"");
  const size_t line = header.find("#include \"line.h\"");
  CHECK(store != std::string::npos, "calibration.h no longer includes the store");
  CHECK(line != std::string::npos, "calibration.h no longer includes the line module");
}

}  // namespace

int main() {
  testStoreChannelKeepsItsOwnName();
  testSweepFeedsBothShapesWithoutTranslation();
  testRestartStagesTheStoreRecordItself();
  testNoMacroSeamRemains();
  return DF_SUMMARIZE("store_seam");
}