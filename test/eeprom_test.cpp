// eeprom_test.cpp — host-side tests for src/rescue_robot/eeprom_store.h.
//
// Build/run: g++ -std=c++14 -Itest -Isrc/rescue_robot test/eeprom_test.cpp \
//              -o /tmp/etest && /tmp/etest

#include "eeprom_store.h"

#include "dryframe.h"

using namespace rescue;

namespace {

// Local aliases: the public slot-size names are long enough that spelling them
// out at every call site would wrap most lines for no extra meaning.
constexpr uint16_t CAL_SLOT_BYTES = STORE_CALIBRATION_SLOT_BYTES;
constexpr uint16_t ACC_SLOT_BYTES = STORE_ACCOUNTING_SLOT_BYTES;
constexpr uint8_t TEST_WRITES_PER_RUN = 12;

// The per-run allowance is delivery policy, supplied to the store as data so
// the header stays free of AVR constants.
WriteBudget budget(uint8_t writes_this_run, uint8_t wear_revision) {
  WriteBudget ledger = {writes_this_run, wear_revision, TEST_WRITES_PER_RUN};
  return ledger;
}

template <typename T>
int asInt(const T& value) {
  return static_cast<int>(value);
}

bool bytesEqual(const uint8_t* actual, const uint8_t* expected, uint16_t count) {
  for (uint16_t index = 0; index < count; ++index) {
    if (actual[index] != expected[index]) return false;
  }
  return true;
}

void fillChannel(StoredChannelCalibration& channel, uint16_t black_level,
                 uint16_t white_level, uint16_t weight) {
  channel.black_level = black_level;
  channel.white_level = white_level;
  channel.weight = weight;
}

CalibrationRecord sampleCalibration() {
  CalibrationRecord record = freshCalibration();
  for (uint8_t index = 0; index < QTR_CHANNEL_COUNT; ++index) {
    fillChannel(record.channels[index], static_cast<uint16_t>(100 + index * 10),
                static_cast<uint16_t>(900 + index * 10),
                static_cast<uint16_t>(index + 1));
  }
  for (uint8_t sensor = 0; sensor < GREEN_BASELINE_SENSOR_COUNT; ++sensor) {
    fillChannel(record.green_baseline[sensor],
                static_cast<uint16_t>(200 + sensor),
                static_cast<uint16_t>(800 + sensor),
                static_cast<uint16_t>(sensor + 1));
  }
  record.contrast = ContrastVerdict::Healthy;
  record.weak_channel_mask = 0x0024u;
  return record;
}

AccountingRecord sampleAccounting() {
  AccountingRecord record;
  record.victims_counted = 3;
  record.deposits_placed = 2;
  record.loop_pending = true;
  record.landmark_counter = 17;
  return record;
}

void checkChannelsEqual(const StoredChannelCalibration* actual,
                        const StoredChannelCalibration* expected,
                        uint8_t count) {
  for (uint8_t index = 0; index < count; ++index) {
    CHECK_EQ(asInt(actual[index].black_level),
             asInt(expected[index].black_level));
    CHECK_EQ(asInt(actual[index].white_level),
             asInt(expected[index].white_level));
    CHECK_EQ(asInt(actual[index].weight), asInt(expected[index].weight));
  }
}

void checkCalibrationEqual(const CalibrationRecord& actual,
                           const CalibrationRecord& expected) {
  checkChannelsEqual(actual.channels, expected.channels, QTR_CHANNEL_COUNT);
  checkChannelsEqual(actual.green_baseline, expected.green_baseline,
                     GREEN_BASELINE_SENSOR_COUNT);
  CHECK(actual.contrast == expected.contrast, "contrast verdict round-trips");
  CHECK_EQ(asInt(actual.weak_channel_mask), asInt(expected.weak_channel_mask));
}

void checkAccountingEqual(const AccountingRecord& actual,
                          const AccountingRecord& expected) {
  CHECK_EQ(asInt(actual.victims_counted), asInt(expected.victims_counted));
  CHECK_EQ(asInt(actual.deposits_placed), asInt(expected.deposits_placed));
  CHECK(actual.loop_pending == expected.loop_pending, "loop-pending round-trips");
  CHECK_EQ(asInt(actual.landmark_counter), asInt(expected.landmark_counter));
}

void checkCalibrationIsFresh(const CalibrationRecord& record) {
  checkCalibrationEqual(record, freshCalibration());
  CHECK(record.contrast == ContrastVerdict::Unknown,
        "fresh calibration reports an unknown contrast verdict");
}

void checkAccountingIsFresh(const AccountingRecord& record) {
  checkAccountingEqual(record, freshAccounting());
}

// --- cycle 1: round trips ---------------------------------------------------

void testCrc8MatchesLinkProtocolVectors() {
  const uint8_t heartbeat[] = {0x02, 0x00};
  CHECK_EQ(asInt(storeCrc8(heartbeat, 2)), 0x2A);
  const uint8_t run_start[] = {0x04, 0x11, 0x00, 0x03};
  CHECK_EQ(asInt(storeCrc8(run_start, 4)), 0x98);
}

void testCalibrationSlotRoundTrip() {
  const CalibrationRecord written = sampleCalibration();
  uint8_t slot[CAL_SLOT_BYTES] = {0};
  const uint16_t bytes = writeCalibrationSlot(slot, CAL_SLOT_BYTES, written, 7);
  CHECK_EQ(asInt(bytes), asInt(CAL_SLOT_BYTES));
  CalibrationRecord read = freshCalibration();
  const LoadResult result = loadCalibration(slot, CAL_SLOT_BYTES, read);
  CHECK(result.outcome == LoadOutcome::Loaded, "calibration slot loads");
  CHECK_EQ(asInt(result.revision), 7);
  checkCalibrationEqual(read, written);
}

void testAccountingSlotRoundTrip() {
  const AccountingRecord written = sampleAccounting();
  uint8_t slot[ACC_SLOT_BYTES] = {0};
  const uint16_t bytes = writeAccountingSlot(slot, ACC_SLOT_BYTES, written, 12);
  CHECK_EQ(asInt(bytes), asInt(ACC_SLOT_BYTES));
  AccountingRecord read = freshAccounting();
  const LoadResult result = loadAccounting(slot, ACC_SLOT_BYTES, read);
  CHECK(result.outcome == LoadOutcome::Loaded, "accounting slot loads");
  CHECK_EQ(asInt(result.revision), 12);
  checkAccountingEqual(read, written);
}

void testChannelMidpointIsBlackWhiteAverage() {
  StoredChannelCalibration channel;
  fillChannel(channel, 200, 901, 3);
  CHECK_EQ(asInt(channelMidpoint(channel)), 550);
}

// --- cycle 2: integrity, as a table of every way a slot can be damaged ------

enum class Damage : uint8_t {
  PayloadFlip,      // a payload byte changes and the stamp is left alone
  StampFlip,        // the stamp itself changes
  PayloadResealed,  // a payload byte changes and the stamp is recomputed
  ForeignVersion,   // the envelope claims a version this build cannot read
  WrongKind,        // the envelope claims the other record kind
  ShortLength,      // the envelope disagrees with the record shape
  ShortBuffer,      // fewer bytes are supplied than the record shape needs
  ErasedFill,       // 0xFF throughout, as a blank device reports
};

struct DamageCase {
  const char* name;
  Damage damage;
  LoadOutcome expected;
  bool fresh_fallback;  // a refused slot must leave fresh state behind
};

const DamageCase DAMAGE_CASES[] = {
    {"a flipped payload byte is caught by the stamp", Damage::PayloadFlip,
     LoadOutcome::ChecksumMismatch, true},
    {"a flipped stamp is caught", Damage::StampFlip,
     LoadOutcome::ChecksumMismatch, true},
    {"a resealed payload loads again, so the stamp covers the whole prefix",
     Damage::PayloadResealed, LoadOutcome::Loaded, false},
    {"a foreign envelope version is refused on the version gate alone",
     Damage::ForeignVersion, LoadOutcome::VersionMismatch, true},
    {"a slot of the wrong kind is refused", Damage::WrongKind,
     LoadOutcome::KindMismatch, true},
    {"a length that disagrees with the record shape is refused",
     Damage::ShortLength, LoadOutcome::LengthMismatch, true},
    {"a short buffer is refused", Damage::ShortBuffer,
     LoadOutcome::SlotTruncated, true},
    {"an erased slot is never loaded", Damage::ErasedFill,
     LoadOutcome::VersionMismatch, true},
};

// Damage a slot in place, then report how many bytes the loader is handed.
uint16_t applyDamage(uint8_t* slot, uint16_t slot_bytes, uint16_t stamped_bytes,
                     Damage damage) {
  const uint16_t other_kind =
      slot[ENVELOPE_KIND_OFFSET] ==
              static_cast<uint8_t>(RecordKind::Calibration)
          ? static_cast<uint8_t>(RecordKind::Accounting)
          : static_cast<uint8_t>(RecordKind::Calibration);
  switch (damage) {
    case Damage::PayloadFlip:
      slot[ENVELOPE_HEADER_BYTES + 2] ^= 0x01;
      break;
    case Damage::StampFlip:
      slot[stamped_bytes] ^= 0xFF;
      break;
    case Damage::PayloadResealed:
      slot[ENVELOPE_HEADER_BYTES + 2] ^= 0x01;
      slot[stamped_bytes] = storeCrc8(slot, stamped_bytes);
      break;
    case Damage::ForeignVersion:
      slot[ENVELOPE_VERSION_OFFSET] = static_cast<uint8_t>(STORE_VERSION + 1);
      slot[stamped_bytes] = storeCrc8(slot, stamped_bytes);
      break;
    case Damage::WrongKind:
      slot[ENVELOPE_KIND_OFFSET] = static_cast<uint8_t>(other_kind);
      slot[stamped_bytes] = storeCrc8(slot, stamped_bytes);
      break;
    case Damage::ShortLength:
      slot[ENVELOPE_LENGTH_OFFSET] = static_cast<uint8_t>(
          (stamped_bytes - ENVELOPE_HEADER_BYTES - 1) & 0xFF);
      slot[stamped_bytes] = storeCrc8(slot, stamped_bytes);
      break;
    case Damage::ShortBuffer:
      return slot_bytes - 1;
    case Damage::ErasedFill:
      for (uint16_t index = 0; index < slot_bytes; ++index) {
        slot[index] = 0xFF;
      }
      break;
  }
  return slot_bytes;
}

void testCalibrationRefusesEveryKindOfDamage() {
  for (const DamageCase& test_case : DAMAGE_CASES) {
    uint8_t slot[CAL_SLOT_BYTES];
    writeCalibrationSlot(slot, CAL_SLOT_BYTES, sampleCalibration(), 4);
    const uint16_t bytes =
        applyDamage(slot, CAL_SLOT_BYTES, CALIBRATION_STAMPED_BYTES,
                    test_case.damage);
    CalibrationRecord read = sampleCalibration();
    const LoadResult result = loadCalibration(slot, bytes, read);
    CHECK(result.outcome == test_case.expected, test_case.name);
    if (test_case.fresh_fallback) checkCalibrationIsFresh(read);
  }
}

void testAccountingRefusesEveryKindOfDamage() {
  for (const DamageCase& test_case : DAMAGE_CASES) {
    uint8_t slot[ACC_SLOT_BYTES];
    writeAccountingSlot(slot, ACC_SLOT_BYTES, sampleAccounting(), 4);
    const uint16_t bytes = applyDamage(slot, ACC_SLOT_BYTES,
                                      ACCOUNTING_STAMPED_BYTES, test_case.damage);
    AccountingRecord read = sampleAccounting();
    const LoadResult result = loadAccounting(slot, bytes, read);
    CHECK(result.outcome == test_case.expected, test_case.name);
    if (test_case.fresh_fallback) checkAccountingIsFresh(read);
  }
}

// --- cycle 3: wear budget ---------------------------------------------------

void testVirginStoreIsWithinBudget() {
  CHECK(!storeExhausted(0), "a never-written device is within budget");
  CHECK(!storeExhausted(STORE_REVISION_LIMIT - 1),
        "the last admissible write is still within budget");
  CHECK(storeExhausted(STORE_REVISION_LIMIT), "the limit itself is exhausted");
}

void testRevisionAdvancesOncePerWrite() {
  uint8_t revision = 0;
  CHECK(advanceRevision(revision), "the first write is admitted");
  CHECK_EQ(asInt(revision), 1);
  CHECK(advanceRevision(revision), "the second write is admitted");
  CHECK_EQ(asInt(revision), 2);
  CHECK_EQ(asInt(storeExhausted(revision)), 0);
}

void testRevisionRidesEveryPersistedRecord() {
  uint8_t revision = 0;
  uint8_t slot[ACC_SLOT_BYTES];
  const AccountingRecord record = sampleAccounting();
  for (uint8_t write = 0; write < 5; ++write) {
    CHECK(advanceRevision(revision), "the budget admits five chained writes");
    writeAccountingSlot(slot, ACC_SLOT_BYTES, record, revision);
    AccountingRecord read = freshAccounting();
    const LoadResult result = loadAccounting(slot, ACC_SLOT_BYTES, read);
    CHECK(result.outcome == LoadOutcome::Loaded, "the chained slot loads");
    CHECK_EQ(asInt(result.revision), asInt(write + 1));
  }
}

void testWearBudgetTripsAfterTheRevisionLimit() {
  uint8_t revision = 0;
  for (uint16_t write = 0; write < STORE_REVISION_LIMIT; ++write) {
    advanceRevision(revision);
  }
  CHECK_EQ(asInt(revision), asInt(STORE_REVISION_LIMIT));
  CHECK(storeExhausted(revision), "the store reports itself exhausted at the limit");
  const uint8_t held_at_limit = revision;
  CHECK(!advanceRevision(revision), "a write past the limit is refused");
  CHECK_EQ(asInt(revision), asInt(held_at_limit));
}

// --- cycle 4: write admission, as a table over both write paths -------------

struct AdmissionCase {
  const char* name;
  uint8_t writes_this_run;
  uint8_t wear_revision;
  bool release;  // the all-or-nothing release path rather than a single write
  WriteAdmission expected;
};

const AdmissionCase ADMISSION_CASES[] = {
    {"a virgin budget admits the first write", 0, 0, false,
     WriteAdmission::Admitted},
    {"the last admissible write is still admitted", TEST_WRITES_PER_RUN - 1,
     STORE_REVISION_LIMIT - 1, false, WriteAdmission::Admitted},
    {"a spent run allowance refuses a write", TEST_WRITES_PER_RUN, 3, false,
     WriteAdmission::RunBudgetExhausted},
    {"the lifetime cap refuses a write", 0, STORE_REVISION_LIMIT, false,
     WriteAdmission::RevisionLimitReached},
    {"a release with one write per slot left is admitted",
     TEST_WRITES_PER_RUN - RELEASE_SLOT_COUNT, 0, true, WriteAdmission::Admitted},
    {"half a slot pair must not commit, so a release is all or nothing",
     TEST_WRITES_PER_RUN - 1, 0, true, WriteAdmission::RunBudgetExhausted},
    {"the lifetime cap refuses a release commit", 0, STORE_REVISION_LIMIT, true,
     WriteAdmission::RevisionLimitReached},
};

void testBothWritePathsShareOneAdmissionMatrix() {
  for (const AdmissionCase& test_case : ADMISSION_CASES) {
    const WriteBudget ledger =
        budget(test_case.writes_this_run, test_case.wear_revision);
    uint8_t revision = 0;
    const WriteAdmission admitted = test_case.release
                                       ? admitRelease(ledger, revision)
                                       : admitWrite(ledger, revision);
    CHECK(admitted == test_case.expected, test_case.name);
    // Admission hands back the very next revision, and nothing when refused.
    const int issued = admitted == WriteAdmission::Admitted
                           ? test_case.wear_revision + 1
                           : 0;
    CHECK_EQ(asInt(revision), issued);
  }
}

void testChargingSpendsExactlyTheWritesPerformed() {
  WriteBudget single = budget(0, 0);
  chargeWrite(single, 1);
  CHECK_EQ(asInt(single.writes_this_run), 1);
  CHECK_EQ(asInt(single.wear_revision), 1);
  WriteBudget release = budget(0, 0);
  chargeRelease(release, 4);
  CHECK_EQ(asInt(release.writes_this_run), asInt(RELEASE_SLOT_COUNT));
  CHECK_EQ(asInt(release.wear_revision), 4);
}

// --- cycle 5: write-before-release ordering ---------------------------------

void testWriteOrderIsCalibrationThenAccounting() {
  CHECK_EQ(asInt(RELEASE_SLOT_COUNT), 2);
  CHECK(RELEASE_WRITE_ORDER[0] == ReleaseSlot::Calibration,
        "the calibration slot is committed first");
  CHECK(RELEASE_WRITE_ORDER[1] == ReleaseSlot::Accounting,
        "the accounting slot is committed last, so a cut mid-window keeps the previous accounting");
}

void testReleaseIsNotPermittedBeforeStaging() {
  ReleaseBuffers buffers = {};
  CHECK(!releasePermitted(buffers),
        "an unstaged release gate is closed, so no mechanism may move");
  CHECK_EQ(asInt(slotLength(buffers, ReleaseSlot::Accounting)), 0);
}

void testStagingFillsBothSlotsWithSerialisedRecords() {
  ReleaseBuffers buffers = {};
  const CalibrationRecord calibration = sampleCalibration();
  const AccountingRecord accounting = sampleAccounting();
  CHECK(stageRelease(buffers, accounting, calibration, 9),
        "staging a release succeeds");
  CHECK(releasePermitted(buffers), "a staged release opens the gate");
  uint8_t expected_calibration[CAL_SLOT_BYTES];
  uint8_t expected_accounting[ACC_SLOT_BYTES];
  writeCalibrationSlot(expected_calibration, CAL_SLOT_BYTES, calibration, 9);
  writeAccountingSlot(expected_accounting, ACC_SLOT_BYTES, accounting, 9);
  CHECK(bytesEqual(buffers.calibration, expected_calibration, CAL_SLOT_BYTES),
        "the staged calibration slot equals the serialised record");
  CHECK(bytesEqual(buffers.accounting, expected_accounting, ACC_SLOT_BYTES),
        "the staged accounting slot equals the serialised record");
  CHECK_EQ(asInt(slotLength(buffers, ReleaseSlot::Calibration)),
           asInt(CAL_SLOT_BYTES));
  CHECK_EQ(asInt(slotLength(buffers, ReleaseSlot::Accounting)),
           asInt(ACC_SLOT_BYTES));
  CHECK(slotBytes(buffers, ReleaseSlot::Calibration) == buffers.calibration,
        "the calibration accessor exposes its own slot");
  CHECK(slotBytes(buffers, ReleaseSlot::Accounting) == buffers.accounting,
        "the accounting accessor exposes its own slot");
}

void testStagedAccountingIsOnTheDeviceBeforeTheRelease() {
  ReleaseBuffers buffers = {};
  const AccountingRecord accounting = sampleAccounting();
  CHECK(stageRelease(buffers, accounting, sampleCalibration(), 9),
        "staging a release succeeds");
  AccountingRecord decoded = freshAccounting();
  const LoadResult result =
      loadAccounting(buffers.accounting, ACC_SLOT_BYTES, decoded);
  CHECK(result.outcome == LoadOutcome::Loaded,
        "the staged accounting decodes back off the bytes that will be committed");
  checkAccountingEqual(decoded, accounting);
  CHECK_EQ(asInt(buffers.revision), 9);
  CHECK_EQ(asInt(result.revision), 9);
}

void testDiscardingAStagedReleaseClosesTheGate() {
  ReleaseBuffers buffers = {};
  CHECK(stageRelease(buffers, sampleAccounting(), sampleCalibration(), 9),
        "staging a release succeeds");
  CHECK(releasePermitted(buffers), "the gate is open while the release is staged");
  discardStagedRelease(buffers);
  CHECK(!releasePermitted(buffers),
        "once the release is committed the gate closes, so it cannot be replayed");
  CHECK_EQ(asInt(slotLength(buffers, ReleaseSlot::Accounting)), 0);
}

}  // namespace

int main() {
  testCrc8MatchesLinkProtocolVectors();
  testCalibrationSlotRoundTrip();
  testAccountingSlotRoundTrip();
  testChannelMidpointIsBlackWhiteAverage();
  testCalibrationRefusesEveryKindOfDamage();
  testAccountingRefusesEveryKindOfDamage();
  testVirginStoreIsWithinBudget();
  testRevisionAdvancesOncePerWrite();
  testRevisionRidesEveryPersistedRecord();
  testWearBudgetTripsAfterTheRevisionLimit();
  testBothWritePathsShareOneAdmissionMatrix();
  testChargingSpendsExactlyTheWritesPerformed();
  testWriteOrderIsCalibrationThenAccounting();
  testReleaseIsNotPermittedBeforeStaging();
  testStagingFillsBothSlotsWithSerialisedRecords();
  testStagedAccountingIsOnTheDeviceBeforeTheRelease();
  testDiscardingAStagedReleaseClosesTheGate();
  return DF_SUMMARIZE("eeprom");
}
