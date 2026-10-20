// eeprom_store.h — persistent calibration + mission accounting store.
//
// Pure host-compilable C++14: no AVR identifiers, no EEPROM access, no output.
// A record serialises into a caller-owned byte buffer laid out as
//   [ envelope header | payload | checksum stamp ]
// with the stamp written LAST, so a power cut inside a write leaves the
// previous consistent record intact and the partial write is rejected on the
// next load. The AVR delivery layer (eeprom_store.ino) owns address mapping,
// the per-run write budget and the write-before-release commit. Load failures
// are SILENT: the caller gets a fresh deterministic record plus a
// machine-readable reason for its own boot log; this layer reports nothing.

#ifndef RESCUE_ROBOT_EEPROM_STORE_H
#define RESCUE_ROBOT_EEPROM_STORE_H

#include <cstdint>

namespace rescue {

// --- Calibration layer shape ----------------------------------------------
// The venue sweep is the only source of sensory truth; these defaults apply
// when no usable record survives. Contrast starts Unknown so nothing may act
// on an uncalibrated channel pair.
constexpr uint8_t QTR_CHANNEL_COUNT = 8;            // QTR-8A channels
constexpr uint8_t GREEN_BASELINE_SENSOR_COUNT = 2;  // green markers, L and R
constexpr uint16_t DEFAULT_BLACK_LEVEL = 0;
constexpr uint16_t DEFAULT_WHITE_LEVEL = 1023;  // QTR-8A reading span
constexpr uint16_t DEFAULT_CHANNEL_WEIGHT = 1;

// Envelope layout: [version | kind | length lo, hi | revision | payload].
constexpr uint8_t ENVELOPE_VERSION_OFFSET = 0;
constexpr uint8_t ENVELOPE_KIND_OFFSET = 1;
constexpr uint8_t ENVELOPE_LENGTH_OFFSET = 2;
constexpr uint8_t ENVELOPE_REVISION_OFFSET = 4;
constexpr uint8_t STORE_VERSION = 1;
constexpr uint8_t ENVELOPE_HEADER_BYTES = ENVELOPE_REVISION_OFFSET + 1;
constexpr uint8_t ENVELOPE_STAMP_BYTES = 1;
constexpr uint8_t CRC8_POLYNOMIAL = 0x07;

// Every byte count derives from the width of the field it describes.
constexpr uint8_t BYTE_FIELD_BYTES = sizeof(uint8_t);
constexpr uint8_t WORD_FIELD_BYTES = sizeof(uint16_t);
constexpr uint8_t CHANNEL_FIELD_COUNT = 3;  // black, white, weight
constexpr uint8_t CHANNEL_CAL_BYTES = CHANNEL_FIELD_COUNT * WORD_FIELD_BYTES;
constexpr uint8_t CONTRAST_VERDICT_BYTES = BYTE_FIELD_BYTES;
constexpr uint8_t WEAK_CHANNEL_MASK_BYTES = WORD_FIELD_BYTES;
constexpr uint8_t VICTIM_COUNT_BYTES = BYTE_FIELD_BYTES;
constexpr uint8_t DEPOSIT_COUNT_BYTES = BYTE_FIELD_BYTES;
constexpr uint8_t LOOP_PENDING_BYTES = BYTE_FIELD_BYTES;
constexpr uint8_t LANDMARK_COUNTER_BYTES = WORD_FIELD_BYTES;
constexpr uint16_t CALIBRATION_PAYLOAD_BYTES =
    (QTR_CHANNEL_COUNT + GREEN_BASELINE_SENSOR_COUNT) * CHANNEL_CAL_BYTES +
    CONTRAST_VERDICT_BYTES + WEAK_CHANNEL_MASK_BYTES;
constexpr uint16_t ACCOUNTING_PAYLOAD_BYTES =
    VICTIM_COUNT_BYTES + DEPOSIT_COUNT_BYTES + LOOP_PENDING_BYTES +
    LANDMARK_COUNTER_BYTES;
constexpr uint16_t CALIBRATION_STAMPED_BYTES =
    ENVELOPE_HEADER_BYTES + CALIBRATION_PAYLOAD_BYTES;
constexpr uint16_t ACCOUNTING_STAMPED_BYTES =
    ENVELOPE_HEADER_BYTES + ACCOUNTING_PAYLOAD_BYTES;
constexpr uint16_t STORE_CALIBRATION_SLOT_BYTES =
    CALIBRATION_STAMPED_BYTES + ENVELOPE_STAMP_BYTES;
constexpr uint16_t STORE_ACCOUNTING_SLOT_BYTES =
    ACCOUNTING_STAMPED_BYTES + ENVELOPE_STAMP_BYTES;

// Lifetime write budget: one revision step per committed write, and at the
// limit the store refuses further writes until the device is erased. The cap
// sits far below the cell endurance rating and fits the single revision byte.
constexpr uint16_t STORE_REVISION_LIMIT = 250;

// CRC8 over a byte range: the wire-link polynomial and seed, so one routine
// serves both.
inline uint8_t storeCrc8(const uint8_t* bytes, uint16_t count) {
  uint8_t crc = 0;
  for (uint16_t index = 0; index < count; ++index) {
    crc ^= bytes[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ CRC8_POLYNOMIAL)
                           : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// --- Records ---------------------------------------------------------------

// Black and white readings for one channel, plus the weight the sweep gave it;
// their midpoint is its threshold. Named for where it lives rather than for
// what it holds, because the line module declares its own same-shaped-pair type
// with the plain name (line.h) and both belong in one translation unit.
struct StoredChannelCalibration {
  uint16_t black_level = DEFAULT_BLACK_LEVEL;
  uint16_t white_level = DEFAULT_WHITE_LEVEL;
  uint16_t weight = DEFAULT_CHANNEL_WEIGHT;
};

enum class ContrastVerdict : uint8_t { Unknown = 0, Healthy = 1, Weak = 2 };

struct CalibrationRecord {
  StoredChannelCalibration channels[QTR_CHANNEL_COUNT];
  StoredChannelCalibration green_baseline[GREEN_BASELINE_SENSOR_COUNT];
  ContrastVerdict contrast = ContrastVerdict::Unknown;
  uint16_t weak_channel_mask = 0;  // bit per channel that failed the health gate
};

// Accounting that must outlive a power cut: loop_pending marks an interrupted
// run and landmark_counter is the expected progress a restart compares its
// re-derived position against. Navigation progress is never stored — it is
// re-derived on every boot.
struct AccountingRecord {
  uint8_t victims_counted = 0;
  uint8_t deposits_placed = 0;
  bool loop_pending = false;
  uint16_t landmark_counter = 0;
};

enum class RecordKind : uint8_t { Calibration = 1, Accounting = 2 };

enum class LoadOutcome : uint8_t {
  Loaded,
  SlotTruncated,
  VersionMismatch,
  KindMismatch,
  LengthMismatch,
  ChecksumMismatch,
};

struct LoadResult {
  LoadOutcome outcome;
  uint8_t revision;  // persisted write counter; zero unless Loaded
};

// Outcome of a commit. Every refusal is named so the caller's diagnostic log
// says why the store declined instead of guessing. It lives here rather than in
// the delivery file that performs the commit, because calibration.ino and
// mission.ino both act on a verdict they did not produce — one compares it
// against Saved, the other reports it — and a forward declaration cannot name
// an enumerator.
enum class SaveResult : uint8_t {
  Saved,
  RunBudgetExhausted,    // this run already spent its write allowance
  RevisionLimitReached,  // lifetime wear limit reached; device wants an erase
  ReleaseNotStaged,      // commit attempted with the release gate closed
  RevisionStale,         // buffers carry a revision the budget would not issue
  SlotRejected,          // record did not fit the destination slot
};

// --- Write-before-release staging ------------------------------------------
// A victim may only leave the cup once its accounting is serialised and
// committed: staging fills both slot buffers and opens the gate, the delivery
// layer commits them, the caller closes the gate before moving anything.
enum class ReleaseSlot : uint8_t { Calibration = 0, Accounting = 1 };
constexpr uint8_t RELEASE_SLOT_COUNT = 2;

// Commit order: calibration first, accounting last. A power cut inside the
// window therefore leaves the accounting at its previous consistent value and
// only the calibration rejected, which the caller simply re-sweeps.
constexpr ReleaseSlot RELEASE_WRITE_ORDER[RELEASE_SLOT_COUNT] = {
    ReleaseSlot::Calibration, ReleaseSlot::Accounting};

struct ReleaseBuffers {
  uint8_t calibration[STORE_CALIBRATION_SLOT_BYTES];
  uint8_t accounting[STORE_ACCOUNTING_SLOT_BYTES];
  uint16_t calibration_bytes;
  uint16_t accounting_bytes;
  uint8_t revision;
  bool staged;
};

// --- Write budget -----------------------------------------------------------
inline bool storeExhausted(uint8_t revision) {
  return static_cast<uint16_t>(revision) >= STORE_REVISION_LIMIT;
}

// A release costs one cell write per staged slot, so admission is all-or-
// nothing: half a slot pair would break the ordering the staging exists for.
struct WriteBudget {
  uint8_t writes_this_run;
  uint8_t wear_revision;
  uint8_t writes_per_run;
};

enum class WriteAdmission : uint8_t {
  Admitted,
  RunBudgetExhausted,
  RevisionLimitReached,
};

inline WriteAdmission admitWrite(const WriteBudget& budget,
                                 uint8_t& revision_out) {
  if (budget.writes_this_run >= budget.writes_per_run) {
    return WriteAdmission::RunBudgetExhausted;
  }
  if (storeExhausted(budget.wear_revision)) {
    return WriteAdmission::RevisionLimitReached;
  }
  revision_out = static_cast<uint8_t>(budget.wear_revision + 1);
  return WriteAdmission::Admitted;
}

inline void chargeWrite(WriteBudget& budget, uint8_t revision) {
  budget.writes_this_run = static_cast<uint8_t>(budget.writes_this_run + 1);
  budget.wear_revision = revision;
}

inline WriteAdmission admitRelease(const WriteBudget& budget,
                                  uint8_t& revision_out) {
  if (budget.writes_per_run - budget.writes_this_run < RELEASE_SLOT_COUNT) {
    return WriteAdmission::RunBudgetExhausted;
  }
  return admitWrite(budget, revision_out);
}

inline void chargeRelease(WriteBudget& budget, uint8_t revision) {
  budget.writes_this_run =
      static_cast<uint8_t>(budget.writes_this_run + RELEASE_SLOT_COUNT);
  budget.wear_revision = revision;
}

// --- Internals: little-endian byte plumbing and acceptance gates ----------

namespace detail {

inline void writeByte(uint8_t*& cursor, uint8_t value) { *cursor++ = value; }
inline uint8_t readByte(const uint8_t*& cursor) { return *cursor++; }
inline void writeU16(uint8_t*& cursor, uint16_t value) {
  writeByte(cursor, static_cast<uint8_t>(value & 0xFFu));
  writeByte(cursor, static_cast<uint8_t>((value >> 8) & 0xFFu));
}
inline uint16_t readU16(const uint8_t*& cursor) {
  const uint16_t low = readByte(cursor);
  const uint16_t high = readByte(cursor);
  return static_cast<uint16_t>(low | (high << 8));
}
inline void writeChannelCal(uint8_t*& cursor, const StoredChannelCalibration& ch) {
  writeU16(cursor, ch.black_level);
  writeU16(cursor, ch.white_level);
  writeU16(cursor, ch.weight);
}
inline StoredChannelCalibration readChannelCal(const uint8_t*& cursor) {
  StoredChannelCalibration channel;
  channel.black_level = readU16(cursor);
  channel.white_level = readU16(cursor);
  channel.weight = readU16(cursor);
  return channel;
}
inline void writeHeader(uint8_t* slot, RecordKind kind, uint16_t payload_bytes,
                        uint8_t revision) {
  slot[ENVELOPE_VERSION_OFFSET] = STORE_VERSION;
  slot[ENVELOPE_KIND_OFFSET] = static_cast<uint8_t>(kind);
  slot[ENVELOPE_LENGTH_OFFSET] = static_cast<uint8_t>(payload_bytes & 0xFFu);
  slot[ENVELOPE_LENGTH_OFFSET + 1] =
      static_cast<uint8_t>((payload_bytes >> 8) & 0xFFu);
  slot[ENVELOPE_REVISION_OFFSET] = revision;
}

// Accept a slot only when it is long enough, its envelope agrees with the
// record shape, and its stamp matches every byte it covers. The gates stand
// alone so a rejected load reports exactly which one refused.
inline LoadOutcome validateHeader(const uint8_t* slot, uint16_t slot_bytes,
                                  RecordKind expected_kind,
                                  uint16_t expected_payload_bytes) {
  const uint16_t stamped_bytes = ENVELOPE_HEADER_BYTES + expected_payload_bytes;
  if (slot_bytes < stamped_bytes + ENVELOPE_STAMP_BYTES) {
    return LoadOutcome::SlotTruncated;
  }
  if (slot[ENVELOPE_VERSION_OFFSET] != STORE_VERSION) {
    return LoadOutcome::VersionMismatch;
  }
  if (slot[ENVELOPE_KIND_OFFSET] != static_cast<uint8_t>(expected_kind)) {
    return LoadOutcome::KindMismatch;
  }
  const uint8_t* length_cursor = slot + ENVELOPE_LENGTH_OFFSET;
  if (readU16(length_cursor) != expected_payload_bytes) {
    return LoadOutcome::LengthMismatch;
  }
  if (slot[stamped_bytes] != storeCrc8(slot, stamped_bytes)) {
    return LoadOutcome::ChecksumMismatch;
  }
  return LoadOutcome::Loaded;
}

}  // namespace detail

// --- Public surface --------------------------------------------------------

inline CalibrationRecord freshCalibration() { return CalibrationRecord(); }
inline AccountingRecord freshAccounting() { return AccountingRecord(); }
inline uint16_t channelMidpoint(const StoredChannelCalibration& channel) {
  return static_cast<uint16_t>((channel.black_level + channel.white_level) / 2);
}
// Charge one write to the budget; a refused write is never counted.
inline bool advanceRevision(uint8_t& revision) {
  if (storeExhausted(revision)) return false;
  revision = static_cast<uint8_t>(revision + 1);
  return true;
}

// Serialise into `capacity` bytes of `slot`, stamping the checksum last.
// Returns the slot size written, or 0 when capacity is too small.
inline uint16_t writeCalibrationSlot(uint8_t* slot, uint16_t capacity,
                                     const CalibrationRecord& record,
                                     uint8_t revision) {
  if (capacity < STORE_CALIBRATION_SLOT_BYTES) return 0;
  detail::writeHeader(slot, RecordKind::Calibration, CALIBRATION_PAYLOAD_BYTES,
                      revision);
  uint8_t* cursor = slot + ENVELOPE_HEADER_BYTES;
  for (uint8_t channel = 0; channel < QTR_CHANNEL_COUNT; ++channel)
    detail::writeChannelCal(cursor, record.channels[channel]);
  for (uint8_t sensor = 0; sensor < GREEN_BASELINE_SENSOR_COUNT; ++sensor)
    detail::writeChannelCal(cursor, record.green_baseline[sensor]);
  detail::writeByte(cursor, static_cast<uint8_t>(record.contrast));
  detail::writeU16(cursor, record.weak_channel_mask);
  slot[CALIBRATION_STAMPED_BYTES] = storeCrc8(slot, CALIBRATION_STAMPED_BYTES);
  return STORE_CALIBRATION_SLOT_BYTES;
}

inline uint16_t writeAccountingSlot(uint8_t* slot, uint16_t capacity,
                                    const AccountingRecord& record,
                                    uint8_t revision) {
  if (capacity < STORE_ACCOUNTING_SLOT_BYTES) return 0;
  detail::writeHeader(slot, RecordKind::Accounting, ACCOUNTING_PAYLOAD_BYTES,
                      revision);
  uint8_t* cursor = slot + ENVELOPE_HEADER_BYTES;
  detail::writeByte(cursor, record.victims_counted);
  detail::writeByte(cursor, record.deposits_placed);
  detail::writeByte(cursor, record.loop_pending ? 1 : 0);
  detail::writeU16(cursor, record.landmark_counter);
  slot[ACCOUNTING_STAMPED_BYTES] = storeCrc8(slot, ACCOUNTING_STAMPED_BYTES);
  return STORE_ACCOUNTING_SLOT_BYTES;
}

inline LoadResult loadCalibration(const uint8_t* slot, uint16_t slot_bytes,
                                  CalibrationRecord& out) {
  out = freshCalibration();
  const LoadOutcome outcome =
      detail::validateHeader(slot, slot_bytes, RecordKind::Calibration,
                             CALIBRATION_PAYLOAD_BYTES);
  if (outcome != LoadOutcome::Loaded) return LoadResult{outcome, 0};
  const uint8_t* cursor = slot + ENVELOPE_HEADER_BYTES;
  for (uint8_t channel = 0; channel < QTR_CHANNEL_COUNT; ++channel)
    out.channels[channel] = detail::readChannelCal(cursor);
  for (uint8_t sensor = 0; sensor < GREEN_BASELINE_SENSOR_COUNT; ++sensor)
    out.green_baseline[sensor] = detail::readChannelCal(cursor);
  out.contrast = static_cast<ContrastVerdict>(detail::readByte(cursor));
  out.weak_channel_mask = detail::readU16(cursor);
  return LoadResult{LoadOutcome::Loaded, slot[ENVELOPE_REVISION_OFFSET]};
}

inline LoadResult loadAccounting(const uint8_t* slot, uint16_t slot_bytes,
                                 AccountingRecord& out) {
  out = freshAccounting();
  const LoadOutcome outcome =
      detail::validateHeader(slot, slot_bytes, RecordKind::Accounting,
                             ACCOUNTING_PAYLOAD_BYTES);
  if (outcome != LoadOutcome::Loaded) return LoadResult{outcome, 0};
  const uint8_t* cursor = slot + ENVELOPE_HEADER_BYTES;
  out.victims_counted = detail::readByte(cursor);
  out.deposits_placed = detail::readByte(cursor);
  out.loop_pending = detail::readByte(cursor) != 0;
  out.landmark_counter = detail::readU16(cursor);
  return LoadResult{LoadOutcome::Loaded, slot[ENVELOPE_REVISION_OFFSET]};
}

inline void discardStagedRelease(ReleaseBuffers& buffers);

// Serialise both records into caller-owned buffers charged with `revision`.
// Pure RAM work: nothing is committed here, so the caller can stage, decide,
// and only then hand the buffers over.
inline bool stageRelease(ReleaseBuffers& buffers,
                         const AccountingRecord& accounting,
                         const CalibrationRecord& calibration,
                         uint8_t revision) {
  const uint16_t accounting_bytes =
      writeAccountingSlot(buffers.accounting, STORE_ACCOUNTING_SLOT_BYTES,
                          accounting, revision);
  const uint16_t calibration_bytes =
      writeCalibrationSlot(buffers.calibration, STORE_CALIBRATION_SLOT_BYTES,
                           calibration, revision);
  if (accounting_bytes == 0 || calibration_bytes == 0) {
    discardStagedRelease(buffers);
    return false;
  }
  buffers.calibration_bytes = accounting_bytes ? calibration_bytes : 0;
  buffers.accounting_bytes = accounting_bytes;
  buffers.revision = revision;
  buffers.staged = true;
  return true;
}

// True only while a staged release is outstanding. The mechanism reads this
// gate before it opens the cup: a release whose accounting is not yet on the
// device may never move.
inline bool releasePermitted(const ReleaseBuffers& buffers) {
  return buffers.staged;
}

inline void discardStagedRelease(ReleaseBuffers& buffers) {
  buffers.calibration_bytes = 0;
  buffers.accounting_bytes = 0;
  buffers.staged = false;
}

inline const uint8_t* slotBytes(const ReleaseBuffers& buffers,
                                ReleaseSlot slot) {
  return slot == ReleaseSlot::Calibration ? buffers.calibration
                                          : buffers.accounting;
}

inline uint16_t slotLength(const ReleaseBuffers& buffers, ReleaseSlot slot) {
  return slot == ReleaseSlot::Calibration ? buffers.calibration_bytes
                                          : buffers.accounting_bytes;
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_EEPROM_STORE_H
