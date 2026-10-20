// eeprom_store.ino — AVR delivery layer for the persistent store.
//
// Owns every EEPROM access on behalf of eeprom_store.h: the reserved header
// addresses, the slot address map, the bounded per-run write budget and the
// write-before-release commit. It contains no record logic — serialisation,
// validation and the fresh-state fallback all live in the header.
//
// Address map (Mega 2560, 4 KB EEPROM):
//   0x0000  reserved header — layout magic and layout version
//   0x0010  calibration slot
//   0x0060  accounting slot
// Each slot's trailing checksum stamp is written LAST, so a power cut inside
// a write leaves the previous consistent record intact and the partial write
// is rejected on the next load.
//
// NOTE: an EEPROM commit is synchronous (avr-libc stalls the CPU for the
// erase/write cycle, ~3.3 ms per byte). It is confined to the two durability
// points — accepting a venue sweep and committing a release — and never runs
// inside a per-tick state transition. EEPROM.put/update skip bytes that are
// already correct, which keeps a commit to only the bytes that changed.

#include <Arduino.h>
#include <EEPROM.h>

#include "eeprom_store.h"

using namespace rescue;

namespace {

// --- EEPROM map -----------------------------------------------------------

constexpr int EEPROM_DEVICE_BYTES = 4096;
constexpr int EEPROM_HEADER_ADDR = 0x0000;
constexpr int EEPROM_HEADER_RESERVED_BYTES = 16;
constexpr int EEPROM_HEADER_MAGIC_ADDR = EEPROM_HEADER_ADDR;
constexpr uint8_t EEPROM_HEADER_MAGIC_HIGH = 0x51;  // 'Q'
constexpr uint8_t EEPROM_HEADER_MAGIC_LOW = 0x34;   // '4'
constexpr int EEPROM_HEADER_LAYOUT_ADDR = EEPROM_HEADER_ADDR + 2;
constexpr uint8_t EEPROM_LAYOUT_VERSION = 1;

constexpr int EEPROM_CALIBRATION_SLOT_ADDR =
    EEPROM_HEADER_ADDR + EEPROM_HEADER_RESERVED_BYTES;
constexpr int EEPROM_ACCOUNTING_SLOT_ADDR =
    EEPROM_CALIBRATION_SLOT_ADDR + STORE_CALIBRATION_SLOT_BYTES;

// Cell updates this run may spend. A release commit costs one per slot, so
// budget enough for a venue sweep plus every deposit of a full mission.
constexpr uint8_t EEPROM_WRITES_PER_RUN = 12;

// Scratch for single-record commits. A release commit uses the caller's own
// staged buffers instead, so the two paths never share a buffer.
uint8_t g_calibrationSlot[STORE_CALIBRATION_SLOT_BYTES];
uint8_t g_accountingSlot[STORE_ACCOUNTING_SLOT_BYTES];

// Every write goes through this ledger, so the per-run allowance and the
// lifetime wear cap are enforced in one host-tested place.
WriteBudget g_budget = {0, 0, EEPROM_WRITES_PER_RUN};

static_assert(EEPROM_DEVICE_BYTES <= E2END + 1, "map exceeds the device");
static_assert(EEPROM_ACCOUNTING_SLOT_ADDR + STORE_ACCOUNTING_SLOT_BYTES <=
                  EEPROM_DEVICE_BYTES,
              "accounting slot runs past the end of EEPROM");

// --- byte plumbing --------------------------------------------------------

// Write a prepared slot: every byte but the trailing checksum, then the
// checksum itself, so the slot is only ever complete or rejected.
void writeSlotAt(int address, const uint8_t* bytes, uint16_t count) {
  const uint16_t stamped_bytes = count - ENVELOPE_STAMP_BYTES;
  EEPROM.put(address, bytes, stamped_bytes);
  EEPROM.update(address + stamped_bytes, bytes[stamped_bytes]);
}

int slotAddress(ReleaseSlot slot) {
  return slot == ReleaseSlot::Calibration ? EEPROM_CALIBRATION_SLOT_ADDR
                                          : EEPROM_ACCOUNTING_SLOT_ADDR;
}

// The wear ledger is the highest revision either slot still holds; a rejected
// slot simply cannot lower it.
uint8_t persistedRevisionAt(int address) {
  return EEPROM.read(address + ENVELOPE_REVISION_OFFSET);
}

bool storeLayoutValid() {
  return EEPROM.read(EEPROM_HEADER_MAGIC_ADDR) == EEPROM_HEADER_MAGIC_HIGH &&
         EEPROM.read(EEPROM_HEADER_MAGIC_ADDR + 1) == EEPROM_HEADER_MAGIC_LOW &&
         EEPROM.read(EEPROM_HEADER_LAYOUT_ADDR) == EEPROM_LAYOUT_VERSION;
}

// --- write budgets --------------------------------------------------------

SaveResult refusal(WriteAdmission admission) {
  return admission == WriteAdmission::RunBudgetExhausted
             ? SaveResult::RunBudgetExhausted
             : SaveResult::RevisionLimitReached;
}

}  // namespace

// ---------------------------------------------------------------------------
// Public surface
// ---------------------------------------------------------------------------

// Highest write counter either slot holds: the lifetime wear figure the
// budget and the boot diagnostic report.
uint8_t storeRevisionCounter() {
  const uint8_t calibration_revision =
      persistedRevisionAt(EEPROM_CALIBRATION_SLOT_ADDR);
  const uint8_t accounting_revision =
      persistedRevisionAt(EEPROM_ACCOUNTING_SLOT_ADDR);
  return calibration_revision > accounting_revision ? calibration_revision
                                                    : accounting_revision;
}

uint8_t storeWritesRemaining() {
  return static_cast<uint8_t>(EEPROM_WRITES_PER_RUN -
                              g_budget.writes_this_run);
}

// Open the store: stamp the layout header if this is a fresh or foreign
// device, read the wear ledger, and open this run's write budget.
void eepromStoreBegin() {
  if (!storeLayoutValid()) {
    EEPROM.update(EEPROM_HEADER_MAGIC_ADDR, EEPROM_HEADER_MAGIC_HIGH);
    EEPROM.update(EEPROM_HEADER_MAGIC_ADDR + 1, EEPROM_HEADER_MAGIC_LOW);
    EEPROM.update(EEPROM_HEADER_LAYOUT_ADDR, EEPROM_LAYOUT_VERSION);
  }
  g_budget.writes_this_run = 0;
  g_budget.wear_revision = storeRevisionCounter();
}

// Read the persisted calibration. Returns false and leaves `out` at fresh
// state when no usable record survives; the store itself stays silent.
bool loadStoredCalibration(CalibrationRecord& out) {
  if (!storeLayoutValid()) {
    out = freshCalibration();
    return false;
  }
  EEPROM.get(EEPROM_CALIBRATION_SLOT_ADDR, g_calibrationSlot,
             STORE_CALIBRATION_SLOT_BYTES);
  return loadCalibration(g_calibrationSlot, STORE_CALIBRATION_SLOT_BYTES, out)
             .outcome == LoadOutcome::Loaded;
}

// Read the persisted accounting, with the same silent fresh-state fallback.
bool loadStoredAccounting(AccountingRecord& out) {
  if (!storeLayoutValid()) {
    out = freshAccounting();
    return false;
  }
  EEPROM.get(EEPROM_ACCOUNTING_SLOT_ADDR, g_accountingSlot,
             STORE_ACCOUNTING_SLOT_BYTES);
  return loadAccounting(g_accountingSlot, STORE_ACCOUNTING_SLOT_BYTES, out)
             .outcome == LoadOutcome::Loaded;
}

// Persist an accepted venue calibration.
SaveResult storeCalibration(const CalibrationRecord& record) {
  uint8_t revision = 0;
  const WriteAdmission gate = admitWrite(g_budget, revision);
  if (gate != WriteAdmission::Admitted) return refusal(gate);
  if (writeCalibrationSlot(g_calibrationSlot, STORE_CALIBRATION_SLOT_BYTES,
                           record, revision) == 0) {
    return SaveResult::SlotRejected;
  }
  writeSlotAt(EEPROM_CALIBRATION_SLOT_ADDR, g_calibrationSlot,
              STORE_CALIBRATION_SLOT_BYTES);
  chargeWrite(g_budget, revision);
  return SaveResult::Saved;
}

// Persist mission accounting outside a release (for example the bookkeeping
// that must be current before a restart drill).
SaveResult storeAccounting(const AccountingRecord& record) {
  uint8_t revision = 0;
  const WriteAdmission gate = admitWrite(g_budget, revision);
  if (gate != WriteAdmission::Admitted) return refusal(gate);
  if (writeAccountingSlot(g_accountingSlot, STORE_ACCOUNTING_SLOT_BYTES,
                          record, revision) == 0) {
    return SaveResult::SlotRejected;
  }
  writeSlotAt(EEPROM_ACCOUNTING_SLOT_ADDR, g_accountingSlot,
              STORE_ACCOUNTING_SLOT_BYTES);
  chargeWrite(g_budget, revision);
  return SaveResult::Saved;
}

// Commit a staged release. The gate must be open, and the whole slot pair must
// fit the run allowance and sit below the lifetime cap — half a pair would
// break the ordering staging exists for. The mechanism may open its cup once
// this returns Saved, and must then close the gate.
SaveResult writeBeforeRelease(const ReleaseBuffers& buffers) {
  if (!releasePermitted(buffers)) return SaveResult::ReleaseNotStaged;
  uint8_t admitted_revision = 0;
  const WriteAdmission gate = admitRelease(g_budget, admitted_revision);
  if (gate != WriteAdmission::Admitted) return refusal(gate);
  if (admitted_revision != buffers.revision) {
    return SaveResult::RevisionStale;
  }
  for (uint8_t index = 0; index < RELEASE_SLOT_COUNT; ++index) {
    const ReleaseSlot slot = RELEASE_WRITE_ORDER[index];
    writeSlotAt(slotAddress(slot), slotBytes(buffers, slot),
                slotLength(buffers, slot));
  }
  chargeRelease(g_budget, buffers.revision);
  return SaveResult::Saved;
}
