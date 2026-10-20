// mission.ino — the sketch's mission pass: one loop of sensing, one decision,
// one drive command.
//
// This file is plumbing. It polls each sensor once per loop, folds the samples
// into one MissionInputs snapshot, decodes the five link payloads the mission
// needs, hands the snapshot to mission.h, posts the single reflex event that
// came out of it, and installs the seam that turns a DriveDemand into motion.
// It decides nothing: every judgement lives in mission.h, and every reflex,
// restart and zone budget lives in the table in fsm.ino.
//
// The restart's own wiring lives here too, because it is plumbing of the same
// kind: the persisted ledger is read once at bring-up and staged into mission.h,
// the sweep request mission.h armed is put on the wire, and the ledger the resume
// publishes is written back through the store's own single-record path.
//
// Non-blocking by construction: no delay(), no waiting for a frame, no retries.
// A link value that is not fresh is simply missing from the snapshot, and
// mission.h already refuses to score any behavior on one source alone.

#include <Arduino.h>

#include "calibration.h"
#include "link.h"
#include "markers.h"
#include "mission.h"
#include "motors.h"
#include "sensors.h"

using namespace rescue;

// The store's entry points and the link's sender live in eeprom_store.ino and
// link.ino, which the Arduino build concatenates into this same translation unit
// — the same arrangement calibration.ino and fsm.ino already rely on. Only the
// names are repeated here, because the host test builds this file on its own.
SaveResult storeAccounting(const AccountingRecord& record);
bool loadStoredAccounting(AccountingRecord& out);
bool linkSend(uint8_t id, const uint8_t* payload, uint8_t payloadLength);

// The bumper plate and the drivetrain entry points come from sensors.ino and
// motors.ino, for the same reason. This file reads the plate (a tap and a held
// zone are different facts) and the victim probes, and it parks the motors
// before a fresh sweep, so it has to be able to name them without including the
// whole delivery layer.
void motorsBegin();
void sensorsBegin();
void updateBumper();
bool consumeAnyBumperEvent(BumperZone& zone);
bool bumperPressed(BumperZone zone);
int readReflectivity();
bool readConductivity();

namespace {

// Payload field offsets and lengths, docs/mega-nicla-protocol.md section 4.3.
// The length checks are a lower bound on purpose: a frame that grew a field
// still decodes, and the reader only ever touches the bytes it names.
constexpr uint8_t GOAL_WIDTH_OFFSET = 4;
constexpr uint8_t GOAL_HEIGHT_OFFSET = 6;
constexpr uint8_t GOAL_CONFIDENCE_OFFSET = 8;
constexpr uint8_t GOAL_PAYLOAD_BYTES = 9;
constexpr uint8_t TOF_MM_OFFSET = 0;
constexpr uint8_t TOF_STATUS_OFFSET = 2;
constexpr uint8_t TOF_PAYLOAD_BYTES = 3;
constexpr uint8_t TOF_STATUS_VALID = 0;
constexpr uint8_t IMU_YAW_OFFSET = 0;
constexpr uint8_t IMU_PITCH_OFFSET = 2;
constexpr uint8_t IMU_FLAGS_OFFSET = 4;
constexpr uint8_t IMU_PAYLOAD_BYTES = 5;
constexpr uint8_t BLOB_KIND_OFFSET = 0;
constexpr uint8_t BLOB_CENTRE_OFFSET = 2;
constexpr uint8_t BLOB_EXTENT_OFFSET = 6;
constexpr uint8_t BLOB_PAYLOAD_BYTES = 7;
constexpr uint8_t RADAR_ZONE_OFFSET = 0;
constexpr uint8_t RADAR_CLASS_OFFSET = 1;
constexpr uint8_t RADAR_PAYLOAD_BYTES = 2;

// The seam mission.h delivers through: the one place a decision becomes motion.
// motors.h clamps to MOTOR_MAX_SPEED, so a demand can never exceed the ceiling.
void driveWheels(const DriveDemand& demand, void* context) {
  (void)context;
  setMotor(demand.left, demand.right);
}

// GOAL_BLOB 0x03: cx,cy,width,height normalized 0..1000 and confidence percent.
// Only the confidence and the long axis reach mission.h, because the goal gate
// asks "seen and big enough", not "where in the frame".
void sampleGoal(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_GOAL_BLOB);
  if (frame == nullptr || frame->payloadLength < GOAL_PAYLOAD_BYTES) return;
  const uint16_t width = readBe16(frame->payload, GOAL_WIDTH_OFFSET);
  const uint16_t height = readBe16(frame->payload, GOAL_HEIGHT_OFFSET);
  in.goalFresh = true;
  in.goalConfidence = frame->payload[GOAL_CONFIDENCE_OFFSET];
  in.goalSize = width > height ? width : height;
}

// TOF_RANGE 0x05: mm:u16, status:u8, where status 0 is the only valid reading.
// The status is checked before the millimetres are used, so a faulted sample
// can never masquerade as "nothing in the way".
void sampleRange(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_TOF_RANGE);
  if (frame == nullptr || frame->payloadLength < TOF_PAYLOAD_BYTES) return;
  if (frame->payload[TOF_STATUS_OFFSET] != TOF_STATUS_VALID) return;
  in.tofFresh = true;
  in.tofMm = readBe16(frame->payload, TOF_MM_OFFSET);
}

// IMU_FRAME 0x07: yaw:i16, pitch:i16, flags:u8 with bit0 meaning valid. Both
// angles arrive in 0.01 deg, which is the unit mission.h already speaks, so no
// scaling happens on this side of the wire.
void sampleImu(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_IMU_FRAME);
  if (frame == nullptr || frame->payloadLength < IMU_PAYLOAD_BYTES) return;
  if ((frame->payload[IMU_FLAGS_OFFSET] & LINK_IMU_FLAG_VALID) == 0) return;
  in.imuFresh = true;
  in.yawCentiDeg = readBeI16(frame->payload, IMU_YAW_OFFSET);
  in.pitchCentiDeg = readBeI16(frame->payload, IMU_PITCH_OFFSET);
}

// TAPE_BLOB 0x08: type:u8, cx:u16, cy:u16, size:u16. Only the type and the
// size reach mission.h: the two bands either end of the zone are told apart by
// their tape colour, and the entry gate is a claim with the bar, not a position.
void sampleTape(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_TAPE_BLOB);
  if (frame == nullptr || frame->payloadLength < BLOB_PAYLOAD_BYTES) return;
  in.evacTapeFresh = true;
  in.evacTapeType = frame->payload[BLOB_KIND_OFFSET];
  in.evacTapeSize = readBe16(frame->payload, BLOB_EXTENT_OFFSET);
}

// TRIANGLE_BLOB 0x04: color:u8, cx:u16, cy:u16, size:u16. The same shape as the
// tape, and the same discipline: which colour and how big, not where in frame.
void sampleTriangle(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_TRIANGLE_BLOB);
  if (frame == nullptr || frame->payloadLength < BLOB_PAYLOAD_BYTES) return;
  in.triangleFresh = true;
  in.triangleColor = frame->payload[BLOB_KIND_OFFSET];
  in.triangleSize = readBe16(frame->payload, BLOB_EXTENT_OFFSET);
}

// The victim probes, read straight and unfiltered. Deciding what they mean is
// mission.h's job, and it is the only reader of these two numbers: a raw sample
// travelling in a snapshot cannot be a decision, whatever it looks like.
void sampleVictimProbes(MissionInputs& in) {
  in.reflectivity = readReflectivity();
  in.conductive = readConductivity();
}

// READY 0x01: proto_major:u8, proto_minor:u8, cal_cached:u8, sensor_ok:u8,
// impl_version:u16. Only the receipt reaches mission.h — whether the handshake
// happened — because the restart asks "did the camera answer", not "which
// version did it answer in". The cache holds READY for the rest of the run, so
// this is a one-pass latch rather than a per-frame decision.
void sampleReady(MissionInputs& in) {
  if (linkReceiveFresh(MSG_READY)) in.readyFresh = true;
}

// RADAR_RESULT 0x06: zone:u8, class:u8, with 0xFF in zone meaning "no result".
// Both bytes travel as they are: mission.h already knows which profiles are worth
// a direction and which are not, and re-deciding that here would give the pure
// layer two places to disagree. The zone/class gate is link's own, and it hands
// over the newest frame of the sweep, not the first.
void sampleRadar(MissionInputs& in) {
  const CachedFrame* frame = linkLastGood(MSG_RADAR_RESULT);
  if (frame == nullptr || frame->payloadLength < RADAR_PAYLOAD_BYTES) return;
  in.radarFresh = true;
  in.radarZone = frame->payload[RADAR_ZONE_OFFSET];
  in.radarClass = frame->payload[RADAR_CLASS_OFFSET];
}

// The sweep mission.h armed has not left the machine yet. Nothing waits on the
// answer — the bar is already hunting — so this is fire-and-forget, and a
// request the wire refused is spent rather than retried.
void flushSweepRequest() {
  if (!lopRadarRequestPending()) return;
  const uint8_t request[2] = {MISSION_LOP_RADAR_STEP_DEG, MISSION_RADAR_SECTOR_COUNT};
  linkSend(MSG_REQUEST_RADAR, request, sizeof(request));
  lopRadarRequestSent();
}

// The resume hands the ledger over once per restart; the store is told once per
// run. The latch is delivery-side, so the restart path itself stays a pure
// function of its own evidence and no EEPROM call sits on a table row.
bool g_lopLedgerWritten = false;

void publishLopLedger() {
  if (g_lopLedgerWritten || !lopLedgerPublished()) return;
  g_lopLedgerWritten = true;
  // The record the resume publishes IS the store's own record, so what is
  // written back is exactly what was re-derived — no translation, and therefore
  // no way for the two to disagree about a count.
  const AccountingRecord record = lopReDerivedAccounting();
  // A refused write is the store's business, not this pass's: the ledger is
  // already correct in RAM and the next restart reads whatever survived.
  storeAccounting(record);
}

// One pass over the sensors. A strike is the tap the plate latched, or a zone
// still held down: a fast tap can begin and end between two passes, so the
// level alone would miss it, while a held plate must keep acting because the
// robot really is still against something.
void sampleInputs(MissionInputs& in) {
  in.line = updateLine();
  in.marker = pollMarkerSensors().state;
  updateBumper();
  BumperZone struck = BumperZone::Left;
  const bool tapped = consumeAnyBumperEvent(struck);
  const bool leftHeld = bumperPressed(BumperZone::Left);
  const bool rightHeld = bumperPressed(BumperZone::Right);
  in.bumperContact = tapped || leftHeld || rightHeld;
  in.bothZones = leftHeld && rightHeld;
  // One held zone names the side; with neither held the latched tap does; with
  // both, bothZones decides the direction and the name does not matter.
  in.bumperZone = leftHeld ? BumperZone::Left
                           : (rightHeld ? BumperZone::Right : struck);
  sampleRange(in);
  sampleGoal(in);
  sampleImu(in);
  sampleTape(in);
  sampleTriangle(in);
  sampleVictimProbes(in);
  sampleReady(in);
  sampleRadar(in);
  in.policy = linkPolicy();
}

}  // namespace

// main.ino owns setup() and loop(): the boot table drives missionBegin() once
// and the loop pass hands the mission its single tick. These are the two entry
// points it calls.
void missionBegin() {
  motorsBegin();
  sensorsBegin();
  lineBegin();
  markersBegin();
  linkBegin();
  eepromStoreBegin();

  // Both green sensors were calibrated against the same white card, and the
  // marker logic takes one shared white reference, so sensor 0 supplies it. An
  // unstored calibration leaves the baseline at 0, and markers.h reports no
  // green for a 0 baseline: the robot then never turns on sensor noise.
  CalibrationRecord calibration;
  if (loadStoredCalibration(calibration)) {
    setMarkerWhiteBaseline(calibration.green_baseline[0].white_level);
  }

  setMissionDrive(&driveWheels, nullptr);
  beginMission();

  // The ledger, read once and staged whole. A store that cannot be read reports
  // a fresh record, whose zero landmark count stages nothing — so a blank or
  // rejected device produces an ordinary clean start rather than a restart that
  // believes it was somewhere.
  AccountingRecord accounting;
  loadStoredAccounting(accounting);
  beginLopRestage(accounting);
  g_lopLedgerWritten = false;
}

void missionTick() {
  linkTick();
  MissionInputs in;
  sampleInputs(in);
  setMissionInputs(in);
  advanceMission();
  flushSweepRequest();
  fsmPostEvent(missionPendingTrigger());
  fsmTick();
  publishLopLedger();
}
