// mission.h — pure host-compilable navigation reflex logic (C++14, no AVR I/O).
//
// One named pure step function per reflex, over a snapshot of the bar, the
// plate, the markers and the link, plus the cross-tick state of the live reflex.
// Nothing here touches a peripheral: mission.ino fills the snapshot once per
// loop and installs the drive seam, and fsm.ino's table callbacks are the only
// callers that move a reflex along. Reflexes claim a tick in a fixed order (a
// physical strike outranks everything, then dead end, lost line, goal, a green
// side, ramp, seesaw) and each carries a residence budget, so a wedged one is
// caught by its guard and, failing that, by the motor backstop.
//
// The evacuation zone is the one part that is a chain rather than a reflex:
// entering it is a two-source claim (the camera's silver-tape blob AND the
// bar's wide-bright class), and the lane that follows — approach, probe,
// classify, deposit, exit — is bounded state by state, with the deposit written
// down before anything is released. While the zone is open the entry token
// holds the tick, so nothing re-enters behind the chain; the rows that hand the
// robot back to Idle are the ones that drop it again.

#ifndef RESCUE_ROBOT_MISSION_H
#define RESCUE_ROBOT_MISSION_H

#include <cstdint>

#include "eeprom_store.h"
#include "fsm.h"
#include "line.h"
#include "link.h"
#include "markers.h"
#include "sensors.h"

namespace rescue {

// --- Events this module owns (the engine's event set is open) --------------
constexpr EventId MISSION_EVENT_CONTACT = 1;
constexpr EventId MISSION_EVENT_GOAL = 2;
constexpr EventId MISSION_EVENT_DEAD_END = 3;
constexpr EventId MISSION_EVENT_GAP = 4;
constexpr EventId MISSION_EVENT_MARKER = 5;
constexpr EventId MISSION_EVENT_RAMP = 6;
constexpr EventId MISSION_EVENT_SEESAW = 7;
constexpr EventId MISSION_EVENT_EVAC = 8;

// --- Speeds, in the PWM duty units motors.h already speaks -----------------
constexpr int MISSION_CRUISE_SPEED = 90;  // reduced base speed the bar tracks at
constexpr int MISSION_GAP_SPEED = 70;
constexpr int MISSION_RAMP_SPEED = 75;
constexpr int MISSION_SEESAW_SPEED = 55;
constexpr int MISSION_TURN_SPEED = 55;  // pivot duty, the two wheels opposed
constexpr int MISSION_OBSTACLE_BIAS_SPEED = 65;
constexpr int MISSION_OBSTACLE_ARC_SPEED = 55;

// A gap is bridged on the last-known output, bounded, and re-acquired on clean
// centroid samples; a 180 is the same pivot demand held longer, so it takes no
// second shape.
constexpr uint32_t MISSION_GAP_MAX_MS = 700;  // a 20 cm gap, entered straight-in
constexpr uint8_t MISSION_GAP_CLEAR_SAMPLES = 2;
constexpr uint32_t MISSION_MARKER_TURN_MS = 420;
constexpr uint32_t MISSION_DEAD_END_PIVOT_MS = 800;

// Centi-degrees of yaw error scaled into a wheel differential, and the cap that
// keeps it an assist: it can never grow into the path turn on a reflex that is
// not allowed to turn.
constexpr int32_t MISSION_HEADING_ASSIST_SCALE = 200;
constexpr int32_t MISSION_HEADING_ASSIST_MAX = 25;
// The wire carries angles as 0.01 deg per LSB, so this is the scale from the
// units IMU_FRAME speaks to the degrees the assist is reasoned about.
constexpr int32_t MISSION_CENTI_DEG_PER_DEG = 100;

// Terrain bands, in centi-degrees of Nicla pitch. The flat dead-band is what
// stops every straight run reading as a seesaw: a crossing starts and ends
// level, while the mock props sit at 15 deg and 22 deg.
constexpr int16_t MISSION_FLAT_PITCH_MAX_CENTI_DEG = 500;
constexpr int16_t MISSION_SEESAW_PITCH_MAX_CENTI_DEG = 2000;
constexpr int16_t MISSION_RAMP_PITCH_MAX_CENTI_DEG = 2500;
constexpr uint32_t MISSION_RAMP_MAX_MS = 2500;
constexpr uint32_t MISSION_SEESAW_CROSS_MS = 1200;

// The carve-out gate: false until the verify-on-mock evidence exists, so the
// gyro-guided slow straight is the only seesaw strategy currently live.
constexpr bool MISSION_SEESAW_MOCK_VERIFIED = false;

constexpr uint16_t MISSION_OBSTACLE_TOF_BIAS_MM = 450;
constexpr uint32_t MISSION_OBSTACLE_MAX_MS = 2500;
constexpr uint8_t MISSION_OBSTACLE_CLEAR_SAMPLES = 2;

constexpr uint8_t MISSION_GOAL_MIN_CONFIDENCE_PCT = 50;
constexpr uint16_t MISSION_GOAL_MIN_SIZE = 20;  // normalised blob diagonal
constexpr uint16_t MISSION_GOAL_TOF_CLOSE_MM = 350;
constexpr uint32_t MISSION_GOAL_HOLD_MS = 5000;

// Spent well before this by every reflex budget; it exists so a guard which
// somehow never passes still stops the motors.
constexpr uint32_t MISSION_STATE_BACKSTOP_MS = 300;

// --- The evacuation zone ----------------------------------------------------
// Entering is a two-source claim — the camera's silver-tape blob and the bar's
// wide-bright class — and leaving is its mirror on the black band. The lane
// between them is five bounded states, and every budget below is named once
// here so a table row and the guard that reads it can never disagree.

// Wire enum values, docs/mega-nicla-protocol.md section 4.3.
constexpr uint8_t MISSION_TAPE_SILVER = 0;     // TAPE_BLOB type: entrance band
constexpr uint8_t MISSION_TAPE_BLACK = 1;      // TAPE_BLOB type: exit band
constexpr uint8_t MISSION_TRIANGLE_RED = 0;    // TRIANGLE_BLOB color: dead victims
constexpr uint8_t MISSION_TRIANGLE_GREEN = 1;  // TRIANGLE_BLOB color: live victims

constexpr uint16_t MISSION_EVAC_TAPE_MIN_SIZE = 20;  // normalised blob diagonal
constexpr uint16_t MISSION_EVAC_TRIANGLE_MIN_SIZE = 20;
constexpr uint16_t MISSION_EVAC_CONTACT_MM = 200;  // this close is under the probes

constexpr int MISSION_EVAC_SEARCH_SPEED = 45;   // creeping for a victim
constexpr int MISSION_EVAC_CARRY_SPEED = 32;    // loaded: an off-centre sphere rolls
constexpr int MISSION_EVAC_EXIT_SPEED = 55;     // driving out along the black tape
constexpr int MISSION_EVAC_BACKOUT_SPEED = 40;  // reversing clear of the victim

constexpr uint32_t MISSION_EVAC_APPROACH_MAX_MS = 2500;
constexpr uint8_t MISSION_EVAC_PROBE_SETTLE_SAMPLES = 3;  // agreeing samples to name a class
constexpr uint32_t MISSION_EVAC_PROBE_MAX_MS = 600;
constexpr uint32_t MISSION_EVAC_CLASSIFY_MAX_MS = 400;
constexpr uint32_t MISSION_EVAC_DEPOSIT_MAX_MS = 800;
constexpr uint32_t MISSION_EVAC_EXIT_MAX_MS = 2000;

// The mechanism gate, frozen shut until its bench numbers exist. Capture is
// best-effort: with this off the lane still locates, classifies, records and
// scores every victim, and the zone exit is never held up waiting for a cup.
constexpr bool MISSION_EVAC_MECHANISM_ENABLED = false;

// --- Loss of position -------------------------------------------------------
// The restart path, in four bounded states. WAIT_READY is the whole of the
// Nicla handshake: the link comes up on its own clock and a press on the user
// button only releases the boot that starts it, so the window is not a wait for
// the operator — it is the time the bar is uselessly stood on bare floor, and
// closing it means the run continues line-only with the error latched. What
// comes after is the bar: a bounded pivot ladder first, the radar sweep asked
// for concurrently with it, and a short resume that has to put the line back
// under the bar or hand the run back deliberately. Every bound below is the
// number its own table row carries, so a row and its bail guard cannot disagree.

constexpr uint32_t MISSION_LOP_READY_WAIT_MS = 5000;   // the Nicla handshake window
constexpr uint32_t MISSION_LOP_SWEEP_ARM_MS = 2000;    // net under the sweep handoff
constexpr uint32_t MISSION_LOP_SEARCH_LEG_MS = 750;    // one leg of the pivot ladder
constexpr uint8_t MISSION_LOP_SEARCH_LEGS = 4;
constexpr uint32_t MISSION_LOP_SEARCH_MAX_MS = MISSION_LOP_SEARCH_LEG_MS * MISSION_LOP_SEARCH_LEGS;
constexpr uint32_t MISSION_LOP_RESUME_MAX_MS = 3000;   // back on the line, or home
constexpr uint8_t MISSION_LOP_CLEAR_SAMPLES = 2;       // agreeing passes that re-prove the line

constexpr int MISSION_LOP_SEARCH_PIVOT_SPEED = 45;  // hunting blind, so slowly
constexpr int MISSION_LOP_SEARCH_ARC_SPEED = 35;    // rolling onto a sector dead ahead
constexpr int MISSION_LOP_RESUME_SPEED = 70;        // back on the line, cautiously

// One sweep is asked for as the search starts, and at most one more when the
// profile comes back ambiguous. The sweep runs concurrently with the search, so
// this cap is a traffic budget, not a search budget — it is not added to the 12 s.
constexpr uint8_t MISSION_LOP_RADAR_REQUEST_CAP = 2;
// REQUEST_RADAR asks for the whole sweep at the documented 20 deg step over 18
// sectors; both fields are the wire's own "use the default" convention, so the
// request says what it wants rather than what the default happens to be.
constexpr uint8_t MISSION_LOP_RADAR_STEP_DEG = 20;

// RADAR_RESULT class: 1 = enclosed, 0 = open, 2 = ambiguous (docs 4.3).
constexpr uint8_t MISSION_RADAR_OPEN = 0;
constexpr uint8_t MISSION_RADAR_ENCLOSED = 1;
constexpr uint8_t MISSION_RADAR_AMBIGUOUS = 2;
// 18 sectors of 20 deg, numbered from the nose: 0 dead ahead, 1..8 to port,
// 10..17 to starboard, 9 astern. A sector outside that spread, or LINK's own
// 0xFF no-result marker, is not a direction.
constexpr uint8_t MISSION_RADAR_SECTOR_COUNT = 18;
constexpr uint8_t MISSION_RADAR_ASTERN_SECTOR = 9;
constexpr uint8_t MISSION_RADAR_FIRST_LEFT = 1;
constexpr uint8_t MISSION_RADAR_LAST_LEFT = 8;
constexpr uint8_t MISSION_RADAR_FIRST_RIGHT = 10;
constexpr uint8_t MISSION_RADAR_LAST_RIGHT = 17;

// A landmark is anything the bar cannot mistake for the next patch of line: a
// green marker on one side, or a strike the plate latched. A crossing is
// counted once, on the rising edge of the claim, however many passes it stays
// under the bar.
constexpr uint8_t MISSION_LOP_LANDMARK_ERROR_BOUND = 1;

// The drill's own bound, and the margin held back inside it (docs section 5).
constexpr uint32_t MISSION_LOP_DRILL_BOUND_MS = 12000;
constexpr uint32_t MISSION_LOP_DRILL_RESERVE_MS = 500;

// Marker side to turn direction. HARDWARE section 9 and the mission contract
// both read "green on one side, turn that way", which is what markers.h
// publishes; the field mockup guide's E2a/E2b prose reads the other way round.
// The mapping is this one flag, so the bench settles it with a one-line edit.
constexpr bool MISSION_MARKER_SIDE_INVERT = false;

// --- Shapes ---------------------------------------------------------------

enum class TurnDirection : uint8_t { Straight, Left, Right, Reverse };
enum class DriveMode : uint8_t { LineFollow, GapHold, Pivot, GyroStraight, Arc, Stop };
enum class Terrain : uint8_t { Flat, Ramp, Seesaw };

// Where the rules send each class: live to the green triangle, dead to the red
// one, and a fake to neither, because a fake is never carried.
enum class DepositTarget : uint8_t { None, Red, Green };

// What the radar is allowed to contribute to the search: a side to start on, or
// nothing at all. There is no enum value for "wait", because nothing here can
// wait — the sweep runs while the bar is already hunting.
enum class SearchBias : uint8_t { Alternating, Straight, Left, Right };

struct DriveDemand {
  DriveMode mode;
  int left;
  int right;
};

struct MissionInputs {
  LineReading line;
  MarkerState marker;
  bool bumperContact;
  BumperZone bumperZone;
  bool bothZones;
  bool tofFresh;
  uint16_t tofMm;
  bool goalFresh;
  uint8_t goalConfidence;
  uint16_t goalSize;
  bool imuFresh;
  int16_t yawCentiDeg;
  int16_t pitchCentiDeg;
  // The zone: the band the camera sees at each end of it, the deposit triangle
  // it can corroborate, and the two raw probe reads the held classifier below
  // turns into a verdict. None of these are decisions; they are evidence.
  bool evacTapeFresh;
  uint8_t evacTapeType;
  uint16_t evacTapeSize;
  bool triangleFresh;
  uint8_t triangleColor;
  uint16_t triangleSize;
  int reflectivity;
  bool conductive;
  LinkPolicy policy;
  // The handshake and the sweep. READY is sticky on the wire, so its flag is
  // the receipt of the first one rather than a per-frame tick; the radar flag
  // is the frame's own TTL verdict, exactly as the fields above it are.
  bool readyFresh;
  bool radarFresh;
  uint8_t radarZone;
  uint8_t radarClass;
};
// The freshness flags above are link.h's own verdict already applied: a stale
// value arrives as "not fresh" and is therefore never a decision source.

typedef void (*MissionDriveFn)(const DriveDemand& demand, void* ctx);

// --- Shared helpers -------------------------------------------------------

// Split a correction across the wheels, the way line.h already splits one; the
// live centroid's steady-state part is what the bar contributes as the control
// signal, so the line never stops being primary.
inline DriveDemand withBias(DriveMode mode, int speed, int32_t correction) {
  return DriveDemand{mode, speed + correction, speed - correction};
}
inline int32_t lineBias(int32_t error) { return steerFor(error).left; }

// Link-sourced values may be used while the policy says so; under
// DegradeToLineOnly every reflex falls back to what the bar and the plate alone
// can prove, which is what keeps the robot off the wall instead of stranded.
inline bool linkUsable(const MissionInputs& in) { return in.policy != LinkPolicy::DegradeToLineOnly; }

inline int32_t headingAssist(int32_t yawErrorCentiDeg) {
  const int32_t scaled =
      yawErrorCentiDeg * MISSION_HEADING_ASSIST_SCALE / MISSION_CENTI_DEG_PER_DEG;
  return scaled > MISSION_HEADING_ASSIST_MAX
             ? MISSION_HEADING_ASSIST_MAX
             : (scaled < -MISSION_HEADING_ASSIST_MAX ? -MISSION_HEADING_ASSIST_MAX : scaled);
}

inline bool lineLost(const LineReading& line) {
  return !line.lineDetected || line.qualifier == LineClass::EDGE ||
         line.qualifier == LineClass::ALL_WHITE;
}
inline bool onLineClear(const LineReading& line) {
  return line.lineDetected && line.qualifier == LineClass::ON_LINE;
}

// --- Per-reflex steps -----------------------------------------------------

inline DriveDemand pivotStep(TurnDirection turn, int speed) {
  switch (turn) {
    case TurnDirection::Left: return DriveDemand{DriveMode::Pivot, speed, -speed};
    case TurnDirection::Right: return DriveDemand{DriveMode::Pivot, -speed, speed};
    case TurnDirection::Reverse: return DriveDemand{DriveMode::Pivot, -speed, -speed};
    default: return DriveDemand{DriveMode::Pivot, speed, speed};
  }
}

// The held correction carries the crossing; a usable link adds the yaw assist on
// top of it, never instead of it.
inline DriveDemand gapStep(const MissionInputs& in, int32_t heldCorrection, int16_t yawAtEntry) {
  const int32_t assist =
      linkUsable(in) ? headingAssist(static_cast<int32_t>(in.yawCentiDeg) - yawAtEntry) : 0;
  return withBias(DriveMode::GapHold, MISSION_GAP_SPEED, heldCorrection + assist);
}

inline TurnDirection turnForMarker(const MissionInputs& in) {
  if (in.marker == MarkerState::None || in.marker == MarkerState::Both) {
    return TurnDirection::Straight;
  }
  const bool greenLeft = (in.marker == MarkerState::Left) != MISSION_MARKER_SIDE_INVERT;
  return greenLeft ? TurnDirection::Left : TurnDirection::Right;
}

// The pressed zone dictates the go-around direction. Both zones at once means the
// nose is in, so the robot backs out rather than picking a side.
inline TurnDirection arcFor(const MissionInputs& in) {
  if (in.bothZones) return TurnDirection::Reverse;
  return in.bumperZone == BumperZone::Left ? TurnDirection::Right : TurnDirection::Left;
}

inline Terrain terrainFor(const MissionInputs& in) {
  if (!in.imuFresh || !linkUsable(in)) return Terrain::Flat;
  const int32_t pitch = in.pitchCentiDeg < 0 ? -in.pitchCentiDeg : in.pitchCentiDeg;
  if (pitch >= MISSION_RAMP_PITCH_MAX_CENTI_DEG) return Terrain::Flat;
  if (pitch >= MISSION_SEESAW_PITCH_MAX_CENTI_DEG) return Terrain::Ramp;
  return pitch > MISSION_FLAT_PITCH_MAX_CENTI_DEG ? Terrain::Seesaw : Terrain::Flat;
}

inline DriveDemand rampStep(const MissionInputs& in) {
  return withBias(DriveMode::LineFollow, MISSION_RAMP_SPEED, lineBias(in.line.error));
}

// Behind the gate the fallback is a gyro-guided slow straight: velocity is held,
// the heading differential is capped and no turn is ever taken.
inline DriveDemand seesawStep(const MissionInputs& in, bool mockVerified, int16_t yawAtEntry) {
  if (mockVerified) {
    return withBias(DriveMode::LineFollow, MISSION_CRUISE_SPEED, lineBias(in.line.error));
  }
  const int32_t assist =
      linkUsable(in) ? headingAssist(static_cast<int32_t>(in.yawCentiDeg) - yawAtEntry) : 0;
  return withBias(DriveMode::GyroStraight, MISSION_SEESAW_SPEED, assist);
}

// Range biases the pass; only a struck zone picks the arc, and with the range
// clear the robot drives on at arc speed while it hunts the same line back.
inline DriveDemand obstacleStep(const MissionInputs& in) {
  if (!in.tofFresh || in.tofMm > MISSION_OBSTACLE_TOF_BIAS_MM) {
    return withBias(DriveMode::Arc, MISSION_OBSTACLE_ARC_SPEED, 0);
  }
  if (!in.bumperContact) {
    return withBias(DriveMode::LineFollow, MISSION_OBSTACLE_BIAS_SPEED, lineBias(in.line.error));
  }
  return pivotStep(arcFor(in), MISSION_OBSTACLE_ARC_SPEED);
}

// --- The zone: the claims the lane is built from ----------------------------

// The bar's half of the entry claim. Silver entrance tape reflects like the
// white floor, so a bar sitting inside the band has no dark channel left
// anywhere: the opposite of the matte black line it just left, and a class the
// centroid cannot manufacture from a line. It is the same coarse class a gap
// gives, which is exactly why this is never a decision on its own.
inline bool barReadsWideBright(const LineReading& line) {
  return !line.lineDetected && line.qualifier == LineClass::ALL_WHITE;
}

// The bar's half of the exit claim: a black band under the bar, which is the
// exit tape with the course line running out of it. Any dark channel qualifies;
// the blob is what says the band is the exit and not the line itself.
inline bool barOnBlackTape(const LineReading& line) { return line.lineDetected; }

// The grey area is everything the entry gate can half-see: a silver-shaped blob
// the bar has not agreed with yet, or a bar already bright with no blob behind
// it. Inside it the robot keeps following the line instead of committing, so a
// single source can never walk it into the zone.
inline bool evacGreyArea(const MissionInputs& in) {
  const bool tapeSeen = in.evacTapeFresh && in.evacTapeType == MISSION_TAPE_SILVER;
  return tapeSeen != barReadsWideBright(in.line);
}

// Entering is the two halves agreeing, and both must be fresh and in range: a
// band too small to be the tape, or a link no longer entitled to be believed,
// is not an entry.
inline bool evacEntryArmed(const MissionInputs& in) {
  if (evacGreyArea(in)) return false;
  if (!linkUsable(in)) return false;
  if (in.evacTapeSize < MISSION_EVAC_TAPE_MIN_SIZE) return false;
  return barReadsWideBright(in.line);
}

// Leaving is the same shape on the black band, cross-validated the same way.
inline bool evacBlackTape(const MissionInputs& in) {
  if (!in.evacTapeFresh || in.evacTapeType != MISSION_TAPE_BLACK) return false;
  if (in.evacTapeSize < MISSION_EVAC_TAPE_MIN_SIZE) return false;
  return barOnBlackTape(in.line);
}

// A fake is never carried and never deposited: the else-branch of the truth
// table is the whole reason the robot probes a victim instead of looking at it.
inline bool evacShouldCarry(VictimClass victim) {
  return victim == VictimClass::Live || victim == VictimClass::Dead;
}

inline DepositTarget evacTargetFor(VictimClass victim) {
  if (victim == VictimClass::Live) return DepositTarget::Green;
  if (victim == VictimClass::Dead) return DepositTarget::Red;
  return DepositTarget::None;
}

// "A victim is here" is a contact claim, made the way every other contact on
// this robot is made: the bumper plate is the never-fail backstop, and a close
// range only corroborates it. Neither is a scored reading — the probes are.
inline bool evacAtProbePoint(const MissionInputs& in) {
  if (in.bumperContact) return true;
  return in.tofFresh && in.tofMm <= MISSION_EVAC_CONTACT_MM;
}

// The triangle a classified victim is carried to is cross-validated as well: a
// fresh blob of that colour. With no target there is nothing to confirm, so the
// no-capture lane is never made to wait for a triangle it will not use.
inline bool evacTargetConfirmed(const MissionInputs& in, DepositTarget target) {
  if (target == DepositTarget::None) return true;
  if (!linkUsable(in)) return false;
  if (!in.triangleFresh) return false;
  if (in.triangleSize < MISSION_EVAC_TRIANGLE_MIN_SIZE) return false;
  return (target == DepositTarget::Green) == (in.triangleColor == MISSION_TRIANGLE_GREEN);
}

// The one place the lane's five budgets live, so a table row and the bail guard
// that reads it can never disagree about how long a state may hold the robot.
inline uint32_t evacResidenceBudgetMs(StateId state) {
  switch (state) {
    case StateId::EvacApproach: return MISSION_EVAC_APPROACH_MAX_MS;
    case StateId::EvacProbe: return MISSION_EVAC_PROBE_MAX_MS;
    case StateId::EvacClassify: return MISSION_EVAC_CLASSIFY_MAX_MS;
    case StateId::EvacDeposit: return MISSION_EVAC_DEPOSIT_MAX_MS;
    case StateId::EvacExit: return MISSION_EVAC_EXIT_MAX_MS;
    default: return RESIDENCE_UNBOUNDED;
  }
}

// --- Arming, in the order reflexes claim a tick ----------------------------

// A fresh red-tape blob AND a fresh close range. Either alone is not a goal, so
// a bare wall never stops the robot and neither does tape with no return.
inline bool goalArmed(const MissionInputs& in) {
  if (!in.goalFresh || !in.tofFresh) return false;
  if (in.goalConfidence < MISSION_GOAL_MIN_CONFIDENCE_PCT) return false;
  return in.goalSize >= MISSION_GOAL_MIN_SIZE && in.tofMm <= MISSION_GOAL_TOF_CLOSE_MM;
}

// The pass presupposes a line to leave, so range only arms it while one is under
// the bar; a struck zone always arms, whether the line is there or not. A close
// range that a red-tape blob corroborates is the goal's own return distance
// rather than something to pass, so the goal is not pre-empted by its evidence.
inline bool obstacleArmed(const MissionInputs& in) {
  if (in.bumperContact) return true;
  if (goalArmed(in)) return false;
  if (!in.tofFresh || in.tofMm > MISSION_OBSTACLE_TOF_BIAS_MM) return false;
  return !lineLost(in.line);
}

// Declared ahead of its definition because the trigger that arms the zone has to
// ask whether the zone is already open; the token itself lives with the lane's
// cross-tick state, below.
inline bool evacLaneActive();

// The zone's only trigger, armable only by the two agreeing sources and only
// from outside the lane: once the token is held the chain is already running,
// so the reflex API can never start a second one behind it.
inline bool evacTrigger(const MissionInputs& in) {
  return !evacLaneActive() && evacEntryArmed(in);
}

inline EventId missionTrigger(const MissionInputs& in) {
  if (obstacleArmed(in)) return MISSION_EVENT_CONTACT;
  if (in.marker == MarkerState::Both) return MISSION_EVENT_DEAD_END;
  // The zone is claimed before the gap reflex, because entering it means the
  // bar is off the black line — the very reading the gap reflex owns.
  if (evacTrigger(in)) return MISSION_EVENT_EVAC;
  if (lineLost(in.line)) return MISSION_EVENT_GAP;
  if (goalArmed(in)) return MISSION_EVENT_GOAL;
  if (in.marker == MarkerState::Left || in.marker == MarkerState::Right) return MISSION_EVENT_MARKER;
  const Terrain terrain = terrainFor(in);
  if (terrain == Terrain::Ramp) return MISSION_EVENT_RAMP;
  return terrain == Terrain::Seesaw ? MISSION_EVENT_SEESAW : EVENT_NONE;
}

// --- Cross-tick state of the live reflex ----------------------------------

// Single-flight by construction: the engine is in one state at a time, so one
// reflex's bookkeeping is all that can ever be live. The table's callbacks are
// the only writers; mission.ino writes inputs and the drive seam.

// What the zone has proved this run. A victim is scored as soon as the probe
// names it, capture or no capture — the classify credit is the rules' and it
// never depends on the mechanism.
struct EvacTally {
  uint8_t live;      // victims the probe proved live
  uint8_t dead;      // ... and dead
  uint8_t ignored;   // fakes, located and rejected
  uint8_t deposits;  // victims actually released at a triangle
};

// One victim, as it is written down. The target is where the rules send this
// class, recorded whether or not the mechanism was live to act on it, so the
// no-capture lane still leaves a trail worth reading back.
struct EvacAccounting {
  uint8_t deposits;  // deposits recorded so far, this one included
  VictimClass victim;
  DepositTarget target;
  bool released;
};

// What must outlive a power cut is the store's own AccountingRecord: the same
// four facts the store serialises, held as the type it serialises them from, so
// the record a restart stages from and the record the resume publishes are the
// same shape with no translation in between — and so nothing can disagree with
// the store about what was persisted.

// The lane's two seams. The delivery layer installs them: the first persists a
// record, the second moves a sphere. Null on either is tolerated, so the
// locate/classify lane works with no mechanism wired up at all.
typedef void (*EvacAccountingFn)(const EvacAccounting& record, void* ctx);
typedef void (*EvacReleaseFn)(DepositTarget target, void* ctx);

struct MissionState {
  MissionInputs inputs;
  DriveDemand demand;
  int32_t heldCorrection;  // the PD output carried across a gap
  int32_t previousError;
  int16_t yawAtEntry;
  EventId suppressed;  // the trigger this reflex is holding a claim on
  uint8_t clearSamples;
  bool seesawMockVerified = MISSION_SEESAW_MOCK_VERIFIED;
  // The zone, on the same single-flight argument as the reflex block above.
  ProbeClassifier probe;
  EvacTally tally = EvacTally{0, 0, 0, 0};
  EvacAccounting lastAccounting = EvacAccounting{0, VictimClass::Fake, DepositTarget::None, false};
  VictimClass victim = VictimClass::Fake;
  VictimClass heldVerdict = VictimClass::Fake;
  DepositTarget target = DepositTarget::None;
  uint8_t stableSamples = 0;
  bool mechanismEnabled = MISSION_EVAC_MECHANISM_ENABLED;
  bool entryToken = false;
  bool decisionRecorded = false;
  bool depositPublished = false;
  bool carrying = false;
  // --- Loss of position, across the restart -------------------------------
  // Restaging is one atomic hand-over of the whole persisted record rather than
  // a field-by-field copy, so the expected landmark count the resume is scored
  // against and the accounting the next restart compares itself to are the same
  // numbers, read from the same place. Held here as the parsed primitives the
  // cross-tick work compares, because the state has to be readable per tick
  // while the record itself only crosses at the two boundaries.
  uint8_t restagedVictims = 0;    // victims the interrupted run had counted
  uint8_t restagedDeposits = 0;   // deposits it had released
  uint16_t restagedLandmarks = 0;  // landmarks it had passed: the expected count
  uint16_t reDerivedLandmarks = 0; // landmarks this restart has actually proved
  int16_t resumeLandmarkError = 0; // the same error, latched at the handoff
  uint8_t radarRequests = 0;       // sweeps asked for this restart, capped
  bool restagedPending = false;    // the store said the run was cut short
  bool radarSweepPending = false;  // a request armed and not yet handed to the wire
  bool landmarkClaimHeld = false;  // the landmark currently under the bar
  bool handshakeFaulted = false;   // the READY window closed on nothing
  bool resumePublished = false;    // the ledger has been handed over, once
};

namespace detail {
struct DriveSeam {
  MissionDriveFn fn;
  void* ctx;
};
// The two calls a deposit makes, installed by the delivery layer. Accounting
// first, release second — the order is a property of the lane, not a habit the
// caller has to remember.
struct EvacHooks {
  EvacAccountingFn onAccounting;
  EvacReleaseFn onRelease;
  void* ctx;
};
inline DriveSeam& seam() { static DriveSeam installed{}; return installed; }
inline EvacHooks& evacHooks() { static EvacHooks installed{}; return installed; }
inline MissionState& state() { static MissionState live{}; return live; }
}  // namespace detail

inline MissionState& missionState() { return detail::state(); }

// Install the delivery layer's motor seam; null detaches it.
inline void setMissionDrive(MissionDriveFn fn, void* ctx) {
  detail::seam().fn = fn;
  detail::seam().ctx = ctx;
}
inline void beginMission() { detail::state() = MissionState{}; }
inline void setMissionInputs(const MissionInputs& in) { detail::state().inputs = in; }

// Hand a demand to the installed seam and record it for the sketch's own reads.
inline void missionDeliver(const DriveDemand& demand) {
  detail::state().demand = demand;
  if (detail::seam().fn != nullptr) detail::seam().fn(demand, detail::seam().ctx);
}

// One delivery pass's bookkeeping: keep the last-known PD output while the line
// is under the bar, and release a finished reflex's claim as soon as the
// condition that armed it is gone — so an aborted bridge does not re-arm on the
// same lost line, while the next one is bridged normally.
inline void advanceMission() {
  MissionState& s = detail::state();
  if (s.inputs.line.lineDetected) {
    s.heldCorrection = pdCorrection(s.inputs.line.error, s.previousError);
    s.previousError = s.inputs.line.error;
  }
  if (s.suppressed != EVENT_NONE && missionTrigger(s.inputs) != s.suppressed) s.suppressed = EVENT_NONE;
}

// The one event worth spending this tick on, or EVENT_NONE to spend nothing. An
// open zone owns its own tick: the chain is already running, so nothing may be
// queued behind it.
inline EventId missionPendingTrigger() {
  const MissionState& s = detail::state();
  if (s.entryToken) return EVENT_NONE;
  return s.suppressed == EVENT_NONE ? missionTrigger(s.inputs) : EVENT_NONE;
}

// Entry bookkeeping shared by all seven reflexes: remember the heading the
// element was entered on and claim this reflex's trigger until the condition
// that armed it clears.
inline void missionEnterReflex() {
  MissionState& s = detail::state();
  s.yawAtEntry = s.inputs.imuFresh ? s.inputs.yawCentiDeg : 0;
  s.clearSamples = 0;
  s.suppressed = missionTrigger(s.inputs);
}

// Re-acquisition is counted here so gap and obstacle agree on what "back on the
// line" means: consecutive clean centroid samples, not a single lucky pass.
inline void missionCountClearSamples(bool clear) {
  MissionState& s = detail::state();
  s.clearSamples = clear ? static_cast<uint8_t>(s.clearSamples + 1) : 0;
}

// Idle follows the centroid, and carries the held PD output on when the line is
// gone, so the baseline state can never strand the robot between reflexes.
inline DriveDemand missionIdleDemand() {
  const MissionState& s = detail::state();
  return lineLost(s.inputs.line)
             ? withBias(DriveMode::GapHold, MISSION_GAP_SPEED, s.heldCorrection)
             : withBias(DriveMode::LineFollow, MISSION_CRUISE_SPEED, lineBias(s.inputs.line.error));
}

inline DriveDemand missionStopDemand() { return DriveDemand{DriveMode::Stop, 0, 0}; }

// --- The lane, across ticks -------------------------------------------------
//
// Five states, each with one job: creep to a victim, read it, name it, put it
// where the rules want it, and leave. Every one of them is bounded, and the
// bounds are the ones the table rows carry.

// Install the deposit seams; null on either detaches it, and the lane keeps
// scoring victims with nothing attached.
inline void setEvacHooks(EvacAccountingFn onAccounting, EvacReleaseFn onRelease, void* ctx) {
  detail::evacHooks().onAccounting = onAccounting;
  detail::evacHooks().onRelease = onRelease;
  detail::evacHooks().ctx = ctx;
}

// Whether the zone is open right now. The token is the proof it was entered on
// two agreeing sources, and while it is held the lane owns the tick: no second
// chain, and no other reflex borrowing the claim.
inline bool evacLaneActive() { return detail::state().entryToken; }
inline const EvacTally& evacTally() { return detail::state().tally; }
inline const EvacAccounting& evacLastAccounting() { return detail::state().lastAccounting; }

// Entering mints the token, and only ever on the two-source claim: the entry
// bookkeeping run against an unarmed snapshot is a no-op, so no other route to
// the token exists and the trigger can never be spent without it. It is also a
// no-op on an open lane, so the five shared on_entry callbacks cannot re-stamp
// the heading the zone is being steered on every time the chain steps along.
inline void missionEnterEvac() {
  MissionState& s = detail::state();
  if (s.entryToken) return;
  if (!evacEntryArmed(s.inputs)) return;
  s.entryToken = true;
  s.yawAtEntry = s.inputs.imuFresh ? s.inputs.yawCentiDeg : 0;
  s.suppressed = MISSION_EVENT_EVAC;
}

// Ending the zone, called by the rows that hand the robot back to Idle. Wipes the
// per-lane bookkeeping, so a second pass through the zone starts from a clean
// slate and a bail out of the middle of the chain does not deafen the robot to
// the zone for the rest of the run.
inline void missionLeaveEvac() {
  MissionState& s = detail::state();
  s.entryToken = false;
  s.suppressed = EVENT_NONE;
  s.probe.reset();
  s.victim = VictimClass::Fake;
  s.heldVerdict = VictimClass::Fake;
  s.target = DepositTarget::None;
  s.stableSamples = 0;
  s.decisionRecorded = false;
  s.depositPublished = false;
  s.carrying = false;
}

// One probe sample per pass, folded into the held classifier so the surface
// verdict survives between passes. A class is named only once the window has
// held the same reading repeatedly: a victim that flickers between signatures
// is a fault, not a victim, and the state times out rather than naming it.
inline void missionEvacProbeTick(const MissionInputs& in) {
  MissionState& s = detail::state();
  const VictimClass verdict = s.probe.classify(in.reflectivity, in.conductive);
  s.stableSamples = verdict == s.heldVerdict ? static_cast<uint8_t>(s.stableSamples + 1) : 1;
  s.heldVerdict = verdict;
  if (s.stableSamples >= MISSION_EVAC_PROBE_SETTLE_SAMPLES) s.victim = verdict;
}
inline bool evacProbeSettled() {
  return detail::state().stableSamples >= MISSION_EVAC_PROBE_SETTLE_SAMPLES;
}

// Naming the class is a one-shot decision. A fake is ignored outright; a live or
// dead victim is carried when the mechanism is live; and either way the
// classification is scored, because that credit is the rules' and must not
// depend on a cup that may never be fitted.
inline void missionEvacDecide() {
  MissionState& s = detail::state();
  if (s.decisionRecorded) return;
  s.decisionRecorded = true;
  s.carrying = s.mechanismEnabled && evacShouldCarry(s.victim);
  s.target = s.carrying ? evacTargetFor(s.victim) : DepositTarget::None;
  if (s.victim == VictimClass::Live) ++s.tally.live;
  else if (s.victim == VictimClass::Dead) ++s.tally.dead;
  else ++s.tally.ignored;
}

// Decided, and its target corroborated — or no target to corroborate, which is
// the no-capture lane and a fake, and neither may wait on a triangle blob.
inline bool evacClassReady() {
  const MissionState& s = detail::state();
  return s.decisionRecorded && evacTargetConfirmed(s.inputs, s.target);
}

// The deposit, in the only order it is allowed to happen: the accounting is
// written first and the release second, in this body rather than in the caller's
// hands. A power cut between the two can leave a recorded deposit with the
// sphere still in the cup; it can never leave a released sphere with nothing
// written down. One call publishes one deposit, whatever the state does
// afterwards.
inline void missionEvacDeposit() {
  MissionState& s = detail::state();
  if (s.depositPublished) return;
  s.depositPublished = true;
  const bool released = s.carrying;
  const uint8_t deposits = static_cast<uint8_t>(s.tally.deposits + (released ? 1 : 0));
  const EvacAccounting record{deposits, s.victim, evacTargetFor(s.victim), released};
  if (released) ++s.tally.deposits;
  s.lastAccounting = record;
  if (detail::evacHooks().onAccounting != nullptr) {
    detail::evacHooks().onAccounting(record, detail::evacHooks().ctx);
  }
  if (released && detail::evacHooks().onRelease != nullptr) {
    detail::evacHooks().onRelease(record.target, detail::evacHooks().ctx);
  }
}
inline bool evacDepositSettled() { return detail::state().depositPublished; }

// The search creeps: slowly enough to nudge a sphere rather than launch it, and
// slower still with a victim in the cup, because a carried off-centre sphere
// rolls on a corner and a fast approach costs the deposit. Off the line it is a
// gyro-guided slow straight, and a degraded link buys no heading at all.
inline DriveDemand evacApproachStep(const MissionState& state) {
  const int speed = state.carrying ? MISSION_EVAC_CARRY_SPEED : MISSION_EVAC_SEARCH_SPEED;
  if (!lineLost(state.inputs.line)) {
    return withBias(DriveMode::LineFollow, speed, lineBias(state.inputs.line.error));
  }
  const bool heading = linkUsable(state.inputs) && state.inputs.imuFresh;
  const int32_t assist = heading ? headingAssist(static_cast<int32_t>(state.inputs.yawCentiDeg) -
                                                 state.yawAtEntry)
                                 : 0;
  return withBias(DriveMode::GyroStraight, speed, assist);
}

// Backing out first: the sphere and the zone's own wall are both behind the
// plate, so the exit reverses until the bar finds black and then follows the
// tape away. The no-capture lane runs this identically — an empty robot has no
// reason to wait for anything.
inline DriveDemand evacExitStep(const MissionState& state) {
  if (barOnBlackTape(state.inputs.line)) {
    return withBias(DriveMode::LineFollow, MISSION_EVAC_EXIT_SPEED,
                    lineBias(state.inputs.line.error));
  }
  return withBias(DriveMode::GyroStraight, -MISSION_EVAC_BACKOUT_SPEED, 0);
}

// --- Loss of position: the restart path -------------------------------------
//
// Four states, each with one job and one bound. The handshake holds the robot
// still for its window and then gives up on the LINK, never on the run; the
// sweep state asks for the concurrent radar sweep and gets out of the way; the
// search is a bounded ladder of pivots the bar hunts the line with; and the
// resume either puts the line back under the bar or hands the run back on
// purpose. Nothing in this section can wait, and nothing in it can be held up by
// the radar: the sweep is evidence like any other, and a stale one is simply
// absent.

// The handshake settled on the first READY. It is sticky on the wire, so there
// is exactly one of these per run and no other shape it can arrive in.
inline bool lopHandshakeSettled(const MissionInputs& in) { return in.readyFresh; }

// The one place the four budgets live, so a table row and the bail guard that
// reads it can never disagree about how long a restart state may hold the robot.
inline uint32_t lopResidenceBudgetMs(StateId state) {
  switch (state) {
    case StateId::LoPLost: return MISSION_LOP_READY_WAIT_MS;
    case StateId::LoPRadarAssist: return MISSION_LOP_SWEEP_ARM_MS;
    case StateId::LoPReacquire: return MISSION_LOP_SEARCH_MAX_MS;
    case StateId::LoPResume: return MISSION_LOP_RESUME_MAX_MS;
    default: return RESIDENCE_UNBOUNDED;
  }
}

// What the sweep is allowed to contribute: a first direction, or nothing. Only
// an open profile names one — enclosed says every sector is close, so "the most
// open of them" names nothing — and ambiguous is the profile that asked for a
// second look, not a direction. A sweep the link is not entitled to be believed
// about is not a sweep at all.
inline SearchBias lopSearchBias(const MissionInputs& in) {
  if (!in.radarFresh || !linkUsable(in)) return SearchBias::Alternating;
  if (in.radarClass != MISSION_RADAR_OPEN) return SearchBias::Alternating;
  if (in.radarZone > MISSION_RADAR_LAST_RIGHT) return SearchBias::Alternating;
  if (in.radarZone == MISSION_RADAR_ASTERN_SECTOR) return SearchBias::Alternating;
  if (in.radarZone >= MISSION_RADAR_FIRST_RIGHT) return SearchBias::Right;
  if (in.radarZone >= MISSION_RADAR_FIRST_LEFT) return SearchBias::Left;
  return SearchBias::Straight;
}

// One rung of the ladder, and the only rung the sweep may choose. A long
// residence cannot index past the last rung: a search that has been hunting
// longer than its budget still asks for the last leg, not for a fifth.
inline uint8_t lopSearchLegIndex(uint32_t residenceMs) {
  uint32_t leg = residenceMs / MISSION_LOP_SEARCH_LEG_MS;
  if (leg >= MISSION_LOP_SEARCH_LEGS) leg = MISSION_LOP_SEARCH_LEGS - 1;
  return static_cast<uint8_t>(leg);
}

// Every leg after the first alternates, so a search the sweep never answered
// still sweeps both sides rather than committing to one.
inline TurnDirection lopSearchTurn(const MissionInputs& in, uint32_t residenceMs) {
  const uint8_t leg = lopSearchLegIndex(residenceMs);
  if (leg != 0) return leg % 2 == 1 ? TurnDirection::Right : TurnDirection::Left;
  switch (lopSearchBias(in)) {
    case SearchBias::Left: return TurnDirection::Left;
    case SearchBias::Right: return TurnDirection::Right;
    case SearchBias::Straight: return TurnDirection::Straight;
    default: return TurnDirection::Left;
  }
}

// The search is the bar's work, so the bar's reading outranks the sweep's: a
// clean centroid follows the line even mid-ladder, and the sweep is what the
// demand falls back to when there is no line to follow.
inline DriveDemand lopSearchStep(const MissionState& state, uint32_t residenceMs) {
  const TurnDirection turn = lopSearchTurn(state.inputs, residenceMs);
  if (onLineClear(state.inputs.line) || turn == TurnDirection::Straight) {
    return withBias(DriveMode::LineFollow, MISSION_LOP_SEARCH_ARC_SPEED,
                    lineBias(state.inputs.line.error));
  }
  return pivotStep(turn, MISSION_LOP_SEARCH_PIVOT_SPEED);
}

// Re-acquired, and re-proved: one lucky pass over an edge is not a line back.
inline bool lopSearchClear(const MissionState& state) {
  return onLineClear(state.inputs.line) && state.clearSamples >= MISSION_LOP_CLEAR_SAMPLES;
}

// The resume is the bar driving again, at resume speed. With no line under it
// there is nothing to follow and nothing to correct from, so it holds the last
// known output rather than inventing one.
inline DriveDemand lopResumeStep(const MissionState& state) {
  if (!onLineClear(state.inputs.line)) {
    return withBias(DriveMode::GapHold, MISSION_GAP_SPEED, state.heldCorrection);
  }
  return withBias(DriveMode::LineFollow, MISSION_LOP_RESUME_SPEED,
                  lineBias(state.inputs.line.error));
}
inline bool lopResumeReady(const MissionState& state) {
  return onLineClear(state.inputs.line) && state.clearSamples >= MISSION_LOP_CLEAR_SAMPLES;
}

// --- The restart's own cross-tick state -------------------------------------

inline bool lopResumeRequired() { return detail::state().restagedPending; }
inline uint16_t lopExpectedLandmarks() { return detail::state().restagedLandmarks; }
inline uint16_t lopReDerivedLandmarks() { return detail::state().reDerivedLandmarks; }
inline bool lopHandshakeFaulted() { return detail::state().handshakeFaulted; }

// The whole persisted record in, in one call. Two things are staged, not one:
// the counts the interrupted run had recorded, and whether it was actually cut
// short. The second is what arms the restart, because a completed run leaves a
// ledger behind it and a blank device leaves none — neither is a power cut.
inline void beginLopRestage(const AccountingRecord& record) {
  MissionState& s = detail::state();
  s.restagedVictims = record.victims_counted;
  s.restagedDeposits = record.deposits_placed;
  s.restagedLandmarks = record.landmark_counter;
  s.reDerivedLandmarks = record.landmark_counter;
  s.restagedPending = record.loop_pending;
  s.resumeLandmarkError = 0;
  s.radarRequests = 0;
  s.radarSweepPending = false;
  s.landmarkClaimHeld = false;
  s.handshakeFaulted = false;
  s.resumePublished = false;
}

// A landmark is a marker on one side of the bar or a strike the plate latched:
// something the centroid cannot mistake for the next patch of line. One crossing
// counts once, however many passes it stays under the bar, and a bar sitting on
// an edge has claimed nothing at all.
inline bool lopLandmarkClaimed(const MissionInputs& in) {
  if (!onLineClear(in.line)) return false;
  if (in.marker != MarkerState::None) return true;
  return in.bumperContact;
}

inline void missionLopTrackLandmark(const MissionInputs& in) {
  MissionState& s = detail::state();
  const bool claimed = lopLandmarkClaimed(in);
  if (claimed && !s.landmarkClaimHeld) ++s.reDerivedLandmarks;
  s.landmarkClaimHeld = claimed;
}

// How far the restart's own count has drifted from the record it started from.
// Signed, so a restart that proved fewer landmarks than the record claims reads
// as a negative error rather than a large positive one. While the restart runs
// this is that comparison; afterwards it is the same number latched at the
// handoff, because a completed restart has no expected count left to be wrong
// about and its error is a thing to read, not to keep recomputing.
inline int16_t lopLandmarkError() {
  const MissionState& s = detail::state();
  if (s.restagedPending) return static_cast<int16_t>(s.reDerivedLandmarks - s.restagedLandmarks);
  return s.resumeLandmarkError;
}
inline bool lopLandmarkErrorWithinBound() {
  const int16_t error = lopLandmarkError();
  const int16_t bound = MISSION_LOP_LANDMARK_ERROR_BOUND;
  return error <= bound && error >= -bound;
}

// What leaves the robot: the counts the interrupted run had recorded, the
// landmark count this restart actually proved, and a run no longer pending — so
// the next cut is compared against what was established rather than against what
// was assumed. Never narrower than the record it came from: a restart may find a
// landmark the record had not counted, never erase one it had.
inline AccountingRecord lopReDerivedAccounting() {
  const MissionState& s = detail::state();
  AccountingRecord record = freshAccounting();
  record.victims_counted = s.restagedVictims;
  record.deposits_placed = s.restagedDeposits;
  record.landmark_counter =
      s.reDerivedLandmarks < s.restagedLandmarks ? s.restagedLandmarks
                                                  : s.reDerivedLandmarks;
  return record;
}

// Handing the ledger over is one-shot, and reachable from both ways out of the
// resume, so a search that re-centred and one that deliberately gave up leave
// the same trail behind them.
inline bool missionLopResumeComplete() {
  MissionState& s = detail::state();
  if (s.resumePublished) return false;
  s.resumePublished = true;
  s.resumeLandmarkError = lopLandmarkError();
  s.restagedPending = false;
  s.restagedLandmarks = 0;
  return true;
}
// Whether the resume has handed the ledger over. The delivery layer reads this
// once per pass and owns the latch that turns it into a single store write, so
// no EEPROM call ever sits on a table row.
inline bool lopLedgerPublished() { return detail::state().resumePublished; }

// --- The sweep ledger --------------------------------------------------------
//
// A cap, not a queue: the restart asks once, and at most once more for the
// profile that asked for it. An armed request is handed to the wire by the
// delivery layer and marked sent, so an ask that never went out still spends its
// budget rather than being retried into a storm.

inline uint8_t lopRadarRequests() { return detail::state().radarRequests; }
inline bool lopRadarRequestPending() { return detail::state().radarSweepPending; }
inline bool lopRadarRequestAllowed(const MissionState& state) {
  return state.radarRequests < MISSION_LOP_RADAR_REQUEST_CAP;
}

inline bool missionLopArmSweep() {
  MissionState& s = detail::state();
  if (!lopRadarRequestAllowed(s)) return false;
  ++s.radarRequests;
  s.radarSweepPending = true;
  return true;
}
inline void lopRadarRequestSent() { detail::state().radarSweepPending = false; }
inline bool lopSweepArmed() { return detail::state().radarRequests > 0; }

// The only profile worth a second look, and only while it is the live reading.
inline bool lopReSweepWanted(const MissionInputs& in) {
  return in.radarFresh && linkUsable(in) && in.radarClass == MISSION_RADAR_AMBIGUOUS;
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_MISSION_H
