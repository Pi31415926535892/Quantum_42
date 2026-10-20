// mission_test.cpp — host-side tests for the navigation reflex module.
//
// The reflex decisions live in mission.h as pure functions over a snapshot, and
// the table that runs them lives in fsm.ino, so this suite includes fsm.ino
// itself: every reflex case below is driven through the SHIPPED table, the
// SHIPPED glue callbacks and the real event queue, not a copy. The two delivery
// entry points the sketch calls are stubbed, because the loop body is what the
// tests are standing in for.
//
// Run: ./test/run_tests.sh

#include <cstdio>

// The sketch's setup()/loop() call these; the pass is driven by hand instead.
void missionBegin() {}
void missionTick() {}

#include "fsm.ino"

using namespace rescue;

namespace {

int g_checks = 0;
int g_failures = 0;

void reportFailure(const char* what, int line) {
  ++g_failures;
  std::printf("FAIL line %d: %s\n", line, what);
}

void check(bool ok, const char* what, int line) {
  ++g_checks;
  if (!ok) reportFailure(what, line);
}

template <typename A, typename B>
void checkEq(const A& actual, const B& expected, const char* what, int line) {
  ++g_checks;
  if (!(actual == expected)) reportFailure(what, line);
}

}  // namespace

#define CHECK(cond) check((cond), #cond, __LINE__)
#define CHECK_EQ(actual, expected) checkEq((actual), (expected), #actual, __LINE__)

namespace {

// How many reflex events the table's Idle row is expected to arm. The zone's own
// entry is a row too, but it is not a reflex: it carries a guard and it is
// proved in evac_test.cpp, which is why this count and this file's reflex loop
// both stop short of it.
constexpr uint8_t kReflexEventCount = 7;
constexpr uint8_t kReflexStateCount = 7;  // rows 2..8, before the zone's five

// --- Snapshots -------------------------------------------------------------
// A clean pass: the bar is centred and reading the line, no markers, no link
// frames, nothing struck. Every builder starts from this, so a test only has to
// state the one thing it is about.

MissionInputs cleanInputs() {
  MissionInputs in{};
  in.line.position = 0;
  in.line.error = 0;
  in.line.lineDetected = true;
  in.line.qualifier = LineClass::ON_LINE;
  return in;
}

MissionInputs onLine(int32_t error = 0) {
  MissionInputs in = cleanInputs();
  in.line.error = error;
  return in;
}

MissionInputs offLine() {
  MissionInputs in = cleanInputs();
  in.line.lineDetected = false;
  in.line.qualifier = LineClass::ALL_WHITE;
  return in;
}

MissionInputs lineAtEnd() {
  MissionInputs in = onLine();
  in.line.qualifier = LineClass::EDGE;
  return in;
}

MissionInputs withMarker(MarkerState marker) {
  MissionInputs in = cleanInputs();
  in.marker = marker;
  return in;
}

MissionInputs withToF(uint16_t mm) {
  MissionInputs in = cleanInputs();
  in.tofFresh = true;
  in.tofMm = mm;
  return in;
}

MissionInputs withPitch(int16_t centiDeg, int32_t error = 0) {
  MissionInputs in = onLine(error);
  in.imuFresh = true;
  in.pitchCentiDeg = centiDeg;
  return in;
}

MissionInputs withYaw(int16_t centiDeg) {
  MissionInputs in = cleanInputs();
  in.imuFresh = true;
  in.yawCentiDeg = centiDeg;
  return in;
}

MissionInputs withStrike(BumperZone zone) {
  MissionInputs in = cleanInputs();
  in.bumperContact = true;
  in.bumperZone = zone;
  return in;
}

MissionInputs withBothStruck() {
  MissionInputs in = withStrike(BumperZone::Left);
  in.bothZones = true;
  return in;
}

MissionInputs withGoal(uint8_t confidence, uint16_t size) {
  MissionInputs in = cleanInputs();
  in.goalFresh = true;
  in.goalConfidence = confidence;
  in.goalSize = size;
  return in;
}

MissionInputs goalInput() {
  MissionInputs in = withGoal(MISSION_GOAL_MIN_CONFIDENCE_PCT, MISSION_GOAL_MIN_SIZE);
  in.tofFresh = true;
  in.tofMm = MISSION_GOAL_TOF_CLOSE_MM;
  return in;
}

MissionInputs degraded(MissionInputs in) {
  in.policy = LinkPolicy::DegradeToLineOnly;
  return in;
}

// --- The rig ---------------------------------------------------------------
// One delivery pass in the order mission.ino performs it: the snapshot lands,
// the bookkeeping runs, at most one reflex event is queued, then the engine gets
// its single tick. The drive seam records what the reflex layer asked for.

struct Rig {
  FsmEngine engine;
  uint32_t stepMs;
  uint32_t passes;
  DriveDemand demand;
  int demandCount;

  Rig()
      : engine(kStateTable, kStateCount), stepMs(10), passes(0), demand(), demandCount(0) {
    setMissionDrive(&Rig::record, this);
    beginMission();
    engine.begin(StateId::Boot);
    // Boot's row is an EVENT_ANY, and the engine deliberately skips EVENT_ANY on
    // an entry tick, so the handoff happens on the second pass — the same two
    // passes a real sketch spends on it.
    engine.tick(stepMs, EVENT_NONE);
    engine.tick(stepMs, EVENT_NONE);
  }

  ~Rig() { setMissionDrive(nullptr, nullptr); }

  static void record(const DriveDemand& d, void* ctx) {
    Rig* self = static_cast<Rig*>(ctx);
    self->demand = d;
    ++self->demandCount;
  }

  void feed(const MissionInputs& in) {
    ++passes;
    setMissionInputs(in);
    advanceMission();
    engine.postEvent(missionPendingTrigger());
    engine.tick(stepMs, EVENT_NONE);
  }

  // Feed the same reading until at least `limitMs` of residence has gone by.
  void feedFor(const MissionInputs& in, uint32_t limitMs) {
    while (engine.residenceMs() < limitMs) feed(in);
  }

  // A reflex's action runs on the pass AFTER the one that entered it, because
  // the engine runs on_action before it resolves a row. This is that extra
  // pass, on the same snapshot, so a test can read what a reflex asked for
  // without counting engine ticks itself.
  void settle(const MissionInputs& in) {
    feed(in);
    feed(in);
  }

  StateId state() const { return engine.state(); }
  DriveDemand last() const { return missionState().demand; }
};

// --- The table itself ------------------------------------------------------

void testTableCoversEveryStateAndBoundsEveryReflex() {
  CHECK_EQ(kStateCount, STATE_REGISTRY_SIZE);
  for (uint8_t index = 0; index < kStateCount; ++index) {
    CHECK(kStateTable[index].id == static_cast<StateId>(index));
  }

  // Boot hands off to Idle, and Idle arms one reflex event per row, plus the
  // zone's entry, which evac_test.cpp owns.
  CHECK(kStateTable[0].transitions[0].next == StateId::Idle);
  CHECK_EQ(kStateTable[1].transition_count, kReflexEventCount + 1);
  for (uint8_t row = 0; row < kReflexEventCount; ++row) {
    const Transition& idle = kStateTable[1].transitions[row];
    CHECK(idle.event != EVENT_NONE && idle.event != EVENT_ANY);
    CHECK(idle.next != StateId::Idle && idle.next != StateId::Boot);
    for (uint8_t other = row + 1; other < kReflexEventCount; ++other) {
      CHECK(kStateTable[1].transitions[other].event != idle.event);
    }
  }

  // Every reflex row: one EVENT_ANY way back to Idle, a finite budget, the
  // shared entry, its own per-tick action, and the motor backstop underneath.
  // The loop stops at the zone's five rows, whose shape is the chain's, not a
  // reflex's.
  for (uint8_t index = 2; index < 2 + kReflexStateCount; ++index) {
    const StateDef& row = kStateTable[index];
    CHECK_EQ(row.transition_count, 1);
    CHECK_EQ(row.transitions[0].event, EVENT_ANY);
    CHECK(row.transitions[0].next == StateId::Idle);
    CHECK(row.transitions[0].guard == reflexDone);
    CHECK(row.max_residence_ms != RESIDENCE_UNBOUNDED);
    CHECK(row.on_entry == reflexEnter);
    CHECK(row.on_action == reflexRun);
    CHECK(row.on_timeout == reflexBackstop);
  }

  // Each row is bounded by the SAME budget its own guard compares against, so
  // the guard normally wins the race and on_timeout is only the net under it.
  // A row wired to another reflex's budget would exit late or early.
  CHECK(kStateTable[2].max_residence_ms == MISSION_GAP_MAX_MS);
  CHECK(kStateTable[3].max_residence_ms == MISSION_MARKER_TURN_MS);
  CHECK(kStateTable[4].max_residence_ms == MISSION_DEAD_END_PIVOT_MS);
  CHECK(kStateTable[5].max_residence_ms == MISSION_RAMP_MAX_MS);
  CHECK(kStateTable[6].max_residence_ms == MISSION_SEESAW_CROSS_MS);
  CHECK(kStateTable[7].max_residence_ms == MISSION_OBSTACLE_MAX_MS);
  CHECK(kStateTable[8].max_residence_ms == MISSION_GOAL_HOLD_MS);
  // And the backstop is the one number none of them may reach on their own.
  CHECK(MISSION_GOAL_HOLD_MS > MISSION_STATE_BACKSTOP_MS);
}

void testBootHandsOffToIdleThroughTheSketchEntryPoints() {
  fsmBegin();
  fsmTick();
  CHECK(g_fsm.state() == StateId::Boot);  // an entry tick spends no EVENT_ANY
  fsmTick();
  CHECK(g_fsm.state() == StateId::Idle);

  // One queued event is spent by exactly one tick, and the reflex it entered
  // leaves only when its own guard says so.
  CHECK(fsmPostEvent(MISSION_EVENT_GAP));
  fsmTick();
  CHECK(g_fsm.state() == StateId::GapBridging);
  fsmTick();
  CHECK_EQ(g_fsm.queuedEvents(), 0);
  CHECK(g_fsm.state() == StateId::GapBridging);

  // A quiet tick posts nothing at all, so the ring can never back up.
  setMissionInputs(onLine());
  CHECK_EQ(missionPendingTrigger(), EVENT_NONE);
  CHECK(!fsmPostEvent(missionPendingTrigger()));
}

// --- The baseline state ----------------------------------------------------

void testIdleFollowsTheCentroidAndCarriesTheLineWhenItGoes() {
  Rig rig;
  rig.feed(onLine(300));
  CHECK(rig.state() == StateId::Idle);
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_CRUISE_SPEED + steerFor(300).left);
  CHECK_EQ(rig.last().right, MISSION_CRUISE_SPEED - steerFor(300).left);

  // On the very pass the line drops away, the baseline state is already
  // carrying the last-known PD output, so the handoff into the gap reflex has
  // no silent zero-drive tick in it.
  const int32_t held = missionState().heldCorrection;
  rig.feed(offLine());
  CHECK(rig.state() == StateId::GapBridging);
  CHECK(rig.last().mode == DriveMode::GapHold);
  CHECK_EQ(rig.last().left, MISSION_GAP_SPEED + held);

  // And the correction is frozen from there: a bar that sees nothing cannot
  // update it, which is the whole point of holding it.
  rig.feed(offLine());
  CHECK_EQ(missionState().heldCorrection, held);
}

// --- Gap -------------------------------------------------------------------

void testGapHoldsTheLastKnownOutputAndLeavesOnTheClearLine() {
  Rig rig;
  rig.feed(onLine(200));
  const int32_t held = missionState().heldCorrection;
  CHECK_EQ(held, pdCorrection(200, 0));

  rig.feed(offLine());
  CHECK(rig.state() == StateId::GapBridging);
  rig.feed(offLine());
  CHECK(rig.last().mode == DriveMode::GapHold);
  CHECK_EQ(rig.last().left, MISSION_GAP_SPEED + held);
  CHECK_EQ(rig.last().right, MISSION_GAP_SPEED - held);

  // Two clean centroid samples, not one lucky pass, and it is back on the line.
  rig.feed(onLine(0));
  CHECK(rig.state() == StateId::GapBridging);
  rig.feed(onLine(0));
  CHECK(rig.state() == StateId::Idle);
}

void testGapAddsTheYawAssistOnlyWhileTheLinkIsUsable() {
  // A yaw error only means anything on a link that may still be used.
  MissionInputs entry = withYaw(0);
  entry.line = offLine().line;  // entered straight-in, as the contract requires
  MissionInputs turned = withYaw(900);
  turned.line = offLine().line;

  Rig rig;
  rig.feed(onLine(0));
  rig.feed(entry);
  CHECK(rig.state() == StateId::GapBridging);

  rig.settle(turned);
  const int32_t held = missionState().heldCorrection;
  const int32_t assist = headingAssist(900);
  CHECK(assist > 0);
  CHECK_EQ(rig.last().left, MISSION_GAP_SPEED + held + assist);
  CHECK_EQ(rig.last().right, MISSION_GAP_SPEED - held - assist);

  // The same yaw error under DegradeToLineOnly buys no assist at all, because
  // the robot is no longer entitled to use a link-sourced number.
  rig.settle(degraded(turned));
  CHECK_EQ(rig.last().left, MISSION_GAP_SPEED + held);
  CHECK_EQ(rig.last().right, MISSION_GAP_SPEED - held);

  // The assist is capped, so it can never grow into a path turn.
  CHECK_EQ(headingAssist(100000), MISSION_HEADING_ASSIST_MAX);
  CHECK_EQ(headingAssist(-100000), -MISSION_HEADING_ASSIST_MAX);
}

void testGapAbortsOnItsBudgetAndWaitsForTheConditionToClearBeforeReArming() {
  Rig rig;
  rig.feed(offLine());
  CHECK(rig.state() == StateId::GapBridging);
  rig.feedFor(offLine(), MISSION_GAP_MAX_MS);
  CHECK(rig.state() == StateId::Idle);

  // The line is still lost, so the same gap must NOT be re-entered: the claim
  // on that trigger is what stops a wedged reflex from looping forever.
  rig.feed(offLine());
  rig.feed(offLine());
  CHECK(rig.state() == StateId::Idle);
  CHECK_EQ(missionPendingTrigger(), EVENT_NONE);

  // Once the line comes back the claim is released, and the next gap is
  // bridged normally.
  rig.feed(onLine(0));
  CHECK_EQ(missionPendingTrigger(), EVENT_NONE);
  rig.feed(offLine());
  CHECK(rig.state() == StateId::GapBridging);
}

// --- Markers and the dead end ---------------------------------------------

void testMarkerTurnsTowardTheGreenSide() {
  CHECK(turnForMarker(withMarker(MarkerState::Left)) == TurnDirection::Left);
  CHECK(turnForMarker(withMarker(MarkerState::Right)) == TurnDirection::Right);
  // Both sides is not a turn: it is the dead end's job, so the marker reflex
  // never claims it.
  CHECK(turnForMarker(withMarker(MarkerState::Both)) == TurnDirection::Straight);
  CHECK(turnForMarker(withMarker(MarkerState::None)) == TurnDirection::Straight);
  CHECK(missionTrigger(withMarker(MarkerState::Both)) == MISSION_EVENT_DEAD_END);
  CHECK(missionTrigger(withMarker(MarkerState::Left)) == MISSION_EVENT_MARKER);

  Rig rig;
  rig.feed(withMarker(MarkerState::Left));
  CHECK(rig.state() == StateId::MarkerTurn);
  rig.feed(withMarker(MarkerState::Left));
  CHECK(rig.last().mode == DriveMode::Pivot);
  CHECK_EQ(rig.last().left, MISSION_TURN_SPEED);
  CHECK_EQ(rig.last().right, -MISSION_TURN_SPEED);

  rig.feedFor(withMarker(MarkerState::Left), MISSION_MARKER_TURN_MS);
  CHECK(rig.state() == StateId::Idle);
}

void testDeadEndPivotsOneHundredEighty() {
  Rig rig;
  rig.feed(withMarker(MarkerState::Both));
  CHECK(rig.state() == StateId::DeadEnd);
  rig.feed(withMarker(MarkerState::Both));
  CHECK(rig.last().mode == DriveMode::Pivot);
  CHECK_EQ(rig.last().left, MISSION_TURN_SPEED);
  CHECK_EQ(rig.last().right, -MISSION_TURN_SPEED);

  // It holds the pivot for its own longer budget, not the marker's.
  rig.feedFor(withMarker(MarkerState::Both), MISSION_MARKER_TURN_MS);
  CHECK(rig.state() == StateId::DeadEnd);
  rig.feedFor(withMarker(MarkerState::Both), MISSION_DEAD_END_PIVOT_MS);
  CHECK(rig.state() == StateId::Idle);
}

// --- Ramps and seesaws -----------------------------------------------------

void testRampKeepsTheBarPrimaryAndLeavesAtTheCrest() {
  const int32_t bias = steerFor(400).left;
  Rig rig;
  rig.settle(withPitch(2200, 400));
  CHECK(rig.state() == StateId::RampClimb);
  // The centroid still steers; only the base speed drops for the climb.
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_RAMP_SPEED + bias);
  CHECK_EQ(rig.last().right, MISSION_RAMP_SPEED - bias);

  // The crest reads as flat again, so the reflex hands back without waiting.
  rig.feed(onLine(400));
  CHECK(rig.state() == StateId::Idle);
}

void testPitchBandPicksTheTerrainAndNeverSeesawOnALevelFloor() {
  CHECK(terrainFor(withPitch(0)) == Terrain::Flat);
  CHECK(terrainFor(withPitch(MISSION_FLAT_PITCH_MAX_CENTI_DEG)) == Terrain::Flat);
  CHECK(terrainFor(withPitch(1500)) == Terrain::Seesaw);
  CHECK(terrainFor(withPitch(-1500)) == Terrain::Seesaw);
  CHECK(terrainFor(withPitch(2000)) == Terrain::Ramp);
  CHECK(terrainFor(withPitch(2500)) == Terrain::Flat);
  // No IMU, or no link to trust, means no terrain claim: flat is the safe read.
  CHECK(terrainFor(cleanInputs()) == Terrain::Flat);
  CHECK(terrainFor(degraded(withPitch(2200))) == Terrain::Flat);
  CHECK(lineLost(offLine().line));
  CHECK(lineLost(lineAtEnd().line));
  CHECK(!lineLost(onLine().line));
}

void testSeesawRunsOnlyTheCarveOutUntilTheMockIsVerified() {
  Rig rig;
  rig.feed(withPitch(1500));
  CHECK(rig.state() == StateId::SeesawCross);
  rig.feed(withPitch(1500));
  // The gate is shut, so the only live strategy is a gyro-guided slow straight:
  // a capped differential and no turn of any kind.
  CHECK(rig.last().mode == DriveMode::GyroStraight);
  CHECK_EQ(rig.last().left, MISSION_SEESAW_SPEED);
  CHECK_EQ(rig.last().right, MISSION_SEESAW_SPEED);
  CHECK(rig.last().mode != DriveMode::Pivot);

  // With the evidence recorded, the mock-verify strategy is the one that runs.
  missionState().seesawMockVerified = true;
  rig.feed(withPitch(1500));
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_CRUISE_SPEED + steerFor(0).left);

  rig.feedFor(withPitch(1500), MISSION_SEESAW_CROSS_MS);
  CHECK(rig.state() == StateId::Idle);
}

// --- Obstacles -------------------------------------------------------------

void testObstacleArmsOnRangeOrOnAStrike() {
  // A pass presupposes a line to leave, so range alone arms nothing off-line.
  CHECK(obstacleArmed(withToF(300)));
  CHECK(!obstacleArmed(withToF(500)));
  CHECK(!obstacleArmed(offLine())); 
  MissionInputs offAndClose = offLine();
  offAndClose.tofFresh = true;
  offAndClose.tofMm = 300;
  CHECK(!obstacleArmed(offAndClose));
  // A struck zone always arms, line or no line.
  MissionInputs struckOff = withStrike(BumperZone::Left);
  struckOff.line = offLine().line;
  CHECK(obstacleArmed(struckOff));
}

void testObstacleBiasesOnRangeAndArcsOnTheStruckZone() {
  const int32_t bias = steerFor(400).left;
  MissionInputs near = onLine(400);
  near.tofFresh = true;
  near.tofMm = 300;

  Rig rig;
  rig.settle(near);
  CHECK(rig.state() == StateId::ObstacleArc);
  // Range alone biases the pass; the bar keeps steering.
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_OBSTACLE_BIAS_SPEED + bias);
  CHECK_EQ(rig.last().right, MISSION_OBSTACLE_BIAS_SPEED - bias);

  // A left strike goes right, a right strike goes left.
  MissionInputs left = near;
  left.bumperContact = true;
  left.bumperZone = BumperZone::Left;
  CHECK(arcFor(left) == TurnDirection::Right);
  rig.settle(left);
  CHECK(rig.last().mode == DriveMode::Pivot);
  CHECK_EQ(rig.last().left, -MISSION_OBSTACLE_ARC_SPEED);
  CHECK_EQ(rig.last().right, MISSION_OBSTACLE_ARC_SPEED);

  MissionInputs right = left;
  right.bumperZone = BumperZone::Right;
  CHECK(arcFor(right) == TurnDirection::Left);
  rig.settle(right);
  CHECK_EQ(rig.last().left, MISSION_OBSTACLE_ARC_SPEED);
  CHECK_EQ(rig.last().right, -MISSION_OBSTACLE_ARC_SPEED);

  // Both zones at once means the nose is in: back out, do not pick a side.
  MissionInputs both = withBothStruck();
  both.line = near.line;
  both.tofFresh = true;
  both.tofMm = 300;
  CHECK(arcFor(both) == TurnDirection::Reverse);
  rig.settle(both);
  CHECK_EQ(rig.last().left, -MISSION_OBSTACLE_ARC_SPEED);
  CHECK_EQ(rig.last().right, -MISSION_OBSTACLE_ARC_SPEED);

  // Still struck, still in the pass: a clean centroid sample cannot end it,
  // because the arc has the robot off the line on purpose.
  rig.feed(both);
  CHECK(rig.state() == StateId::ObstacleArc);

  // The contact going away is what starts re-acquisition, and one clean sample
  // is not enough to call the same line found again.
  MissionInputs clear = near;
  clear.tofMm = 700;
  rig.feed(clear);
  CHECK(rig.state() == StateId::ObstacleArc);
  rig.feed(clear);
  CHECK(rig.state() == StateId::Idle);
}

// --- The goal --------------------------------------------------------------

void testGoalNeedsBothTheBlobAndTheRange() {
  CHECK(goalArmed(goalInput()));
  CHECK(!goalArmed(withGoal(MISSION_GOAL_MIN_CONFIDENCE_PCT, MISSION_GOAL_MIN_SIZE)));
  CHECK(!goalArmed(withGoal(MISSION_GOAL_MIN_CONFIDENCE_PCT - 1, MISSION_GOAL_MIN_SIZE)));
  CHECK(!goalArmed(withGoal(MISSION_GOAL_MIN_CONFIDENCE_PCT, MISSION_GOAL_MIN_SIZE - 1)));

  MissionInputs far = goalInput();
  far.tofMm = MISSION_GOAL_TOF_CLOSE_MM + 1;
  CHECK(!goalArmed(far));
  MissionInputs stale = goalInput();
  stale.tofFresh = false;
  CHECK(!goalArmed(stale));
  MissionInputs staleBlob = goalInput();
  staleBlob.goalFresh = false;
  CHECK(!goalArmed(staleBlob));

  // The goal's own close range must not arm the obstacle pass, or the robot
  // would drive around the goal it just cross-validated. A strike still wins.
  CHECK(!obstacleArmed(goalInput()));
  CHECK(missionTrigger(goalInput()) == MISSION_EVENT_GOAL);
  MissionInputs struckAtGoal = goalInput();
  struckAtGoal.bumperContact = true;
  struckAtGoal.bumperZone = BumperZone::Left;
  CHECK(obstacleArmed(struckAtGoal));
  CHECK(missionTrigger(struckAtGoal) == MISSION_EVENT_CONTACT);
}

void testGoalStopsAndHoldsForItsFullBudget() {
  Rig rig;
  rig.feed(goalInput());
  CHECK(rig.state() == StateId::GoalHold);
  rig.feed(goalInput());
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
  CHECK_EQ(rig.last().right, 0);

  // It holds for the full five seconds; a goal is never a momentary stop.
  rig.feedFor(goalInput(), MISSION_GOAL_HOLD_MS - 2 * rig.stepMs);
  CHECK(rig.state() == StateId::GoalHold);
  rig.feedFor(goalInput(), MISSION_GOAL_HOLD_MS);
  CHECK(rig.state() == StateId::Idle);
}

// --- Ordering, delivery and the backstop -----------------------------------

void testReflexesClaimTheTickInPriorityOrder() {
  CHECK_EQ(missionTrigger(cleanInputs()), EVENT_NONE);
  CHECK(missionTrigger(offLine()) == MISSION_EVENT_GAP);
  CHECK(missionTrigger(goalInput()) == MISSION_EVENT_GOAL);
  CHECK(missionTrigger(withPitch(2200)) == MISSION_EVENT_RAMP);
  CHECK(missionTrigger(withPitch(1500)) == MISSION_EVENT_SEESAW);

  // A goal outranks a marker, and a marker outranks terrain.
  MissionInputs goalAndMarker = goalInput();
  goalAndMarker.marker = MarkerState::Left;
  CHECK(missionTrigger(goalAndMarker) == MISSION_EVENT_GOAL);
  CHECK(missionTrigger(withPitch(2200)) != MISSION_EVENT_SEESAW);

  // A dead end outranks a goal, and a lost line outranks both.
  MissionInputs everything = goalInput();
  everything.marker = MarkerState::Both;
  everything.line = offLine().line;
  everything.bumperContact = true;
  everything.bumperZone = BumperZone::Left;
  everything.imuFresh = true;
  everything.pitchCentiDeg = 1500;
  // The strike is the never-fail backstop, so it outranks all of it.
  CHECK(missionTrigger(everything) == MISSION_EVENT_CONTACT);
  everything.bumperContact = false;
  CHECK(missionTrigger(everything) == MISSION_EVENT_DEAD_END);
  everything.marker = MarkerState::None;
  CHECK(missionTrigger(everything) == MISSION_EVENT_GAP);
}

void testTheDriveSeamSeesEveryDemand() {
  Rig rig;
  const int before = rig.demandCount;
  rig.feed(onLine(0));
  rig.feed(offLine());
  rig.feed(offLine());
  CHECK(rig.demandCount == before + 3);
  CHECK(rig.last().mode == DriveMode::GapHold);

  // Detaching the seam stops the motors being written, but the reflex layer
  // still records what it decided.
  setMissionDrive(nullptr, nullptr);
  const int quiet = rig.demandCount;
  rig.feed(offLine());
  CHECK_EQ(rig.demandCount, quiet);
  CHECK(rig.last().mode == DriveMode::GapHold);
  setMissionDrive(&Rig::record, &rig);
}

void testTheBackstopStopsTheMotors() {
  Rig rig;
  rig.feed(onLine(0));
  const int before = rig.demandCount;
  reflexBackstop(rig.engine);
  CHECK(rig.demandCount == before + 1);
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
  CHECK_EQ(rig.last().right, 0);
}

}  // namespace

int main() {
  testTableCoversEveryStateAndBoundsEveryReflex();
  testBootHandsOffToIdleThroughTheSketchEntryPoints();
  testIdleFollowsTheCentroidAndCarriesTheLineWhenItGoes();
  testGapHoldsTheLastKnownOutputAndLeavesOnTheClearLine();
  testGapAddsTheYawAssistOnlyWhileTheLinkIsUsable();
  testGapAbortsOnItsBudgetAndWaitsForTheConditionToClearBeforeReArming();
  testMarkerTurnsTowardTheGreenSide();
  testDeadEndPivotsOneHundredEighty();
  testRampKeepsTheBarPrimaryAndLeavesAtTheCrest();
  testPitchBandPicksTheTerrainAndNeverSeesawOnALevelFloor();
  testSeesawRunsOnlyTheCarveOutUntilTheMockIsVerified();
  testObstacleArmsOnRangeOrOnAStrike();
  testObstacleBiasesOnRangeAndArcsOnTheStruckZone();
  testGoalNeedsBothTheBlobAndTheRange();
  testGoalStopsAndHoldsForItsFullBudget();
  testReflexesClaimTheTickInPriorityOrder();
  testTheDriveSeamSeesEveryDemand();
  testTheBackstopStopsTheMotors();

  std::printf("mission_test: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
