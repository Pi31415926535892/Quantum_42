// lop_test.cpp — host-side tests for the loss-of-position restart path.
//
// Like mission_test.cpp and evac_test.cpp this includes fsm.ino itself, so every
// case below is driven through the SHIPPED table, the SHIPPED glue callbacks and
// the real event queue. A row wired to the wrong guard, a budget the bail
// disagrees with, or entry bookkeeping that was left out cannot pass here.
//
// What is proved: the restart chain sits ahead of the zone in the state
// registry; WAIT_READY holds for its window and then latches the error flag and
// carries on line-only; the radar picks the search's first leg and can never
// gate, stall or end it; the bar search is the primary mechanism and is
// bounded; the ledger is re-derived from the persisted record and survives the
// restart; sweeps are capped; and no arrangement of readings can strand the
// robot inside the chain.
//
// Run: ./test/run_tests.sh

#include <cstdio>

// The store's per-channel struct carries its own name (StoredChannelCalibration)
// and the line module's does too (ChannelCalibration), so the two headers below
// can be included in either order and the restart's ledger is staged as the
// store's own AccountingRecord — no translation, nothing to disagree about.
#include "calibration.h"

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

// The restart chain, in the order it runs and in the order the registry holds
// it. The second of those is the point: re-localisation is finished before the
// zone can be entered.
const StateId kLopChain[] = {StateId::LoPLost, StateId::LoPRadarAssist,
                             StateId::LoPReacquire, StateId::LoPResume};
constexpr uint8_t kLopChainLength = 4;

// Where a chain row hands to: on its own condition, and on its own budget. Only
// the resume bails home — the first three bail FORWARD, because a search that has
// spent its window has not finished its job, it has only stopped trying.
StateId lopHandoffFrom(StateId state, bool budgetSpent) {
  switch (state) {
    case StateId::LoPLost: return StateId::LoPRadarAssist;
    case StateId::LoPRadarAssist: return StateId::LoPReacquire;
    case StateId::LoPReacquire: return StateId::LoPResume;
    default: return budgetSpent || state == StateId::LoPResume ? StateId::Idle
                                                              : StateId::LoPResume;
  }
}

// --- Snapshots --------------------------------------------------------------
// A restart is run with the bar over bare floor: nothing under it, no link
// frames, no markers. Every builder starts there, so a case only states the
// one thing it is about.

MissionInputs onBareFloor() {
  MissionInputs in{};
  in.line.lineDetected = false;
  in.line.qualifier = LineClass::ALL_WHITE;
  return in;
}

// The line is back under the bar, centred.
MissionInputs lineFound() {
  MissionInputs in = onBareFloor();
  in.line.lineDetected = true;
  in.line.qualifier = LineClass::ON_LINE;
  return in;
}

// The bar is on something dark but running off one end of itself: not a
// re-acquisition, and never counted as one.
MissionInputs lineAtEnd() {
  MissionInputs in = lineFound();
  in.line.qualifier = LineClass::EDGE;
  return in;
}

MissionInputs withReady() {
  MissionInputs in = onBareFloor();
  in.readyFresh = true;
  return in;
}

MissionInputs withRadar(uint8_t zone, uint8_t profile) {
  MissionInputs in = onBareFloor();
  in.radarFresh = true;
  in.radarZone = zone;
  in.radarClass = profile;
  return in;
}

MissionInputs withMarker() {
  MissionInputs in = lineFound();
  in.marker = MarkerState::Left;
  return in;
}

MissionInputs withStrike() {
  MissionInputs in = lineFound();
  in.bumperContact = true;
  in.bumperZone = BumperZone::Left;
  return in;
}

MissionInputs degraded(MissionInputs in) {
  in.policy = LinkPolicy::DegradeToLineOnly;
  return in;
}

// --- The ledger -------------------------------------------------------------
// What the store still holds when the rail comes back: the run was cut short
// with two landmarks passed and two deposits already written down.

AccountingRecord cutShortRun(uint16_t landmarks) {
  AccountingRecord record = freshAccounting();
  record.victims_counted = 3;
  record.deposits_placed = 2;
  record.loop_pending = true;
  record.landmark_counter = landmarks;
  return record;
}

// --- The rig ----------------------------------------------------------------
// One delivery pass in the order mission.ino performs it: the snapshot lands,
// the bookkeeping runs, at most one reflex event is queued, then the engine gets
// its single tick. The restart itself is staged exactly as missionBegin does it
// — a fresh state, then the persisted record handed to the restaging call.

struct LoPRig {
  FsmEngine engine;
  uint32_t stepMs;
  DriveDemand demand;
  int demandCount;

  explicit LoPRig(const AccountingRecord& stored = cutShortRun(2))
      : engine(kStateTable, kStateCount), stepMs(10), demand(), demandCount(0) {
    setMissionDrive(&LoPRig::record, this);
    beginMission();
    beginLopRestage(stored);
    engine.begin(StateId::Boot);
    engine.tick(stepMs, EVENT_NONE);
    engine.tick(stepMs, EVENT_NONE);
  }

  ~LoPRig() { setMissionDrive(nullptr, nullptr); }

  static void record(const DriveDemand& d, void* ctx) {
    LoPRig* self = static_cast<LoPRig*>(ctx);
    self->demand = d;
    ++self->demandCount;
  }

  void feed(const MissionInputs& in) {
    setMissionInputs(in);
    advanceMission();
    engine.postEvent(missionPendingTrigger());
    engine.tick(stepMs, EVENT_NONE);
  }

  // Feed the same reading until this state is left, or its own budget is spent,
  // whichever comes first. The last pass is fed outside the loop so the state
  // that spends a budget is always observed leaving.
  void feedUntilLeave(const MissionInputs& in, uint32_t limitMs) {
    const StateId entered = engine.state();
    while (engine.state() == entered && engine.residenceMs() < limitMs) feed(in);
    // One more pass, so a state that only leaves when its budget tips over is
    // always observed leaving. Only when it has NOT left already: the chain
    // states hand on within a single pass, so an unconditional one would walk
    // two rows at once and land somewhere the caller never asked for.
    if (engine.state() == entered) feed(in);
  }

  // The chain, walked to the resume on bare floor: past the handshake, past the
  // sweep, into the search.
  void walkToSearch(const MissionInputs& dark) {
    feedUntilLeave(dark, MISSION_LOP_READY_WAIT_MS);
    CHECK(state() == StateId::LoPRadarAssist);
    feedUntilLeave(dark, MISSION_LOP_SWEEP_ARM_MS);
    CHECK(state() == StateId::LoPReacquire);
  }

  StateId state() const { return engine.state(); }
  DriveDemand last() const { return missionState().demand; }
  uint32_t residence() const { return engine.residenceMs(); }
};

// Walk the chain on bare floor as far as the search, with no link of any kind
// available at any point.
void runChainToSearch(LoPRig& rig, const MissionInputs& dark) {
  CHECK(rig.state() == StateId::LoPLost);
  rig.walkToSearch(dark);
}

// --- The table --------------------------------------------------------------

void testTheRegistryGrowsTheRestartChainAheadOfTheZone() {
  CHECK_EQ(kStateCount, STATE_REGISTRY_SIZE);
  for (uint8_t index = 0; index < kStateCount; ++index) {
    CHECK(kStateTable[index].id == static_cast<StateId>(index));
  }

  // Re-localisation completes before a zone can be entered: the whole chain
  // sits ahead of the zone's five states in the registry.
  CHECK(static_cast<uint8_t>(StateId::LoPResume) <
        static_cast<uint8_t>(StateId::EvacApproach));

  for (uint8_t step = 0; step < kLopChainLength; ++step) {
    const StateId state = kLopChain[step];
    const StateDef& row = kStateTable[static_cast<uint8_t>(state)];
    CHECK_EQ(row.transition_count, 2);
    CHECK_EQ(row.transitions[0].event, EVENT_ANY);
    CHECK_EQ(row.transitions[1].event, EVENT_ANY);
    // Forward on this state's own condition; forward again once its own budget
    // is spent — except the resume, which hands home either way.
    CHECK(row.transitions[0].next == lopHandoffFrom(state, false));
    CHECK(row.transitions[1].next == lopHandoffFrom(state, true));
    // Bounded, and bounded by the same number the bail guard compares against,
    // so the guard normally wins the race and the timeout is the net under it.
    CHECK(row.max_residence_ms != RESIDENCE_UNBOUNDED);
    CHECK_EQ(row.max_residence_ms, lopResidenceBudgetMs(state));
    CHECK(row.on_entry == lopEnter);
    CHECK(row.on_exit == nullptr);
    CHECK(row.on_action == lopRun);
    CHECK(row.on_timeout == lopBackstop);
  }

  // The READY window closing is the fault the restart reports, so the bail row
  // carries the latch rather than letting the timeout do it: a timeout reports,
  // it does not move the engine, and a fault raised from one would leave the
  // robot holding stopped wheels inside the chain.
  const StateDef& lost = kStateTable[static_cast<uint8_t>(StateId::LoPLost)];
  CHECK(lost.transitions[0].guard == lopHandshakeSettledNow);
  CHECK(lost.transitions[0].action == nullptr);
  CHECK(lost.transitions[1].guard == lopBudgetSpent);
  CHECK(lost.transitions[1].action == lopHandshakeExpired);

  // Both ways out of the resume publish the ledger exactly once, so a search
  // that re-centred and one that deliberately gave up leave the same trail. The
  // guards are the resume's own completion condition and its own budget — not
  // the search's, and not the link's.
  const StateDef& resume = kStateTable[static_cast<uint8_t>(StateId::LoPResume)];
  CHECK(resume.transitions[0].guard == lopAdvance);
  CHECK(resume.transitions[1].guard == lopBudgetSpent);
  CHECK_EQ(resume.transitions[0].action, lopResumeOver);
  CHECK_EQ(resume.transitions[1].action, lopResumeOver);

  // Boot gained one row and kept its hand-off first: a clean power-up walks
  // straight into the mission, and only a ledger that says the last run was cut
  // short falls through to the restart chain.
  const StateDef& boot = kStateTable[static_cast<uint8_t>(StateId::Boot)];
  CHECK_EQ(boot.transition_count, 2);
  CHECK(boot.transitions[0].next == StateId::Idle);
  CHECK(boot.transitions[0].guard == lopRestartNotPending);
  CHECK(boot.transitions[1].next == StateId::LoPLost);
  CHECK(boot.transitions[1].guard == lopRestartPending);
  CHECK_EQ(boot.max_residence_ms, RESIDENCE_UNBOUNDED);
}

void testTheRestartIsArmedOnlyByTheStoredLedger() {
  {
    LoPRig cut(cutShortRun(2));
    CHECK(cut.state() == StateId::LoPLost);
    CHECK(lopResumeRequired());
  }
  {
    // A clean record is a clean start: the mission begins, no restart runs.
    LoPRig clean(freshAccounting());
    CHECK(clean.state() == StateId::Idle);
    CHECK(!lopResumeRequired());
  }
  {
    // Counts on the record but no cut-short flag is a run that FINISHED: it left
    // a ledger behind it and owes no restart, so Boot walks into Idle.
    AccountingRecord finished = freshAccounting();
    finished.victims_counted = 3;
    finished.deposits_placed = 2;
    finished.landmark_counter = 2;
    finished.loop_pending = false;
    LoPRig done(finished);
    CHECK(done.state() == StateId::Idle);
    CHECK(!lopResumeRequired());
  }
}

void testEveryChainStateCarriesItsOwnFiniteBudget() {
  // Every one is finite and none of them is the backstop's 300 ms net.
  for (uint8_t step = 0; step < kLopChainLength; ++step) {
    const uint32_t budget = lopResidenceBudgetMs(kLopChain[step]);
    CHECK(budget != RESIDENCE_UNBOUNDED);
    CHECK(budget > MISSION_STATE_BACKSTOP_MS);
  }
  CHECK_EQ(lopResidenceBudgetMs(StateId::Idle), RESIDENCE_UNBOUNDED);
  CHECK_EQ(lopResidenceBudgetMs(StateId::EvacApproach), RESIDENCE_UNBOUNDED);
  // The whole restart, plus the reserve the drill claims, fits the bound it is
  // held to. The sweep runs concurrently with the search and is never added.
  CHECK(MISSION_LOP_READY_WAIT_MS + MISSION_LOP_SEARCH_MAX_MS +
            MISSION_LOP_RESUME_MAX_MS + MISSION_LOP_DRILL_RESERVE_MS <=
        MISSION_LOP_DRILL_BOUND_MS);
  CHECK_EQ(MISSION_LOP_SEARCH_LEG_MS * MISSION_LOP_SEARCH_LEGS,
           MISSION_LOP_SEARCH_MAX_MS);
}

void testTheBackstopUnderEveryChainStateStopsTheMotors() {
  LoPRig rig;
  const int before = rig.demandCount;
  lopBackstop(rig.engine);
  CHECK_EQ(rig.demandCount, before + 1);
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
  CHECK_EQ(rig.last().right, 0);
}

// --- The handshake ----------------------------------------------------------

void testWaitReadyHoldsForItsWindowThenDegradesToLineOnly() {
  CHECK_EQ(MISSION_LOP_READY_WAIT_MS, 5000u);
  CHECK(lopHandshakeSettled(withReady()));
  CHECK(!lopHandshakeSettled(onBareFloor()));

  // Nothing has arrived: the robot holds, wheels stopped, for the whole window.
  LoPRig rig;
  CHECK(rig.state() == StateId::LoPLost);
  CHECK(!lopHandshakeFaulted());
  rig.feed(onBareFloor());
  CHECK(rig.state() == StateId::LoPLost);
  CHECK(rig.last().mode == DriveMode::Stop);
  rig.feedUntilLeave(onBareFloor(), MISSION_LOP_READY_WAIT_MS - rig.stepMs * 2);
  CHECK(rig.state() == StateId::LoPLost);
  CHECK(!lopHandshakeFaulted());

  // The window closes with no decoded READY: the error flag latches and the run
  // carries on, because a restart is not allowed to become a dead end.
  rig.feedUntilLeave(onBareFloor(), MISSION_LOP_READY_WAIT_MS);
  CHECK(rig.state() == StateId::LoPRadarAssist);
  CHECK(lopHandshakeFaulted());

  // The fault is the line-only discipline, not a refusal to move: everything
  // below the handshake runs with no link at all.
  CHECK(!linkUsable(degraded(onBareFloor())));
  rig.feedUntilLeave(onBareFloor(), MISSION_LOP_SWEEP_ARM_MS);
  CHECK(rig.state() == StateId::LoPReacquire);
}

void testReadyInsideTheWindowSkipsTheFault() {
  LoPRig rig;
  rig.feedUntilLeave(onBareFloor(), MISSION_LOP_READY_WAIT_MS / 2);
  CHECK(rig.state() == StateId::LoPLost);
  CHECK(!lopHandshakeFaulted());
  rig.feed(withReady());
  CHECK(rig.state() == StateId::LoPRadarAssist);
  CHECK(!lopHandshakeFaulted());
}

// --- The radar assist -------------------------------------------------------

void testRadarAssistPicksADirectionAndOnlyWhileItIsFresh() {
  // No sweep at all: the search runs blind, alternating.
  CHECK(lopSearchBias(onBareFloor()) == SearchBias::Alternating);
  // A stale sweep is not a decision source, and the only way to say "stale" is
  // not to report one.
  MissionInputs notFresh = withRadar(1, MISSION_RADAR_OPEN);
  notFresh.radarFresh = false;
  CHECK(lopSearchBias(notFresh) == SearchBias::Alternating);
  // A degraded link is not entitled to its own answer.
  CHECK(lopSearchBias(degraded(withRadar(1, MISSION_RADAR_OPEN))) ==
        SearchBias::Alternating);

  // An open profile: the sector names the way. Sectors are 20 degrees, numbered
  // from straight ahead — 1..8 left of the nose, 10..17 right of it, and 9
  // dead astern, where a blind search is as good an answer as any.
  CHECK(lopSearchBias(withRadar(0, MISSION_RADAR_OPEN)) == SearchBias::Straight);
  CHECK(lopSearchBias(withRadar(1, MISSION_RADAR_OPEN)) == SearchBias::Left);
  CHECK(lopSearchBias(withRadar(8, MISSION_RADAR_OPEN)) == SearchBias::Left);
  CHECK(lopSearchBias(withRadar(10, MISSION_RADAR_OPEN)) == SearchBias::Right);
  CHECK(lopSearchBias(withRadar(17, MISSION_RADAR_OPEN)) == SearchBias::Right);
  CHECK(lopSearchBias(withRadar(9, MISSION_RADAR_OPEN)) == SearchBias::Alternating);
  // A sector outside the sweep, or the "no result" marker, is not an answer.
  CHECK(lopSearchBias(withRadar(MISSION_RADAR_SECTOR_COUNT, MISSION_RADAR_OPEN)) ==
        SearchBias::Alternating);
  CHECK(lopSearchBias(withRadar(LINK_ZONE_NO_RESULT, MISSION_RADAR_OPEN)) ==
        SearchBias::Alternating);

  // Enclosed means every sector is close, so "the most open one" says nothing:
  // the assist stands down rather than pointing the search at a wall.
  CHECK(lopSearchBias(withRadar(1, MISSION_RADAR_ENCLOSED)) ==
        SearchBias::Alternating);
  CHECK(lopSearchBias(withRadar(10, MISSION_RADAR_ENCLOSED)) ==
        SearchBias::Alternating);
  // Ambiguity is the one profile worth a second look, and it is not a direction.
  CHECK(lopSearchBias(withRadar(1, MISSION_RADAR_AMBIGUOUS)) ==
        SearchBias::Alternating);

  // Which becomes the search's FIRST leg, and only the first: every leg after
  // it alternates, so a search that never gets an answer still sweeps both
  // sides of the robot.
  CHECK(lopSearchTurn(withRadar(1, MISSION_RADAR_OPEN), 0) == TurnDirection::Left);
  CHECK(lopSearchTurn(withRadar(10, MISSION_RADAR_OPEN), 0) == TurnDirection::Right);
  CHECK(lopSearchTurn(withRadar(0, MISSION_RADAR_OPEN), 0) == TurnDirection::Straight);
  CHECK(lopSearchTurn(onBareFloor(), 0) == TurnDirection::Left);
  CHECK(lopSearchTurn(withRadar(1, MISSION_RADAR_OPEN), MISSION_LOP_SEARCH_LEG_MS) ==
        TurnDirection::Right);
  CHECK(lopSearchTurn(withRadar(1, MISSION_RADAR_OPEN), MISSION_LOP_SEARCH_LEG_MS * 2) ==
        TurnDirection::Left);
  CHECK(lopSearchTurn(onBareFloor(), MISSION_LOP_SEARCH_LEG_MS * 3) ==
        TurnDirection::Right);
  CHECK_EQ(lopSearchLegIndex(0), 0);
  CHECK_EQ(lopSearchLegIndex(MISSION_LOP_SEARCH_LEG_MS - 1), 0);
  CHECK_EQ(lopSearchLegIndex(MISSION_LOP_SEARCH_LEG_MS), 1);
  CHECK_EQ(lopSearchLegIndex(MISSION_LOP_SEARCH_LEG_MS * 3), 3);
  // A long residence can never index past the last leg of the ladder.
  CHECK_EQ(lopSearchLegIndex(MISSION_LOP_SEARCH_MAX_MS), MISSION_LOP_SEARCH_LEGS - 1);
  CHECK_EQ(lopSearchLegIndex(MISSION_LOP_SEARCH_MAX_MS * 4),
           MISSION_LOP_SEARCH_LEGS - 1);
}

void testRadarNeverGatesTheSearch() {
  // The assist has no row of its own and no way to end the chain: the only rows
  // out of every restart state are the state's own condition and its own
  // budget, and no guard of either can be about the link.
  for (uint8_t step = 0; step < kLopChainLength; ++step) {
    const StateDef& row = kStateTable[static_cast<uint8_t>(kLopChain[step])];
    const bool isHandshake = kLopChain[step] == StateId::LoPLost;
    for (uint8_t index = 0; index < row.transition_count; ++index) {
      // The bail row is the budget guard; the other row is this state's own
      // completion condition, and the handshake's own condition is its receipt.
      CHECK_EQ(row.transitions[index].guard == lopBudgetSpent, index != 0);
      if (index == 0 && !isHandshake) {
        CHECK(row.transitions[index].guard == lopAdvance);
      }
      if (index == 0 && isHandshake) {
        CHECK(row.transitions[index].guard == lopHandshakeSettledNow);
      }
    }
  }

  // Proven by the run rather than by the wiring: with no radar frame at any
  // point, the search still terminates on the bar and the chain still reaches
  // the resume. The sweep state, meanwhile, never spends a pass waiting for a
  // sweep nobody has given it.
  LoPRig rig;
  CHECK_EQ(lopRadarRequests(), 0);
  CHECK(!lopRadarRequestPending());
  runChainToSearch(rig, onBareFloor());
  CHECK_EQ(lopRadarRequests(), 1);
  rig.feed(lineFound());
  rig.feed(lineFound());
  CHECK(rig.state() == StateId::LoPResume);
  // And a search the bar never satisfies is bounded rather than endless.
  LoPRig blind;
  runChainToSearch(blind, onBareFloor());
  blind.feedUntilLeave(onBareFloor(), MISSION_LOP_SEARCH_MAX_MS);
  CHECK(blind.state() == StateId::LoPResume);
  blind.feedUntilLeave(onBareFloor(), MISSION_LOP_RESUME_MAX_MS);
  CHECK(blind.state() == StateId::Idle);
}

void testSweepsAreCappedPerRestart() {
  CHECK_EQ(MISSION_LOP_RADAR_REQUEST_CAP, 2);
  beginMission();
  CHECK_EQ(lopRadarRequests(), 0);
  CHECK(lopRadarRequestAllowed(missionState()));
  CHECK(!lopRadarRequestPending());

  // One arming for the sweep the restart asks for while the search runs.
  CHECK(missionLopArmSweep());
  CHECK_EQ(lopRadarRequests(), 1);
  CHECK(lopRadarRequestPending());
  lopRadarRequestSent();
  CHECK(!lopRadarRequestPending());

  // An ambiguous profile is the one case worth a second look.
  CHECK(lopReSweepWanted(withRadar(1, MISSION_RADAR_AMBIGUOUS)));
  CHECK(!lopReSweepWanted(withRadar(1, MISSION_RADAR_OPEN)));
  CHECK(!lopReSweepWanted(withRadar(1, MISSION_RADAR_ENCLOSED)));
  MissionInputs noSweep = withRadar(1, MISSION_RADAR_AMBIGUOUS);
  noSweep.radarFresh = false;
  CHECK(!lopReSweepWanted(noSweep));
  CHECK(missionLopArmSweep());
  CHECK_EQ(lopRadarRequests(), 2);
  CHECK(lopRadarRequestPending());
  lopRadarRequestSent();
  CHECK(!lopRadarRequestPending());

  // And from here the cap holds, however long the profile stays ambiguous.
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    CHECK(!missionLopArmSweep());
  }
  CHECK_EQ(lopRadarRequests(), MISSION_LOP_RADAR_REQUEST_CAP);
  CHECK(!lopRadarRequestAllowed(missionState()));
  CHECK(!lopRadarRequestPending());

  // Through the table: one sweep on the way in, one re-sweep while the search
  // runs, and no third request however long the profile stays ambiguous.
  LoPRig rig;
  runChainToSearch(rig, onBareFloor());
  CHECK_EQ(lopRadarRequests(), 1);
  rig.feed(withRadar(3, MISSION_RADAR_AMBIGUOUS));
  CHECK_EQ(lopRadarRequests(), 2);
  for (uint8_t pass = 0; pass < 4; ++pass) {
    rig.feed(withRadar(3, MISSION_RADAR_AMBIGUOUS));
  }
  CHECK_EQ(lopRadarRequests(), MISSION_LOP_RADAR_REQUEST_CAP);
}

// --- The search -------------------------------------------------------------

void testTheBarSearchIsPrimaryAndBounded() {
  beginMission();
  setMissionInputs(lineFound());
  missionCountClearSamples(false);
  CHECK(!lopSearchClear(missionState()));
  missionCountClearSamples(true);
  CHECK(!lopSearchClear(missionState()));
  missionCountClearSamples(true);
  CHECK(lopSearchClear(missionState()));
  // One lucky pass is not a re-acquisition, and an edge is not a line.
  missionCountClearSamples(false);
  CHECK(!lopSearchClear(missionState()));
  setMissionInputs(lineAtEnd());
  missionCountClearSamples(false);
  missionCountClearSamples(true);
  missionCountClearSamples(true);
  CHECK(!lopSearchClear(missionState()));

  CHECK(lopResidenceBudgetMs(StateId::LoPReacquire) == MISSION_LOP_SEARCH_MAX_MS);
  CHECK(MISSION_LOP_SEARCH_MAX_MS > MISSION_STATE_BACKSTOP_MS);

  // The demand is a pivot, not a guess: with no sweep it alternates sides, and
  // it never carries a line correction it has no bar reading for.
  setMissionInputs(onBareFloor());
  missionCountClearSamples(false);
  const DriveDemand first = lopSearchStep(missionState(), 0);
  CHECK(first.mode == DriveMode::Pivot);
  CHECK_EQ(first.left, MISSION_LOP_SEARCH_PIVOT_SPEED);
  CHECK_EQ(first.right, -MISSION_LOP_SEARCH_PIVOT_SPEED);
  const DriveDemand other = lopSearchStep(missionState(), MISSION_LOP_SEARCH_LEG_MS);
  CHECK_EQ(other.left, -MISSION_LOP_SEARCH_PIVOT_SPEED);
  CHECK_EQ(other.right, MISSION_LOP_SEARCH_PIVOT_SPEED);
  // An open sector dead ahead is the one case that is not a pivot: the line is
  // in front, so the search rolls onto it.
  setMissionInputs(withRadar(0, MISSION_RADAR_OPEN));
  const DriveDemand ahead = lopSearchStep(missionState(), 0);
  CHECK(ahead.mode == DriveMode::LineFollow);
  CHECK_EQ(ahead.left, MISSION_LOP_SEARCH_ARC_SPEED);
  CHECK_EQ(ahead.right, MISSION_LOP_SEARCH_ARC_SPEED);
  setMissionInputs(withRadar(10, MISSION_RADAR_OPEN));
  const DriveDemand assisted = lopSearchStep(missionState(), 0);
  CHECK_EQ(assisted.left, -MISSION_LOP_SEARCH_PIVOT_SPEED);
  CHECK_EQ(assisted.right, MISSION_LOP_SEARCH_PIVOT_SPEED);
}

void testTheSearchFindsTheLineTheBarBringsBack() {
  LoPRig rig;
  runChainToSearch(rig, onBareFloor());
  rig.feed(lineAtEnd());
  CHECK(rig.state() == StateId::LoPReacquire);
  rig.feed(lineFound());
  CHECK(rig.state() == StateId::LoPReacquire);
  rig.feed(lineFound());
  CHECK(rig.state() == StateId::LoPResume);
  // The resume is the line-following run: the bar steers again, at resume speed.
  rig.feed(lineFound());
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_LOP_RESUME_SPEED + lineBias(0));

  // And a search that finds nothing hands the run back deliberately.
  LoPRig blind;
  runChainToSearch(blind, onBareFloor());
  blind.feedUntilLeave(onBareFloor(), MISSION_LOP_SEARCH_MAX_MS);
  CHECK(blind.state() == StateId::LoPResume);
  blind.feedUntilLeave(onBareFloor(), MISSION_LOP_RESUME_MAX_MS);
  CHECK(blind.state() == StateId::Idle);
}

// --- The ledger -------------------------------------------------------------

void testTheLedgerIsReDerivedFromTheStoreAcrossTheRestart() {
  // The record as it survives the cut, read back through the store's own
  // serialiser rather than handed over in memory.
  const AccountingRecord cut = cutShortRun(2);
  uint8_t slot[STORE_ACCOUNTING_SLOT_BYTES];
  CHECK_EQ(writeAccountingSlot(slot, sizeof(slot), cut, 4),
           STORE_ACCOUNTING_SLOT_BYTES);
  AccountingRecord stored = freshAccounting();
  const LoadResult loaded = loadAccounting(slot, sizeof(slot), stored);
  CHECK(loaded.outcome == LoadOutcome::Loaded);
  CHECK(stored.loop_pending);
  CHECK_EQ(stored.landmark_counter, 2);
  CHECK_EQ(stored.victims_counted, 3);
  CHECK_EQ(stored.deposits_placed, 2);

  // Restaging takes the whole record, not a count of its own.
  beginMission();
  beginLopRestage(stored);
  CHECK(lopResumeRequired());
  CHECK_EQ(lopExpectedLandmarks(), 2);
  CHECK_EQ(lopReDerivedLandmarks(), 2);
  CHECK_EQ(lopLandmarkError(), 0);
  CHECK(lopLandmarkErrorWithinBound());

// The re-derived half is the count the robot can prove from where it stands.
  // A crossing is counted on its rising edge, so one landmark held under the bar
  // for many passes is still one landmark.
  CHECK(lopLandmarkClaimed(withMarker()));
  CHECK(lopLandmarkClaimed(withStrike()));
  CHECK(!lopLandmarkClaimed(lineFound()));
  CHECK(!lopLandmarkClaimed(lineAtEnd()));
  missionLopTrackLandmark(withMarker());
  CHECK_EQ(lopReDerivedLandmarks(), 3);
  missionLopTrackLandmark(withMarker());
  CHECK_EQ(lopReDerivedLandmarks(), 3);  // still the same landmark, not a second
  missionLopTrackLandmark(lineFound());
  CHECK_EQ(lopReDerivedLandmarks(), 3);
  CHECK_EQ(lopLandmarkError(), 1);
  CHECK(lopLandmarkErrorWithinBound());
  missionLopTrackLandmark(withStrike());
  CHECK_EQ(lopReDerivedLandmarks(), 4);
  missionLopTrackLandmark(lineFound());
  CHECK_EQ(lopReDerivedLandmarks(), 4);
  missionLopTrackLandmark(withMarker());
  CHECK_EQ(lopReDerivedLandmarks(), 5);
  // Error 3 is a failure, and the bound says so.
  CHECK_EQ(lopLandmarkError(), 3);
  CHECK(!lopLandmarkErrorWithinBound());
  CHECK_EQ(MISSION_LOP_LANDMARK_ERROR_BOUND, 1);
}

void testTheRestartPublishesItsLedgerExactlyOnce() {
  beginMission();
  beginLopRestage(cutShortRun(2));
  CHECK(missionLopResumeComplete());
  CHECK(!missionLopResumeComplete());
  CHECK(!lopResumeRequired());

  // What leaves the robot is the same accounting, the re-derived landmark
  // count, and a run that is no longer pending — so the next cut is compared
  // against what this restart actually proved.
  const AccountingRecord published = lopReDerivedAccounting();
  CHECK_EQ(published.victims_counted, 3);
  CHECK_EQ(published.deposits_placed, 2);
  CHECK_EQ(published.landmark_counter, 2);
  CHECK(!published.loop_pending);

  // And it survives being written and read back, which is the whole point: a
  // completed restart stages nothing on the next power-up.
  uint8_t slot[STORE_ACCOUNTING_SLOT_BYTES];
  CHECK(writeAccountingSlot(slot, sizeof(slot), published, 5) != 0);
  AccountingRecord readBack = freshAccounting();
  CHECK(loadAccounting(slot, sizeof(slot), readBack).outcome == LoadOutcome::Loaded);
  CHECK(!readBack.loop_pending);
  CHECK_EQ(readBack.landmark_counter, 2);
  CHECK_EQ(readBack.victims_counted, 3);
  beginLopRestage(readBack);
  CHECK(!lopResumeRequired());
  CHECK(!lopLedgerPublished());
}

void testTheAccountingIsCorrectAfterAResume() {
  // End to end: the run is cut short, the robot restages from the store, hunts
  // the line back and re-walks one landmark on the way in. The ledger it
  // publishes is the one the next restart compares against.
  LoPRig rig(cutShortRun(2));
  runChainToSearch(rig, onBareFloor());
  rig.feed(lineFound());
  rig.feed(lineFound());
  CHECK(rig.state() == StateId::LoPResume);
  rig.feed(withMarker());
  rig.feed(lineFound());
  rig.feed(lineFound());
  CHECK(rig.state() == StateId::Idle);

  const AccountingRecord published = lopReDerivedAccounting();
  CHECK_EQ(published.landmark_counter, 3);
  CHECK_EQ(published.victims_counted, 3);
  CHECK_EQ(published.deposits_placed, 2);
  CHECK(!published.loop_pending);
  // One landmark more than the record claimed, which is inside the accepted
  // error — and the error latched at the handoff outlives it, so the restart
  // still reports what it proved once the run has moved on.
  CHECK(lopLandmarkErrorWithinBound());
  CHECK_EQ(lopLandmarkError(), 1);
  // The chain is over: the run is no longer owed a restart, and the entry token
  // is not something this pass left held.
  CHECK(!lopResumeRequired());
  CHECK(!evacLaneActive());
}

// --- Never stranding --------------------------------------------------------

void testTheRestartNeverStrands() {
  // No arrangement of readings may keep the robot inside the chain: every state
  // hands on within its own budget, on its own budget, to a state that also
  // hands on. Run the chain on a bar that never finds anything.
  LoPRig rig;
  rig.walkToSearch(onBareFloor());
  const StateId path[] = {StateId::LoPReacquire, StateId::LoPResume};
  for (uint8_t step = 0; step < 2; ++step) {
    CHECK(rig.state() == path[step]);
    rig.feedUntilLeave(onBareFloor(), lopResidenceBudgetMs(path[step]));
    CHECK(rig.state() != path[step]);
  }
  CHECK(rig.state() == StateId::Idle);

  // A snapshot that keeps every link claim alive and never settles the bar
  // still ends in Idle, and the ledger it published is still the one the store
  // handed over: it passed no landmark it did not actually pass.
  LoPRig busy(cutShortRun(5));
  MissionInputs noisy = withRadar(2, MISSION_RADAR_AMBIGUOUS);
  noisy.readyFresh = true;
  noisy.line.lineDetected = true;
  noisy.line.qualifier = LineClass::EDGE;
  for (uint32_t pass = 0; pass < 900; ++pass) busy.feed(noisy);
  CHECK(busy.state() == StateId::Idle);
  CHECK_EQ(lopReDerivedAccounting().landmark_counter, 5);
  CHECK(lopRadarRequests() <= MISSION_LOP_RADAR_REQUEST_CAP);
}

void testTheLoPConditionsAreClaimsAboutEvidence() {
  // A bare-floor bar is the hardest case there is, and the one the restart is
  // built for: nothing under it and nothing on the wire.
  LoPRig rig;
  runChainToSearch(rig, onBareFloor());
  CHECK(lopHandshakeFaulted());
  CHECK(!lopHandshakeSettled(onBareFloor()));

  // A fresh sweep that reports no result, and a sweep that is not fresh, are
  // the same thing to the search: nothing to steer by.
  beginMission();
  setMissionInputs(withRadar(LINK_ZONE_NO_RESULT, MISSION_RADAR_OPEN));
  CHECK(lopSearchBias(missionState().inputs) == SearchBias::Alternating);
  const DriveDemand blind = lopSearchStep(missionState(), 0);
  CHECK(blind.mode == DriveMode::Pivot);

  // A resume with no line under the bar is not a follow: it is the last-known
  // PD output held, which is what a robot with no bar reading must do.
  beginMission();
  setMissionInputs(onBareFloor());
  const DriveDemand holding = lopResumeStep(missionState());
  CHECK(holding.mode == DriveMode::GapHold);
  missionCountClearSamples(false);
  CHECK(!lopResumeReady(missionState()));
  setMissionInputs(lineFound());
  const DriveDemand following = lopResumeStep(missionState());
  CHECK(following.mode == DriveMode::LineFollow);
  missionCountClearSamples(false);
  missionCountClearSamples(true);
  CHECK(!lopResumeReady(missionState()));
  missionCountClearSamples(true);
  CHECK(lopResumeReady(missionState()));
}

}  // namespace

int main() {
  testTheRegistryGrowsTheRestartChainAheadOfTheZone();
  testTheRestartIsArmedOnlyByTheStoredLedger();
  testEveryChainStateCarriesItsOwnFiniteBudget();
  testTheBackstopUnderEveryChainStateStopsTheMotors();
  testWaitReadyHoldsForItsWindowThenDegradesToLineOnly();
  testReadyInsideTheWindowSkipsTheFault();
  testRadarAssistPicksADirectionAndOnlyWhileItIsFresh();
  testRadarNeverGatesTheSearch();
  testSweepsAreCappedPerRestart();
  testTheBarSearchIsPrimaryAndBounded();
  testTheSearchFindsTheLineTheBarBringsBack();
  testTheLedgerIsReDerivedFromTheStoreAcrossTheRestart();
  testTheRestartPublishesItsLedgerExactlyOnce();
  testTheAccountingIsCorrectAfterAResume();
  testTheRestartNeverStrands();
  testTheLoPConditionsAreClaimsAboutEvidence();

  std::printf("lop_test: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
