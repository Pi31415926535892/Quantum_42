// evac_test.cpp — host-side tests for the evacuation-zone lane and the victim
// classification pipeline.
//
// Like mission_test.cpp this includes fsm.ino itself, so every case below is
// driven through the SHIPPED table, the SHIPPED glue callbacks and the real
// event queue. A row wired to the wrong guard, or a budget the bail disagrees
// with, cannot pass here.
//
// What is proved: the two-source entry claim (silver-tape blob AND a wide-bright
// bar), the grey area between them, the entry token, the probe truth table
// (live / dead / fake and nothing else), a flickering sample that is never
// named, the write-before-release accounting order, the black-tape exit, the
// bound on every lane state, and the no-capture lane that still scores and still
// leaves.
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

// The probe readings the truth table is driven with. Higher is more reflective
// — the polarity the probe delivery layer and its own header both state, and
// the one thing inverting here would silently swap live and dead with.
constexpr int kReflective = 900;  // silver
constexpr int kOpaque = 100;      // matte black

// --- Snapshots --------------------------------------------------------------
// Every builder starts from a clean pass — bar centred on the black line, no
// link frames, nothing struck — so a case only has to state the claim it is
// about.

MissionInputs following() {
  MissionInputs in{};
  in.line.lineDetected = true;
  in.line.qualifier = LineClass::ON_LINE;
  return in;
}

// The bar inside the silver band: no dark channel left anywhere.
MissionInputs insideSilverBand() {
  MissionInputs in = following();
  in.line.lineDetected = false;
  in.line.qualifier = LineClass::ALL_WHITE;
  return in;
}

// A silver-tape blob and a bar that has not agreed with it yet: the grey area.
MissionInputs silverBlobOnly() {
  MissionInputs in = following();
  in.evacTapeFresh = true;
  in.evacTapeType = MISSION_TAPE_SILVER;
  in.evacTapeSize = MISSION_EVAC_TAPE_MIN_SIZE;
  return in;
}

MissionInputs silverBandAgreed() {
  MissionInputs in = insideSilverBand();
  in.evacTapeFresh = true;
  in.evacTapeType = MISSION_TAPE_SILVER;
  in.evacTapeSize = MISSION_EVAC_TAPE_MIN_SIZE;
  return in;
}

MissionInputs blackBand() {
  MissionInputs in = following();
  in.evacTapeFresh = true;
  in.evacTapeType = MISSION_TAPE_BLACK;
  in.evacTapeSize = MISSION_EVAC_TAPE_MIN_SIZE;
  return in;
}

MissionInputs withContact(MissionInputs in) {
  in.bumperContact = true;
  in.bumperZone = BumperZone::Left;
  return in;
}

MissionInputs withTriangle(MissionInputs in, uint8_t color) {
  in.triangleFresh = true;
  in.triangleColor = color;
  in.triangleSize = MISSION_EVAC_TRIANGLE_MIN_SIZE;
  return in;
}

MissionInputs probing(MissionInputs in, int reflectivity, bool conductive) {
  in.reflectivity = reflectivity;
  in.conductive = conductive;
  return in;
}

// --- The deposit seam -------------------------------------------------------
// The two calls the lane makes when it publishes a deposit, in the order they
// arrived. Accounting must be the first of them, every time.

constexpr uint8_t kLogCapacity = 16;
char g_log[kLogCapacity];
uint8_t g_logCount;
EvacAccounting g_records[kLogCapacity];
uint8_t g_recordCount;

void logToken(char token) {
  if (g_logCount < kLogCapacity) g_log[g_logCount] = token;
  ++g_logCount;
}

void recordAccounting(const EvacAccounting& record, void*) {
  logToken('A');
  if (g_recordCount < kLogCapacity) g_records[g_recordCount] = record;
  ++g_recordCount;
}

void recordRelease(DepositTarget, void*) { logToken('R'); }

void clearLog() {
  g_logCount = 0;
  g_recordCount = 0;
  for (uint8_t i = 0; i < kLogCapacity; ++i) g_log[i] = 0;
}

// --- The rig ----------------------------------------------------------------
// One delivery pass in the order mission.ino performs it: the snapshot lands,
// the bookkeeping runs, at most one reflex event is queued, then the engine gets
// its single tick.

struct ZoneRig {
  FsmEngine engine;
  uint32_t stepMs;
  uint32_t leftAtResidenceMs;  // residence the last state left on
  DriveDemand demand;
  int demandCount;

  ZoneRig()
      : engine(kStateTable, kStateCount),
        stepMs(10),
        leftAtResidenceMs(0),
        demand(),
        demandCount(0) {
    setMissionDrive(&ZoneRig::record, this);
    setEvacHooks(&recordAccounting, &recordRelease, nullptr);
    beginMission();
    engine.begin(StateId::Boot);
    engine.tick(stepMs, EVENT_NONE);
    engine.tick(stepMs, EVENT_NONE);
  }

  ~ZoneRig() {
    setMissionDrive(nullptr, nullptr);
    setEvacHooks(nullptr, nullptr, nullptr);
  }

  static void record(const DriveDemand& d, void* ctx) {
    ZoneRig* self = static_cast<ZoneRig*>(ctx);
    self->demand = d;
    ++self->demandCount;
  }

  void feed(const MissionInputs& in) {
    const StateId before = engine.state();
    const uint32_t residenceBefore = engine.residenceMs();
    setMissionInputs(in);
    advanceMission();
    engine.postEvent(missionPendingTrigger());
    engine.tick(stepMs, EVENT_NONE);
    // Recorded rather than counted: a probe hold is three passes by design, so
    // the honest question is how long a state held the robot, not how many.
    if (engine.state() != before) leftAtResidenceMs = residenceBefore;
  }

  // Feed until this state is left, or its own budget is spent, whichever is
  // first. Stopping on the leave matters: a bail hands back to Idle with the
  // reflex event already queued, and feeding on would drive straight back into
  // whatever the snapshot still claims.
  void feedUntilLeave(const MissionInputs& in, uint32_t limitMs) {
    feedAlternatingUntilLeave(in, in, limitMs);
  }

  // The same, for a test about a reading that never settles: the two snapshots
  // trade places every pass, so nothing here can ever fill a settling window.
  void feedAlternatingUntilLeave(const MissionInputs& a, const MissionInputs& b,
                                 uint32_t limitMs) {
    const StateId entered = engine.state();
    bool takeA = true;
    while (engine.state() == entered && engine.residenceMs() < limitMs) {
      feed(takeA ? a : b);
      takeA = !takeA;
    }
    if (engine.state() == entered) feed(a);  // the pass that spends the budget
  }

  StateId state() const { return engine.state(); }
  DriveDemand last() const { return missionState().demand; }
  const EvacTally& tally() const { return evacTally(); }
};

// Enter the zone from a clean pass: the two agreeing sources arm the lane.
void enterZone(ZoneRig& rig) {
  rig.feed(following());
  rig.feed(silverBandAgreed());
  CHECK(rig.state() == StateId::EvacApproach);
}

// Approach -> Probe on a contact claim.
void reachProbe(ZoneRig& rig, const MissionInputs& contact) {
  rig.feed(contact);
  CHECK(rig.state() == StateId::EvacProbe);
}

// Probe -> Classify once the sampling window has filled with agreeing samples.
// Feeds only the passes the window still needs, so a caller that already spent
// one while checking what the probe asks for does not overshoot it.
void reachClassify(ZoneRig& rig, const MissionInputs& probeSnapshot) {
  while (rig.state() == StateId::EvacProbe &&
         missionState().stableSamples < MISSION_EVAC_PROBE_SETTLE_SAMPLES) {
    rig.feed(probeSnapshot);
  }
  CHECK(rig.state() == StateId::EvacClassify);
}

// Classify -> Deposit: the decision is made and its target is either confirmed
// or, on the no-capture lane, not needed at all.
void reachDeposit(ZoneRig& rig, const MissionInputs& classifySnapshot) {
  rig.feed(classifySnapshot);
  CHECK(rig.state() == StateId::EvacDeposit);
}

// --- The table --------------------------------------------------------------

// The lane's five states, in the order they run. Shared by every test that has
// to name a state, so no test can quietly walk a different chain.
const StateId kEvacChain[] = {StateId::EvacApproach, StateId::EvacProbe,
                              StateId::EvacClassify, StateId::EvacDeposit,
                              StateId::EvacExit};
constexpr uint8_t kEvacChainLength = 5;

void testTableGrowsTheEvacChainAndBoundsEveryState() {
  CHECK_EQ(kStateCount, STATE_REGISTRY_SIZE);
  for (uint8_t index = 0; index < kStateCount; ++index) {
    CHECK(kStateTable[index].id == static_cast<StateId>(index));
  }

  // The lane is a chain in enumeration order, so the table can be walked.
  for (uint8_t step = 0; step < kEvacChainLength; ++step) {
    const StateId state = kEvacChain[step];
    const StateDef& row = kStateTable[static_cast<uint8_t>(state)];
    CHECK(row.id == state);
    // Two rows: advance to the next state, or hand back to Idle.
    CHECK_EQ(row.transition_count, 2);
    CHECK_EQ(row.transitions[0].event, EVENT_ANY);
    CHECK_EQ(row.transitions[1].event, EVENT_ANY);
    CHECK(row.transitions[0].next ==
          (step + 1 < kEvacChainLength ? kEvacChain[step + 1] : StateId::Idle));
    CHECK(row.transitions[1].next == StateId::Idle);
    // The teardown rides the rows that land back on Idle, and only those: a
    // shared on_exit could not tell a step along the chain from a bail out of
    // it, and guessing wrong would drop the held verdict on every step.
    for (uint8_t index = 0; index < row.transition_count; ++index) {
      const bool endsZone = row.transitions[index].next == StateId::Idle;
      CHECK_EQ(row.transitions[index].action == evacZoneOver, endsZone);
    }
    // Bounded, and bounded by the same number the bail guard compares against,
    // so the guard normally wins the race and the backstop is the net under it.
    CHECK(row.max_residence_ms != RESIDENCE_UNBOUNDED);
    CHECK_EQ(row.max_residence_ms, evacResidenceBudgetMs(state));
    CHECK(row.on_entry == evacEnter);
    CHECK(row.on_exit == nullptr);
    CHECK(row.on_action == evacRun);
    CHECK(row.on_timeout == evacBackstop);
  }

  // Idle gained exactly one row, and it is the only way into the zone: armed
  // from the two-source claim, not from the entry bookkeeping alone.
  const uint8_t idleRows = kStateTable[static_cast<uint8_t>(StateId::Idle)].transition_count;
  uint8_t evacRows = 0;
  for (uint8_t row = 0; row < idleRows; ++row) {
    const Transition& idle = kStateTable[1].transitions[row];
    if (idle.next == StateId::EvacApproach) {
      ++evacRows;
      CHECK_EQ(idle.event, MISSION_EVENT_EVAC);
      CHECK(idle.guard == evacEntryArmedNow);
    }
  }
  CHECK_EQ(evacRows, 1);
}

void testTheBackstopUnderEveryLaneStateStopsTheMotors() {
  ZoneRig rig;
  const int before = rig.demandCount;
  evacBackstop(rig.engine);
  CHECK_EQ(rig.demandCount, before + 1);
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
  CHECK_EQ(rig.last().right, 0);
}

// --- Entry ------------------------------------------------------------------

void testEntryNeedsTheBlobAndTheBrightClassTogether() {
  CHECK(barReadsWideBright(insideSilverBand().line));
  CHECK(!barReadsWideBright(following().line));
  CHECK(barOnBlackTape(following().line));
  CHECK(!barOnBlackTape(insideSilverBand().line));

  CHECK(evacEntryArmed(silverBandAgreed()));
  CHECK(!evacGreyArea(silverBandAgreed()));
  CHECK(evacTrigger(silverBandAgreed()));
  CHECK(missionTrigger(silverBandAgreed()) == MISSION_EVENT_EVAC);

  // Either source alone is the grey area, and neither is an entry.
  MissionInputs blobOnly = silverBlobOnly();
  CHECK(evacGreyArea(blobOnly));
  CHECK(!evacEntryArmed(blobOnly));
  CHECK(!evacTrigger(blobOnly));

  MissionInputs brightOnly = insideSilverBand();
  CHECK(evacGreyArea(brightOnly));
  CHECK(!evacEntryArmed(brightOnly));
  CHECK(missionTrigger(brightOnly) != MISSION_EVENT_EVAC);

  // A band too small to be the tape is not an entry either.
  MissionInputs smallBand = silverBandAgreed();
  smallBand.evacTapeSize = MISSION_EVAC_TAPE_MIN_SIZE - 1;
  CHECK(!evacEntryArmed(smallBand));
  MissionInputs staleBand = silverBandAgreed();
  staleBand.evacTapeFresh = false;
  CHECK(!evacEntryArmed(staleBand));
  MissionInputs exitBand = silverBandAgreed();
  exitBand.evacTapeType = MISSION_TAPE_BLACK;
  CHECK(!evacEntryArmed(exitBand));
  // A bar still partly on the black line has not read wide-bright.
  MissionInputs edgeBar = silverBandAgreed();
  edgeBar.line.lineDetected = true;
  edgeBar.line.qualifier = LineClass::EDGE;
  CHECK(!evacEntryArmed(edgeBar));

  // A degraded link is not entitled to the vision half of the claim.
  MissionInputs degraded = silverBandAgreed();
  degraded.policy = LinkPolicy::DegradeToLineOnly;
  CHECK(!evacEntryArmed(degraded));

  // The entry claim outranks the gap reflex, which would otherwise own every
  // sample the bar is off the line.
  CHECK(missionTrigger(insideSilverBand()) == MISSION_EVENT_GAP);
}

void testTriggerNeverFiresWithoutTheEntryToken() {
  // Nothing entered, so nothing may claim a tick: a zone that was never
  // cross-validated has no entry trigger at all.
  beginMission();
  CHECK(!evacLaneActive());
  missionEnterEvac();
  CHECK(!evacLaneActive());
  CHECK(missionTrigger(following()) != MISSION_EVENT_EVAC);
  CHECK(missionTrigger(insideSilverBand()) != MISSION_EVENT_EVAC);

  ZoneRig rig;
  rig.feed(following());
  CHECK(!evacLaneActive());
  enterZone(rig);
  CHECK(evacLaneActive());

  // Inside the lane the entry claim is the token's job, so the reflex API
  // cannot re-enter the zone from inside it and queue a second chain.
  rig.feed(silverBandAgreed());
  CHECK(evacLaneActive());
  CHECK_EQ(missionPendingTrigger(), EVENT_NONE);
  CHECK(missionTrigger(silverBandAgreed()) != MISSION_EVENT_EVAC);

  // And the entry bookkeeping happens once, not on every step along the chain:
  // the heading the zone is steered on is the one from the band it was entered
  // on, and a re-stamp halfway across the zone would be a different turn.
  const int16_t entryYaw = missionState().yawAtEntry;
  reachProbe(rig, withContact(insideSilverBand()));
  CHECK_EQ(missionState().yawAtEntry, entryYaw);
}

void testLeavingTheZoneReleasesTheEntryToken() {
  ZoneRig rig;
  enterZone(rig);
  CHECK(evacLaneActive());

  // Walking out of the lane by hand (a budget bail) ends the entry, exactly as
  // the normal exit does — otherwise the robot would be deaf to the zone for
  // the rest of the run.
  missionLeaveEvac();
  CHECK(!evacLaneActive());
  CHECK(missionTrigger(silverBandAgreed()) == MISSION_EVENT_EVAC);
}

// --- The truth table --------------------------------------------------------

void testProbeTruthTableNamesLiveDeadAndFakeAndNothingElse() {
  CHECK(evacShouldCarry(VictimClass::Live));
  CHECK(evacShouldCarry(VictimClass::Dead));
  CHECK(!evacShouldCarry(VictimClass::Fake));
  CHECK(evacTargetFor(VictimClass::Live) == DepositTarget::Green);
  CHECK(evacTargetFor(VictimClass::Dead) == DepositTarget::Red);
  CHECK(evacTargetFor(VictimClass::Fake) == DepositTarget::None);

  // The four signatures, each held long enough to name.
  const struct {
    int reflectivity;
    bool conductive;
    VictimClass expected;
  } kCases[] = {{kReflective, true, VictimClass::Live},
                {kReflective, false, VictimClass::Fake},
                {kOpaque, false, VictimClass::Dead},
                {kOpaque, true, VictimClass::Fake}};
  for (const auto& testCase : kCases) {
    ZoneRig rig;
    // Capture live, so the truth table is visible in what the lane decides: a
    // shut mechanism would hide it behind "carries nothing".
    missionState().mechanismEnabled = true;
    enterZone(rig);
    reachProbe(rig, withContact(insideSilverBand()));
    CHECK(rig.state() == StateId::EvacProbe);
    reachClassify(rig, probing(insideSilverBand(), testCase.reflectivity, testCase.conductive));
    missionEvacDecide();
    CHECK(missionState().victim == testCase.expected);
    CHECK(missionState().carrying == evacShouldCarry(testCase.expected));
    CHECK(missionState().target == evacTargetFor(testCase.expected));
    // A fake is scored as located-and-classified, and counted as ignored.
    CHECK_EQ(rig.tally().live + rig.tally().dead + rig.tally().ignored, 1);
    if (testCase.expected == VictimClass::Fake) {
      CHECK_EQ(rig.tally().ignored, 1);
      CHECK_EQ(rig.tally().deposits, 0);
    }
  }
}

void testAVictimThatFlickersIsNeverNamed() {
  ZoneRig rig;
  enterZone(rig);
  reachProbe(rig, withContact(insideSilverBand()));

  // Alternating signatures never fill the settling window, so the lane times
  // out on its own budget and names nothing: a flickering sample is not a
  // victim, it is a fault.
  rig.feedAlternatingUntilLeave(probing(insideSilverBand(), kReflective, true),
                                probing(insideSilverBand(), kOpaque, false),
                                MISSION_EVAC_PROBE_MAX_MS);
  CHECK(rig.state() == StateId::Idle);
  CHECK(!evacLaneActive());
  CHECK(missionState().victim == VictimClass::Fake);
  CHECK_EQ(rig.tally().live + rig.tally().dead + rig.tally().ignored, 0);
}

// --- Deposit accounting -----------------------------------------------------

void testAccountingIsWrittenBeforeTheRelease() {
  clearLog();
  ZoneRig rig;
  missionState().mechanismEnabled = true;
  enterZone(rig);
  reachProbe(rig, withContact(insideSilverBand()));
  reachClassify(rig, probing(insideSilverBand(), kReflective, true));  // live
  reachDeposit(rig, withTriangle(following(), MISSION_TRIANGLE_GREEN));
  CHECK(missionState().carrying);
  CHECK(missionState().target == DepositTarget::Green);

  rig.feed(following());
  CHECK(rig.state() == StateId::EvacExit);
  // A power cut between the two can only leave a recorded deposit, never a
  // released sphere with nothing written down.
  CHECK_EQ(g_logCount, 2);
  CHECK_EQ(g_log[0], 'A');
  CHECK_EQ(g_log[1], 'R');
  CHECK_EQ(g_recordCount, 1);
  CHECK(g_records[0].victim == VictimClass::Live);
  CHECK(g_records[0].target == DepositTarget::Green);
  CHECK(g_records[0].released);
  CHECK_EQ(g_records[0].deposits, 1);
  CHECK_EQ(rig.tally().deposits, 1);
  CHECK(evacLastAccounting().released);

  // The publish is one-shot: a second tick in the same state does not write a
  // second deposit or release a second sphere.
  clearLog();
  missionEvacDeposit();
  missionEvacDeposit();
  CHECK_EQ(g_logCount, 0);
  CHECK_EQ(rig.tally().deposits, 1);
}

void testAFakeIsScoredAndNeverReleased() {
  clearLog();
  ZoneRig rig;
  missionState().mechanismEnabled = true;
  enterZone(rig);
  reachProbe(rig, withContact(insideSilverBand()));
  reachClassify(rig, probing(insideSilverBand(), kReflective, false));  // silver, not conductive
  CHECK(missionState().victim == VictimClass::Fake);
  CHECK(!missionState().carrying);
  CHECK(missionState().target == DepositTarget::None);
  // A fake needs no triangle confirmed, so it is never made to wait for one.
  reachDeposit(rig, following());
  rig.feed(following());
  CHECK(rig.state() == StateId::EvacExit);
  CHECK_EQ(g_logCount, 1);
  CHECK_EQ(g_log[0], 'A');
  CHECK(g_records[0].target == DepositTarget::None);
  CHECK(!g_records[0].released);
  CHECK_EQ(g_records[0].deposits, 0);
  CHECK_EQ(rig.tally().ignored, 1);
  CHECK_EQ(rig.tally().deposits, 0);
}

void testTheDepositTargetIsCrossValidatedAgainstTheTriangle() {
  ZoneRig rig;
  missionState().mechanismEnabled = true;
  enterZone(rig);
  reachProbe(rig, withContact(insideSilverBand()));
  reachClassify(rig, probing(insideSilverBand(), kReflective, true));  // live
  CHECK(evacTargetConfirmed(withTriangle(following(), MISSION_TRIANGLE_GREEN), DepositTarget::Green));
  CHECK(!evacTargetConfirmed(withTriangle(following(), MISSION_TRIANGLE_RED), DepositTarget::Green));
  CHECK(!evacTargetConfirmed(following(), DepositTarget::Green));
  MissionInputs smallTriangle = withTriangle(following(), MISSION_TRIANGLE_GREEN);
  smallTriangle.triangleSize = MISSION_EVAC_TRIANGLE_MIN_SIZE - 1;
  CHECK(!evacTargetConfirmed(smallTriangle, DepositTarget::Green));
  MissionInputs staleTriangle = withTriangle(following(), MISSION_TRIANGLE_GREEN);
  staleTriangle.triangleFresh = false;
  CHECK(!evacTargetConfirmed(staleTriangle, DepositTarget::Green));
  // No target to confirm means nothing to wait for.
  CHECK(evacTargetConfirmed(following(), DepositTarget::None));

  // Held in the classify state while the right triangle has not shown up, and
  // released the moment it does. The decision itself is made on the classify
  // state's first pass, which is this one: entering a state runs the previous
  // state's action, not its own.
  rig.feed(following());
  CHECK(rig.state() == StateId::EvacClassify);
  CHECK(missionState().target == DepositTarget::Green);
  rig.feed(withTriangle(following(), MISSION_TRIANGLE_RED));
  CHECK(rig.state() == StateId::EvacClassify);
  rig.feed(withTriangle(following(), MISSION_TRIANGLE_GREEN));
  CHECK(rig.state() == StateId::EvacDeposit);
}

// --- Exit -------------------------------------------------------------------

void testExitIsDetectedOnBlackTapeAndOnlyOnBlackTape() {
  CHECK(evacBlackTape(blackBand()));
  MissionInputs silver = blackBand();
  silver.evacTapeType = MISSION_TAPE_SILVER;
  CHECK(!evacBlackTape(silver));
  MissionInputs stale = blackBand();
  stale.evacTapeFresh = false;
  CHECK(!evacBlackTape(stale));
  MissionInputs brightBar = blackBand();
  brightBar.line = insideSilverBand().line;
  CHECK(!evacBlackTape(brightBar));

  // Backing out runs until the bar finds black; on black it follows the tape
  // away, carrying or not.
  beginMission();
  setMissionInputs(insideSilverBand());
  missionState().carrying = true;
  CHECK_EQ(evacExitStep(missionState()).left, -MISSION_EVAC_BACKOUT_SPEED);
  CHECK_EQ(evacExitStep(missionState()).right, -MISSION_EVAC_BACKOUT_SPEED);
  setMissionInputs(blackBand());
  CHECK_EQ(evacExitStep(missionState()).left, MISSION_EVAC_EXIT_SPEED);
  CHECK_EQ(evacExitStep(missionState()).right, MISSION_EVAC_EXIT_SPEED);
}

// --- The no-capture lane ----------------------------------------------------

void testTheNoCaptureLaneStillScoresAndStillExits() {
  clearLog();
  ZoneRig rig;
  // The gate is shut, which is the default: capture is best-effort and the
  // locate/classify credit must never depend on it.
  CHECK(!missionState().mechanismEnabled);
  enterZone(rig);
  CHECK(rig.state() == StateId::EvacApproach);

  reachProbe(rig, withContact(insideSilverBand()));
  CHECK(rig.state() == StateId::EvacProbe);
  reachClassify(rig, probing(insideSilverBand(), kOpaque, false));  // dead -> red
  reachDeposit(rig, following());
  rig.feed(following());
  CHECK(rig.state() == StateId::EvacExit);
  CHECK(rig.last().mode == DriveMode::Stop);  // holding still, releasing nothing

  // Scored and classified, with no capture and no release at all.
  CHECK_EQ(rig.tally().dead, 1);
  CHECK_EQ(rig.tally().deposits, 0);
  CHECK_EQ(g_logCount, 1);
  CHECK_EQ(g_log[0], 'A');
  CHECK(g_records[0].target == DepositTarget::Red);
  CHECK(!g_records[0].released);
  CHECK(!missionState().carrying);

  // And the exit is the black tape, not a timeout: the exit gave the robot back
  // on the first pass that showed black, well inside its own budget.
  rig.feed(blackBand());
  CHECK(rig.state() == StateId::Idle);
  CHECK(!evacLaneActive());
  CHECK(rig.leftAtResidenceMs < MISSION_EVAC_EXIT_MAX_MS);
  // The lane's own copy of the verdict is gone with the entry; the record is not,
  // which is the whole reason the accounting is written before anything moves.
  CHECK(missionState().victim == VictimClass::Fake);
  CHECK(g_records[0].victim == VictimClass::Dead);
}

// What the lane asks the motors for. A state's own demand lands on the pass after
// it is entered, because the engine runs the action of whatever state it is in
// when the pass starts — so each check here is one pass past the transition.
void testTheLaneCreepsWhileSearchingAndHoldsWhileWorking() {
  ZoneRig rig;
  enterZone(rig);
  rig.feed(following());
  CHECK(rig.last().mode == DriveMode::LineFollow);
  CHECK_EQ(rig.last().left, MISSION_EVAC_SEARCH_SPEED);
  // Inside the band the black line is gone, so the search is a gyro-guided slow
  // straight rather than a line follow.
  rig.feed(silverBandAgreed());
  CHECK(rig.last().mode == DriveMode::GyroStraight);
  CHECK_EQ(rig.last().left, MISSION_EVAC_SEARCH_SPEED);

  // Holding still is how a victim is read: a robot that drives on takes the tips
  // off the sphere inside the sampling window.
  reachProbe(rig, withContact(insideSilverBand()));
  rig.feed(probing(insideSilverBand(), kOpaque, false));
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
  CHECK_EQ(missionState().stableSamples, 1);

  // And holding still is how a deposit is made: the accounting is written and
  // the release made without the robot moving a millimetre.
  reachClassify(rig, probing(insideSilverBand(), kOpaque, false));
  reachDeposit(rig, following());
  rig.feed(following());
  CHECK(rig.state() == StateId::EvacExit);
  CHECK(rig.last().mode == DriveMode::Stop);
  CHECK_EQ(rig.last().left, 0);
}

void testACarriedVictimIsApproachedMoreSlowly() {
  beginMission();
  setMissionInputs(following());
  missionState().carrying = false;
  CHECK_EQ(evacApproachStep(missionState()).left, MISSION_EVAC_SEARCH_SPEED);
  missionState().carrying = true;
  CHECK_EQ(evacApproachStep(missionState()).left, MISSION_EVAC_CARRY_SPEED);
  CHECK(evacApproachStep(missionState()).left < MISSION_EVAC_SEARCH_SPEED);

  // Off the line it is a gyro-guided slow straight, and a degraded link buys
  // no heading assist at all.
  setMissionInputs(insideSilverBand());
  missionState().carrying = false;
  MissionInputs turned = insideSilverBand();
  turned.imuFresh = true;
  turned.yawCentiDeg = static_cast<int16_t>(MISSION_CENTI_DEG_PER_DEG * 4);
  setMissionInputs(turned);
  const int32_t assist = headingAssist(4 * MISSION_CENTI_DEG_PER_DEG);
  CHECK(assist > 0);
  CHECK_EQ(evacApproachStep(missionState()).left, MISSION_EVAC_SEARCH_SPEED + assist);
  turned.policy = LinkPolicy::DegradeToLineOnly;
  setMissionInputs(turned);
  CHECK_EQ(evacApproachStep(missionState()).left, MISSION_EVAC_SEARCH_SPEED);
}

// --- Bounds -----------------------------------------------------------------

void testEveryLaneStateCarriesItsOwnFiniteBudget() {
  // The bounds are named, finite, and the same numbers the rows carry.
  for (uint8_t step = 0; step < kEvacChainLength; ++step) {
    const StateId state = kEvacChain[step];
    const uint32_t budget = evacResidenceBudgetMs(state);
    CHECK(budget != RESIDENCE_UNBOUNDED);
    CHECK(budget > 0);
    CHECK_EQ(kStateTable[static_cast<uint8_t>(state)].max_residence_ms, budget);
  }
  CHECK_EQ(evacResidenceBudgetMs(StateId::Idle), RESIDENCE_UNBOUNDED);
  CHECK_EQ(evacResidenceBudgetMs(StateId::Boot), RESIDENCE_UNBOUNDED);
}

void testAStateThatCannotFinishGivesTheRobotBack() {
  // Approach: nothing under the plate, so the search runs out its own budget.
  {
    ZoneRig rig;
    enterZone(rig);
    rig.feedUntilLeave(insideSilverBand(), MISSION_EVAC_APPROACH_MAX_MS);
    CHECK(rig.state() == StateId::Idle);
    CHECK(!evacLaneActive());
  }
  // Classify: a live victim whose green triangle never shows up.
  {
    ZoneRig rig;
    clearLog();
    missionState().mechanismEnabled = true;
    enterZone(rig);
    reachProbe(rig, withContact(insideSilverBand()));
    reachClassify(rig, probing(insideSilverBand(), kReflective, true));
    rig.feedUntilLeave(following(), MISSION_EVAC_CLASSIFY_MAX_MS);
    CHECK(rig.state() == StateId::Idle);
    CHECK(!evacLaneActive());
    CHECK_EQ(g_recordCount, 0);  // nothing deposited, so nothing recorded
  }
  // Exit: no black tape, so the back-out runs out its own budget.
  {
    ZoneRig rig;
    enterZone(rig);
    reachProbe(rig, withContact(insideSilverBand()));
    reachClassify(rig, probing(insideSilverBand(), kOpaque, false));
    reachDeposit(rig, following());
    rig.feed(following());  // the deposit completes itself, handing over to the exit
    CHECK(rig.state() == StateId::EvacExit);
    rig.feedUntilLeave(insideSilverBand(), MISSION_EVAC_EXIT_MAX_MS);
    CHECK(rig.state() == StateId::Idle);
    CHECK(!evacLaneActive());
  }
  // A second pass through the zone starts clean, because the bail dropped the
  // token: the robot is not deaf to the zone for the rest of the run.
  {
    ZoneRig rig;
    enterZone(rig);
    rig.feedUntilLeave(insideSilverBand(), MISSION_EVAC_APPROACH_MAX_MS);
    enterZone(rig);
    CHECK(rig.state() == StateId::EvacApproach);
  }
}

void testTheDepositStateCompletesItself() {
  // The state has nothing to wait for, so its bound is the backstop alone.
  ZoneRig rig;
  enterZone(rig);
  reachProbe(rig, withContact(insideSilverBand()));
  reachClassify(rig, probing(insideSilverBand(), kOpaque, false));
  reachDeposit(rig, following());
  rig.feedUntilLeave(following(), MISSION_EVAC_DEPOSIT_MAX_MS);
  CHECK(rig.state() != StateId::EvacDeposit);
  CHECK(evacDepositSettled());
}

void testATriangleClassAloneNeverEndsTheZone() {
  // A fresh red triangle out on the course is a deposit target, not an entry:
  // the zone opens on the silver band, not on a marker blob.
  CHECK(!evacEntryArmed(withTriangle(following(), MISSION_TRIANGLE_RED)));
  CHECK(missionTrigger(withTriangle(following(), MISSION_TRIANGLE_GREEN)) == EVENT_NONE);
}

// --- The lane's own conditions ----------------------------------------------
// The table calls these four on every pass, and the chain tests above assert
// what they do through the shipped rows. Called here as well, so each one is
// visibly a claim about evidence rather than a callback that happens to fire.

void testTheLaneConditionsAreClaimsAboutEvidence() {
  // Contact is the never-fail backstop; a close range only corroborates it.
  CHECK(evacAtProbePoint(withContact(following())));
  MissionInputs close = following();
  close.tofFresh = true;
  close.tofMm = MISSION_EVAC_CONTACT_MM;
  CHECK(evacAtProbePoint(close));
  MissionInputs far = close;
  far.tofMm = MISSION_EVAC_CONTACT_MM + 1;
  CHECK(!evacAtProbePoint(far));
  // A reading that is not fresh is no contact whatever its number says: the
  // validity flag is the status byte, and it was checked at the edge.
  MissionInputs stale = close;
  stale.tofFresh = false;
  CHECK(!evacAtProbePoint(stale));

  // One sample a pass, and the window fills only on agreement.
  beginMission();
  const MissionInputs live = probing(following(), kReflective, true);
  missionEvacProbeTick(live);
  CHECK_EQ(missionState().stableSamples, 1);
  CHECK(!evacProbeSettled());
  missionEvacProbeTick(live);
  missionEvacProbeTick(live);
  CHECK(evacProbeSettled());
  CHECK(missionState().victim == VictimClass::Live);
  // Disagreement resets the window rather than adding to it, and the class
  // already named stands: a victim does not un-become itself mid-sampling.
  missionEvacProbeTick(probing(following(), kOpaque, false));
  CHECK_EQ(missionState().stableSamples, 1);
  CHECK(!evacProbeSettled());
  CHECK(missionState().victim == VictimClass::Live);

  // Ready means decided, and its target corroborated — or no target to wait for.
  missionState().decisionRecorded = false;
  missionState().target = DepositTarget::Green;
  setMissionInputs(following());
  CHECK(!evacClassReady());
  missionState().decisionRecorded = true;
  CHECK(!evacClassReady());
  setMissionInputs(withTriangle(following(), MISSION_TRIANGLE_GREEN));
  CHECK(evacClassReady());
  missionState().target = DepositTarget::None;
  setMissionInputs(following());
  CHECK(evacClassReady());
}

}  // namespace

int main() {
  testTableGrowsTheEvacChainAndBoundsEveryState();
  testTheBackstopUnderEveryLaneStateStopsTheMotors();
  testEntryNeedsTheBlobAndTheBrightClassTogether();
  testTriggerNeverFiresWithoutTheEntryToken();
  testLeavingTheZoneReleasesTheEntryToken();
  testProbeTruthTableNamesLiveDeadAndFakeAndNothingElse();
  testAVictimThatFlickersIsNeverNamed();
  testAccountingIsWrittenBeforeTheRelease();
  testAFakeIsScoredAndNeverReleased();
  testTheDepositTargetIsCrossValidatedAgainstTheTriangle();
  testExitIsDetectedOnBlackTapeAndOnlyOnBlackTape();
  testTheNoCaptureLaneStillScoresAndStillExits();
  testTheLaneCreepsWhileSearchingAndHoldsWhileWorking();
  testACarriedVictimIsApproachedMoreSlowly();
  testEveryLaneStateCarriesItsOwnFiniteBudget();
  testAStateThatCannotFinishGivesTheRobotBack();
  testTheDepositStateCompletesItself();
  testATriangleClassAloneNeverEndsTheZone();
  testTheLaneConditionsAreClaimsAboutEvidence();

  std::printf("evac_test: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
