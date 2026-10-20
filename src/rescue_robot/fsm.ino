// fsm.ino — AVR delivery layer for the mission FSM engine, and the table itself.
//
// Owns the three things fsm.h deliberately knows nothing about: the elapsed time
// source (millis), the watchdog peripheral, and the per-loop tick call. Every
// mission decision lives in the transition table below or in the named handlers
// in mission.h that the table calls — not here.
//
// Non-blocking by construction: this file never calls delay() and never spins.
// The whole tick is table-walking over a fixed-size structure.

#include <Arduino.h>
#include <avr/wdt.h>

#include "fsm.h"
#include "mission.h"

using namespace rescue;

// The mission pass's two entry points. main.ino owns setup()/loop() — the boot
// table brings the engine up with fsmBegin(), and the loop pass hands the
// mission its single tick. Their definitions live in mission.ino.
void missionBegin();
void missionTick();

namespace {

// --- Reflex handlers: the glue between the table and the named steps -------
//
// Entry bookkeeping is shared by all seven reflexes; the per-tick action picks
// the shape its own state calls for; the guard is the reflex's exit condition.
// The engine runs on_action before it resolves a row, so a reflex that sees the
// line come back leaves on that same pass instead of a pass later.

void reflexEnter(FsmEngine&) { missionEnterReflex(); }

void reflexIdle(FsmEngine&) { missionDeliver(missionIdleDemand()); }

void reflexRun(FsmEngine& engine) {
  MissionState& state = missionState();
  switch (engine.state()) {
    case StateId::GapBridging:
      missionCountClearSamples(onLineClear(state.inputs.line));
      missionDeliver(gapStep(state.inputs, state.heldCorrection, state.yawAtEntry));
      return;
    case StateId::MarkerTurn:
      missionDeliver(pivotStep(turnForMarker(state.inputs), MISSION_TURN_SPEED));
      return;
    case StateId::DeadEnd:
      missionDeliver(pivotStep(TurnDirection::Left, MISSION_TURN_SPEED));
      return;
    case StateId::RampClimb:
      missionDeliver(rampStep(state.inputs));
      return;
    case StateId::SeesawCross:
      missionDeliver(seesawStep(state.inputs, state.seesawMockVerified, state.yawAtEntry));
      return;
    case StateId::ObstacleArc: {
      const MissionInputs& in = state.inputs;
      // While a zone is still pressed the robot is deliberately off the line,
      // so a clean centroid sample means nothing. Re-acquisition only starts
      // counting once the contact has gone and the bar can find the line again.
      missionCountClearSamples(!in.bumperContact && onLineClear(in.line));
      missionDeliver(obstacleStep(in));
      return;
    }
    default:
      missionDeliver(missionStopDemand());
      return;
  }
}

// The exit condition each reflex's own row asks before it will hand back to
// Idle: the line came back cleanly, or the budget it was granted is spent.
bool reflexDone(FsmEngine& engine) {
  const MissionState& state = missionState();
  const uint32_t residence = engine.residenceMs();
  switch (engine.state()) {
    case StateId::GapBridging:
      return state.clearSamples >= MISSION_GAP_CLEAR_SAMPLES || residence >= MISSION_GAP_MAX_MS;
    case StateId::MarkerTurn:
      return residence >= MISSION_MARKER_TURN_MS;
    case StateId::DeadEnd:
      return residence >= MISSION_DEAD_END_PIVOT_MS;
    case StateId::RampClimb:
      return terrainFor(state.inputs) != Terrain::Ramp || residence >= MISSION_RAMP_MAX_MS;
    case StateId::SeesawCross:
      return residence >= MISSION_SEESAW_CROSS_MS;
    case StateId::ObstacleArc:
      return state.clearSamples >= MISSION_OBSTACLE_CLEAR_SAMPLES ||
             residence >= MISSION_OBSTACLE_MAX_MS;
    case StateId::GoalHold:
      return residence >= MISSION_GOAL_HOLD_MS;
    default:
      return false;
  }
}

// The backstop: if a residence guard ever failed to pass, stop rather than drive.
void reflexBackstop(FsmEngine&) { missionDeliver(missionStopDemand()); }

// --- The zone: a bounded chain, not a reflex --------------------------------
// The five chain states share one entry and one per-tick handler. Each state's
// two rows are tried in order — advance to the next state, then give the tick
// back to Idle — and each row's budget is the number the bail guard compares
// against, so the bail normally wins the race and the backstop is the net under
// it.

void evacEnter(FsmEngine&) { missionEnterEvac(); }
void evacBackstop(FsmEngine&) { missionDeliver(missionStopDemand()); }

// Ending the zone is a row action, not a state's on_exit. By the time on_exit
// runs the engine has not yet decided where it is going as far as the callback
// can see, so one shared exit hook could not tell stepping along the chain from
// bailing out of it — and mistaking the two would drop the entry token, the
// held probe verdict and the tally on every ordinary step. Arming the teardown
// on the rows that land back on Idle is the same decision, made where it is
// actually knowable: the bail row and the exit's own finish row, and nothing
// else.
void evacZoneOver(FsmEngine&) { missionLeaveEvac(); }

void evacRun(FsmEngine& engine) {
  MissionState& state = missionState();
  switch (engine.state()) {
    case StateId::EvacApproach:
      missionDeliver(evacApproachStep(state));
      return;
    case StateId::EvacProbe:
      // Reading a victim is a hold: the probe tips have to stay on one sphere
      // for the sampling window, and a robot that drives on moves them.
      missionEvacProbeTick(state.inputs);
      missionDeliver(missionStopDemand());
      return;
    case StateId::EvacClassify:
      missionEvacDecide();
      missionDeliver(missionStopDemand());
      return;
    case StateId::EvacDeposit:
      // Publishing the accounting before the release is the whole job here.
      missionEvacDeposit();
      missionDeliver(missionStopDemand());
      return;
    case StateId::EvacExit:
      missionDeliver(evacExitStep(state));
      return;
    default:
      missionDeliver(missionStopDemand());
      return;
  }
}

// Advance: this state's own completion condition, or nothing.
bool evacAdvance(FsmEngine& engine) {
  const MissionState& state = missionState();
  switch (engine.state()) {
    case StateId::EvacApproach: return evacAtProbePoint(state.inputs);
    case StateId::EvacProbe: return evacProbeSettled();
    case StateId::EvacClassify: return evacClassReady();
    case StateId::EvacDeposit: return evacDepositSettled();
    case StateId::EvacExit: return evacBlackTape(state.inputs);
    default: return false;
  }
}

// Give up: this state's own budget is spent.
bool evacBudgetSpent(FsmEngine& engine) {
  return engine.residenceMs() >= evacResidenceBudgetMs(engine.state());
}

// Entering the zone is armed from the two-source claim, re-read at the moment
// the row fires: a snapshot that went stale between the post and the resolve
// does not open the zone.
bool evacEntryArmedNow(FsmEngine&) { return evacEntryArmed(missionState().inputs); }

// --- The restart: the same bounded shape, four times ------------------------
//
// Four states sharing one entry, one per-tick handler, one backstop. Each has
// two rows: hand forward when its own condition is met, then hand forward again
// once its own budget is spent — except the resume, whose bail row lands on Idle
// because a search that never re-centres has to give the run back on purpose.
// Every budget is the single number the bail guard reads, and the sweep is armed
// on entry so the radar is running concurrently with the search rather than
// before it.

void lopBackstop(FsmEngine&) { missionDeliver(missionStopDemand()); }

void lopEnter(FsmEngine& engine) {
  MissionState& state = missionState();
  switch (engine.state()) {
    case StateId::LoPRadarAssist:
      // Ask for the sweep here, once, before the search starts: the radar takes
      // seconds to come back and the bar must not sit still waiting for it.
      missionLopArmSweep();
      return;
    case StateId::LoPReacquire:
    case StateId::LoPResume:
      // Both are a line that has to be proved again from this state's own first
      // pass, so neither inherits the search's or the handshake's samples.
      state.clearSamples = 0;
      return;
    default:
      return;
  }
}

void lopRun(FsmEngine& engine) {
  MissionState& state = missionState();
  const uint32_t residence = engine.residenceMs();
  switch (engine.state()) {
    case StateId::LoPRadarAssist:
    case StateId::LoPLost:
      // Standing still is the whole job of both: the bar is over bare floor and
      // any wheel motion here is motion away from a line it cannot see.
      missionDeliver(missionStopDemand());
      return;
    case StateId::LoPReacquire:
      missionCountClearSamples(onLineClear(state.inputs.line));
      // The one re-sweep the profile that asked for it is worth. Asked here,
      // while the search runs, so it too is concurrent and never waited on.
      if (lopReSweepWanted(state.inputs)) missionLopArmSweep();
      missionDeliver(lopSearchStep(state, residence));
      return;
    case StateId::LoPResume:
      missionCountClearSamples(onLineClear(state.inputs.line));
      missionLopTrackLandmark(state.inputs);
      missionDeliver(lopResumeStep(state));
      return;
    default:
      missionDeliver(missionStopDemand());
      return;
  }
}

// Each state's own completion condition, or nothing: the handshake settled, the
// sweep is armed, the bar re-acquired, the line back under it. None of them can
// be about the link being kind.
bool lopAdvance(FsmEngine& engine) {
  const MissionState& state = missionState();
  switch (engine.state()) {
    case StateId::LoPRadarAssist: return lopSweepArmed();
    case StateId::LoPReacquire: return lopSearchClear(state);
    case StateId::LoPResume: return lopResumeReady(state);
    default: return false;
  }
}

// Give up on this state's own budget. Handing forward is still the answer for
// the first three: a search that has spent its window still has to rejoin the
// line before the run may be handed back.
bool lopBudgetSpent(FsmEngine& engine) {
  return engine.residenceMs() >= lopResidenceBudgetMs(engine.state());
}

// The READY window closing with no handshake is the restart's own fault report,
// and it is raised from the bail row rather than from the timeout: a timeout
// only reports, and a fault raised from one would leave the engine holding this
// state with stopped wheels. The flag latches for the run, because it describes
// what happened at the window and not what the link is doing now.
void lopHandshakeExpired(FsmEngine&) { missionState().handshakeFaulted = true; }

// Re-read at the moment the row fires, like the zone's entry claim: a READY
// that went stale between the post and the resolve does not settle a handshake.
bool lopHandshakeSettledNow(FsmEngine&) { return lopHandshakeSettled(missionState().inputs); }

// Both ways out of the resume publish the ledger, so a search that re-centred
// and one that deliberately gave up leave the same trail.
void lopResumeOver(FsmEngine&) { missionLopResumeComplete(); }

// Boot hands off to the restart chain only when the store says the last run was
// cut short with progress on it; a clean record, or one that never started, goes
// straight to Idle.
bool lopRestartPending(FsmEngine&) { return lopResumeRequired(); }
bool lopRestartNotPending(FsmEngine&) { return !lopResumeRequired(); }

// --- The table ------------------------------------------------------------
//
// Row N describes StateId(N); begin() verifies that mapping. Boot hands off to
// Idle on the next tick, once power and the timer are up. Idle lists one row per
// reflex event plus the zone's entry, each of which both arms and enters its own
// state, and carries no EVENT_ANY row on purpose: an uneventful tick must leave
// the state untouched, and a self-transition would reset the residence clock and
// re-fire on_entry on every single pass.
//
// Each reflex row then has exactly one transition back to Idle — EVENT_ANY with
// the reflex's own guard — plus the residence budget it was granted and the
// backstop as its on_timeout. Because the budget appears in both the guard and
// the row, the guard normally wins the race and the timeout is the net under it.
//
// The zone's five rows are that same shape with one row more: advance to the
// next chain state, or hand back to Idle. Each budget is the single number the
// bail guard reads, and the rows that land back on Idle are the ones that end
// the zone, so a bail out of the middle of the chain costs one pass, not the
// whole run.
//
// The restart's four rows are that shape twice over, with two differences worth
// naming. Three of the four bail FORWARD rather than home: a search that has
// spent its window has not finished its job, it has only stopped trying, and the
// run must not be handed back until the resume has had its own bounded turn to
// put the line back. And the handshake's bail row carries an action, because
// closing that window on nothing is the restart's own fault report and the flag
// has to be latched from the row that moved rather than from a timeout that only
// reports.

// Boot gained one row when the restart chain was added: the persisted ledger
// decides between the mission and the restart, and the mission is still the
// first hand-off, so a clean power-up still walks straight into Idle.
const Transition kBootTransitions[] = {
    {EVENT_ANY, StateId::Idle, lopRestartNotPending, nullptr},
    {EVENT_ANY, StateId::LoPLost, lopRestartPending, nullptr},
};

const Transition kIdleTransitions[] = {
    {MISSION_EVENT_CONTACT, StateId::ObstacleArc, nullptr, nullptr},
    {MISSION_EVENT_GOAL, StateId::GoalHold, nullptr, nullptr},
    {MISSION_EVENT_DEAD_END, StateId::DeadEnd, nullptr, nullptr},
    {MISSION_EVENT_EVAC, StateId::EvacApproach, evacEntryArmedNow, nullptr},
    {MISSION_EVENT_GAP, StateId::GapBridging, nullptr, nullptr},
    {MISSION_EVENT_MARKER, StateId::MarkerTurn, nullptr, nullptr},
    {MISSION_EVENT_RAMP, StateId::RampClimb, nullptr, nullptr},
    {MISSION_EVENT_SEESAW, StateId::SeesawCross, nullptr, nullptr},
};

// The one shape every reflex row shares: leave when the guard says so.
const Transition kReflexTransitions[] = {
    {EVENT_ANY, StateId::Idle, reflexDone, nullptr},
};

// The restart shape: settle on this state's own condition, then bail on its own
// budget. The handshake is the one state whose condition is a link receipt, and
// even it hands FORWARD when the window closes rather than home — the restart is
// not allowed to become a dead end. Both of the resume's rows publish the ledger,
// so the two ways out leave the same trail.
const Transition kLopLostTransitions[] = {
    {EVENT_ANY, StateId::LoPRadarAssist, lopHandshakeSettledNow, nullptr},
    {EVENT_ANY, StateId::LoPRadarAssist, lopBudgetSpent, lopHandshakeExpired},
};
const Transition kLopAssistTransitions[] = {
    {EVENT_ANY, StateId::LoPReacquire, lopAdvance, nullptr},
    {EVENT_ANY, StateId::LoPReacquire, lopBudgetSpent, nullptr},
};
const Transition kLopReacquireTransitions[] = {
    {EVENT_ANY, StateId::LoPResume, lopAdvance, nullptr},
    {EVENT_ANY, StateId::LoPResume, lopBudgetSpent, nullptr},
};
const Transition kLopResumeTransitions[] = {
    {EVENT_ANY, StateId::Idle, lopAdvance, lopResumeOver},
    {EVENT_ANY, StateId::Idle, lopBudgetSpent, lopResumeOver},
};

// The chain shape: advance first, then bail. Both rows name EVENT_ANY, because
// while the entry token is held nothing outside the zone can move the robot.
// Only the rows that land back on Idle carry the teardown.
const Transition kEvacApproachTransitions[] = {
    {EVENT_ANY, StateId::EvacProbe, evacAdvance, nullptr},
    {EVENT_ANY, StateId::Idle, evacBudgetSpent, evacZoneOver},
};
const Transition kEvacProbeTransitions[] = {
    {EVENT_ANY, StateId::EvacClassify, evacAdvance, nullptr},
    {EVENT_ANY, StateId::Idle, evacBudgetSpent, evacZoneOver},
};
const Transition kEvacClassifyTransitions[] = {
    {EVENT_ANY, StateId::EvacDeposit, evacAdvance, nullptr},
    {EVENT_ANY, StateId::Idle, evacBudgetSpent, evacZoneOver},
};
const Transition kEvacDepositTransitions[] = {
    {EVENT_ANY, StateId::EvacExit, evacAdvance, nullptr},
    {EVENT_ANY, StateId::Idle, evacBudgetSpent, evacZoneOver},
};
const Transition kEvacExitTransitions[] = {
    {EVENT_ANY, StateId::Idle, evacAdvance, evacZoneOver},
    {EVENT_ANY, StateId::Idle, evacBudgetSpent, evacZoneOver},
};

const StateDef kStateTable[] = {
    // id,             transitions,        count, max_residence,            on_entry,    on_exit, on_action,     on_timeout
    {StateId::Boot,      kBootTransitions,  2, RESIDENCE_UNBOUNDED,        nullptr,      nullptr, nullptr,      nullptr},
    {StateId::Idle,      kIdleTransitions,  8, RESIDENCE_UNBOUNDED,        nullptr,      nullptr, reflexIdle,   nullptr},
    {StateId::GapBridging, kReflexTransitions, 1, MISSION_GAP_MAX_MS,      reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::MarkerTurn, kReflexTransitions, 1, MISSION_MARKER_TURN_MS,   reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::DeadEnd, kReflexTransitions, 1, MISSION_DEAD_END_PIVOT_MS, reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::RampClimb, kReflexTransitions, 1, MISSION_RAMP_MAX_MS,      reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::SeesawCross, kReflexTransitions, 1, MISSION_SEESAW_CROSS_MS, reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::ObstacleArc, kReflexTransitions, 1, MISSION_OBSTACLE_MAX_MS, reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::GoalHold, kReflexTransitions, 1, MISSION_GOAL_HOLD_MS,      reflexEnter,  nullptr, reflexRun,    reflexBackstop},
    {StateId::LoPLost, kLopLostTransitions, 2, MISSION_LOP_READY_WAIT_MS, lopEnter, nullptr, lopRun, lopBackstop},
    {StateId::LoPRadarAssist, kLopAssistTransitions, 2, MISSION_LOP_SWEEP_ARM_MS, lopEnter, nullptr, lopRun, lopBackstop},
    {StateId::LoPReacquire, kLopReacquireTransitions, 2, MISSION_LOP_SEARCH_MAX_MS, lopEnter, nullptr, lopRun, lopBackstop},
    {StateId::LoPResume, kLopResumeTransitions, 2, MISSION_LOP_RESUME_MAX_MS, lopEnter, nullptr, lopRun, lopBackstop},
    {StateId::EvacApproach, kEvacApproachTransitions, 2, MISSION_EVAC_APPROACH_MAX_MS, evacEnter, nullptr, evacRun, evacBackstop},
    {StateId::EvacProbe, kEvacProbeTransitions, 2, MISSION_EVAC_PROBE_MAX_MS, evacEnter, nullptr, evacRun, evacBackstop},
    {StateId::EvacClassify, kEvacClassifyTransitions, 2, MISSION_EVAC_CLASSIFY_MAX_MS, evacEnter, nullptr, evacRun, evacBackstop},
    {StateId::EvacDeposit, kEvacDepositTransitions, 2, MISSION_EVAC_DEPOSIT_MAX_MS, evacEnter, nullptr, evacRun, evacBackstop},
    {StateId::EvacExit, kEvacExitTransitions, 2, MISSION_EVAC_EXIT_MAX_MS, evacEnter, nullptr, evacRun, evacBackstop},
};

constexpr uint8_t kStateCount = static_cast<uint8_t>(sizeof(kStateTable) / sizeof(kStateTable[0]));
static_assert(kStateCount == STATE_REGISTRY_SIZE, "every StateId needs a table row");

FsmEngine g_fsm(kStateTable, kStateCount);

uint32_t g_lastTickMs = 0;

}  // namespace

// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------
//
// The software miss counter in the engine is the authority during bring-up;
// the hardware watchdog is armed only after it is confirmed on the bench.
//
// WHY THE HARDWARE ENABLE IS COMMENTED OUT: on the Mega the first activation
// of the WDT changes the reset vector to the watchdog routine while the
// program-lock bits are still set, so a board that hangs once can no longer
// be re-flashed over USB until those bits are cleared by a timed ISP session.
// Arming it while the reset source (external reset button vs. brownout) and
// the timeout window are still unconfirmed risks exactly that lockout on a
// board that is hard to replace days before a qualifier. Uncomment only after:
//   * the WDT reset source is confirmed to be the hardware reset pin, and
//   * the timeout is long enough that a normal loop never approaches it, and
//   * the lock-bit clear procedure has been rehearsed once.
// With the WDT disabled, wdt_reset() below is a harmless no-op write.
void watchdogBegin() {
  // wdt_enable(WDTO_15MS);  // ARM ON THE BENCH ONLY — see the note above.
  (void)0;
}

// Kick the peripheral and tell the engine the loop is alive. The engine counts
// a loop pass that reaches tick() without this call as a missed kick and raises
// a latched fault after WATCHDOG_MISS_LIMIT in a row.
void watchdogKick() {
  wdt_reset();
  g_fsm.watchdogKick();
}

// ---------------------------------------------------------------------------
// Elapsed time
// ---------------------------------------------------------------------------
// The engine is handed the time that passed; it never reads a clock itself.
// The unsigned subtraction is wrap-safe across the ~49-day millis() rollover.
uint32_t elapsedSinceLastTick() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - g_lastTickMs;
  g_lastTickMs = now;
  return elapsed;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

// Arm the engine before the first loop pass.
void fsmBegin() {
  watchdogBegin();
  g_lastTickMs = millis();
  g_fsm.begin(StateId::Boot);
}

// One engine pass. Called once per main-loop iteration; the returned report is
// the per-loop slice the diagnostics menu reads (state, timeout, watchdog).
TickReport fsmTick() {
  watchdogKick();
  return g_fsm.tick(elapsedSinceLastTick(), EVENT_NONE);
}

// Queue one reflex event. The delivery layer decides WHEN an event is raised,
// never what it means; a false return means the ring was full, which the engine
// counts as a dropped event.
bool fsmPostEvent(EventId event) { return g_fsm.postEvent(event); }
