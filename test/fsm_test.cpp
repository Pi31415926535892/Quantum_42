// fsm_test.cpp — host-side tests for src/rescue_robot/fsm.h (pure C++14).
// Build/run:
//   g++ -std=c++14 -Itest -Isrc/rescue_robot test/fsm_test.cpp -o /tmp/ftest
//
// The engine is fed scripted event sequences and elapsed-time slices; every
// callback records into a shared trace so assertions can see what fired.

#include <string>

#include "dryframe.h"
#include "fsm.h"

using namespace rescue;

// dryframe's CHECK_EQ is a bare two-operand compare; this variant keeps the
// caller's sentence, which matters where the same comparison proves two
// different claims in the same test.
#define CHECK_EQ_MSG(a, b, msg)                                     \
  do {                                                              \
    const auto& _cma = (a);                                        \
    const auto& _cmb = (b);                                        \
    dryframe::report(_cma == _cmb,                                 \
                     std::string(msg) + " [" #a " == " #b "]",     \
                     __FILE__, __LINE__);                           \
  } while (0)

namespace {

// Elapsed-time slice and the Boot liveness bound it probes against.
constexpr uint32_t kSliceMs = 10;
constexpr uint32_t kBootResidenceMs = 40;

// Scripted events — each handler owns its own event ids.
constexpr EventId kEventGo = 1;
constexpr EventId kEventStop = 2;

// Observed callback trace.
int g_entries = 0;
int g_exits = 0;
int g_actions = 0;
int g_timeouts = 0;
int g_rowFires = 0;
uint16_t g_guardCalls = 0;
bool g_guardPass = false;

FsmEngine* g_engine = nullptr;
bool g_reenterArmed = false;
int g_nestedTransitions = 0;

void bootEntry(FsmEngine&) { ++g_entries; }
void idleEntry(FsmEngine&) { ++g_entries; }
void bootTimeout(FsmEngine&) { ++g_timeouts; }
void idleExit(FsmEngine&) { ++g_exits; }
void rowFire(FsmEngine&) { ++g_rowFires; }

void idleAction(FsmEngine&) {
  ++g_actions;
  if (!g_reenterArmed) return;
  g_reenterArmed = false;
  if (g_engine->tick(kSliceMs, EVENT_NONE).transitioned) ++g_nestedTransitions;
}

bool guard(FsmEngine&) {
  ++g_guardCalls;
  return g_guardPass;
}

// Boot carries one event-driven guarded row; Idle carries one default row.
const Transition kBootRows[] = {
    {kEventGo, StateId::Idle, guard, rowFire},
};
const Transition kIdleRows[] = {
    {EVENT_ANY, StateId::Boot, guard, rowFire},
};
const StateDef kTable[] = {
    {StateId::Boot, kBootRows, 1, kBootResidenceMs, bootEntry, nullptr, nullptr,
     bootTimeout},
    {StateId::Idle, kIdleRows, 1, RESIDENCE_UNBOUNDED, idleEntry, idleExit,
     idleAction, nullptr},
};
constexpr uint8_t kTableSize = 2;

void resetTrace() {
  g_entries = 0;
  g_exits = 0;
  g_actions = 0;
  g_timeouts = 0;
  g_rowFires = 0;
  g_guardCalls = 0;
  g_guardPass = false;
  g_reenterArmed = false;
  g_nestedTransitions = 0;
}

// --- cases -----------------------------------------------------------------

void testBeginRejectsUnregisteredState() {
  FsmEngine engine(kTable, kTableSize);
  CHECK(!engine.begin(StateId::Count),
        "the registry sentinel is not a valid start state");
  FsmEngine empty(nullptr, 0);
  CHECK(!empty.begin(StateId::Boot), "an empty table cannot start");
  const TickReport report = empty.tick(kSliceMs, EVENT_NONE);
  CHECK(!report.transitioned, "a stopped engine ignores ticks");
  CHECK_EQ(int(empty.ticks()), 0);
}

void testDefaultTransition() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  CHECK(engine.begin(StateId::Idle), "begin on a registered state succeeds");
  g_guardPass = true;

  const TickReport entry = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(!entry.transitioned,
        "a default row does not fire on the state-entry tick");
  CHECK_EQ(int(entry.next), int(StateId::Idle));

  const TickReport settled = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(settled.transitioned, "the default row fires once the state has been ticked");
  CHECK_EQ(int(settled.next), int(StateId::Boot));
  CHECK_EQ(int(engine.state()), int(StateId::Boot));
  CHECK_EQ(g_rowFires, 1);
  CHECK_EQ(g_guardCalls, 1);
}

void testEventDrivenTransition() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  g_guardPass = true;
  CHECK(engine.begin(StateId::Boot), "begin on Boot");

  const TickReport idle = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(!idle.transitioned, "an empty queue leaves the state alone");
  CHECK(!idle.event_consumed, "no queued event means nothing was spent");
  CHECK_EQ(g_entries, 1);

  CHECK(engine.postEvent(kEventGo), "the event is accepted by the queue");
  const TickReport moved = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(moved.event_consumed, "the posted event is spent on its tick");
  CHECK(moved.transitioned, "an event-driven row fires");
  CHECK_EQ(int(moved.next), int(StateId::Idle));
  CHECK_EQ(int(engine.state()), int(StateId::Idle));
  CHECK_EQ(g_rowFires, 1);
  CHECK_EQ(g_entries, 2);
  CHECK_EQ(g_actions, 0);
}

void testResidenceTimeoutFiresOnce() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  g_guardPass = false;
  CHECK(engine.begin(StateId::Boot), "begin on Boot");

  engine.tick(kSliceMs, EVENT_NONE);
  engine.tick(kSliceMs, EVENT_NONE);
  engine.tick(kSliceMs, EVENT_NONE);
  CHECK_EQ_MSG(g_timeouts, 0, "under the residence bound nothing fires");
  CHECK_EQ(int(engine.residenceMs()), 30);

  engine.tick(kSliceMs, EVENT_NONE);
  CHECK_EQ_MSG(g_timeouts, 1, "reaching the residence bound raises the timeout");
  CHECK_EQ_MSG(int(engine.state()), int(StateId::Boot), "a timeout reports; it does not move the engine");

  for (int slice = 0; slice < 5; ++slice) {
    engine.tick(kSliceMs, EVENT_NONE);
  }
  CHECK_EQ_MSG(g_timeouts, 1, "the timeout fires exactly once per residence");
}

void testGuardRejection() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  CHECK(engine.begin(StateId::Boot), "begin on Boot");
  g_guardPass = false;
  CHECK(engine.postEvent(kEventGo), "the event is accepted");

  const TickReport rejected = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(rejected.event_consumed, "a guard rejection still spends the event");
  CHECK(!rejected.transitioned, "a false guard blocks the row");
  CHECK_EQ(int(engine.state()), int(StateId::Boot));
  CHECK_EQ_MSG(g_rowFires, 0, "the row action is skipped when the guard is false");
  CHECK_EQ(g_guardCalls, 1);

  g_guardPass = true;
  CHECK(engine.postEvent(kEventGo), "the event is re-posted");
  const TickReport allowed = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(allowed.transitioned, "an open guard lets the row fire");
  CHECK_EQ(int(engine.state()), int(StateId::Idle));
  CHECK_EQ(g_rowFires, 1);

  CHECK(engine.begin(StateId::Idle), "restart on Idle");
  g_guardPass = false;
  engine.tick(kSliceMs, EVENT_NONE);
  const TickReport blocked = engine.tick(kSliceMs, EVENT_NONE);
  CHECK(!blocked.transitioned, "a false guard also blocks a default row");
  CHECK_EQ(int(engine.state()), int(StateId::Idle));
}

void testEventQueueIsBounded() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  g_guardPass = false;  // no row may fire: this case only watches the queue
  CHECK(engine.begin(StateId::Idle), "begin on Idle");

  CHECK(!engine.postEvent(EVENT_NONE), "EVENT_NONE is never queued");
  for (uint8_t slot = 0; slot < EVENT_QUEUE_CAPACITY; ++slot) {
    CHECK(engine.postEvent(kEventStop), "a free slot accepts the event");
  }
  CHECK_EQ(int(engine.queuedEvents()), int(EVENT_QUEUE_CAPACITY));
  CHECK(!engine.postEvent(kEventGo), "a full queue rejects a new event");
  CHECK_EQ(int(engine.droppedEvents()), 1);
  CHECK_EQ_MSG(int(engine.queuedEvents()), int(EVENT_QUEUE_CAPACITY), "a rejected event changes nothing");
  CHECK_EQ_MSG(int(engine.pendingEvent()), int(kEventStop), "the oldest queued event is served first");

  for (uint8_t drain = 0; drain < EVENT_QUEUE_CAPACITY; ++drain) {
    engine.tick(kSliceMs, EVENT_NONE);
  }
  CHECK_EQ_MSG(int(engine.queuedEvents()), 0, "one event drains per tick");
  CHECK_EQ_MSG(int(engine.pendingEvent()), int(EVENT_NONE), "an empty queue reports no event");
  CHECK_EQ_MSG(g_rowFires, 0, "draining an unmatched event moves nothing");
}

void testWatchdogFaultsOnMissedKicks() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  g_guardPass = false;
  CHECK(engine.begin(StateId::Boot), "begin on Boot");

  for (uint8_t loop = 0; loop < WATCHDOG_MISS_LIMIT + 2; ++loop) {
    engine.watchdogKick();
    engine.tick(kSliceMs, EVENT_NONE);
  }
  CHECK_EQ_MSG(int(engine.missedKicks()), 0, "a kick every tick records no miss");
  CHECK(!engine.watchdogFaulted(), "a kicked loop raises no watchdog fault");
  CHECK_EQ(int(engine.watchdogFaultCount()), 0);

  for (uint8_t loop = 0; loop < WATCHDOG_MISS_LIMIT - 1; ++loop) {
    engine.tick(kSliceMs, EVENT_NONE);
  }
  CHECK(!engine.watchdogFaulted(), "a fault needs the full miss run");
  engine.tick(kSliceMs, EVENT_NONE);
  CHECK(engine.watchdogFaulted(), "the miss run raises a watchdog fault");
  CHECK_EQ(int(engine.watchdogFaultCount()), 1);
  engine.tick(kSliceMs, EVENT_NONE);
  CHECK_EQ_MSG(int(engine.watchdogFaultCount()), 1, "the fault latches");

  engine.watchdogKick();
  engine.tick(kSliceMs, EVENT_NONE);
  CHECK_EQ_MSG(int(engine.missedKicks()), 0, "a resumed kick clears the miss run");
  CHECK(engine.watchdogFaulted(), "the fault record stays latched");
  engine.clearWatchdogFault();
  CHECK(!engine.watchdogFaulted(), "the fault clears on request");
}

void testTickIsNotReentrant() {
  resetTrace();
  FsmEngine engine(kTable, kTableSize);
  g_engine = &engine;
  g_guardPass = false;
  CHECK(engine.begin(StateId::Idle), "begin on Idle");
  engine.tick(kSliceMs, EVENT_NONE);
  const int actionsAfterEntry = g_actions;

  g_reenterArmed = true;
  engine.tick(kSliceMs, EVENT_NONE);
  CHECK_EQ_MSG(int(engine.reentrancyBlockedCount()), 1, "a nested tick is refused");
  CHECK_EQ_MSG(g_nestedTransitions, 0, "the refused nested tick changes nothing");
  CHECK_EQ(int(engine.state()), int(StateId::Idle));
  CHECK_EQ_MSG(int(engine.ticks()), 2, "only the outer ticks are counted");
  CHECK_EQ_MSG(g_actions, actionsAfterEntry + 1, "the nested tick does not re-run callbacks");
}

}  // namespace

int main() {
  testBeginRejectsUnregisteredState();
  testDefaultTransition();
  testEventDrivenTransition();
  testResidenceTimeoutFiresOnce();
  testGuardRejection();
  testEventQueueIsBounded();
  testWatchdogFaultsOnMissedKicks();
  testTickIsNotReentrant();
  return DF_SUMMARIZE("fsm_test");
}
