// fsm.h — generic non-blocking mission-FSM engine (C++14, no AVR I/O).
//
// The engine owns no behavior: it walks a transition table, runs the callbacks
// the table names, and reports what happened. The reflex rows, the loss-of-
// position restart chain and the evacuation zone's chain rows now live in fsm.ino
// beside the table; mission.h owns the decisions those rows call, so nothing
// here needed rewriting to grow the mission — the zone was added the same way the
// reflexes were, as rows.
//
// Guarantees: one tick per main-loop pass with no blocking primitive on the
// path; a tick is never re-entered (a callback calling tick() is refused and
// counted); every state with a finite max-residence bound raises on_timeout
// exactly once per residence, so a wedged state cannot hold the robot silently;
// the event queue is a fixed-size ring, so a burst can never grow memory or
// stall the loop.
//
// PROGMEM / flash-resident tables: StateDef and Transition are plain aggregates
// of scalars and function pointers — no vtable, no RTTI, no std container, no
// heap — so the table can live in flash on the AVR. The engine never holds a
// pointer into the table; it copies the active StateDef and one Transition at a
// time (loadState/loadRow), so those two helpers are the ONLY places needing a
// flash read (memcpy_P) and no other line knows where the table was stored.
// State ids index the table directly, so row N must describe StateId(N) —
// begin() verifies that mapping on the way in.
#ifndef RESCUE_ROBOT_FSM_H
#define RESCUE_ROBOT_FSM_H

#include <cstdint>

namespace rescue {

// Event ids are an open value set, not a closed enum: every handler owns its
// own ids, so a new reflex never requires editing this header.
typedef uint16_t EventId;

constexpr EventId EVENT_NONE = 0;      // nothing happened this tick
constexpr EventId EVENT_ANY = 0xFFFF;  // default row: evaluated every tick

// Fixed ring depth. Overflow policy: REJECT THE NEW EVENT — the newest sensor
// event is what a reflex most wants, and a visible rejection lets the producer
// retry, whereas a silent overwrite loses the trigger with nothing to show.
constexpr uint8_t EVENT_QUEUE_CAPACITY = 8;
constexpr uint8_t EVENT_DROP_CEILING = 255;

// One enumerator per state the mission can occupy; the table grows with it and
// row N describes StateId(N). Count doubles as the not-started / invalid marker.
enum class StateId : uint8_t {
  Boot,
  Idle,
  GapBridging,
  MarkerTurn,
  DeadEnd,
  RampClimb,
  SeesawCross,
  ObstacleArc,
  GoalHold,
  // The restart chain, in the order it runs and ahead of the zone: wait out the
  // link handshake, arm the concurrent sweep, hunt the line back with the bar,
  // and resume on it. Each hands forward on its own condition and, spending its
  // own budget, forward again — except the last, which hands back to Idle on
  // purpose rather than searching for ever.
  LoPLost,
  LoPRadarAssist,
  LoPReacquire,
  LoPResume,
  // The zone chain, in the order it runs. Each state hands to the next when its
  // own condition is met and back to Idle when its own budget is spent.
  EvacApproach,
  EvacProbe,
  EvacClassify,
  EvacDeposit,
  EvacExit,
  Count,
};

constexpr uint8_t STATE_REGISTRY_SIZE = static_cast<uint8_t>(StateId::Count);

constexpr uint32_t RESIDENCE_UNBOUNDED = 0;    // state opts out of the guard
constexpr uint8_t WATCHDOG_MISS_LIMIT = 4;      // consecutive misses -> fault
constexpr uint8_t WATCHDOG_MISS_CEILING = 255;  // clamp on the miss counter

class FsmEngine;

// Callbacks receive the engine, so a handler can read state(), residenceMs(),
// or post its own. Null means "not interested".
typedef void (*StateCallback)(FsmEngine& engine);
typedef bool (*GuardCallback)(FsmEngine& engine);

// One row: the event that arms it, its destination, an optional guard that
// must pass first, and an optional action run when the row fires.
struct Transition {
  EventId event;          // a specific id, or EVENT_ANY for a default row
  StateId next;
  GuardCallback guard;    // null = unguarded; true = permitted
  StateCallback action;   // optional; runs when the row fires
};

struct StateDef {
  StateId id;
  const Transition* transitions;
  uint8_t transition_count;
  uint32_t max_residence_ms;  // RESIDENCE_UNBOUNDED = no liveness guard
  StateCallback on_entry;     // optional; on entry only
  StateCallback on_exit;      // optional; on leaving only
  StateCallback on_action;    // optional; every tick while active
  StateCallback on_timeout;   // optional; once per residence over the bound
};

// What one tick did; `next` is the state occupied at the end of it.
struct TickReport {
  bool transitioned;    // the state changed on this tick
  bool timed_out;       // the residence guard fired on this tick
  bool event_consumed;  // a queued event was spent (matched or not)
  StateId next;
};

class FsmEngine {
 public:
  // The table is indexed by state id and must outlive the engine.
  FsmEngine(const StateDef* table, uint8_t state_count);
  // Enter the initial state. False when it is not in the table.
  bool begin(StateId initial);
  // One main-loop pass: serves at most one queued event, adds elapsed_ms to the
  // residence clock, resolves at most one transition.
  TickReport tick(uint32_t elapsed_ms, EventId event);

  // Called every pass; a pass reaching tick() without one is a missed kick.
  void watchdogKick();
  // Queue an event. False when the queue is full or the event is EVENT_NONE.
  bool postEvent(EventId event);
  // Oldest queued event, EVENT_NONE when the queue is empty.
  EventId pendingEvent() const {
    return queued_ == 0 ? EVENT_NONE : queue_[head_];
  }
  uint8_t queuedEvents() const { return queued_; }
  uint8_t droppedEvents() const { return dropped_; }
  StateId state() const { return state_; }
  uint32_t residenceMs() const { return residence_ms_; }
  uint32_t ticks() const { return ticks_; }
  uint16_t missedKicks() const { return missed_kicks_; }
  bool watchdogFaulted() const { return faulted_; }
  uint32_t watchdogFaultCount() const { return fault_count_; }
  void clearWatchdogFault() { faulted_ = false; }
  uint32_t reentrancyBlockedCount() const { return reentrancy_blocked_; }

 private:
  bool registered(StateId id) const {
    if (table_ == nullptr) return false;
    const uint8_t index = static_cast<uint8_t>(id);
    return index < state_count_ && table_[index].id == id;
  }

  // The two flash-read seams — see the PROGMEM note at the top of the file.
  void loadState(StateId id, StateDef& out) const {
    out = table_[static_cast<uint8_t>(id)];
  }
  void loadRow(uint8_t index, Transition& out) const {
    out = current_.transitions[index];
  }

  bool tryRow(const Transition& row);
  bool resolveTransition(EventId event);
  bool residenceExpired() const;
  EventId takeEvent(EventId fallback);
  void accountWatchdog();
  TickReport runTick(uint32_t elapsed_ms, EventId event);
  const StateDef* table_;
  StateDef current_;
  Transition row_;
  EventId queue_[EVENT_QUEUE_CAPACITY];
  StateId state_;
  uint32_t residence_ms_;
  uint32_t ticks_;
  uint32_t fault_count_;
  uint32_t reentrancy_blocked_;
  uint16_t missed_kicks_;
  uint8_t state_count_;
  uint8_t head_;
  uint8_t tail_;
  uint8_t queued_;
  uint8_t dropped_;
  bool faulted_;
  bool kicked_;
  bool in_tick_;
  bool entered_this_tick_;
  bool timeout_fired_;
};

inline FsmEngine::FsmEngine(const StateDef* table, uint8_t state_count)
    : table_(table), state_(StateId::Count), state_count_(state_count) {}

// Enter the initial state and zero every run-scoped counter. False when the
// state is not in the table, which leaves the engine stopped.
inline bool FsmEngine::begin(StateId initial) {
  head_ = tail_ = queued_ = dropped_ = 0;
  ticks_ = fault_count_ = reentrancy_blocked_ = missed_kicks_ = 0;
  residence_ms_ = 0;
  faulted_ = kicked_ = in_tick_ = timeout_fired_ = false;
  state_ = StateId::Count;
  if (!registered(initial)) return false;
  loadState(initial, current_);
  state_ = initial;
  entered_this_tick_ = true;
  if (current_.on_entry != nullptr) current_.on_entry(*this);
  return true;
}

inline TickReport FsmEngine::tick(uint32_t elapsed_ms, EventId event) {
  const TickReport idle{false, false, false, state_};
  if (in_tick_) {
    ++reentrancy_blocked_;  // re-entrancy is a defect, not a supported call
    return idle;
  }
  if (state_ == StateId::Count) return idle;  // never began
  in_tick_ = true;
  const TickReport report = runTick(elapsed_ms, event);
  in_tick_ = false;
  return report;
}

inline TickReport FsmEngine::runTick(uint32_t elapsed_ms, EventId event) {
  ++ticks_;
  accountWatchdog();

  TickReport report{false, false, queued_ > 0, state_};
  const EventId served = takeEvent(event);
  residence_ms_ += elapsed_ms;

  // Per-tick work runs first: a reflex handler refreshes the conditions its own
  // guards read, so it must run before a row is resolved against them.
  if (current_.on_action != nullptr) current_.on_action(*this);

  if (resolveTransition(served)) {
    report.transitioned = true;
  } else if (residenceExpired()) {
    timeout_fired_ = true;
    report.timed_out = true;
    if (current_.on_timeout != nullptr) current_.on_timeout(*this);
  }

  entered_this_tick_ = false;
  report.next = state_;
  return report;
}

// Serve the oldest queued event, falling back to the caller's argument when
// the queue is empty (the fast path for callers that never queue).
inline EventId FsmEngine::takeEvent(EventId fallback) {
  if (queued_ == 0) return fallback;
  const EventId event = queue_[head_];
  head_ = static_cast<uint8_t>((head_ + 1) % EVENT_QUEUE_CAPACITY);
  --queued_;
  return event;
}

// Two ordered passes, so an explicit event always wins over a default row:
//   1. rows naming the served event (a state entry tick included)
//   2. EVENT_ANY rows, skipped on the entry tick so a freshly entered state
//      always gets one full tick to settle. A guard that must observe
//      settling time belongs in pass 2.
inline bool FsmEngine::resolveTransition(EventId event) {
  for (uint8_t index = 0; index < current_.transition_count; ++index) {
    loadRow(index, row_);
    if (row_.event == event && tryRow(row_)) return true;
  }
  if (entered_this_tick_) return false;
  for (uint8_t index = 0; index < current_.transition_count; ++index) {
    loadRow(index, row_);
    if (row_.event == EVENT_ANY && tryRow(row_)) return true;
  }
  return false;
}

// Guard -> action -> exit -> enter. A false guard spends the event without
// moving, so a permanently-closed guard can never wedge the queue.
inline bool FsmEngine::tryRow(const Transition& row) {
  if (row.guard != nullptr && !row.guard(*this)) return false;
  if (row.action != nullptr) row.action(*this);
  if (current_.on_exit != nullptr) current_.on_exit(*this);
  loadState(row.next, current_);
  state_ = row.next;
  residence_ms_ = 0;
  timeout_fired_ = false;
  entered_this_tick_ = true;
  if (current_.on_entry != nullptr) current_.on_entry(*this);
  return true;
}

inline bool FsmEngine::residenceExpired() const {
  if (timeout_fired_) return false;
  if (current_.max_residence_ms == RESIDENCE_UNBOUNDED) return false;
  return residence_ms_ >= current_.max_residence_ms;
}

inline bool FsmEngine::postEvent(EventId event) {
  if (event == EVENT_NONE) return false;
  if (queued_ >= EVENT_QUEUE_CAPACITY) {
    if (dropped_ < EVENT_DROP_CEILING) ++dropped_;
    return false;
  }
  queue_[tail_] = event;
  tail_ = static_cast<uint8_t>((tail_ + 1) % EVENT_QUEUE_CAPACITY);
  ++queued_;
  return true;
}

// A pass without a kick counts as missed. The fault latches at the limit: once
// the loop has stalled that long the record must outlive its recovery.
inline void FsmEngine::accountWatchdog() {
  if (kicked_) {
    missed_kicks_ = 0;
  } else if (missed_kicks_ < WATCHDOG_MISS_CEILING) {
    ++missed_kicks_;
  }
  kicked_ = false;
  if (missed_kicks_ >= WATCHDOG_MISS_LIMIT && !faulted_) {
    faulted_ = true;
    ++fault_count_;
  }
}

inline void FsmEngine::watchdogKick() { kicked_ = true; }

}  // namespace rescue

#endif  // RESCUE_ROBOT_FSM_H
