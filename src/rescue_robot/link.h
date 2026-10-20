// link.h — pure host-compilable Mega <-> Nicla host-link codec and state (C++14).
// Implements docs/mega-nicla-protocol.md: framing, CRC-8 (poly 0x07, init 0x00,
// no final xor, MSB-first), big-endian fields, the HUNT/FRAME re-sync machine,
// the last-good cache with per-type freshness, liveness with its first-transition
// events, and the live-run hold-and-degrade policy. LEN is authoritative: a
// receiver never infers payload size from a message id. No AVR identifiers, I/O
// or waiting here — link.ino owns Serial1, millis() and the TX ring.

#ifndef RESCUE_ROBOT_LINK_H
#define RESCUE_ROBOT_LINK_H
#include <cstddef>
#include <cstdint>
namespace rescue {

// Framing, cadence and re-sync (protocol 2.4, 3, 5.4, 5.5).
constexpr uint32_t LINK_BAUD = 115200;  // 8N1, no flow control, both ends
constexpr uint8_t LINK_SYNC0 = 0xAA;
constexpr uint8_t LINK_SYNC1 = 0x55;
constexpr uint8_t LINK_CRC8_POLY = 0x07;      // MSB-first, init 0x00, no xor
constexpr uint8_t LINK_CRC8_MSB = 0x80;
constexpr uint8_t LINK_CRC8_BITS = 8;
constexpr uint8_t LINK_LEN_MIN = 2;   // LEN counts message-id + payload + CRC
constexpr uint16_t LINK_LEN_MAX = 255;
constexpr uint8_t LINK_BODY_OVERHEAD = 2;   // message-id + CRC
constexpr uint8_t LINK_FRAME_OVERHEAD = 5;  // sync0 + sync1 + LEN + id + CRC
constexpr uint8_t LINK_MAX_PAYLOAD = LINK_LEN_MAX - LINK_BODY_OVERHEAD;
constexpr uint16_t LINK_MAX_FRAME_BYTES = LINK_MAX_PAYLOAD + LINK_FRAME_OVERHEAD;
constexpr uint8_t LINK_TX_BUFFER_BYTES = 12;   // widest Mega -> Nicla frame
constexpr uint8_t LINK_RX_BYTE_BUDGET = 64;   // bytes drained per loop tick
constexpr uint8_t LINK_BAD_FRAME_LIMIT = 3;   // bad frames before a re-sync
constexpr uint32_t LINK_HEARTBEAT_MS = 250;   // send cadence, never faster
// Liveness, freshness and degrade windows (protocol 5.1-5.3).
constexpr uint32_t LINK_LIVENESS_FRESH_MS = 1000;  // age <  this -> FRESH
constexpr uint32_t LINK_LIVENESS_LOST_MS = 2500;   // age >= this -> LOST
constexpr uint32_t LINK_TTL_BLOB_MS = 500;         // GOAL/TRIANGLE/TAPE blobs
constexpr uint32_t LINK_TTL_TOF_MS = 1000;
constexpr uint32_t LINK_TTL_IMU_MS = 1000;
constexpr uint32_t LINK_TTL_RADAR_MS = 2500;
constexpr uint32_t LINK_TTL_STICKY = 0xFFFFFFFFu;  // READY, MODE_ACK
constexpr uint32_t LINK_TTL_NONE = 0;              // not held by the cache
constexpr uint8_t LINK_CACHED_TYPES = 8;           // one slot per cacheable id
constexpr uint8_t LINK_MAX_CACHED_PAYLOAD = 9;     // GOAL_BLOB is the widest
constexpr uint16_t LINK_DEGRADE_ESCALATIONS = 2;   // escalations -> line-only
// Payload scales both ends must agree on (protocol 4.3). Frames carry the raw
// wire values; these bound the documented ranges.
constexpr uint16_t LINK_COORD_MAX = 1000;           // cx/cy/width/height/size
constexpr uint8_t LINK_CONFIDENCE_MAX_PCT = 100;
constexpr uint16_t LINK_TOF_MAX_MM = 4000;
constexpr int16_t LINK_YAW_CENTI_DEG_MAX = 18000;   // 0.01 deg/LSB
constexpr uint8_t LINK_ZONE_NO_RESULT = 0xFF;       // RADAR_RESULT zone
constexpr uint8_t LINK_IMU_FLAG_VALID = 0x01;       // IMU_FRAME flags bit0
// Message ids (protocol 4); any other id is reserved and ignored by the codec.
enum MessageId : uint8_t {
  MSG_HEARTBEAT = 0x00, MSG_READY = 0x01, MSG_MODE_ACK = 0x02,
  MSG_GOAL_BLOB = 0x03, MSG_TRIANGLE_BLOB = 0x04, MSG_TOF_RANGE = 0x05,
  MSG_RADAR_RESULT = 0x06, MSG_IMU_FRAME = 0x07, MSG_TAPE_BLOB = 0x08,
  MSG_DIAG_REPORT = 0x09, MSG_CAL_COMPLETE = 0x0A, MSG_ERROR = 0x0B,
  MSG_MODE_SET = 0x10, MSG_RUN_START = 0x11, MSG_DIAG_GET = 0x12,
  MSG_CAL_START = 0x13, MSG_CAL_ABORT = 0x14, MSG_REQUEST_BLOB = 0x15,
  MSG_REQUEST_RANGE = 0x16, MSG_REQUEST_RADAR = 0x17, MSG_CONFIG_PUSH = 0x18,
  MSG_SENSOR_GATE = 0x19, MSG_VERSION = 0x1A,
};

// CRC-8 and big-endian field access (protocol 3.1, 4).
inline uint8_t crc8(const uint8_t* data, uint16_t length) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < LINK_CRC8_BITS; ++bit) {
      crc = (crc & LINK_CRC8_MSB) ? static_cast<uint8_t>((crc << 1) ^ LINK_CRC8_POLY)
                                   : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
inline uint16_t readBe16(const uint8_t* d, uint16_t o) {
  return static_cast<uint16_t>((static_cast<uint16_t>(d[o]) << 8) | d[o + 1]);
}
inline int16_t readBeI16(const uint8_t* d, uint16_t o) {
  return static_cast<int16_t>(readBe16(d, o));
}
inline uint32_t readBe32(const uint8_t* d, uint16_t o) {
  return (static_cast<uint32_t>(d[o]) << 24) |
         (static_cast<uint32_t>(d[o + 1]) << 16) |
         (static_cast<uint32_t>(d[o + 2]) << 8) | d[o + 3];
}
inline void writeBe16(uint8_t* d, uint16_t o, uint16_t v) {
  d[o] = static_cast<uint8_t>(v >> 8);
  d[o + 1] = static_cast<uint8_t>(v);
}
inline void writeBe32(uint8_t* d, uint16_t o, uint32_t v) {
  d[o] = static_cast<uint8_t>(v >> 24);
  d[o + 1] = static_cast<uint8_t>(v >> 16);
  d[o + 2] = static_cast<uint8_t>(v >> 8);
  d[o + 3] = static_cast<uint8_t>(v);
}

// Borrowed view of a decoded frame: payload points into the decoder's buffer
// and is valid only until the next feed(), so keep what you need.
struct FrameView {
  uint8_t id;
  const uint8_t* payload;
  uint8_t payloadLength;
};

// Write SYNC0 SYNC1 LEN ID payload CRC and return the byte count, or 0 when
// capacity or the protocol payload ceiling would be exceeded.
inline uint8_t encodeFrame(uint8_t* out, uint16_t capacity, uint8_t id,
                           const uint8_t* payload, uint8_t payloadLength) {
  if (payloadLength > LINK_MAX_PAYLOAD) return 0;
  const uint8_t len = static_cast<uint8_t>(payloadLength + LINK_BODY_OVERHEAD);
  const uint8_t total = static_cast<uint8_t>(payloadLength + LINK_FRAME_OVERHEAD);
  if (capacity < total) return 0;
  out[0] = LINK_SYNC0;
  out[1] = LINK_SYNC1;
  out[2] = len;
  out[3] = id;
  for (uint8_t i = 0; i < payloadLength; ++i) out[4 + i] = payload[i];
  out[total - 1] = crc8(out + 2, len);  // CRC covers LEN + id + payload
  return total;
}

// HUNT/FRAME re-sync state machine (protocol 5.4). A CRC mismatch or an
// out-of-range LEN is one bad frame; the third drops the partial frame and
// re-hunts the preamble. A decoded frame clears the bad-frame count.
class FrameDecoder {
 public:
  enum class RxState : uint8_t { Hunt, Preamble, Body };
  enum class FeedResult : uint8_t { NoFrame, FrameComplete, BadFrame, Resynced };
  FrameDecoder() { reset(); }
  void reset() {
    state_ = RxState::Hunt;
    haveLen_ = false;
    bodyIndex_ = 0;
    badFrames_ = 0;
    resyncs_ = 0;
  }
  // Consume one byte. On FrameComplete, out describes the decoded frame.
  FeedResult feed(uint8_t byte, FrameView& out) {
    if (state_ == RxState::Hunt) {
      if (byte == LINK_SYNC0) state_ = RxState::Preamble;
      return FeedResult::NoFrame;
    }
    if (state_ == RxState::Preamble) {
      // A lone 0xAA is dropped and the next byte is re-examined for a preamble,
      // so noise containing 0xAA cannot strand the hunt.
      state_ = (byte == LINK_SYNC1) ? RxState::Body
             : (byte == LINK_SYNC0) ? RxState::Preamble : RxState::Hunt;
      if (state_ != RxState::Body) return FeedResult::NoFrame;
      return startBody();
    }
    return feedBody(byte, out);
  }
  RxState state() const { return state_; }
  uint8_t badFrames() const { return badFrames_; }
  uint16_t resyncs() const { return resyncs_; }

 private:
  FeedResult startBody() {
    haveLen_ = false;
    bodyIndex_ = 0;
    return FeedResult::NoFrame;
  }
  void hunt() {
    state_ = RxState::Hunt;
    startBody();
  }
  FeedResult reject() {
    ++badFrames_;
    hunt();
    if (badFrames_ < LINK_BAD_FRAME_LIMIT) return FeedResult::BadFrame;
    ++resyncs_;
    return FeedResult::Resynced;
  }
  // wire_[0] = LEN, wire_[1] = id, wire_[2 .. LEN-1] = payload, wire_[LEN] = CRC.
  FeedResult feedBody(uint8_t byte, FrameView& out) {
    if (!haveLen_) {  // the first body byte is LEN
      if (byte < LINK_LEN_MIN) return reject();
      wire_[0] = byte;
      haveLen_ = true;
      return FeedResult::NoFrame;
    }
    wire_[bodyIndex_ + 1] = byte;
    if (++bodyIndex_ < wire_[0]) return FeedResult::NoFrame;
    if (crc8(wire_, wire_[0]) != wire_[wire_[0]]) return reject();
    out.id = wire_[1];
    out.payload = wire_ + LINK_BODY_OVERHEAD;
    out.payloadLength = static_cast<uint8_t>(wire_[0] - LINK_BODY_OVERHEAD);
    badFrames_ = 0;
    hunt();
    return FeedResult::FrameComplete;
  }
  uint8_t wire_[LINK_LEN_MAX + 1];
  RxState state_;
  uint8_t bodyIndex_;
  uint8_t badFrames_;
  uint16_t resyncs_;
  bool haveLen_;
};

// LINK_TTL_NONE marks an id this receiver does not hold: HEARTBEAT drives
// liveness instead, and every Mega -> Nicla id is a command, not a reading.
inline uint32_t cacheTtlMs(uint8_t id) {
  switch (id) {
    case MSG_GOAL_BLOB:
    case MSG_TRIANGLE_BLOB:
    case MSG_TAPE_BLOB:
      return LINK_TTL_BLOB_MS;
    case MSG_TOF_RANGE: return LINK_TTL_TOF_MS;
    case MSG_IMU_FRAME: return LINK_TTL_IMU_MS;
    case MSG_RADAR_RESULT: return LINK_TTL_RADAR_MS;
    case MSG_READY:
    case MSG_MODE_ACK: return LINK_TTL_STICKY;
    default: return LINK_TTL_NONE;
  }
}
struct CachedFrame {
  bool valid;
  uint32_t timestampMs;
  uint8_t id;
  uint8_t payloadLength;  // as framed: LEN is authoritative, never assumed
  uint8_t payload[LINK_MAX_CACHED_PAYLOAD];
};

// A cached value is consumable only while fresh: get() returns null once the TTL
// has passed, so a stale entry can never become a decision source.
class LastGoodCache {
 public:
  LastGoodCache() { reset(); }
  void reset() {
    for (CachedFrame& slot : slots_) slot.valid = false;
  }
  void store(const FrameView& frame, uint32_t nowMs) {
    const int index = indexOf(frame.id);
    if (index < 0) return;
    CachedFrame& slot = slots_[index];
    const uint8_t kept = frame.payloadLength < LINK_MAX_CACHED_PAYLOAD
                             ? frame.payloadLength : LINK_MAX_CACHED_PAYLOAD;
    for (uint8_t i = 0; i < kept; ++i) slot.payload[i] = frame.payload[i];
    slot.id = frame.id;
    slot.payloadLength = frame.payloadLength;
    slot.timestampMs = nowMs;
    slot.valid = true;
  }
  const CachedFrame* get(uint8_t id, uint32_t nowMs) const {
    const int index = indexOf(id);
    if (index < 0) return nullptr;
    const CachedFrame& slot = slots_[index];
    const uint32_t ttl = cacheTtlMs(id);
    const bool fresh =
        slot.valid && (ttl == LINK_TTL_STICKY || (nowMs - slot.timestampMs) < ttl);
    return fresh ? &slot : nullptr;
  }
  bool isFresh(uint8_t id, uint32_t nowMs) const { return get(id, nowMs) != nullptr; }

 private:
  // Slot holding id, else the first free slot, else -1 for an uncached id.
  int indexOf(uint8_t id) const {
    if (cacheTtlMs(id) == LINK_TTL_NONE) return -1;
    for (uint8_t i = 0; i < LINK_CACHED_TYPES; ++i) {
      if (slots_[i].valid && slots_[i].id == id) return i;
    }
    for (uint8_t i = 0; i < LINK_CACHED_TYPES; ++i) {
      if (!slots_[i].valid) return i;
    }
    return -1;
  }
  CachedFrame slots_[LINK_CACHED_TYPES];
};

// Liveness (protocol 5.1) and its first-transition events (5.3). Liveness is
// measured from the last frame of any type, so any traffic counts. Before the
// first frame the link is LOST: the boot sequence is WAIT_READY.
enum class Liveness : uint8_t { Fresh, Stale, Lost };
enum class LinkEvent : uint8_t { None, Stale, Lost, Revived };
class LinkHealth {
 public:
  LinkHealth() { reset(); }
  void reset() {
    liveness_ = Liveness::Lost;
    lastFrameMs_ = 0;
    pending_ = LinkEvent::None;
    seen_ = false;
  }
  void onFrame(uint32_t nowMs) {
    lastFrameMs_ = nowMs;
    seen_ = true;
  }
  Liveness update(uint32_t nowMs) {
    const Liveness next =
        seen_ ? livenessForAge(nowMs - lastFrameMs_) : Liveness::Lost;
    if (next != liveness_) {
      pending_ = eventForTransition(liveness_, next);
      liveness_ = next;
    }
    return liveness_;
  }
  Liveness liveness() const { return liveness_; }
  bool seen() const { return seen_; }
  uint32_t ageMs(uint32_t nowMs) const { return seen_ ? nowMs - lastFrameMs_ : 0; }
  LinkEvent takeEvent() {
    const LinkEvent event = pending_;
    pending_ = LinkEvent::None;
    return event;
  }

 private:
  static Liveness livenessForAge(uint32_t ageMs) {
    if (ageMs < LINK_LIVENESS_FRESH_MS) return Liveness::Fresh;
    if (ageMs < LINK_LIVENESS_LOST_MS) return Liveness::Stale;
    return Liveness::Lost;
  }
  // Revived is the first frame after LOST; a STALE -> FRESH recovery is not
  // one, and a first transition into a state reports that state.
  static LinkEvent eventForTransition(Liveness from, Liveness to) {
    if (to == Liveness::Fresh) {
      return from == Liveness::Lost ? LinkEvent::Revived : LinkEvent::None;
    }
    if (to == Liveness::Stale) return LinkEvent::Stale;
    return from == Liveness::Lost ? LinkEvent::None : LinkEvent::Lost;
  }
  Liveness liveness_;
  uint32_t lastFrameMs_;
  LinkEvent pending_;
  bool seen_;
};

// Live-run policy: hold per reflex, degrade after repeated loss. Driven by the
// link's first-transition events: Stale holds last-good values while still
// waiting, Lost holds and counts an escalation, Revived clears the degrade and
// the escalation history. A hold that runs out of budget counts an escalation
// too, so repeated loss degrades the robot to QTR-based line-only reflexes
// instead of leaving a reflex waiting on data that will not arrive.
enum class HoldSlot : uint8_t {
  GapCross, MarkerTurn, ObstacleAvoid, GoalApproach, EvacEntry, EvacExit, Count
};
constexpr uint8_t HOLD_SLOT_COUNT = static_cast<uint8_t>(HoldSlot::Count);
enum class LinkPolicy : uint8_t { LinkFresh, HoldLastGood, DegradeToLineOnly };
// Default hold budgets: no budget may exceed the window in which the link can
// still report LOST, so every reflex gives up before the data is declared gone.
constexpr uint32_t LINK_HOLD_GAP_CROSS_MS = 2000;
constexpr uint32_t LINK_HOLD_MARKER_TURN_MS = 1000;
constexpr uint32_t LINK_HOLD_OBSTACLE_MS = 1500;
constexpr uint32_t LINK_HOLD_GOAL_MS = 1000;
constexpr uint32_t LINK_HOLD_EVAC_MS = 1000;
inline uint32_t defaultHoldBudgetMs(HoldSlot slot) {
  static const uint32_t kBudgets[HOLD_SLOT_COUNT] = {
      LINK_HOLD_GAP_CROSS_MS, LINK_HOLD_MARKER_TURN_MS, LINK_HOLD_OBSTACLE_MS,
      LINK_HOLD_GOAL_MS, LINK_HOLD_EVAC_MS, LINK_HOLD_EVAC_MS};
  const uint8_t index = static_cast<uint8_t>(slot);
  return index < HOLD_SLOT_COUNT ? kBudgets[index] : 0;
}
struct PolicyCounters {
  uint16_t holdTimeouts;
  uint16_t linkLosses;
  uint16_t revivals;
  uint16_t degradeEntries;
};
class LiveLinkPolicy {
 public:
  LiveLinkPolicy() { reset(); }
  void reset() {
    for (uint8_t i = 0; i < HOLD_SLOT_COUNT; ++i) {
      holds_[i] = Hold{0, defaultHoldBudgetMs(static_cast<HoldSlot>(i)), false, false};
    }
    counters_ = PolicyCounters{0, 0, 0, 0};
    escalations_ = 0;
    linkWaiting_ = false;
  }
  void setHoldBudget(HoldSlot slot, uint32_t budgetMs) { at(slot).budgetMs = budgetMs; }
  uint32_t budgetMs(HoldSlot slot) const { return at(slot).budgetMs; }
  // Enter the bounded hold a reflex spends on last-good link values.
  void beginHold(HoldSlot slot, uint32_t nowMs) {
    Hold& hold = at(slot);
    hold.startedMs = nowMs;
    hold.active = true;
    hold.timedOut = false;
  }
  void endHold(HoldSlot slot) {
    at(slot).active = false;
    at(slot).timedOut = false;
  }
  void onLinkEvent(LinkEvent event) {
    if (event == LinkEvent::None) return;
    linkWaiting_ = true;
    if (event == LinkEvent::Revived) {
      linkWaiting_ = false;
      ++counters_.revivals;
      escalations_ = 0;
    } else if (event == LinkEvent::Lost) {
      ++counters_.linkLosses;
      escalate();
    }
  }
  // Expire every hold that has run out of budget.
  void update(uint32_t nowMs) {
    for (uint8_t i = 0; i < HOLD_SLOT_COUNT; ++i) {
      Hold& hold = holds_[i];
      if (!hold.active || (nowMs - hold.startedMs) < hold.budgetMs) continue;
      hold.active = false;
      hold.timedOut = true;
      ++counters_.holdTimeouts;
      escalate();
    }
  }
  LinkPolicy policy() const {
    if (degraded()) return LinkPolicy::DegradeToLineOnly;
    if (linkWaiting_ || escalations_ > 0 || activeHolds() > 0) {
      return LinkPolicy::HoldLastGood;
    }
    return LinkPolicy::LinkFresh;
  }
  bool degraded() const { return escalations_ >= LINK_DEGRADE_ESCALATIONS; }
  bool holdActive(HoldSlot slot) const { return at(slot).active; }
  bool holdTimedOut(HoldSlot slot) const { return at(slot).timedOut; }
  uint8_t activeHolds() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < HOLD_SLOT_COUNT; ++i) count += holds_[i].active;
    return count;
  }
  const PolicyCounters& counters() const { return counters_; }

 private:
  struct Hold {
    uint32_t startedMs;
    uint32_t budgetMs;
    bool active;
    bool timedOut;
  };
  Hold& at(HoldSlot slot) { return holds_[static_cast<uint8_t>(slot)]; }
  const Hold& at(HoldSlot slot) const { return holds_[static_cast<uint8_t>(slot)]; }
  // One escalation is a hold; crossing the threshold counts one degrade entry.
  void escalate() {
    if (escalations_ < 0xFFFF) ++escalations_;
    if (escalations_ == LINK_DEGRADE_ESCALATIONS) ++counters_.degradeEntries;
  }
  Hold holds_[HOLD_SLOT_COUNT];
  PolicyCounters counters_;
  uint16_t escalations_;
  bool linkWaiting_;
};

}  // namespace rescue
#endif  // RESCUE_ROBOT_LINK_H
