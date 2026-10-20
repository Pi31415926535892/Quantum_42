// link_test.cpp — host-side tests for src/rescue_robot/link.h (pure C++14).
// Build/run: g++ -std=c++14 test/link_test.cpp -o /tmp/linktest && /tmp/linktest
// Normative source: docs/mega-nicla-protocol.md (frame format, CRC-8, ids,
// freshness, re-sync, live-run degrade). The payload-length column below is
// transcribed from that document so the tests never validate the code against
// itself.

#include "../src/rescue_robot/link.h"

#include <cstdio>
#include <cstring>

namespace {

int gFailures = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::printf("FAIL %d: %s\n", __LINE__, #cond);                       \
      ++gFailures;                                                         \
    }                                                                      \
  } while (0)

#define CHECK_EQ(a, b)                                                     \
  do {                                                                     \
    const long va = (long)(a);                                             \
    const long vb = (long)(b);                                             \
    if (va != vb) {                                                        \
      std::printf("FAIL %d: %s == %s (got %ld, want %ld)\n", __LINE__, #a,  \
                  #b, va, vb);                                             \
      ++gFailures;                                                         \
    }                                                                      \
  } while (0)

using namespace rescue;

struct DocRow {
  uint8_t id;
  uint8_t payloadLength;  // LEN = payloadLength + 2
};
const DocRow DOC_ROWS[] = {
    {0x00, 0}, {0x01, 6}, {0x02, 2},  {0x03, 9}, {0x04, 6}, {0x05, 3},
    {0x06, 2}, {0x07, 5}, {0x08, 6},  {0x09, 5}, {0x0A, 1}, {0x0B, 3},
    {0x10, 2}, {0x11, 2}, {0x12, 1},  {0x13, 1}, {0x14, 0}, {0x15, 2},
    {0x16, 0}, {0x17, 2}, {0x18, 7},  {0x19, 2}, {0x1A, 4},
};
const size_t DOC_ROW_COUNT = sizeof(DOC_ROWS) / sizeof(DOC_ROWS[0]);

uint8_t docPayloadLength(uint8_t id) {
  for (size_t i = 0; i < DOC_ROW_COUNT; ++i) {
    if (DOC_ROWS[i].id == id) return DOC_ROWS[i].payloadLength;
  }
  return LINK_LEN_MAX;  // deliberately unreachable for a documented id
}

// Deterministic filler: distinct per id and per index.
uint8_t patternByte(uint8_t id, uint8_t index) {
  return static_cast<uint8_t>((id * 13u) + (index * 7u) + 3u);
}

// Encode id with the documented payload length and the pattern payload.
uint8_t buildFrame(uint8_t* frame, uint16_t capacity, uint8_t id) {
  const uint8_t payloadLength = docPayloadLength(id);
  uint8_t payload[LINK_MAX_PAYLOAD];
  for (uint8_t i = 0; i < payloadLength; ++i) {
    payload[i] = patternByte(id, i);
  }
  return encodeFrame(frame, capacity, id, payload, payloadLength);
}

// The delivery-loop shape: drain a frame, cache it on completion. A verdict is
// remembered but the rest of the corrupt frame is still drained, exactly as
// the serial ring delivers it.
FrameDecoder::FeedResult consume(FrameDecoder& decoder, LastGoodCache& cache,
                                 const uint8_t* frame, uint8_t length,
                                 FrameView& view, uint32_t nowMs) {
  FrameDecoder::FeedResult verdict = FrameDecoder::FeedResult::NoFrame;
  for (uint8_t i = 0; i < length; ++i) {
    const FrameDecoder::FeedResult result = decoder.feed(frame[i], view);
    if (result == FrameDecoder::FeedResult::FrameComplete) {
      cache.store(view, nowMs);
      return result;
    }
    if (result != FrameDecoder::FeedResult::NoFrame) verdict = result;
  }
  return verdict;
}

bool decoderAccepts(FrameDecoder& decoder, const uint8_t* frame,
                    uint8_t length, FrameView& out) {
  for (uint8_t i = 0; i < length; ++i) {
    if (decoder.feed(frame[i], out) ==
        FrameDecoder::FeedResult::FrameComplete) {
      return i == length - 1;  // a frame completes on its final byte
    }
  }
  return false;
}

// A fresh cache entry, or a failing dummy so a missing entry reads as zeros
// rather than dereferencing null.
const CachedFrame kAbsentFrame = {false, 0, 0, 0, {0}};
const CachedFrame* expectFresh(LastGoodCache& cache, uint8_t id,
                               uint32_t nowMs) {
  const CachedFrame* entry = cache.get(id, nowMs);
  if (entry == nullptr) {
    std::printf("FAIL %d: no fresh entry for id 0x%02X at %u ms\n", __LINE__,
                id, nowMs);
    ++gFailures;
    return &kAbsentFrame;
  }
  return entry;
}

// (a) Every documented message type round-trips, and (b) both CRC vectors
// assert byte for byte.
void testFramingAndCrcVectors() {
  const uint8_t heartbeatScope[] = {0x02, 0x00};
  const uint8_t runStartScope[] = {0x04, 0x11, 0x00, 0x03};
  CHECK_EQ(crc8(heartbeatScope, 2), 0x2A);
  CHECK_EQ(crc8(runStartScope, 4), 0x98);
  CHECK_EQ(crc8(heartbeatScope, 0), 0x00);  // init 0x00, no final xor

  uint8_t frame[LINK_MAX_FRAME_BYTES];
  const uint8_t wantHeartbeat[] = {0xAA, 0x55, 0x02, 0x00, 0x2A};
  CHECK_EQ(buildFrame(frame, sizeof(frame), MSG_HEARTBEAT), 5);
  CHECK(std::memcmp(frame, wantHeartbeat, 5) == 0);

  uint8_t payload[2];
  writeBe16(payload, 0, 0x0003);
  const uint8_t wantRunStart[] = {0xAA, 0x55, 0x04, 0x11, 0x00, 0x03, 0x98};
  CHECK_EQ(encodeFrame(frame, sizeof(frame), MSG_RUN_START, payload, 2), 7);
  CHECK(std::memcmp(frame, wantRunStart, 7) == 0);

  // Named ids must equal the document's values.
  CHECK_EQ(MSG_HEARTBEAT, 0x00);
  CHECK_EQ(MSG_READY, 0x01);
  CHECK_EQ(MSG_MODE_ACK, 0x02);
  CHECK_EQ(MSG_GOAL_BLOB, 0x03);
  CHECK_EQ(MSG_TOF_RANGE, 0x05);
  CHECK_EQ(MSG_TAPE_BLOB, 0x08);
  CHECK_EQ(MSG_ERROR, 0x0B);
  CHECK_EQ(MSG_MODE_SET, 0x10);
  CHECK_EQ(MSG_RUN_START, 0x11);
  CHECK_EQ(MSG_VERSION, 0x1A);

  // Big-endian accessors.
  writeBe16(payload, 0, 0x1234);
  CHECK_EQ(payload[0], 0x12);
  CHECK_EQ(readBe16(payload, 0), 0x1234);
  CHECK_EQ(readBeI16(payload, 0), 0x1234);
  writeBe32(payload, 0, 0x89ABCDEFu);
  CHECK_EQ(payload[0], 0x89);
  CHECK_EQ(payload[3], 0xEF);
  CHECK_EQ(readBe32(payload, 0), 0x89ABCDEFu);
  writeBe16(payload, 2, 0xFF00);
  CHECK_EQ(readBeI16(payload, 2), -256);
  writeBe32(payload, 0, 0x00000001u);
  CHECK_EQ(readBe32(payload, 0), 1);

  // The published scale contract both ends agree on (protocol 4.3).
  CHECK_EQ(LINK_COORD_MAX, 1000);
  CHECK_EQ(LINK_CONFIDENCE_MAX_PCT, 100);
  CHECK_EQ(LINK_TOF_MAX_MM, 4000);
  CHECK_EQ(LINK_YAW_CENTI_DEG_MAX, 18000);
  CHECK_EQ(LINK_ZONE_NO_RESULT, 0xFF);
  CHECK_EQ(LINK_IMU_FLAG_VALID, 0x01);
  CHECK_EQ(LINK_BAUD, 115200);
  CHECK_EQ(LINK_HEARTBEAT_MS, 250);
  CHECK_EQ(LINK_LIVENESS_FRESH_MS, 1000);
  CHECK_EQ(LINK_LIVENESS_LOST_MS, 2500);
  CHECK_EQ(LINK_BAD_FRAME_LIMIT, 3);
  CHECK_EQ(LINK_RX_BYTE_BUDGET, 64);
  CHECK_EQ(LINK_TX_BUFFER_BYTES, 12);
  CHECK_EQ(LINK_DEGRADE_ESCALATIONS, 2);
  CHECK_EQ(LINK_MAX_PAYLOAD, 253);
  CHECK_EQ(LINK_MAX_FRAME_BYTES, 258);
  CHECK_EQ(LINK_MAX_CACHED_PAYLOAD, 9);
  CHECK_EQ(LINK_CACHED_TYPES, 8);

  // Capacity and the protocol payload ceiling are refused, never truncated.
  uint8_t tiny[4];
  CHECK_EQ(encodeFrame(tiny, sizeof(tiny), MSG_HEARTBEAT, nullptr, 0), 0);
  CHECK_EQ(encodeFrame(frame, sizeof(frame), MSG_GOAL_BLOB, payload,
                       LINK_MAX_PAYLOAD + 1),
           0);
}

void testEveryMessageRoundTrips() {
  FrameDecoder decoder;
  FrameView view;
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  for (size_t row = 0; row < DOC_ROW_COUNT; ++row) {
    const uint8_t id = DOC_ROWS[row].id;
    const uint8_t payloadLength = DOC_ROWS[row].payloadLength;
    const uint8_t length = buildFrame(frame, sizeof(frame), id);
    if (length != payloadLength + LINK_FRAME_OVERHEAD) {
      std::printf("FAIL id 0x%02X: %u bytes on the wire, want %u\n", id, length,
                  payloadLength + LINK_FRAME_OVERHEAD);
      ++gFailures;
      continue;
    }
    CHECK_EQ(frame[0], LINK_SYNC0);
    CHECK_EQ(frame[1], LINK_SYNC1);
    CHECK_EQ(frame[2], payloadLength + LINK_BODY_OVERHEAD);
    CHECK_EQ(frame[3], id);
    CHECK_EQ(frame[length - 1], crc8(frame + 2, length - 3));
    CHECK(decoderAccepts(decoder, frame, length, view));
    CHECK_EQ(view.id, id);
    CHECK_EQ(view.payloadLength, payloadLength);
    for (uint8_t i = 0; i < payloadLength; ++i) {
      CHECK_EQ(view.payload[i], patternByte(id, i));
    }
    CHECK_EQ(decoder.badFrames(), 0);
  }
}

// (c) Corruption is rejected and the last-good cache is left untouched.
void testCorruptionKeepsLastGood() {
  FrameDecoder decoder;
  LastGoodCache cache;
  FrameView view;
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  const uint8_t length = buildFrame(frame, sizeof(frame), MSG_GOAL_BLOB);
  uint8_t corrupt[LINK_MAX_FRAME_BYTES];
  CHECK(consume(decoder, cache, frame, length, view, 1000) ==
        FrameDecoder::FeedResult::FrameComplete);
  const CachedFrame snapshot = *expectFresh(cache, MSG_GOAL_BLOB, 1000);

  for (uint8_t i = 0; i < length; ++i) corrupt[i] = frame[i];
  corrupt[6] = static_cast<uint8_t>(corrupt[6] ^ 0x01);  // payload bit flip
  CHECK(consume(decoder, cache, corrupt, length, view, 1010) ==
        FrameDecoder::FeedResult::BadFrame);
  const CachedFrame* afterPayload = expectFresh(cache, MSG_GOAL_BLOB, 1010);
  CHECK_EQ(afterPayload->timestampMs, snapshot.timestampMs);
  for (uint8_t i = 0; i < snapshot.payloadLength; ++i) {
    CHECK_EQ(afterPayload->payload[i], snapshot.payload[i]);
  }

  corrupt[6] = frame[6];
  corrupt[length - 1] = static_cast<uint8_t>(frame[length - 1] ^ 0x80);  // CRC
  CHECK(consume(decoder, cache, corrupt, length, view, 1020) ==
        FrameDecoder::FeedResult::BadFrame);
  CHECK_EQ(decoder.badFrames(), 2);
  CHECK_EQ(expectFresh(cache, MSG_GOAL_BLOB, 1020)->timestampMs,
           snapshot.timestampMs);
}

void testFramingErrors() {
  FrameDecoder decoder;
  LastGoodCache cache;
  FrameView view;
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  const uint8_t length = buildFrame(frame, sizeof(frame), MSG_GOAL_BLOB);
  uint8_t badLen[LINK_MAX_FRAME_BYTES];
  for (uint8_t i = 0; i < length; ++i) badLen[i] = frame[i];
  badLen[2] = 0x01;  // LEN below the minimum
  CHECK(consume(decoder, cache, badLen, length, view, 100) ==
        FrameDecoder::FeedResult::BadFrame);
  CHECK(decoder.state() == FrameDecoder::RxState::Hunt);

  // A truncated frame stays in FRAME: the next frame is swallowed as its body
  // and fails the CRC, and the frame after that decodes. That one-frame cost
  // is the re-sync price of a byte stream with no framing signal.
  CHECK(consume(decoder, cache, frame, length - 1, view, 200) ==
        FrameDecoder::FeedResult::NoFrame);
  CHECK(consume(decoder, cache, frame, length, view, 300) ==
        FrameDecoder::FeedResult::BadFrame);
  CHECK(consume(decoder, cache, frame, length, view, 400) ==
        FrameDecoder::FeedResult::FrameComplete);
  CHECK_EQ(decoder.badFrames(), 0);
  CHECK_EQ(expectFresh(cache, MSG_GOAL_BLOB, 400)->timestampMs, 400);

  // LEN is authoritative: a known id framed wider than the table's payload is
  // stored as it arrived, so the consumer sees the real length.
  uint8_t widePayload[12];
  uint8_t wideFrame[LINK_MAX_FRAME_BYTES];
  for (uint8_t i = 0; i < 12; ++i) widePayload[i] = patternByte(MSG_GOAL_BLOB, i);
  const uint8_t wideLength =
      encodeFrame(wideFrame, sizeof(wideFrame), MSG_GOAL_BLOB, widePayload, 12);
  CHECK_EQ(wideFrame[2], 14);
  CHECK(consume(decoder, cache, wideFrame, wideLength, view, 500) ==
        FrameDecoder::FeedResult::FrameComplete);
  CHECK_EQ(view.payloadLength, 12);
  CHECK_EQ(expectFresh(cache, MSG_GOAL_BLOB, 500)->payloadLength, 12);
}

// (d) Three bad frames re-sync on the next valid preamble.
void testResyncAfterThreeBadFrames() {
  FrameDecoder decoder;
  LastGoodCache cache;
  FrameView view;
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  const uint8_t length = buildFrame(frame, sizeof(frame), MSG_TOF_RANGE);
  uint8_t corrupt[LINK_MAX_FRAME_BYTES];
  for (uint8_t i = 0; i < length; ++i) corrupt[i] = frame[i];
  corrupt[length - 1] = static_cast<uint8_t>(corrupt[length - 1] ^ 0xFF);

  CHECK(consume(decoder, cache, corrupt, length, view, 0) ==
        FrameDecoder::FeedResult::BadFrame);
  CHECK_EQ(decoder.badFrames(), 1);
  CHECK(consume(decoder, cache, corrupt, length, view, 0) ==
        FrameDecoder::FeedResult::BadFrame);
  CHECK_EQ(decoder.badFrames(), 2);
  CHECK(decoder.state() == FrameDecoder::RxState::Hunt);

  // The third bad frame drops the partial frame and re-hunts the preamble.
  CHECK(consume(decoder, cache, corrupt, length, view, 0) ==
        FrameDecoder::FeedResult::Resynced);
  CHECK_EQ(decoder.badFrames(), LINK_BAD_FRAME_LIMIT);
  CHECK_EQ(decoder.resyncs(), 1);
  CHECK(decoder.state() == FrameDecoder::RxState::Hunt);

  // The next valid preamble is picked up and the counter clears.
  CHECK(consume(decoder, cache, frame, length, view, 100) ==
        FrameDecoder::FeedResult::FrameComplete);
  CHECK_EQ(view.id, MSG_TOF_RANGE);
  CHECK_EQ(decoder.badFrames(), 0);
  CHECK_EQ(decoder.resyncs(), 1);

  // HUNT scan: a lone 0xAA is dropped and the next byte is re-examined.
  FrameDecoder scanner;
  FrameView scanView;
  const uint8_t noise[] = {0x00, 0xAA, 0x13, 0xAA, 0xAA, 0x55};
  for (uint8_t i = 0; i < sizeof(noise); ++i) {
    CHECK(scanner.feed(noise[i], scanView) !=
          FrameDecoder::FeedResult::BadFrame);
  }
  CHECK(scanner.state() == FrameDecoder::RxState::Body);
  CHECK_EQ(scanner.badFrames(), 0);

  // A reserved id is decoded without counting a frame error, and is not
  // cached (no TTL covers it).
  uint8_t reservedFrame[LINK_MAX_FRAME_BYTES];
  const uint8_t reservedLength =
      encodeFrame(reservedFrame, sizeof(reservedFrame), 0x7F, noise, 4);
  FrameDecoder reservedDecoder;
  CHECK(decoderAccepts(reservedDecoder, reservedFrame, reservedLength,
                       scanView));
  CHECK_EQ(scanView.id, 0x7F);
  CHECK_EQ(reservedDecoder.badFrames(), 0);
  CHECK_EQ(cacheTtlMs(0x7F), LINK_TTL_NONE);
  CHECK(cache.get(0x7F, 0) == nullptr);  // never stored

  decoder.reset();
  CHECK_EQ(decoder.badFrames(), 0);
  CHECK_EQ(decoder.resyncs(), 0);
  CHECK(decoder.state() == FrameDecoder::RxState::Hunt);
}

// (e) A heartbeat gap ages the link out and expires the per-type caches.
void testLivenessAndFreshness() {
  LinkHealth health;
  CHECK(health.liveness() == Liveness::Lost);
  CHECK(!health.seen());
  CHECK_EQ(health.ageMs(0), 0);
  CHECK(health.update(0) == Liveness::Lost);
  CHECK(health.takeEvent() == LinkEvent::None);  // no frame yet, so no event
  health.onFrame(1000);
  CHECK(health.seen());
  CHECK(health.update(1000) == Liveness::Fresh);
  CHECK(health.takeEvent() == LinkEvent::Revived);
  CHECK(health.takeEvent() == LinkEvent::None);  // events are one-shot
  CHECK(health.update(1999) == Liveness::Fresh);
  CHECK_EQ(health.ageMs(1999), 999);
  CHECK(health.update(2000) == Liveness::Stale);
  CHECK(health.takeEvent() == LinkEvent::Stale);
  CHECK(health.update(3499) == Liveness::Stale);
  CHECK(health.takeEvent() == LinkEvent::None);  // still stale, not a transition
  CHECK(health.update(3500) == Liveness::Lost);
  CHECK(health.takeEvent() == LinkEvent::Lost);
  health.onFrame(3500);
  CHECK(health.update(3500) == Liveness::Fresh);
  CHECK(health.takeEvent() == LinkEvent::Revived);
  // Ages are unsigned, so the millis() wrap is not a false LOST.
  health.onFrame(0xFFFFFF00u);
  CHECK(health.update(0x00000100u) == Liveness::Fresh);
  CHECK_EQ(health.ageMs(0x00000100u), 512);
  health.reset();
  CHECK(!health.seen());
  CHECK(health.update(100000) == Liveness::Lost);

  struct TtlCase {
    uint8_t id;
    uint32_t ttl;
  };
  const TtlCase ttlCases[] = {
      {MSG_GOAL_BLOB, LINK_TTL_BLOB_MS},   {MSG_TRIANGLE_BLOB, LINK_TTL_BLOB_MS},
      {MSG_TAPE_BLOB, LINK_TTL_BLOB_MS},  {MSG_TOF_RANGE, LINK_TTL_TOF_MS},
      {MSG_IMU_FRAME, LINK_TTL_IMU_MS},   {MSG_RADAR_RESULT, LINK_TTL_RADAR_MS},
      {MSG_READY, LINK_TTL_STICKY},       {MSG_MODE_ACK, LINK_TTL_STICKY},
  };
  for (size_t i = 0; i < sizeof(ttlCases) / sizeof(ttlCases[0]); ++i) {
    CHECK_EQ(cacheTtlMs(ttlCases[i].id), ttlCases[i].ttl);
  }
  CHECK_EQ(cacheTtlMs(MSG_HEARTBEAT), LINK_TTL_NONE);
  CHECK_EQ(cacheTtlMs(MSG_RUN_START), LINK_TTL_NONE);

  // One slot per cacheable id: all eight fit at once.
  FrameDecoder decoder;
  LastGoodCache cache;
  FrameView view;
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  for (size_t i = 0; i < sizeof(ttlCases) / sizeof(ttlCases[0]); ++i) {
    const uint8_t length = buildFrame(frame, sizeof(frame), ttlCases[i].id);
    CHECK(consume(decoder, cache, frame, length, view, 0) ==
          FrameDecoder::FeedResult::FrameComplete);
  }
  for (size_t i = 0; i < sizeof(ttlCases) / sizeof(ttlCases[0]); ++i) {
    const CachedFrame* entry = expectFresh(cache, ttlCases[i].id, 0);
    CHECK_EQ(entry->id, ttlCases[i].id);
    CHECK_EQ(entry->payloadLength, docPayloadLength(ttlCases[i].id));
  }
  // Each type expires on its own TTL; READY and MODE_ACK are sticky.
  for (size_t i = 0; i < 6; ++i) {  // the six reading types
    CHECK(cache.isFresh(ttlCases[i].id, ttlCases[i].ttl - 1));
    CHECK(!cache.isFresh(ttlCases[i].id, ttlCases[i].ttl));
  }
  CHECK(cache.isFresh(MSG_READY, 600000));
  CHECK(cache.isFresh(MSG_MODE_ACK, 600000));
  // A second sighting refreshes the entry; a reset empties the cache.
  const uint8_t length = buildFrame(frame, sizeof(frame), MSG_GOAL_BLOB);
  CHECK(consume(decoder, cache, frame, length, view, 400) ==
        FrameDecoder::FeedResult::FrameComplete);
  CHECK_EQ(expectFresh(cache, MSG_GOAL_BLOB, 400)->timestampMs, 400);
  CHECK(cache.isFresh(MSG_GOAL_BLOB, 400 + LINK_TTL_BLOB_MS - 1));
  CHECK(!cache.isFresh(MSG_GOAL_BLOB, 400 + LINK_TTL_BLOB_MS));
  cache.reset();
  CHECK(cache.get(MSG_READY, 0) == nullptr);
  CHECK(cache.get(MSG_GOAL_BLOB, 0) == nullptr);
}

// (f) Live-run degrade: hold per reflex, line-only after repeated loss,
// resumed by a fresh frame.
void testPolicyDegradeAndResume() {
  for (uint8_t slot = 0; slot < static_cast<uint8_t>(HoldSlot::Count); ++slot) {
    // No budget may outlast the window in which the link can report LOST.
    CHECK(defaultHoldBudgetMs(static_cast<HoldSlot>(slot)) > 0);
    CHECK(defaultHoldBudgetMs(static_cast<HoldSlot>(slot)) <=
          LINK_LIVENESS_LOST_MS);
  }
  CHECK_EQ(defaultHoldBudgetMs(HoldSlot::GapCross), LINK_HOLD_GAP_CROSS_MS);
  CHECK_EQ(defaultHoldBudgetMs(HoldSlot::ObstacleAvoid), LINK_HOLD_OBSTACLE_MS);

  LiveLinkPolicy policy;
  CHECK(policy.policy() == LinkPolicy::LinkFresh);
  CHECK_EQ(policy.activeHolds(), 0);
  // A reflex holding link data stays on last-good values inside its budget.
  const uint32_t budget = defaultHoldBudgetMs(HoldSlot::ObstacleAvoid);
  policy.beginHold(HoldSlot::ObstacleAvoid, 0);
  CHECK_EQ(policy.activeHolds(), 1);
  CHECK(policy.holdActive(HoldSlot::ObstacleAvoid));
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);
  policy.update(budget - 1);
  CHECK(policy.holdActive(HoldSlot::ObstacleAvoid));
  CHECK_EQ(policy.counters().holdTimeouts, 0);
  // At the budget the hold times out and the reflex is flagged to give up.
  policy.update(budget);
  CHECK(!policy.holdActive(HoldSlot::ObstacleAvoid));
  CHECK(policy.holdTimedOut(HoldSlot::ObstacleAvoid));
  CHECK_EQ(policy.counters().holdTimeouts, 1);
  CHECK_EQ(policy.activeHolds(), 0);
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);  // one escalation: hold
  // Repeated escalation degrades to line-only.
  policy.beginHold(HoldSlot::GoalApproach, 1000);
  policy.update(1000 + defaultHoldBudgetMs(HoldSlot::GoalApproach));
  CHECK_EQ(policy.counters().holdTimeouts, 2);
  CHECK(policy.policy() == LinkPolicy::DegradeToLineOnly);
  CHECK(policy.degraded());
  policy.endHold(HoldSlot::EvacExit);  // harmless when never started
  CHECK_EQ(policy.activeHolds(), 0);

  // Stale waits on last-good values; the first loss still only holds.
  policy.reset();
  policy.onLinkEvent(LinkEvent::Stale);
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);
  CHECK_EQ(policy.counters().linkLosses, 0);
  CHECK(!policy.degraded());
  policy.onLinkEvent(LinkEvent::Lost);
  CHECK_EQ(policy.counters().linkLosses, 1);
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);
  policy.onLinkEvent(LinkEvent::Lost);
  CHECK(policy.policy() == LinkPolicy::DegradeToLineOnly);
  CHECK_EQ(policy.counters().degradeEntries, 1);
  // A fresh frame resumes the link-sourced reflexes and clears the history.
  policy.onLinkEvent(LinkEvent::Revived);
  CHECK_EQ(policy.counters().revivals, 1);
  CHECK(!policy.degraded());
  CHECK(policy.policy() == LinkPolicy::LinkFresh);
  policy.onLinkEvent(LinkEvent::Lost);
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);
  // A hold running across a revival keeps its reflex on last-good values.
  policy.beginHold(HoldSlot::GapCross, 3000);
  policy.onLinkEvent(LinkEvent::Revived);
  CHECK(policy.policy() == LinkPolicy::HoldLastGood);
  policy.endHold(HoldSlot::GapCross);
  CHECK(policy.policy() == LinkPolicy::LinkFresh);
  policy.update(10000);
  CHECK_EQ(policy.activeHolds(), 0);
  // A tuned budget overrides the default table entry for one reflex only.
  LiveLinkPolicy tuned;
  CHECK_EQ(tuned.budgetMs(HoldSlot::MarkerTurn), LINK_HOLD_MARKER_TURN_MS);
  tuned.setHoldBudget(HoldSlot::MarkerTurn, 250);
  CHECK_EQ(tuned.budgetMs(HoldSlot::MarkerTurn), 250);
  CHECK_EQ(tuned.budgetMs(HoldSlot::EvacEntry), LINK_HOLD_EVAC_MS);
  tuned.beginHold(HoldSlot::MarkerTurn, 0);
  tuned.update(249);
  CHECK(tuned.holdActive(HoldSlot::MarkerTurn));
  tuned.update(250);
  CHECK(tuned.holdTimedOut(HoldSlot::MarkerTurn));
}

}  // namespace

int main() {
  testFramingAndCrcVectors();
  testEveryMessageRoundTrips();
  testCorruptionKeepsLastGood();
  testFramingErrors();
  testResyncAfterThreeBadFrames();
  testLivenessAndFreshness();
  testPolicyDegradeAndResume();

  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", gFailures);
  return 1;
}
