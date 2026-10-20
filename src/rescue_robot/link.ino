// link.ino — Mega-side delivery for the Nicla host link.
//
// Serial1 at 115200 8N1, no flow control. On the Mega 2560 Serial1 is fixed to
// D18 (TX) / D19 (RX) and the board wires those through the BSS138 level
// shifter to the Nicla; HardwareSerial binds them at begin(), so there is no
// pin to configure here.
//
// This file only moves bytes and advances time: the transmit FIFO, the receive
// pump, the heartbeat cadence and the per-type last-good cache. All wire logic
// (framing, CRC, re-sync, liveness, degrade policy) lives in link.h.
//
// SRAM cost: 256 B decoder buffer + 96 B TX ring + 8 B TX lengths + 160 B
// last-good cache + ~72 B health/policy state = ~590 B, inside the 2 KB budget.
// No heap, no String, no blocking calls.

#include "link.h"
#include <Arduino.h>

// Public delivery API, declared here so the sketch is valid C++ without
// relying on the IDE's generated prototypes. Liveness transitions are applied
// to the policy inside linkTick(), so callers read the resulting policy state
// rather than the raw first-transition events.
void linkBegin();
void linkTick();
bool linkSend(uint8_t id, const uint8_t* payload, uint8_t payloadLength);
rescue::Liveness linkLiveness();
rescue::LinkPolicy linkPolicy();
bool linkReceiveFresh(uint8_t id);
const rescue::CachedFrame* linkLastGood(uint8_t id);
uint16_t linkTxOverflows();
uint8_t linkTxQueued();

namespace {

using namespace rescue;  // NOLINT(link.h types are used unqualified below)

// Transmit FIFO sizing: the protocol asks for at least 8 frames or 64 bytes.
// Mega -> Nicla frames are at most LINK_TX_BUFFER_BYTES (12 B, the widest
// command being CONFIG_PUSH), so eight slots give 96 B in 96 + 8 B of SRAM.
// Inbound frames are never queued here: they are decoded as they arrive.
constexpr uint8_t LINK_TX_FRAMES = 8;
constexpr uint16_t LINK_TX_RING_BYTES =
    static_cast<uint16_t>(LINK_TX_FRAMES * LINK_TX_BUFFER_BYTES);
// Bound the work per loop tick: a full 12 B frame is ~1 ms at 115200, so two
// frames cannot starve the control loop.
constexpr uint8_t LINK_TX_FRAMES_PER_TICK = 2;

uint8_t gTxBytes[LINK_TX_RING_BYTES];
uint8_t gTxLengths[LINK_TX_FRAMES];
uint8_t gTxHead = 0;  // next slot to fill
uint8_t gTxTail = 0;  // oldest queued slot
uint8_t gTxCount = 0;
uint16_t gTxOverflows = 0;
uint32_t gLastHeartbeatMs = 0;

FrameDecoder gDecoder;
LastGoodCache gCache;
FrameView gView;
LinkHealth gHealth;
LiveLinkPolicy gPolicy;
Liveness gLiveness = Liveness::Lost;

// Drop the oldest queued frame: a slow consumer loses the stalest telemetry
// rather than the newest control word.
void txDropOldest() {
  gTxTail = static_cast<uint8_t>((gTxTail + 1) % LINK_TX_FRAMES);
  --gTxCount;
}

bool txPush(const uint8_t* frame, uint8_t length) {
  if (frame == nullptr || length == 0 || length > LINK_TX_BUFFER_BYTES) {
    return false;
  }
  if (gTxCount == LINK_TX_FRAMES) {
    txDropOldest();
    ++gTxOverflows;
  }
  const uint16_t base = static_cast<uint16_t>(gTxHead * LINK_TX_BUFFER_BYTES);
  for (uint8_t i = 0; i < length; ++i) gTxBytes[base + i] = frame[i];
  gTxLengths[gTxHead] = length;
  gTxHead = static_cast<uint8_t>((gTxHead + 1) % LINK_TX_FRAMES);
  ++gTxCount;
  return true;
}

void txPump() {
  uint8_t budget = LINK_TX_FRAMES_PER_TICK;
  while (budget > 0 && gTxCount > 0 && Serial1.availableForWrite() > 0) {
    const uint16_t base = static_cast<uint16_t>(gTxTail * LINK_TX_BUFFER_BYTES);
    for (uint8_t i = 0; i < gTxLengths[gTxTail]; ++i) {
      Serial1.write(gTxBytes[base + i]);
    }
    gTxTail = static_cast<uint8_t>((gTxTail + 1) % LINK_TX_FRAMES);
    --gTxCount;
    --budget;
  }
}

// Drain at most LINK_RX_BYTE_BUDGET bytes and return to the caller as soon as
// one frame completes, so a long burst cannot delay the control loop.
void rxPump() {
  uint8_t budget = LINK_RX_BYTE_BUDGET;
  while (budget > 0 && Serial1.available() > 0) {
    --budget;
    const int incoming = Serial1.read();
    if (incoming < 0) break;
    const uint8_t byte = static_cast<uint8_t>(incoming);
    if (gDecoder.feed(byte, gView) != FrameDecoder::FeedResult::FrameComplete) {
      continue;
    }
    gCache.store(gView, millis());
    gHealth.onFrame(millis());
    return;
  }
}

}  // namespace

void linkBegin() {
  Serial1.begin(LINK_BAUD);
  gDecoder.reset();
  gCache.reset();
  gHealth.reset();
  gPolicy.reset();
  gTxHead = 0;
  gTxTail = 0;
  gTxCount = 0;
  gTxOverflows = 0;
  gLastHeartbeatMs = millis();
  gLiveness = gHealth.liveness();
}

// One non-blocking pass: receive, age the link, advance the policy, transmit,
// then emit the heartbeat if its cadence is due. millis() deltas are compared
// as signed offsets so the 49-day wrap cannot stall or fire the heartbeat.
void linkTick() {
  const uint32_t now = millis();
  rxPump();
  gLiveness = gHealth.update(now);
  gPolicy.onLinkEvent(gHealth.takeEvent());
  gPolicy.update(now);
  txPump();
  if (static_cast<int32_t>(now - gLastHeartbeatMs) >=
      static_cast<int32_t>(LINK_HEARTBEAT_MS)) {
    gLastHeartbeatMs = now;
    linkSend(MSG_HEARTBEAT, nullptr, 0);
  }
}

// Encode and queue one Mega -> Nicla frame. Returns false when the frame is
// malformed or wider than a transmit slot, in which case nothing is queued.
bool linkSend(uint8_t id, const uint8_t* payload, uint8_t payloadLength) {
  uint8_t frame[LINK_MAX_FRAME_BYTES];
  const uint8_t length =
      encodeFrame(frame, sizeof(frame), id, payload, payloadLength);
  if (length == 0 || length > LINK_TX_BUFFER_BYTES) return false;
  return txPush(frame, length);
}

rescue::Liveness linkLiveness() { return gLiveness; }

rescue::LinkPolicy linkPolicy() { return gPolicy.policy(); }

bool linkReceiveFresh(uint8_t id) { return gCache.isFresh(id, millis()); }

// A fresh last-good frame for id, or null when absent, uncached or expired.
const rescue::CachedFrame* linkLastGood(uint8_t id) {
  return gCache.get(id, millis());
}

uint16_t linkTxOverflows() { return gTxOverflows; }

uint8_t linkTxQueued() { return gTxCount; }
