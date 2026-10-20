// markers.h — pure host-compilable green-marker decision logic (C++14).
//
// Two downward-facing APDS-9960 colour sensors sit beside the black line (one
// each side) and answer only at 0x39, so they are reached through a TCA9548A
// mux: CH0 = LEFT, CH1 = RIGHT. Green on one side -> turn that way, green on
// both sides -> dead end, nothing green -> straight.
//
// Detection is active-low on the RGB green channel: the calibrated white
// baseline from the calibration sweep is the reference, and a side counts as
// green when its green-channel reading has dropped far enough below it. The
// dead-end case additionally needs MARKER_CONFIRM_SAMPLES agreeing sample pairs
// so a transient cannot trigger a 180 turn.
//
// No AVR identifiers live here. markers.ino is the only delivery layer and owns
// every address, register and bus call.

#ifndef RESCUE_ROBOT_MARKERS_H
#define RESCUE_ROBOT_MARKERS_H

#include <cstdint>

namespace rescue {

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

// I2C clock the delivery layer programs on the marker bus.
constexpr uint32_t MARKER_BUS_HZ = 400000;

// Per-side mux slice budget (select + sensor read + deselect). Every slice must
// finish inside 1 ms so a marker poll never stalls the drive loop.
constexpr uint32_t MARKER_SAMPLE_BUDGET_US = 1000;

// A side counts as green when its green-channel reading sits at or below this
// percentage of the calibrated white baseline. Green absorbs green light, hence
// the active-low test.
constexpr uint8_t MARKER_GREEN_DROP_PERCENT = 35;

// Consecutive sample pairs that must agree on "green on both sides" before the
// dead end is published.
constexpr uint8_t MARKER_CONFIRM_SAMPLES = 2;

// Re-arm hold after a published dead end: the robot is still sitting on the
// marker, so further readings are suppressed until the window expires.
constexpr uint32_t MARKER_DEADEND_BLIND_MS = 600;

// Mux channel masks: CH0 carries the LEFT sensor, CH1 the RIGHT sensor.
constexpr uint8_t LEFT_MUX_CHANNEL = 0x01;
constexpr uint8_t RIGHT_MUX_CHANNEL = 0x02;

// ---------------------------------------------------------------------------
// Green test
// ---------------------------------------------------------------------------

enum class MarkerState { None, Left, Right, Both };

// Side-level green test against the white baseline. An uncalibrated baseline
// reports false so the robot never turns on noise.
inline bool isGreen(uint16_t reading, uint16_t whiteBaseline) {
  if (whiteBaseline == 0) return false;
  const uint32_t retainedPercent = static_cast<uint32_t>(reading) * 100u;
  const uint32_t greenCeiling =
      static_cast<uint32_t>(whiteBaseline) *
      (100u - static_cast<uint32_t>(MARKER_GREEN_DROP_PERCENT));
  return retainedPercent <= greenCeiling;
}

inline const char* markerStateName(MarkerState state) {
  switch (state) {
    case MarkerState::None: return "none";
    case MarkerState::Left: return "left";
    case MarkerState::Right: return "right";
    case MarkerState::Both: return "both";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Timing model
// ---------------------------------------------------------------------------

// Wall-clock breakdown of one select / read / deselect slice.
struct MarkerSampleTiming {
  uint32_t selectUs = 0;
  uint32_t readUs = 0;
  uint32_t deselectUs = 0;

  uint32_t totalUs() const { return selectUs + readUs + deselectUs; }
  bool withinBudget() const { return totalUs() <= MARKER_SAMPLE_BUDGET_US; }
};

// ---------------------------------------------------------------------------
// Delivery seam
// ---------------------------------------------------------------------------
// The logic drives the bus through these callbacks so mux discipline (select ->
// read -> deselect) is testable on the host and identical on the robot.

using MarkerSelectFn = void (*)(uint8_t channelMask, void* ctx);
using MarkerReadGreenFn = bool (*)(uint16_t* green, void* ctx);
using MarkerDeselectFn = void (*)(void* ctx);
using MarkerMicrosFn = uint32_t (*)();
using MarkerMillisFn = uint32_t (*)();

struct MarkerBus {
  MarkerSelectFn select;
  MarkerReadGreenFn readGreen;
  MarkerDeselectFn deselect;
  MarkerMicrosFn micros;
  void* ctx;
};

struct MarkerRead {
  uint16_t green = 0;
  bool ok = false;
  MarkerSampleTiming timing;
};

struct MarkerPoll {
  MarkerState state = MarkerState::None;
  uint16_t leftGreen = 0;
  uint16_t rightGreen = 0;
  bool leftOk = false;
  bool rightOk = false;
  MarkerSampleTiming leftTiming;
  MarkerSampleTiming rightTiming;

  bool withinBudget() const {
    return leftTiming.withinBudget() && rightTiming.withinBudget();
  }
};

// ---------------------------------------------------------------------------
// Decision logic
// ---------------------------------------------------------------------------

// Side-independent marker decision. The white baseline is injected by the
// calibration sweep; the millis source is injected by the delivery layer and
// may be null when the caller does not need the dead-end blind window.
class MarkerLogic {
 public:
  explicit MarkerLogic(MarkerMillisFn millisSource = nullptr)
      : millisFn_(millisSource),
        whiteBaseline_(0),
        pendingBothSamples_(0),
        blindHeld_(false),
        blindSinceMs_(0),
        deadEndConfirmed_(false) {}

  void setWhiteBaseline(uint16_t whiteBaseline) {
    whiteBaseline_ = whiteBaseline;
  }

  uint16_t whiteBaseline() const { return whiteBaseline_; }

  // Feed one interleaved left/right sample pair; yields the published state.
  MarkerState poll(uint16_t leftGreen, uint16_t rightGreen) {
    deadEndConfirmed_ = false;
    if (insideBlindWindow(nowMs())) {
      pendingBothSamples_ = 0;
      return MarkerState::None;
    }
    const bool leftGreenSeen = isGreen(leftGreen, whiteBaseline_);
    const bool rightGreenSeen = isGreen(rightGreen, whiteBaseline_);
    if (!leftGreenSeen || !rightGreenSeen) {
      pendingBothSamples_ = 0;
      if (leftGreenSeen) return MarkerState::Left;
      if (rightGreenSeen) return MarkerState::Right;
      return MarkerState::None;
    }
    if (pendingBothSamples_ < MARKER_CONFIRM_SAMPLES - 1) {
      ++pendingBothSamples_;
      return MarkerState::None;
    }
    pendingBothSamples_ = 0;
    blindHeld_ = true;
    blindSinceMs_ = nowMs();
    deadEndConfirmed_ = true;
    return MarkerState::Both;
  }

  // Consecutive both-green sample pairs held so far (0 once published).
  uint8_t pendingBothSamples() const { return pendingBothSamples_; }

  // True only on the poll that published MarkerState::Both.
  bool deadEndConfirmed() const { return deadEndConfirmed_; }

  void reset() {
    pendingBothSamples_ = 0;
    blindHeld_ = false;
    deadEndConfirmed_ = false;
  }

 private:
  uint32_t nowMs() const { return millisFn_ != nullptr ? millisFn_() : 0; }

  bool insideBlindWindow(uint32_t now) const {
    return blindHeld_ && (now - blindSinceMs_) < MARKER_DEADEND_BLIND_MS;
  }

  MarkerMillisFn millisFn_;
  uint16_t whiteBaseline_;
  uint8_t pendingBothSamples_;
  bool blindHeld_;
  uint32_t blindSinceMs_;
  bool deadEndConfirmed_;
};

// ---------------------------------------------------------------------------
// Poll driver
// ---------------------------------------------------------------------------

// One mux slice for a single side: select -> read -> deselect. The bus is
// always returned to idle, even on a failed read.
inline MarkerRead readMarkerSide(const MarkerBus& bus, uint8_t channelMask) {
  MarkerRead result;
  const uint32_t startUs = bus.micros();
  bus.select(channelMask, bus.ctx);
  const uint32_t selectedUs = bus.micros();
  result.ok = bus.readGreen(&result.green, bus.ctx);
  const uint32_t readUs = bus.micros();
  bus.deselect(bus.ctx);
  const uint32_t releasedUs = bus.micros();
  result.timing.selectUs = selectedUs - startUs;
  result.timing.readUs = readUs - selectedUs;
  result.timing.deselectUs = releasedUs - readUs;
  return result;
}

// Interleaved left/right poll across separate mux reads, then the decision.
// A failed read is treated as "no green" on that side.
inline MarkerPoll pollMarkers(const MarkerBus& bus, MarkerLogic& logic,
                              uint8_t leftChannelMask,
                              uint8_t rightChannelMask) {
  const MarkerRead left = readMarkerSide(bus, leftChannelMask);
  const MarkerRead right = readMarkerSide(bus, rightChannelMask);

  MarkerPoll result;
  result.leftGreen = left.green;
  result.rightGreen = right.green;
  result.leftOk = left.ok;
  result.rightOk = right.ok;
  result.leftTiming = left.timing;
  result.rightTiming = right.timing;
  result.state = logic.poll(left.ok ? left.green : logic.whiteBaseline(),
                            right.ok ? right.green : logic.whiteBaseline());
  return result;
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_MARKERS_H
