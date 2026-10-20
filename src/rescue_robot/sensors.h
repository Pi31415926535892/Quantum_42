// sensors.h — pure host-compilable logic for the front bumper plate and the
// victim probe pair (C++14, no AVR I/O).
//
// Two independent jobs live here:
//
//   1. Bumper contact. A front plate carries two soft push buttons (left and
//      right zones). Whatever the ranging stack believes, the plate physically
//      cannot miss a swing-and-hit, so a struck zone is DEBOUNCED and LATCHED
//      as a single event. The state machine reads the latch once and clears it
//      explicitly, so a contact that lands between two reads is still seen and
//      is never counted twice.
//
//   2. Victim classification, by physical property only. A reflectivity
//      reading (higher = more reflective) plus a conductivity boolean resolve
//      to a victim class:
//
//        silver  AND conductive     -> live   (deposit)
//        black   AND non-conductive -> dead   (deposit)
//        anything else              -> fake   (ignore, never deposit)
//
// Both are non-blocking: every entry point takes the caller's clock reading
// and returns immediately. Nothing here waits, polls, or busy-loops.

#ifndef RESCUE_ROBOT_SENSORS_H
#define RESCUE_ROBOT_SENSORS_H

#include <cstdint>

namespace rescue {

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

// A bumper push button is soft and mechanical; contact chatter spans tens of
// milliseconds. A press must be steady for this long before it counts.
constexpr uint16_t BUTTON_DEBOUNCE_MS = 20;

// Reflectivity decision band, in 10-bit ADC counts. A surface only turns
// silver when it reads at or above THRESHOLD + HYSTERESIS, and only turns
// black when it falls to or below THRESHOLD - HYSTERESIS. Between the two
// bands the previous reading stands, which keeps a victim resting on the
// band edge from flickering between live and dead on sampling noise.
constexpr int REFLECTIVITY_SILVER_THRESHOLD = 512;
constexpr int REFLECTIVITY_HYSTERESIS = 40;

// ---------------------------------------------------------------------------
// Bumper contact
// ---------------------------------------------------------------------------

// A momentary push button, debounced and latched, sampled entirely from the
// caller's millis() reading. The first update() only establishes the resting
// level, so a button held at boot latches on the next steady sample rather
// than on power-up noise.
class DebouncedButton {
 public:
  explicit DebouncedButton(uint16_t debounceMs = BUTTON_DEBOUNCE_MS)
      : debounceMs_(debounceMs) {}

  // Feed one raw sample. `rawPressed` is true for a made contact; `nowMs` is
  // the caller's unsigned millis() reading.
  void update(bool rawPressed, uint32_t nowMs) {
    if (!hasSample_) {
      hasSample_ = true;
      rawSample_ = rawPressed;
      candidateSinceMs_ = nowMs;
      return;
    }
    if (rawPressed != rawSample_) {
      rawSample_ = rawPressed;
      candidateSinceMs_ = nowMs;
      return;
    }
    commitIfSteady(nowMs);
  }

  // Debounced level the button is holding right now.
  bool isPressed() const { return stablePressed_; }

  // True while a press is waiting to be read. Peeking never clears it.
  bool hasPendingEvent() const { return latched_; }

  // Read a pending press and clear it. Returns false when nothing is pending,
  // so a single read cannot report the same strike twice.
  bool consumeEvent() {
    if (!latched_) return false;
    latched_ = false;
    return true;
  }

  // Forget the sample history and any latched press.
  void reset() {
    hasSample_ = false;
    rawSample_ = false;
    stablePressed_ = false;
    latched_ = false;
    candidateSinceMs_ = 0;
  }

 private:
  // Promote the candidate to the stable level once it has been held long
  // enough. Only a settled press latches; a settled release never does. The
  // subtraction is unsigned so a millis() rollover mid-window still measures.
  void commitIfSteady(uint32_t nowMs) {
    if (rawSample_ == stablePressed_) return;
    if (nowMs - candidateSinceMs_ < debounceMs_) return;
    stablePressed_ = rawSample_;
    if (stablePressed_) latched_ = true;
  }

  uint16_t debounceMs_;
  uint32_t candidateSinceMs_ = 0;
  bool hasSample_ = false;
  bool rawSample_ = false;
  bool stablePressed_ = false;
  bool latched_ = false;
};

// Which half of the bumper plate was struck.
enum class BumperZone { Left, Right };

// The plate: two independent debounced zones that never share state, so a
// left strike cannot mask a right one.
class BumperPlate {
 public:
  // Feed both raw button levels from the same millis() reading.
  void update(bool rawLeftPressed, bool rawRightPressed, uint32_t nowMs) {
    left_.update(rawLeftPressed, nowMs);
    right_.update(rawRightPressed, nowMs);
  }

  bool hasEvent(BumperZone zone) const {
    return zoneOf(zone).hasPendingEvent();
  }

  // Read and clear one zone's latched press.
  bool consumeEvent(BumperZone zone) { return zoneOf(zone).consumeEvent(); }

  // Read and clear the first pending press, reporting which side took it.
  // When both zones are latched the left side is reported, so the caller can
  // simply poll again for the remaining side.
  bool consumeAnyZone(BumperZone& zone) {
    if (left_.consumeEvent()) {
      zone = BumperZone::Left;
      return true;
    }
    if (right_.consumeEvent()) {
      zone = BumperZone::Right;
      return true;
    }
    return false;
  }

  // Current debounced level of one zone.
  bool isPressed(BumperZone zone) const { return zoneOf(zone).isPressed(); }

  // Forget both zones' history and latched presses.
  void reset() {
    left_.reset();
    right_.reset();
  }

 private:
  DebouncedButton& zoneOf(BumperZone zone) {
    return zone == BumperZone::Left ? left_ : right_;
  }
  const DebouncedButton& zoneOf(BumperZone zone) const {
    return zone == BumperZone::Left ? left_ : right_;
  }

  DebouncedButton left_;
  DebouncedButton right_;
};

// ---------------------------------------------------------------------------
// Victim classification
// ---------------------------------------------------------------------------

// Reflectivity verdict for the surface currently under the probe.
enum class SurfaceKind { Black, Silver };

// What the probe pair concludes about a victim. Anything off the truth table
// lands on Fake, which the mission logic ignores and never deposits.
enum class VictimClass { Live, Dead, Fake };

// Conductivity verdicts the other half of the table.
enum class Conductivity { Insulating, Conductive };

// Evaluate the truth table from the two physical readings.
constexpr VictimClass classifyFromReadings(SurfaceKind surface,
                                           Conductivity conductivity) {
  if (surface == SurfaceKind::Silver) {
    return conductivity == Conductivity::Conductive ? VictimClass::Live
                                                    : VictimClass::Fake;
  }
  return conductivity == Conductivity::Insulating ? VictimClass::Dead
                                                  : VictimClass::Fake;
}

// Apply one reflectivity sample to the previous verdict, honouring the
// hysteresis band. The first sample has no history to lean on, so it is read
// conservatively: only a decisively bright reading proves silver.
constexpr SurfaceKind applyReflectivity(SurfaceKind previous,
                                        int reflectivityReading) {
  const int enterSilver = REFLECTIVITY_SILVER_THRESHOLD + REFLECTIVITY_HYSTERESIS;
  const int leaveSilver = REFLECTIVITY_SILVER_THRESHOLD - REFLECTIVITY_HYSTERESIS;
  if (previous == SurfaceKind::Silver) {
    return reflectivityReading >= leaveSilver ? SurfaceKind::Silver
                                              : SurfaceKind::Black;
  }
  return reflectivityReading >= enterSilver ? SurfaceKind::Silver
                                            : SurfaceKind::Black;
}

// Holds the probe's current surface verdict and turns consecutive samples
// into a victim class. Held across samples so a victim resting on the
// decision band edge cannot flicker between live and dead.
class ProbeClassifier {
 public:
  // Classify one sample pair. `reflectivityReading` is a 10-bit count where a
  // higher number means a more reflective surface; `conductive` is the
  // conductivity probe's boolean verdict.
  VictimClass classify(int reflectivityReading, bool conductive) {
    surface_ = applyReflectivity(surface_, reflectivityReading);
    return classifyFromReadings(
        surface_, conductive ? Conductivity::Conductive : Conductivity::Insulating);
  }

  // Surface the probe currently believes it is touching.
  SurfaceKind surface() const { return surface_; }

  // Forget the surface verdict; the next sample re-proves it from scratch.
  void reset() { surface_ = SurfaceKind::Black; }

 private:
  SurfaceKind surface_ = SurfaceKind::Black;
};

}  // namespace rescue

#endif  // RESCUE_ROBOT_SENSORS_H
