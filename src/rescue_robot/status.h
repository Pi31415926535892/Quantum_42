// status.h — the sketch's status lamp logic (pure C++14).
//
// One lamp on the fused LED (config.h STATUS_LED_PIN) carries the whole health
// vocabulary: a steady light while the robot trusts the field, a blink shape per
// latched reason it stopped trusting it. Nothing here touches a pin — main.ino
// reads the pattern and the lamp state each loop and drives the LED with them,
// and the tick's cadence is handed in as elapsed time, so nothing blocks.
//
// The vocabulary is deliberately small. Boot and LossOfPower are *starts*: the
// next Ready proves they are over and clears them. NiclaMissing and
// CalibrationAborted are *faults*: they latch — the reason the robot is running
// blind survives every healthy pass until the device is reset, because a robot
// that forgot why it is stopped is a robot that starts again on nothing. A
// later fault replaces an earlier one, so the newest reason is the one shown.

#ifndef RESCUE_ROBOT_STATUS_H
#define RESCUE_ROBOT_STATUS_H

#include <cstdint>

namespace rescue {

enum class StatusSignal : uint8_t {
  Boot = 0,               // power-on; cleared by the first Ready
  Ready,                  // a healthy pass: sensors, link and basis are up
  NiclaMissing,           // the host closed its READY window; latches
  CalibrationAborted,     // the venue sweep refused to start; latches
  LossOfPower,            // the restart signed back in; a start, not a fault
};

// A blink shape; onMs and offMs in milliseconds, offMs == 0 meaning "on".
struct StatusPattern {
  uint32_t onMs;
  uint32_t offMs;
};

namespace detail {

// The lamp's shapes. A fault blinks (an alarm deserves to be seen), a start
// blinks slowly (the robot is present but not yet sure), and a running robot
// sits on a steady light.
inline StatusPattern patternFor(StatusSignal signal) {
  switch (signal) {
    case StatusSignal::NiclaMissing:
    case StatusSignal::CalibrationAborted:
      return StatusPattern{250, 250};
    case StatusSignal::LossOfPower:
      return StatusPattern{2000, 500};
    case StatusSignal::Boot:
    case StatusSignal::Ready:
      return StatusPattern{1000, 0};
  }
  return StatusPattern{1000, 0};
}

// A fault is the reason the robot runs blind, so only a reset clears it; a
// start is proven over by the next Ready.
inline bool latches(StatusSignal signal) {
  return signal == StatusSignal::NiclaMissing ||
         signal == StatusSignal::CalibrationAborted;
}

struct StatusState {
  StatusSignal signal = StatusSignal::Boot;
  StatusPattern pattern = {1000, 0};
  bool lampOn = true;
  uint32_t elapsedAccum = 0;
};

// One instance per program, shared by every translation unit that includes this
// header: the lamp is a single global instrument, not one per TU. (Named for
// its owner because other modules keep their own state() in this namespace.)
inline StatusState& statusState() {
  static StatusState instance;
  return instance;
}

}  // namespace detail

// Return the lamp to the power-on position. Also the reset the bench reaches
// for: after a fault has latched, only this clears it.
inline void statusReset() {
  detail::StatusState& st = detail::statusState();
  st.signal = StatusSignal::Boot;
  st.pattern = detail::patternFor(StatusSignal::Boot);
  st.lampOn = true;
  st.elapsedAccum = 0;
}

// Install a signal. Ready clears a start (Boot, LossOfPower) but never a
// latched fault; any other signal replaces whatever the lamp was showing.
inline void statusSet(StatusSignal signal) {
  detail::StatusState& st = detail::statusState();
  if (signal == StatusSignal::Ready && detail::latches(st.signal)) return;
  st.signal = signal;
  st.pattern = detail::patternFor(signal);
  st.lampOn = true;
  st.elapsedAccum = 0;
}

inline StatusSignal statusSignal() { return detail::statusState().signal; }

inline StatusPattern statusPattern() { return detail::statusState().pattern; }

inline bool statusLampOn() { return detail::statusState().lampOn; }

// Advance the lamp's cadence with elapsed time handed in. A call that sees no
// time in front of it changes nothing; a steady pattern never times out, and an
// elapsed span larger than the current half-period crosses that many edges.
inline void statusTick(uint32_t elapsedMs) {
  detail::StatusState& st = detail::statusState();
  const uint32_t period = st.lampOn ? st.pattern.onMs : st.pattern.offMs;
  if (period == 0) {
    st.lampOn = true;
    st.elapsedAccum = 0;
    return;
  }
  st.elapsedAccum += elapsedMs;
  while (st.elapsedAccum >= period) {
    st.elapsedAccum -= period;
    st.lampOn = !st.lampOn;
  }
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_STATUS_H