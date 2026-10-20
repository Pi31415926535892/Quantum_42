// sketch.h — the one sketch's boot order and loop-pass plan (pure C++14).
//
// main.ino is the only file with a setup()/loop(). This header gives it the two
// decisions that belong to nobody else: the order the modules come up in, and
// which pass owns each loop iteration while the venue sweep is open. No AVR
// identifiers live here, so the boot table and the pass plan are host-testable
// exactly as they run.
//
// The boot table is a plain array of function pointers — one entry per module,
// single order, nothing hidden inside a setup() call. The steps themselves live
// in the delivery files and only declare their names here.

#ifndef RESCUE_ROBOT_SKETCH_H
#define RESCUE_ROBOT_SKETCH_H

#include <cstddef>
#include <cstdint>

namespace rescue {

// One boot step: brings a module up and returns. The context pointer is thread-
// ed through for a future step that needs a boot-time argument; nothing uses it
// today, and the passing caller (setup()) always hands it nullptr.
typedef void (*BootFn)();

// Walk the boot table once, in order, calling each step. The context pointer is
// reserved for a future step that takes a boot-time argument; today every step
// is no-arg, so it is deliberately unused and the caller always passes nullptr.
void runBootOrder(const BootFn* steps, size_t stepCount, void* context) {
  (void)context;
  for (size_t index = 0; index < stepCount; ++index) {
    steps[index]();
  }
}

// Which pass owns this loop iteration. While a calibration window is open the
// sweep owns the bar (it is sweeping the same eight sensors the mission reads),
// so the mission must not sample it in the same wall-clock window.
enum class LoopPass : uint8_t {
  CalibrationSweep = 0,  // tick the sweep; the mission waits
  Mission = 1,           // the sweep is closed; the mission owns the pass
};

inline LoopPass planLoopPass(bool sweepRunning) {
  return sweepRunning ? LoopPass::CalibrationSweep : LoopPass::Mission;
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_SKETCH_H