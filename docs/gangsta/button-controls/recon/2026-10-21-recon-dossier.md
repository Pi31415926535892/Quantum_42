---
heist: button-controls
date: 2026-10-21
status: pending-review
---

# Reconnaissance Dossier: button-controls

## Objective

Give the robot two physical controls, replacing today's laptop/serial-only operation:

1. **Calibration button (Arduino Mega D43)** — held for **3 seconds** puts the robot into the
   calibration state, runs the calibration, and **stores the resulting calibration data in
   EEPROM**.
2. **StartRun button (Arduino Mega D41)** — first press **starts** the run; the next press
   **pauses**; the next press **resumes from where it left off**, alternating indefinitely.

Restated as firmware intent: add non-blocking, debounced, long-press-aware button input in the
Arduino glue layer, expose the gesture/state logic as host-testable pure code, add a first-class
**Paused** condition to the robot state machine, and persist calibration in EEPROM.

## Don's Decisions (2026-10-21) — scope adjustment

These supersede the corresponding open questions in this dossier's first draft:

1. **The journal is to be commented out** — keep the code in place for possible later use, but
   stop calling it. This frees the whole 4096-byte EEPROM for calibration persistence.
2. **Calibration happens only on a D43 long-press.** There is **no calibration sweep at boot**.
   Boot loads the stored calibration instead.
3. **Missing or corrupt calibration produces an LED-strip diagnostic** telling the operator to
   run calibration first. The robot must not silently run on hand-tuned defaults.

## Codebase Overview

Active firmware is the **firmware repo** `~/Documents/Arduino/Robocup.ino` (HEAD `6643479`,
2026-10-20 23:38 "Stream QTR telemetry by default; colour every fifth line"; one uncommitted
edit in `Robocup.ino.ino` — a Kp change at `:194`). The `firmware/` directory in this project repo
is a read-only pinned submodule (`REPOS.md` §3, §5) — **do not edit it**.

| Concern | Finding | Citation |
|---|---|---|
| D41 / D43 | **Unused.** No `D41`, `D43`, `pinMode(41/43)`, or any digital-input reference anywhere in the sketch or headers. Both pins are free. | repo-wide search of `Robocup.ino.ino` + all `*.h` |
| Pin style | `const int` / arrays, not `constexpr`; e.g. `LEFT_IN1 = 2` … `sensorPins[8] = {A14, A12, A10, A8, A6, A4, A2, A0}` | `Robocup.ino.ino:161-164`, `:185-188` |
| Buttons today | **None.** Zero `digitalRead`, `INPUT_PULLUP`, debounce, or ISR code. All operator input is single-character **Serial** commands. | `handleSerialCommands()` `Robocup.ino.ino:1354` |
| Start / pause today | Robot **auto-starts**: `setup()` → `Startup → Calibrate → LineFollow`. No pause state exists. `s` is a momentary `stopMotors()`; `g`/`j` are test readout modes; the warm-boot `logHold` pins the robot until any serial key. | `Robocup.ino.ino:1673-1752`, `:1810-1855`, `:1403-1407`, `:1830-1833` |
| State machine | Six states — `Startup, Calibrate, LineFollow, MarkerSettle, MarkerArmed, MarkerTurn`; ten events. **The single state assignment is `postEvent()`** → `nextState()` (pure, host-tested). | `RobotState.h:22-41`, `:53-77`; `Robocup.ino.ino:474-478` |
| Main loop | `handleSerialCommands(); journalPump(); … if (logHold) {stopMotors(); return;} switch (robotState) {…}` at ~200 Hz (`delay(5)` in `lineFollowTick`) | `Robocup.ino.ino:1810-1855`, `:1803` |
| Calibration flow | `calibrateSensors()` — **blocking 10 s sweep** (`while (millis()-start < 10000)`), captures `sensorMin[8]`/`sensorMax[8]` and per-flank `GreenThresholds`, then renders green. Called from `setup()` and from serial `k`. | `Robocup.ino.ino:303-380`, `:1739`, `:1408-1421` |
| Calibration derivation | Pure, I/O-free: histograms, band/plateau split, `thresholdsFromRef()`, `colourRefValid()`, `looksGreenCalibrated()`. | `ColourCalibration.h:52-267` |
| **Persistence today** | Calibration is **RAM-only** by explicit earlier Don decision. Comment: "Derived at boot from the sweep and kept in RAM only … they die at power-off". | `Robocup.ino.ino:35-39`; `docs/gangsta/colour-calibration/specs/2026-10-18-contract.md:334` |
| EEPROM consumer | **Journal only.** Magic 0–3, generation 4–5, then `227 × 18`-byte slots = 6–4091. **Only 4092–4095 (4 bytes) are free.** | `Journal.h:46-67`; `Robocup.ino.ino:793-804` |
| EEPROM writes | Raw AVR registers via `EEDR/EECR/EEPE` with a bounded queue drained by `journalPump()` | `Robocup.ino.ino:793-827` |
| Timed logic | `MARKER_POLL_MS 30`, `MARKER_REARM_MS 270`, `MARKER_EXPIRY_MS 1500`, `SETTLE_READ_MS 100`, telemetry 50 ms, journal heartbeat 500/50 | `Robocup.ino.ino:683-695`, `:923-928` |

## Existing Test Coverage

- Runner: `cd ~/Documents/Arduino/Robocup.ino && ./test/run_tests.sh` — bash glob of
  `test/*_test.cpp`, compiled with `g++ -std=c++14 -Wall -Wextra -Wpedantic`, run natively
  (`test/run_tests.sh:16-42`). 13 suites on disk; `CHECKPOINT.md:8-9` still claims 12 (stale).
- Assertions: header-only `CHECK` / `CHECK_EQ` / `RUN` in `test/check.h:18-41`.
- **No Arduino stubs exist and none are needed**: headers are pure `namespace q42` logic with
  `<stdint.h>` only; `millis()`/`digitalRead`/`EEPROM` live exclusively in `Robocup.ino.ino`,
  which is verified by `arduino-cli compile --fqbn arduino:avr:mega .`.
- Test files exercise the same shape we need: `robot_state_test.cpp` pins `nextState()`
  transitions, `colour_calibration_test.cpp` pins pure derivation, `journal_test.cpp` pins
  encode/decode + bounded queue.
- **Nothing tests buttons, pause, or EEPROM persistence** (they do not exist yet).

## Dependencies

None new. Existing toolchain: Arduino AVR core (`EEPROM`/`eeprom_read_byte` already used),
`Adafruit_NeoPixel`, `Wire`, `SoftWire`, plus g++ C++14 for host tests. No button library needed —
a ~30-line debounce/long-press header fits the codebase's style better than a dependency.

## Relevant Ledger Entries

Neither repo has a `docs/gangsta/` Ledger or `constitution.md`
(`docs/gangsta/rcj-rescue-line-2026/recon/2026-09-20-recon-dossier.md:36`). The de-facto
constitution is the signed contracts.

### Applicable Insights (standing conventions)

- **Pure logic in host-testable headers; Arduino I/O is compile-verified glue.**
  `colour-calibration/.../2026-10-18-contract.md:270`.
  → Button debounce + long-press + pause-gate logic belongs in a `q42` header with tests;
  only `pinMode`/`digitalRead`/`millis` plumbing belongs in `Robocup.ino.ino`.
- **The only state assignment is the single transition function.**
  `colour-calibration/.../2026-10-18-contract.md:272`.
  → Pause/resume must be an event through `nextState()`, not a side-channel boolean.
- **TDD: a failing test precedes every behaviour.** Same file `:271`.
- **Untethered field operation; no laptop required.** `:274`.

### Applicable Negative Constraints

- **NEVER require a laptop, a serial key, or any interactive input for a run** —
  Source: `uturn-confirm/specs/2026-10-16-contract.md:242`. Buttons satisfy this; they must not
  become a gate that bricks an untethered start.
- **NEVER let the sweep run with the motors live** — Source: same file `:288`. Button-triggered
  calibration must `stopMotors()` first (the existing `calibrateSensors()` already does).
- **NEVER let a failure require a power cycle to clear** — Source: same file `:289`. Calibration
  must be re-triggerable by the D43 button.
- **NEVER ship a magic number** — Source: `rcj-rescue-line-2026/specs/2026-09-22-contract.md:123`.
  Debounce, hold-time, and EEPROM addresses become named constants.
- **NEVER `delay()` block in a mission FSM state** — Source: same file `:124`, `:60`.
  The existing 10 s blocking sweep is already a known deviation (`CHECKPOINT.md:377`); a button
  makes it worse, because a dead 10 s window cannot hear the StartRun button.

## Risks and Unknowns

1. **RESOLVED by Don's decision 1 — EEPROM space.** Disabling the journal frees 0–4095. The
   calibration blob is then unconstrained in practice (~40–64 B). The residual risk moves to the
   *blast radius of commenting the journal out*: `journalPump()` in `loop()`, the EEPROM journal
   dump in `setup()`, and the warm-boot `logHold` all assume the journal exists
   (`Robocup.ino.ino:1812`, `:1709-1719`, `:1681-1697`, `:1830-1833`). `logHold` must be removed
   or re-based, otherwise a warm boot still pins the robot until a serial key — which now
   contradicts "start is a button". `Journal.h` and `journal_test.cpp` can stay untouched and
   green; only the calls go.
2. **The Don previously declined persisting colour calibration.** The Grilling log records
   "RAM-only — the Don explicitly declined EEPROM"
   (`colour-calibration/checkpoints/2026-10-18-grilling.md:34`). **This request reverses that.**
   The spec must state the reversal explicitly so the old contract line does not contradict the
   new one.
3. **RESOLVED by Don's decision 2 — boot semantics.** `setup()` no longer sweeps; it loads and
   validates the stored blob, then waits for the StartRun button. Residual unknowns: what
   `setup()` does when the blob is absent/corrupt (decision 3 says LED diagnostic, but does the
   robot *refuse to start* until a calibration is taken, or allow a start on defaults?), and
   whether a **failed** button calibration preserves the previous stored blob or invalidates it.
4. **Pause semantics are unspecified for the marker states.** Pausing mid-`MarkerSettle`,
   `MarkerArmed`, or a `MarkerTurn` pivot is not the same as pausing on a clean line segment.
   Options: pause only from `LineFollow` (simple, but a press during a marker does nothing) vs.
   pause from any active state and restore that state on resume (needs a saved-state slot and
   defined behaviour for expired timers, e.g. `MARKER_EXPIRY_MS`).
5. **Resume must reset the control time base.** `lineFollowTick()` derives PID `dt` from
   `micros()` (`Robocup.ino.ino:428-447`). A resume after 30 s would see a huge `dt` and a
   windup spike. Resume must reseed the PID clock — the same discipline the `k` re-run already
   applies (`:1418-1419`).
6. **Blocking code deafens the buttons.** The 10 s sweep and the red failure cue (up to
   3 × 950 ms) ignore input. With calibration now button-triggered, the sweep still blocks for
   10 s and cannot hear the StartRun button. The spec must define whether the sweep is made
   non-blocking (poll the buttons inside it) or whether button input is simply ignored during
   calibration. A long-press of D41 = start, so a press during the sweep must not queue a start
   that fires the moment the sweep ends.
7. **Physical wiring / documentation drift.** `HARDWARE.md:38-40` currently says the start button
   is *not used* by the sketch and forbids inferring pins; `HARDWARE.md:113-114` requires updating
   the pin table **and** the sketch together, then `scripts/check-hardware.py` must pass. Adding
   D41/D43 means `HARDWARE.md` §2.5 gets two rows with an evidence tag and
   `check-hardware.py` learns to verify them.
8. **Button electrical convention is unknown from the code.** No existing input pin to copy.
   Assumed: `INPUT_PULLUP`, active-LOW to GND, ~20–30 ms debounce, 3 000 ms long-press. This is
   an assumption to confirm with the Don — a button wired to +5 V with a pull-down would invert
   everything.
9. **D41/D43 on a Mega are ordinary digital pins**, but D41 is adjacent to the LED strip data pin
   D35 only in the sense of the header row; no timer/peripheral conflict is introduced by plain
   `digitalRead`. No PWM is involved.
10. **NeoPixel feedback is now specified in part, unspecified in the rest.** Don's decision 3 adds
    a required **"no calibration — run calibration first"** diagnostic pattern. Still open: the
    exact pattern (colour, blink rate, repeat count), whether it is shown once at boot or repeats
    until calibration is taken, whether the StartRun button is *refused* while it is showing, and
    whether entry into calibration and the Paused condition get their own patterns.
    `LedDisplay.h` currently has calibration/green/red frames (`LedDisplay.h:121-131`); diagnostic
    and paused frames would be new.
11. **The two buttons are not independent during a run.** A D43 long-press while paused/line-following
    must be defined: accepted as a re-calibration (stopping the run), or ignored unless idle.
12. **Losing the journal removes in-run diagnostics.** The journal is the only flight recorder
    (`Journal.h`, `LogBuffer.h`, `journalPump()`); commenting it out means the serial telemetry
    streams (`:1598-1660`) become the sole debug channel, and the field "what happened?" post-mortem
    is gone until it is re-enabled. This is the Don's call, logged here as the accepted cost.

## Recommended Scope

**In scope**

- **Journal disable (Don's decision 1).** Stop calling `journalPump()`, the `setup()` previous-session
  dump, and the `logHold` warm-boot gate; leave `Journal.h` / `LogBuffer.h` and their tests intact
  with a comment on how to re-enable. `loop()` then has no EEPROM or serial-gated hold.
- `ButtonInput.h` (new, pure, host-tested): debounce, edge detect, `heldFor(3 s)` long-press
  latch, and a per-tick gesture result per button (`none` / `shortPress` / `longPressHeld`).
  Both buttons are polled once per `loop()` pass — never inside a blocking loop.
- `RobotState.h`: add `Idle` (calibrated, waiting for StartRun), `AwaitingCalibration`
  (no valid blob — LED diagnostic, StartRun refused) and `Paused`, plus the events that drive
  them, keeping `nextState()` the only assignment site. Host tests pin every transition,
  including "resume returns to the state it paused from".
- `CalibrationStore.h` (new, pure, host-tested): encode/decode + magic + version + CRC for a
  fixed-size blob at a named base address. Tests cover round-trip, missing magic, torn/partial
  write, version mismatch, and CRC failure.
- `Robocup.ino.ino` glue: `pinMode(41/43, INPUT_PULLUP)`; D43 long-press → `stopMotors()` →
  calibration sweep → validate → EEPROM write → return to `Idle`; boot → EEPROM load → `Idle`
  or `AwaitingCalibration`; D41 press toggles `Idle → LineFollow → Paused → LineFollow`;
  PID time-base reseed and marker-timer handling on resume.
- LED diagnostic frame(s) in `LedDisplay.h` for "run calibration first", plus a calibration-entry
  cue and a Paused cue (patterns still to be pinned in the spec).
- `HARDWARE.md` §2.5 two new pin rows + `scripts/check-hardware.py` verification.
- Firmware `README.md` / `CHECKPOINT.md` updated: no boot sweep, button controls, EEPROM layout,
  journal disabled.

**Out of scope (unless the Don says otherwise)**

- Deleting the journal code — it is commented/disabled for later use, not removed.
- Debouncing via interrupts/ISRs (polling at ~200 Hz is ample).
- A button-driven menu, multi-click gestures, or a second-run/reset gesture.
- Persisting PID gains or any config beyond calibration.
- Replacing the existing serial commands (`k`, `s`, `g`, `j`, …) — `k` should keep working as the
  bench equivalent of the D43 long-press, now also writing EEPROM.

**Open questions the Grilling must settle** — does `AwaitingCalibration` hard-refuse StartRun
(risk 3); pause allowed in marker states (risk 4); button electrical convention (risk 8); the
diagnostic / paused / calibration LED patterns and their repeat behaviour (risk 10); D43 during a
run (risk 11); whether a failed button calibration overwrites or preserves the last-good blob
(risk 3); how resume interacts with `MARKER_EXPIRY_MS` and mid-turn phase clocks (risk 5).
