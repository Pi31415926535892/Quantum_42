---
heist: rcj-rescue-line-2026
date: 2026-09-22
revision: 2
total-work-packages: 23
territories: 5
estimated-total-budget: 105000
---

# Execution Plan (Rev 2): RCJ Rescue Line 2026 — Team Quantum 42

Approved contract: `specs/2026-09-22-contract.md` (signed 2026-09-22).
**Don directive 2026-09-22:** original Rev 1 scope is NOT honest for a 1-month deadline.
Rev 2 trims to a month-doable, guaranteed-scoring envelope. Gates unchanged
(P0 → Q−21; mechanism decision → Q−14; P3 freeze → Q−7; Q ≈ 2026-10-22).

## Scope Adjustment Record (Rev 2 vs Rev 1)

| Item | Rev 1 | Rev 2 | Rationale |
|------|-------|-------|-----------|
| LoP radar sweep | 4-class structure profile (evac rect / sparse line / obstacle spike / ambiguous) | **2-class coarse** (enclosed-vs-open) + re-sweep retry; radar is an ASSIST — primary re-localization is bounded QTR line reacquisition | Profile classification is the slowest part; QTR reacquisition earns the resume regardless of radar |
| Nicla vision | goal tape + triangles + victim size/color hint | **goal tape + triangle blobs only**; victim hint is a stretch, never on critical path | Classification is 100% probes (contractual); trims vision.py risk |
| Victim mechanism | contained-cup POC, mechanism decided at Q−14 on evidence | **single simple contained-cup servo POC**, NO alternate variant; decide Q−14; no-capture lane is the default scoring path if bench evidence is weak | One mechanism, one shrug |
| Diag menu | motor/line/marker/probe/link/mechanism/fsm | **motor/line/link/probe only** | Dry-run harness covers the rest headlessly |
| Mega workers | 2 parallel | **1 sequential stream** | Parallelism across Mega *files* adds merge overhead; stream is faster here |
| Parallelism kept | — | **Mega ⇄ Nicla ⇄ Docs** run in parallel | The real independent edges |
| NEW | — | **WP-000 hardware inventory + power decision, Day 1–2** | Go/no-go on Nicla-required scoring; locks power Option A/B via the P0 EMI measurement |

## Territories

### Territory: HardwareGo (NEW)
**Crew Lead Domain:** Constitution CMD-01 — verify hardware truth BEFORE code claims it.
**Files:** `HARDWARE.md` (lock current §1–9), procurement note
**WPs:** WP-000
**Workers:** 1
**Budget:** 1000

### Territory: MegaCore
**Crew Lead Domain:** Real-time drive brain — config, motors, line, calibration, EEPROM, FSM engine. ONE sequential stream.
**Files:** `src/rescue_robot/{config.h, motors.ino, line.ino, calibration.ino, fsm.ino, eeprom_store.*}`
**WPs:** WP-001, WP-002, WP-003, WP-004, WP-005 (+ WP-000 scope overlap)
**Workers:** 1
**Budget:** 26000

### Territory: LinkIO
**Files:** `src/rescue_robot/{link.*, markers.*, sensors.*}`, `docs/mega-nicla-protocol.md`
**WPs:** WP-006, WP-007, WP-008, WP-009
**Workers:** 1
**Budget:** 19000

### Territory: NiclaPerception
**Files:** `nicla/{main.py, config.py, link.py, vision.py, tof.py, imu.py}`
**WPs:** WP-010, WP-011(T), WP-012(T), WP-013, WP-014
**Workers:** 1 (parallel stream)
**Budget:** 22000

### Territory: MissionLogic
**Files:** `src/rescue_robot/{victim.*, fsm reflex additions}`, `test/dryrun.ino`
**WPs:** WP-015, WP-016, WP-017(T), WP-018(T), WP-019
**Workers:** 1 (sequential — all touch the Mega FSM)
**Budget:** 28000

### Territory: DocsInfra
**Files:** `HARDWARE.md`, `README.md`, `PWM Test`, `Robocup.ino` (archive), `docs/field-mockup-guide.md`, `.gitignore`
**WPs:** WP-020, WP-021, WP-022(T)
**Workers:** 1 (parallel stream)
**Budget:** 9000

## Work Packages

**Prevention Guidance (all Work Packages):** Do NOT reproduce Gangsta-internal spec identifiers (FR-xxx, NFR-xxx, WP-xxx, where xxx is one or more digits) in any deliverable — source code, test files, code comments, or documentation. These identifiers belong only in `docs/gangsta/`.

### WP-000: Hardware inventory + power decision (Day 1–2, GO/NO-GO)
**Territory:** HardwareGo
**Contract Clause:** wiring ground truth; power Option A/B.
**Files:**
- Modify: `HARDWARE.md` (status flags per component)
**Acceptance Criteria:**
1. Every component confirmed present + testable: Mega, 4 motors, L298N, QTR-8A, **Nicla Vision (+USB/camera cable)** — this is the make-or-break line, **BSS138**, **TCA9548A**, **2× APDS-9960**, bumper buttons, reflective + conductive probe materials, servo + cup materials.
2. P0 EMI measurement (motors running → Nicla rail) done → power Option A or B locked.
**Decision branch:** Nicla unavailable within 2 weeks → fallback profile (see WP-000 note in Risks) — QTR-based goal/exit detection, probes-only evac classify, LoP via QTR reacquisition only. Field score still winnable; evac-zone triangle detect degrades to edge-tracking toward silver tape.
**Verification:** Day 1 meeting with Don; HARDWARE.md flags green.
**Budget:** 1000
**Dependencies:** None. FIRST.

### WP-001: Config + corrected motor layer
**Territory:** MegaCore
**Contract Clause:** drive remap; timer allocation; config single-source.
**Files:**
- Create: `src/rescue_robot/config.h`, `src/rescue_robot/motors.ino`
**Acceptance Criteria:**
1. LEFT/RIGHT constants match physical channels (LEFT=Ch B, RIGHT=Ch A), correct per-channel forward polarity.
2. Single motor constrain point; bidirectional `setMotor`; straight-trim offset slot.
3. Timer4 PWM on D6/D7; all thresholds/speeds/pins in `config.h`.
**Verification:** Compile (Arduino CLI); bench: `150,150` straight / `150,-150` pivot.
**Budget:** 4000
**Dependencies:** WP-000

### WP-002: Calibration sweep + health gate
**Territory:** MegaCore
**Contract Clause:** on-site calibration; rejection path.
**Files:** Create: `src/rescue_robot/calibration.ino`
**Acceptance Criteria:** 10 s sweep → per-sensor midpoints + weights; contrast gate rejects weak channels; retry ×2 then logged degrade; diagnostic dump.
**Verification:** Dry-run simulated channel contrast; reject/retry path asserted.
**Budget:** 4000
**Dependencies:** WP-004, WP-001

### WP-003: Line classifier + centroid PD
**Territory:** MegaCore
**Contract Clause:** centroid PID error; 4-state classes as qualifiers only.
**Files:** Create: `src/rescue_robot/line.ino`
**Acceptance Criteria:** weighted 0–7000 centroid; 4-state class enum (never control); small positive Kp, Kd disabled.
**Verification:** Dry-run synthetic sensor vectors → centroid + class assertions.
**Budget:** 5000
**Dependencies:** WP-001

### WP-004: EEPROM store (cal + accounting)
**Territory:** MegaCore
**Contract Clause:** persisted cal + hybrid accounting; write-before-release; wear budget.
**Files:** Create: `src/rescue_robot/eeprom_store.h`, `.ino`
**Acceptance Criteria:** versioned structs (cal / victim-accounting / LoP−landmark); write-before-release; bounded writes; corrupt load → silent fresh state.
**Verification:** Dry-run write→load→corrupt round-trips.
**Budget:** 4000
**Dependencies:** WP-001

### WP-005: FSM engine (transition table)
**Territory:** MegaCore
**Contract Clause:** non-blocking FSM; max-residence; watchdog.
**Files:** Create: `src/rescue_robot/fsm.h`, `.ino`
**Acceptance Criteria:** PROGMEM transition table; one tick/loop; per-state max-residence; watchdog kick each tick; no blocking in engine path.
**Verification:** Dry-run scripted event sequences + residence timeouts.
**Budget:** 5000
**Dependencies:** WP-001

### WP-006: Protocol specification document
**Territory:** LinkIO
**Files:** Create: `docs/mega-nicla-protocol.md`
**Acceptance Criteria:** frame format, sync, CRC8, message IDs both directions, heartbeat, staleness, resync rule; level-shifter wiring section.
**Verification:** Cross-checked against WP-007/WP-010.
**Budget:** 3000
**Dependencies:** None

### WP-007: Mega link layer
**Territory:** LinkIO
**Files:** Create: `src/rescue_robot/link.h`, `.ino`
**Acceptance Criteria:** Serial1 CRC8 frames; non-blocking ring; freshness flag; heartbeat ≥250 ms; resync after 3 bad frames; live-run degrade (hold-timeout → line-only).
**Verification:** Loopback corruption soak under motor run.
**Budget:** 6000
**Dependencies:** WP-001, WP-006

### WP-008: Marker pair (APDS-9960 + mux)
**Territory:** LinkIO
**Files:** Create: `src/rescue_robot/markers.h`, `.ino`
**Acceptance Criteria:** TCA9548A select/deselect per read (CH0=L, CH1=R), ≤1 ms time-box; G-poll with white baseline; proximity IR off; dead-end 2-sample confirm.
**Verification:** Bench: green card under each → LEFT/RIGHT/BOTH.
**Budget:** 4000
**Dependencies:** WP-001

### WP-009: Bumper + probes
**Territory:** LinkIO
**Files:** Create: `src/rescue_robot/sensors.h`, `.ino`
**Acceptance Criteria:** bumper debounce + latched left/right; reflectivity + conductivity probes with truth-table live/dead/fake (else=fake); non-blocking.
**Verification:** Bench: buttons; silver/black/plain touches.
**Budget:** 5000
**Dependencies:** WP-001

### WP-010: Nicla link mirror + config
**Territory:** NiclaPerception
**Files:** Create: `nicla/link.py`, `nicla/config.py`
**Acceptance Criteria:** MicroPython CRC8/IDs/heartbeat/resync mirror; responder returns IDLE if Mega silent >1000 ms; all thresholds in `config.py`.
**Verification:** Loopback vs WP-007 fixtures.
**Budget:** 4000
**Dependencies:** WP-006

### WP-011(TRIMMED): Nicla vision (goal + triangles)
**Territory:** NiclaPerception
**Files:** Create: `nicla/vision.py`
**Acceptance Criteria:** red goal-tape blob; red/green triangle blobs; arena color cal from card <90 s, QSPI-cached. Victim size/color hint = stretch, not gating.
**Verification:** OpenMV IDE test images → label assertions.
**Budget:** 5000
**Dependencies:** WP-010

### WP-012(TRIMMED): Nicla ToF + coarse radar
**Territory:** NiclaPerception
**Files:** Create: `nicla/tof.py`
**Acceptance Criteria:** continuous ranging; radar = coarse rotation sample with **2-class outcome** (enclosed / open) + re-sweep retry on ambiguity; out-of-range binning per effective range.
**Verification:** Recorded range arrays → class assertions.
**Budget:** 4000
**Dependencies:** WP-010

### WP-013: Nicla IMU
**Files:** Create: `nicla/imu.py`
**Acceptance Criteria:** gated yaw frames; pitch band for ramp-vs-seesaw discriminator.
**Verification:** Replayed samples → angular frames.
**Budget:** 3000
**Dependencies:** WP-010

### WP-014: Nicla main (boot/ready/modes)
**Files:** Create: `nicla/main.py`
**Acceptance Criteria:** boot→IDLE→READY; QSPI cal-cache load < READY timeout; mode control; radio-energy off.
**Verification:** Link bring-up vs WP-007.
**Budget:** 4000
**Dependencies:** WP-010, WP-011, WP-012, WP-013

### WP-015: Navigation reflex handlers
**Territory:** MissionLogic
**Files:** Modify `fsm.ino`, `config.h`
**Acceptance Criteria:** gap (hold-own-PD + timer-abort), marker turns + 180°, ramp (QTR stays primary, pitch band discriminator), seesaw (verify-on-mock then gyro-guided slow straight, declared carve-out), obstacle (ToF bias-away + bumper arc + reacquire), goal (vision ∧ ToF stop + 5 s hold).
**Verification:** Dry-run event→transition assertions; mock-field element passes (M3).
**Budget:** 8000
**Dependencies:** WP-005, WP-003, WP-007, WP-008, WP-009

### WP-016: Evac zone FSM + classify lane
**Territory:** MissionLogic
**Files:** Modify `fsm.ino`, `config.h`
**Acceptance Criteria:** entry = silver-tape blob ∧ QTR bright class; exit = black-tape; bounded states; probe truth-table live/dead/fake; no-capture lane always scores and never blocks exit bonus.
**Verification:** Dry-run evac script + bench probe classification.
**Budget:** 6000
**Dependencies:** WP-015, WP-009, WP-014

### WP-017(TRIMMED): Victim mechanism — single simple POC
**Territory:** MissionLogic
**Files:** Create `src/rescue_robot/victim.h`, `.ino`
**Acceptance Criteria:** ONE contained-cup servo design (Timer5); states idle/deploy/contain/release/home; non-blocking; behind isolated interface; deposit writes accounting before release; enable flag frozen at Q−14 on bench evidence. No alternate variant.
**Verification:** Dry-run state assertions; bench servo cycle.
**Budget:** 3000
**Dependencies:** WP-016, WP-005, WP-004

### WP-018(TRIMMED): LoP re-localization
**Territory:** MissionLogic
**Files:** Modify `fsm.ino`, `link.ino`
**Acceptance Criteria:** restart path (WAIT_READY 5 s → error LED → line-only); radar assist (2-class) to pick reacquisition direction; bounded QTR reacquisition search is the PRIMARY mechanism; accounting survives; re-sweeps capped.
**Verification:** Power-cut drill resumes ≤12 s, ≤1 landmark error — within radar assist OR with radar fully absent.
**Budget:** 4000
**Dependencies:** WP-015, WP-012, WP-004

### WP-019: Dry-run harness (foundation, parallel)
**Territory:** MissionLogic
**Files:** Create `test/dryrun.ino`
**Acceptance Criteria:** build-time `-DDRYRUN` feeds synthetic events into FSM with assertions (calibration, transitions, probes, accounting); headless CLI-compile pass.
**Verification:** Arduino CLI compile + assert exit 0.
**Budget:** 4000
**Dependencies:** WP-005; authored incrementally as modules land (starts in Group 1)

### WP-020: Mock-field guide
**Territory:** DocsInfra
**Files:** Create `docs/field-mockup-guide.md`
**Acceptance Criteria:** surrogate props per element; ≤120 s pre-run sequence; timed-rehearsal protocol; per-element test matrix.
**Verification:** Review; matches M3–M5 drills.
**Budget:** 3000
**Dependencies:** None

### WP-021: HARDWARE.md + README
**Territory:** DocsInfra
**Files:** Modify `HARDWARE.md` (§6 marked APPLIED; power-A/B + EMI section; level-shifter/Serial1 pins), `README.md`, Create `.gitignore`
**Acceptance Criteria:** hardware doc FIRST (CMD-01); README covers build/flash/wiring.
**Verification:** Review + cross-check vs WP-007 pins.
**Budget:** 4000
**Dependencies:** WP-000 (content lands incrementally)

### WP-022(TRIMMED): Bench remap + minimal diag
**Territory:** DocsInfra
**Files:** Modify `PWM Test`; archive-notes `Robocup.ino`
**Acceptance Criteria:** `PWM Test` passes straight/pivot; diag menu = motor/line/link/probe only.
**Verification:** Bench polarity; headless menu exercise.
**Budget:** 2000
**Dependencies:** WP-001

## Milestones (Don revision 2026-09-22: fully up and running by Oct 15, tune/calibrate Oct 15–22, qualifier ≈ Oct 22)

| Gate | Date | Exits |
|------|------|-------|
| M1 | Sep 26 (D+4) | WP-000 green (inventory + power locked); WP-001/003/022 bench-passing straight+pivot; HARDWARE.md wiring truth current |
| M2 | **Oct 1 (D+9) = Q−21 / P0** | WP-006/007/010 link soak clean; WP-008 markers; WP-009 probes; WP-002/004 cal+EEPROM round-trip |
| M3 | Oct 5 (D+13) | WP-005/015 nav reflexes pass dry-run + mock field elements; WP-011/012/013/014 Nicla online; mock field built (WP-020) |
| M4 | **Oct 8 (D+16) = Q−14 / mechanism gate** | WP-016 evac classify lane (no-capture) + WP-018 LoP drill ≤12 s; **first full-system assembly complete → running the full mission loop end-to-end**; mechanism POC bench verdict → WP-017 en/disable; ≥1 clean timed full run (cal ≤120 s) |
| M5 | **Oct 15 (D+23) = Q−7 / P3 FREEZE** | **Robot FULLY up and running** — both brains, all sensors, every element passing: ≥2 consecutive clean timed full runs recorded. Mission-logic frozen; ONLY tuning + calibration + rehearsal allowed Oct 15–22 |
| M6 | Oct 15–22 | Tiger-team window: tune, calibrate, rehearse, rehearse the 8-min clock, dry-run the Technical-Challenge chair test. No new mission-logic changes |
| M7 | Oct 22 (D+30) | Qualifier |

**Buffer tactic:** every milestone lands with a hard freeze — anything unready is **excised at the end of that day, never slipped into the next milestone** (scope discipline is the 1-month weapon; the scoring consequences of each excised element are quantified in the Contract's gate criteria). M4's mechanism gate is hard: the POC either passes bench evidence or the no-capture lane becomes the plan — no second chance, because Oct 15 is inviolable.

## Execution Order

1. **Group 0 (FIRST, blocking):** WP-000
2. **Group 1 (parallel after WP-000):** WP-001, WP-006, WP-012, WP-013, WP-019, WP-020, WP-021
3. **Group 2:** WP-003 (needs WP-001), WP-004 (needs WP-001), WP-005 (needs WP-001), WP-007 (needs WP-001, WP-006), WP-008 (needs WP-001), WP-009 (needs WP-001), WP-010 (needs WP-006)
4. **Group 3:** WP-002 (needs WP-004), WP-011 (needs WP-010), WP-014 (needs WP-010/011/012/013), WP-022 (needs WP-001)
5. **Group 4:** WP-015 (needs WP-005/003/007/008/009)
6. **Group 5:** WP-016 (needs WP-015/009/014), WP-018 (needs WP-015/012/004)
7. **Group 6:** WP-017 (Q−14-informed)
8. **Group 7:** integration per gates; WP-021/020 final sweep; P3 freeze.

**Conflict note:** WP-015/016/018 all edit the Mega FSM — strictly sequential within MissionLogic. Nicla stream (WP-010…014) and DocsInfra run in parallel with MegaCore — these are the real parallel edges.

## Baseline Verification

- Tests: none exist (dry-run harness is a new deliverable; do not gate on broken `Robocup.ino`).
- Dependencies: QTRSensors assumed in Arduino libs (verify at M1 on bench). No new third-party runtime deps beyond config-gated optional handling (mux/APDS inline).
- Branch: `heist/rcj-rescue-line-2026` from `main` (2026-09-22). Untracked baseline: `.DS_Store`, `HARDWARE.md`, `docs/` (intentional Heist artifacts).

## Execution Status (updated 2026-09-22, The Hit — Group 1 complete)

### Landed, tests green
| WP | Artifacts | Verification |
|----|-----------|--------------|
| WP-001 | `src/rescue_robot/config.h`, `motors.h`, `motors.ino`; `test/motors_test.cpp` | `ALL PASS` (g++ -std=c++14) — pins {6,4,2}=LEFT/{7,3,5}=RIGHT, polarity fixed, 150/150 straight + pivot asserted. `.ino` unverified (no AVR toolchain); bench per HARDWARE §6/§7 |
| WP-006 | `docs/mega-nicla-protocol.md` (323 lines) | Self-check CRC8 worked vectors: `AA 55 02 00 2A`, `AA 55 04 11 00 03 98` |
| WP-012 | `nicla/tof.py` (148 l), `nicla/__init__.py`, `test/tof_test.py`, `test/__init__.py` | 12/12 unittest OK; 2-class radar (enclosed/open), out-of-range bin 4000 mm, retry ≤2 |
| WP-013 | `nicla/imu.py` (98 l), `test/imu_test.py` | 11/11 unittest OK; deg/deg/s, yaw gate, pitch bands (ramp 20–25°) |
| WP-019 | `test/run_tests.sh`, `test/dryframe.h` (CHECK/CHECK_EQ/CHECK_NEAR), `test/example_test.cpp`, `test/dryrun.ino` | `run_tests.sh` exit 0; dryrun.ino host-compiles with -DDRYRUN |

Run everything: `./test/run_tests.sh` (must exit 0).

### Next dispatch (Group 2 — no bench needed, parallel)
Ordered by dependency; MegaCore workers strictly sequential against shared files:
1. `WP-010` nicla link mirror (needs WP-006; owns `nicla/link.py`) — first, unblocks the whole Nicla stream
2. `WP-007` Mega link layer (needs WP-001, WP-006; owns `src/rescue_robot/link.*`)
3. `WP-003` line classifier + PD (needs WP-001; owns `line.ino`)
4. `WP-004` EEPROM store (needs WP-001; owns `eeprom_store.*`)
5. `WP-008` markers (needs WP-001; owns `markers.*`)
6. `WP-009` bumper + probes (needs WP-001; owns `sensors.*`)
7. `WP-005` FSM engine (needs WP-001; owns `fsm.*`) → then WP-002 calibration (needs WP-004)

### Reconciliations to fold in while writing link/markers (from WP-012/013 reports)
- `nicla/tof.py` radar wire shape is **provisional**: `{"bins_mm":[int]×18, "outcome":"enclosed"|"open", "retry":bool}` — WP-010's `link.py` must adopt this into the protocol message layout (or bump the protocol doc).
- `imu.py` units (deg/deg/s, gyro FS1000, accel FS8g constants) must match `docs/mega-nicla-protocol.md` yaw/pitch field scale; adjust constants only, logic is fixed.
- `tof.py` VL53L1 driver call shape unverified (sealed seam `Vl53L1xDriver`) — one-line swap at first Nicla bring-up.

### Known-benign LSP diagnostics (do not chase)
- `nicla/tof.py`: unresolved `machine`, `vl53l1x` imports = MicroPython-only modules behind the driver seam.
- `test/tof_test.py`: unresolved `nicla.tof` = cwd/package-path artifact; `python3 -m unittest test.tof_test` from repo root passes (empty `__init__.py` added in both packages to defeat stdlib `test` shadowing).

### Blocked on the Don (WP-000, Day-1 GO/NO-GO)
Physical inventory of TO-ACQUIRE items + P0 EMI measurement (see BoM in HARDWARE.md §1/§5/§9 and the HoW summary) — NEEDS the Don's bench. Nicla Vision availability is the make-or-break line; if unacquirable by ~Oct 6 use the WP-000 fallback profile.

### Group 2 (all landed 2026-09-22)
| WP | Artifacts | Verification | Notes |
|----|-----------|--------------|-------|
| WP-010 | `nicla/link.py` (692 l), `nicla/config.py`, `test/link_test.py` | 35/35 unittest OK; full suite 11 suites green | Radar mapping resolved: tof.py profile → RADAR_RESULT zone/class per doc. **First attempt returned blank work, re-dispatched.** Files exceed 300 (see exemptions) |
| WP-007 | `src/rescue_robot/link.h` (444 l), `link.ino`, `test/link_test.cpp` (531 l) | ALL PASS, off-target link.ino OK | Cross-checked TX/RX slots: Mega TX=12 (Mega→Nicla), Nicla TX=14 (widest blob PL9+5). NOT a defect — decoder is protocol-plenum sized (256 B) |
| WP-003 | `line.h` (300 l), `line.ino`, `test/line_test.cpp` | 65 asserts PASS | Polarity is calibration data; pre-run sweep mandatory; placeholder cal = full-scale (highest-consequence assumption). Pins live in line.ino, NOT config.h — follow-up move to config.h noted |
| WP-004 | `eeprom_store.h` (399 l), `eeprom_store.ino`, `test/eeprom_test.cpp` (447 l) | 390 asserts PASS | Write budget now host-tested; release bypass gap closed (RevisionStale guard) |
| WP-005 | `fsm.h` (300 l), `fsm.ino`, `test/fsm_test.cpp` | 79 asserts PASS | Engine API; event-row-before-EVENT_ANY resolution; reject-newest queue; WDT enable left commented (lock-bit caution) |
| WP-008 | `markers.h` (252 l), `markers.ino`, `test/markers_test.cpp` (323 l) | 54 asserts PASS | 1 ms mux slice budget; APDS reg consts unbench-verified (ID read + G dump first bring-up); poll on millis cadence (31 ms integration) |
| WP-009 | `sensors.h` (241 l), `sensors.ino`, `test/sensors_test.cpp` (299 l) | 86 asserts PASS | Provisional pins: bumper LEFT=D8/RIGHT=D9, reflectivity=A1, conductivity=D22 — LOCK in HARDWARE.md before first flash. **Probe polarity unverified: first bench task flips `==HIGH` if pull-up probe. else→FAKE is fail-safe direction** |
| WP-020 | `docs/field-mockup-guide.md` (399 l) | ≤120 s pre-run sums to 110+10 reserve; LoP 11.5 s ≤ 12 s bound | **Key finding: evac zone needs a real ~12 cm vertical wall** or radar always reads open → re-localization drills invalid; made mandatory + pre-drill bench check. Radar sweep runs concurrent with primary QTR reacquisition |

### WP-000 UPDATE (2026-09-22, Don): inventory = **GO**
Don confirmed all TO-ACQUIRE items are in hand, incl. Nicla Vision + cable, 2× APDS-9960, servo+cup, mock-field props. No fallback needed. **Still pending on the bench:** P0 EMI measurement (motors running → Nicla rail) to lock power Option A vs B. HARDWARE.md gets the locked pins/options in WP-021 once the Don runs it.

### Don-approved long-file exemptions (2026-09-22) — named in the Contract so Laundering passes
| File | Lines | Note |
|------|-------|------|
| `src/rescue_robot/eeprom_store.h` | 399 | protocol+coverage floor, Don-approved |
| `test/eeprom_test.cpp` | 447 | explicit assertions are the evidence, Don-approved |
| `src/rescue_robot/link.h` | 444 | 23 ids both directions + policy, Don-approved |
| `test/link_test.cpp` | 531 | 6-group protocol suite, Don-approved |

Also pending Don decision: `nicla/link.py` (692) + `test/link_test.py` (559) exceed the 500 hard block — waiver request or split still open.

### Group 3a (all landed 2026-09-22) + Don waiver for the 300–500 band
| WP | Artifacts | Verification | Notes |
|----|-----------|--------------|-------|
| WP-002 | `calibration.h` (381 l), `calibration.ino`, `test/calibration_test.cpp` (474 l) | 165 asserts PASS | 10 s sweep, midpoint thresholds, contrast-health gate (11.2 mm height), retry ×3 → Persisted/LastGoodPersisted/Degraded/Abort. **Flagged:** `rescue::ChannelCalibration` name collision between line.h and eeprom_store.h — see reconciliation #6 |
| WP-011 | `nicla/vision.py` (357 l), `test/vision_test.py` (541 l) | 29/29 PASS | goal + red/green triangle blobs; **lab tolerance unbench-valued**; cache record NVC1+CRC32+JSON; goal-vs-red disambiguation driven by REQUEST_BLOB mask bits (main.py duty) |
| WP-022 | `PWM Test` remapped; `Robocup.ino` archived-header; `diag.ino` + `diag_parse.h`; `test/run_tests.sh` enhanced | 17 suites green; `./test/run_tests.sh` exit 0 | PWM Test standalone (MAX_SPEED 200, straight/pivot); Robocup.ino Kp=0.10,Ki=0,Kd=0, fixes line-lost predicate; diag = motor/line/link/probe menu; run_tests.sh now includes stub include path (shared-file change, keeper) |

**DON WAIVER (2026-09-22, approved):** extend the exemption list with the four files above
(`calibration.h` 381, `test/calibration_test.cpp` 474, `nicla/vision.py` 357,
`test/vision_test.py` 541) — same protocol-coverage floor as the four already named in the
Contract. Also approved earlier: `nicla/link.py` (692) + `test/link_test.py` (559).

### Group 3b + Group 4 (all landed 2026-09-30)
| WP | Artifacts | Verification | Notes |
|----|-----------|--------------|-------|
| WP-014 | `nicla/main.py` (300 l), `test/main_test.py` (591 l) | WP: 31/31 unittest; full suite 118 tests OK | boot→IDLE→READY (READY is purely Nicla self-check, no Mega dependency); QSPI cal-cache under READY timeout (empty-cache readies with cal_cached=0; unreadable flash does NOT); CAL_START→CAL_COMPLETE {0/1/2/3}; REQUEST_BLOB mask drives emission; heartbeat during slow cal; radios compile-off. **Hand-off fact:** REQUEST_RANGE one-shot is gated on streams_allowed(GATE_TOF) in landed link.py — Mega must set TOF gate bit before issuing it |
| WP-015 | `mission.h` (347 l), `mission.ino` (143 l), extended `fsm.h` (308 l)/`fsm.ino` (229 l), `test/mission_test.cpp` (670 l) | 246 checks PASS on the REAL table; full suite **19 suites green** | Gap (hold-PD+timer-abort+reacquire), marker turns + 180° (2-sample confirm), ramp (QTR primary + pitch band), seesaw (gyro-guided slow straight, carve-out gate `MISSION_..._VERIFY`), obstacle (ToF bias + bumper arc, drain-in-loop, reacquire), goal (GOAL_BLOB ∧ fresh ToF close → STOP_HOLD 5 s). **Marker side = turn toward green (INVERT=false)** — matches contract FR-008/HARDWARE §9; the mock-guide E2a/E2b was inverted and has been corrected. **Bumper = latched tap OR held zone** (fast tap can start/end between passes). Marker reflex disarmed until calibration exists (white baseline=0 → no green). Goal size = long axis of GOAL_BLOB. Defect elsewhere: `ChannelCalibration` collision (reconciliation #6) |

**Group 3b/4 long-file requests pending Don:** `test/mission_test.cpp` (670) + `test/main_test.py` (591) exceed the 500 hard block; `mission.h` (347) + `fsm.h` (308) exceed the 300 target (within the approved 300–500 band — recommend same blanket). Precedent: every prior test/coverage-floor waiver granted. — **RESOLVED 2026-09-30: Don approved `mission_test.cpp` 670 + `main_test.py` 591, and folds the 300–500 band for `mission.h`/`fsm.h`.**

### Group 5a (all landed 2026-09-30)
| WP | Artifacts | Verification | Notes |
|----|-----------|--------------|-------|
| WP-016 | `mission.h` (347→**699** l), `mission.ino` (143→179), `fsm.h` (305→316), `fsm.ino` (292→338), `test/evac_test.cpp` (**854** l), `test/mission_test.cpp` (670→677) | 331 evac checks + 251 mission checks; full suite **20 suites green** | Entry = fresh silver TAPE_BLOB ≥min size ∧ link usable ∧ bar wide-bright; grey-area (XOR of blob/bar) never enters; claim outranks gap (bar is off the line in the zone); exit = fresh black TAPE_BLOB ∧ dark channel, gyro back-out. **Followed the contract (TAPE_BLOB) over a wrong "triangle blob" phrase in the Underboss brief** — correct. Four-signature probe truth table; flicker-timeout; **accounting written (hook A) before release (hook R), proven in-order, one-shot** — power cut between them can orphan the cup but never unrecorded. `MISSION_EVAC_MECHANISM_ENABLED=false` frozen; no-capture lane still locates/probes/classifies/records/scores AND still exits on black tape. 5 bounded states; VP entry stamps idempotent; on_exit shared hook dropped (engine ordering), teardown is row action. Risks: bright-class = "no line + ALL_WHITE" (same coarse class as gap — never a decision alone); contact = bumper-first ∧ ToF ≤200 mm; speeds/budgets unbenched |

**NEW over-500 growth from WP-016 (pending Don waiver, same coverage floor):** `mission.h` (699) and `test/evac_test.cpp` (854). — **RESOLVED 2026-09-30: Don approved both.**

### Group 5 (all landed 2026-09-30 — MissionLogic COMPLETE; only WP-017 + WP-021 remain)
| WP | Artifacts | Verification | Notes |
|----|-----------|--------------|-------|
| WP-018 | `mission.h` (699→**1013** l), `mission.ino` (179→264), `fsm.h` (316→326), `fsm.ino` (338→477), `test/lop_test.cpp` (**829** l), `test/sketch_syntax_test.cpp` (stubs) | 303 evac…lop checks + 255 mission checks; full suite **21 suites green**; sketch_syntax clean -Wall -Wextra | WAIT_READY 5 s handshake arming; radar 2-class assist biases ONLY the first search leg (never gates — QTR reacquisition is PRIMARY); bounded ladder (search 3 s, resume 3 s, re-sweep cap=2); accounting re-derived from store, published once at handoff, ±1 landmark bound; **never-strand chain** (every LoP state has condition row + budget row). Tests caught 2 real bugs: chain handed on TWO rows per pass (wrong helper + wrong policy), and staging keyed off landmark count restarted COMPLETED runs (separated cut-short flag from progress count). Defect in-workaround: `mission.h` cannot include `eeprom_store.h` because of the `ChannelCalibration` ODR collision → local `Accounting` seam translated in mission.ino (REINFORCES reconciliation #7: rename the store's `ChannelCalibration` → `StoredChannelCalibration` and drop the `#define`, now a Group 7 integration MUST-DO before full-sketch assembly) |

**NEW over-500 growth from WP-018 (pending Don waiver, same coverage floor):** `mission.h` (1013) — it was 699 when approved, LoP added ~314 lines (the designated decision home) — and `test/lop_test.cpp` (829). **Recommend: approve the band, one final time for mission.h (it is the decision homeland; Group 7 rename will NOT move it under 500 — accept a stable 1k-class decision file) + lop_test.cpp.**

### Reconciliation outcomes (additional, folded 2026-09-30)
6. **Marker-side direction = turn toward green** — contract FR-008/HARDWARE §9/markers.h all agree; the mock-field guide had E2a/E2b inverted. Code `MISSION_MARKER_SIDE_INVERT=false` is CORRECT; the doc was fixed (2026-09-30).
7. **`rescue::ChannelCalibration` collision (latent build landmine):** `line.h:87` and `eeprom_store.h:88` each define a different `rescue::ChannelCalibration`; `calibration.h:32` papers over it with `#define ChannelCalibration StoredChannelCalibration` + include-ordering (must include calibration.h first). WORKS today only through that seam — rename one type (planned: store's → `StoredChannelCalibration`) and drop the `#define`, to be executed before the full-sketch assembly (Group 7) so no TU can silently break.
   - **RESOLVED (Group 7 integration pass, 2026-09-30).** The store's per-channel type is `rescue::StoredChannelCalibration`; the line module keeps `rescue::ChannelCalibration` for its own `{black, white}`. `calibration.h` includes both headers plainly, in any order, with no `#define`/`#undef`. (Include guards were never the issue — each header had one; the clash was two *different* headers declaring the same name in one TU.) Consequences: `mission.h` now includes `eeprom_store.h` and stages/restages `AccountingRecord` directly — the local `Accounting` struct and the two translation sites in `mission.ino` are deleted, and `test/lop_test.cpp`'s copies of that translation are gone. `test/store_seam_test.cpp` (68 asserts) includes `mission.h`, `eeprom_store.h` and `calibration.h` in the previously-hostile order to hold it shut.
8. **REQUEST_RANGE gate:** landed `link.py` only answers a REQUEST_RANGE one-shot when the TOF gate bit is set (streams_allowed) — the Mega FSM must set bit2 before issuing the request.
9. **Vision caveat:** red-triangle path reports the largest red blob; a goal card in frame can satisfy a red mask with a size that overflows `size:u8` → link drops the frame. main.py must drive mask bits to avoid false red emissions (see WP-014 hand-off).
10. WP-002 calibrations tie into markers work: markers stay disarmed until a stored white baseline exists (`missionBegin` feeds it; no record → no green → safe).

### Group 7 integration (COMPLETE 2026-10-01 — full-sketch assembly landed; suite GREEN 23/23, exit 0)
- **`ChannelCalibration` rename (reconciliation #7):** DONE and kept (store → `StoredChannelCalibration`, seam deleted, local `Accounting` translation removed; store_seam_test 67 checks guard it).
- **NEW `src/rescue_robot/sketch.h` (49 l) + `status.h` (126 l):** pure C++14, no Arduino deps → host-testable. `runBootOrder`/`planLoopPass` drive the sketch; status-lamp logic latches faults (NiclaMissing, CalibrationAborted) that Ready can never clear; Boot/LossOfPower are transient and Ready clears them.
- **NEW `src/rescue_robot/main.ino` (134 l):** sole setup()/loop(). Boot table = statusBegin, eepromStoreBegin, motorsBegin, sensorsBegin, lineBegin, markersBegin, linkBegin, fsmBegin, beginCalRun, missionBegin. loop(): lampTick → diagTick → planLoopPass — sweep branch ticks the calibration sweep + settles once (CalFallback::LastGoodThenDegrade; Abort → latched CalibrationAborted), mission branch runs missionTick + sampleHealth (link Lost → NiclaMissing else Ready). No delay() anywhere.
- **`fsm.ino` (477→468):** setup()/loop() moved out; entry-point comment updated. **`mission.ino` (+8):** forward decls `readReflectivity`/`readConductivity` added (sensors.ino sorts after mission.ino — latent undeclared-call break).
- **`config.h` (78→87):** `STATUS_LED_PIN = 13`; `LINE_SENSOR_PINS` single owner here (line.ino consumes + static_asserts A-names).
- **`test/sketch_assembly_test.cpp` (228→254, REWRITTEN):** the delivered version could not compile (global mocks clashed with the real delivery Begin() definitions + dryframe misuse). Now: file-local mocks, real single-TU include of every .ino, boot-order/loop-pass/status behavior, source-wholeness, boot-table text scan, no-delay, pins-in-config. **53 PASS.**
- **`test/sketch_syntax_test.cpp` (+3):** SaveResult ambiguity fixed (global `enum class SaveResult` forward-decl removed; stub returns `rescue::SaveResult{}`). **ALL PASS.**

### Remaining dispatch order (Group 5 → eldest, updated 2026-10-01)
- **Group 6 (Q−14-informed, Oct 8):** WP-017 mechanism (needs WP-016/005/004 — all landed; the POC code can be authored now, the en/disable flag freezes on Oct 8 bench verdict). **NOT yet authorized by the Don — re-ask before dispatch.**
- **WP-021 (parallel edge, needs Don EMI):** HARDWARE.md (§6 APPLIED, WP-009 pins, power A/B), README.md, `.gitignore`. Power A/B still locked on the pending P0 EMI run.

## Risks (Rev 2)

1. **Nicla unavailability (CRITICAL external):** if unacquirable by ~Oct 6 → fallback profile in WP-000: QTR goal/exit detection, probes-only evac classify, line-only LoP. Field score still winnable; the plan already isolates Nicla to goal/triangle blobs + radar-assist so the blast radius is contained.
2. **Bench time:** mitigations: mock field built by M3, rehearsals by M4, milestone freezes.
3. **Qualifier rules delta:** config-driven (contract) — expected rules update absorbed at the Technical Challenge layer, not in mission logic.