---
heist: rcj-rescue-line-2026
date: 2026-09-20
status: pending-review
mode: greenfield
---

# Reconnaissance Dossier: RCJ Rescue Line 2026

## Objective

The Don wants Team Quantum 42's robot to compete in RoboCupJunior Rescue Line 2026 and score
points across the full course. The interpreted objective: transform the current (broken)
PID-only line-follower harness into an **autonomous, rules-compliant rescue robot** that
follows the black line, handles every scored obstacle (gaps, speed bumps, intersections,
dead ends, ramps, seesaws, obstacles), enters the evacuation zone, rescues the right victims
to the right evacuation points, and reaches the goal tile — within the 8-minute game,
fully autonomously, with a single-button LoP restart and no pre-mapping.

Basis: RCJ Rescue Line 2026 rules (final, published 2026-03-29). **The Don notes an updated
version will be released later** → design must be config-driven so rule deltas are cheap.

## Workspace State

Small Arduino repo (18 commits of Kp tuning). Existing artifacts:

- `Robocup.ino` — PID line follower + QTR-8A calibration. **Diagnosed broken:** LEFT/RIGHT
  channel constants swapped vs physical wiring (Left pair = Ch B), Ch B driven reversed
  (robot pivots on `150,150`); `Kp = -10` wrong-sign + ~500x too large (bang-bang);
  line-lost predicate inverted. Pin-remap fix documented in HARDWARE.md §6, not yet applied.
- `PWM Test` — bidirectional motor bench test; same swapped constants.
- `HARDWARE.md` — BOM, wiring tables, ASCII diagrams, discrepancy log, checklist, tuning ref.
- `RoboCupBot/` — chassis CAD (Base.stl, Robocup.f3d). 4 BIO motors, L298N, QTR-8A,
  sensor ride-height ~11.2 mm.

No `docs/gangsta/` exists → no institutional memory, no constitution (Ledger search: empty).

## Architecture — Two-Processor Split (Don's addition)

**Decision (Don):** the evacuation/rescue perception will run on an **Arduino Nicla Vision**
co-processor with its built-in ToF. This places Quantum 42 in the same "brain + real-time
base" architecture as every serious analogue (Airborne, BAM, FermiLabs).

Verified specs (docs.arduino.cc/pro/hardware-product-nicla-vision; docs.openmv.io Nicla
quickref): 22.86 × 22.86 mm; STM32H747AII6 dual-core (Cortex-M7 400–480 MHz + Cortex-M4
200–240 MHz); GC2145 2MP color camera; **VL53L1CB ToF (up to ~4 m, on internal I2C2)**;
LSM6DSOX 6-axis IMU; MP34DT06 mic; Murata 1DX Wi-Fi + BLE; 2MB flash + 16MB QSPI; Li-Po
charger + fuel gauge; 13 user I/O (4 LPIO, 3× 1.8V ADC, I2C1 on SCL/SDA, SPI4); OpenMV
firmware (MicroPython) or Arduino Mbed core.

Proposed division of labor:

| Processor | Owns | Notes |
|-----------|------|-------|
| **Mega 2560 Pro** (real-time base) | Line following PID, motor control, FSM, gap/intersection/ramp/seesaw/obstacle steering, **green-marker detection (2 × APDS-9960, I2C via TCA9548A)**, button start, LoP restart, calibration | 16 MHz AVR; keep loops tight; never block |
| **Nicla Vision** (perception) | Color/vision: red goal tape, red/green evac triangles; victim detection (camera, with reflectivity/conductivity probes relayed from Mega); ToF obstacle/wall ranging; evac-zone situational heading | OpenMV MicroPython; streams small result frames to Mega (markers are NOT its job — they live on the Mega APDS pair) |

Interconnect: **I2C1 (SCL/SDA)** or UART (LPUART1 on D1/D2) with a tiny binary/magma message
protocol in both directions (Mega asks "what do you see?", Nicla answers a compact enum +
distance). Integration risks to resolve in the contract:

- **Logic levels:** Nicla I/O is 3.3 V (VDDIO_EXT), Mega is 5 V → level-shift I2C/UART or run
  3.3 V pullups on the I2C bus. Verify before wiring. The APDS-9960 marker pair + its
  TCA9548A mux are 3.3 V devices too (0x39 fixed / 0x70 mux) — same level discipline, and the
  two marker reads add I2C traffic on the shared Wire bus (bandwidth trade → the Nicla upstream
  may prefer UART to leave the I²C bus for the marker pair; pin down in the contract).
- **Boot/startup discipline:** OpenMV boots a scripting runtime; the single-button LoP restart
  (§4.2.8) must bring BOTH boards to a deterministic state (Nicla powers up and enters its
  ready state before Mega leaves STOP). A power-cut restarts the Nicla mid-run — it must
  re-ready itself from the button press without needing Mega's field knowledge.
- **Camera lighting risk (§3.11):** silver victims reflect the scene; blob/camera detection of
  silver is fragile → keep the reflectivity + **conductivity probe** as the non-vision
  classifier on the Mega/analog side; the camera handles color (markers, triangles, red goal)
  which is its strong suit.
- **Legality (§4.2.3):** Nicla Vision is a general-purpose vision board (not marketed for RCJ
  Rescue) — compliant, but team must explain it and its OpenMV program at inspection.
- **Wireless:** Wi-Fi/BLE is legal (§1.3.1, 2.4 GHz <100 mW EIRP) but the scoring run should
  use the wired link for determinism; wireless reserved for calibration/debug and SuperTeam.

## Idea Framing

- **Problem:** Complete the entire autonomous rescue mission on an unknown modular field and
  score, under hard rules constraints (autonomy, no pre-mapping, single button, 8 min).
- **Who has it:** Team Quantum 42 (junior students, RCJ international rules).
- **Evidence:** 2026 final rules PDF (provided; text extracted, 30 pp), broken current code,
  physical hardware (Mega 2560 Pro, L298N, 4 BIO motors, QTR-8A, 3D chassis).
- **Hidden assumptions (risk if wrong):**
  - Existing chassis can accept added sensors + victim mechanism — risk HIGH (mechanical space/CoG).
  - QTR-8A at 11.2 mm works for line tracking — risk HIGH (Pololu spec tops out ~6 mm; contrast
    collapses; height must be verified or lowered/shielded).
  - Mega 2560 can host all required I/O within 8KB SRAM/timer budget — risk MEDIUM.
  - A victim pick-up + deposit mechanism fits the platform's size/cost/CoG — risk HIGH.
  - Team can fabricate mechanical additions in time — risk HIGH (mechanical/electrical crew roles).

## Analogues

Verified live by Associates (URLs are the evidence; unverifiable items flagged):

| Name | What it does | Strengths | Weaknesses | License/Hosting |
|------|--------------|-----------|-----------|-----------------|
| Team Airborne (2025) | Full Rescue Line: line, terrain, obstacles, silver-tape evac entry, dual-servo rescue arm, victim pick-up | Brain/real-time split (RPi5+Pico); YOLOv8 on Coral for victims+silver; ToF+IMU stack; ships TDP/poster/video | Heavy HW cost; camera lighting fragility; no gap/dead-end/seesaw detail | MIT — github.com/JamesBond6873/Airborne_Rescue_Line_2025 |
| Team BAM | Line trace + obstacles + intersections + victim room | Uno+RPi split; encoder distance; servo-tilt camera | Dated; encoder dead-reckoning risks §4.1.3; no ramp/seesaw/gap docs | site — teambamrcj.weebly.com/hardware.html |
| RoboCore | Open-source rescue electronics base (ESP32-S3, custom PCBs, phototransistor line bar) | Modular HW; OLED+buttons UI; VL53L1X + MPU6050 infra | Electronics base only; no mission logic; immature ADC | GPL-3.0 — github.com/codemarv42/RoboCore |
| FermiLabs "Yellow Radiators" (2026) | Vision-first line+markers+rescues, web calibration dashboard | On-site calibration weapon vs 8-min clock + variable lighting | No public repo; camera dependence fragile in glare/flash | blog — blog.arduino.cc/2026/05/08/... |
| luccaSartore simulation | Camera-robot simulation harness for line-follow algorithm dev | Simulation-first workflow; used by a claimed champion team (unverified claim) | Camera only; no IR array / victim mechanics | github.com/lucaSartore/Robocup-Rescue-Line-simulation |
| lujain Ibrahim (Mega project) | Arduino Mega line follower with green-turn + obstacle avoid | Issue-credible: TCS34725+mux, TCS3200 cal, PID, ramps; directly on Mega | 2019 coursework, not hardened; no evac/victim | github.com/lujainibrahim/embedded-systems-project |

### International champions — last 3 years (strengths AND weaknesses)

| Year | Team | Verified result | Strengths | Weaknesses |
|------|------|-----------------|-----------|-----------|
| 2023 | **Team BitFlip** (Engelbert-von-Berg-Gymnasium, Wipperfürth, DE) | 1st Rescue Line, World Champs Bordeaux (verified: RCJ 2023 awards PDF) | 3 generations of public code (robocup22/23/24 repos); gyro-assist heading-hold through gaps; iterated, long-running build | Line-sensor+IMU stack, no camera evidence publicly; sparse docs of the physical robot; tuned-constant heavy to replicate |
| 2024 | **Team Overengineering²** (GBG Seelze, DE) | 1st Individual Team, World Champs Eindhoven (verified: own GitHub README + results) | Fully camera-based (Raspberry Pi); max engineering docs (Journal/TDP/Poster); years-long iteration, clean 2024 sweep | Heavy compute + camera pipeline (lighting-sensitive); team graduated/stopped in 2025; complexity is a porting tax |
| 2025 | **Team BrioBrause** ("Bio Brause", Kassel, DE) | Rescue Line 1st, Salvador — per the Don; independent confirmation pending. Verified independently: World Champion **Maze 2024** Eindhoven (beat 32 teams); announced Rescue Line entry for 2025 | Maze world-champ pedigree; fast iteration (DE/EU titles); proven autonomous-navigation fundamentals | No public code/TDP of the Rescue-Line build found; Maze→Rescue-Line switch changes sensing/mission; little hard detail to copy |

Adjacent analogues: SayanSeth (PID line+follower sonar base), pradhyuman511 (T/+ junction +
90°/180° pivots — replacement for sequential counter w/ green markers), andynuisud (line-lost
auto-recovery = gap reacquisition pattern), Ajay02423 (3-sensor "all-on ⇒ straight").
Platforms: rescue.rcj.cloud (official docs/PDFs + clarifications), junior.robocup.org Rescue
Line page (forum threads incl. Huskylens, ESP-NOW for SuperTeam), github.com/robocup-junior,
rcjrescue-iran field editor.

## Prior Art & Patterns

- **QTR-8A PID line following** — position 0–1000/7000 → error → differential steering; boot
  calibration into min/max. Pololu QTR docs (pololu.com/docs/0j19/all). *Canonical sign rule
  is folklore — confirm empirically at calibration.*
- **Gaps (≤20 cm, ≥5 cm straight before)** — hold heading/burst across, timer-based abort →
  sweep. Forum §3.3 confirmation; microline.hr/EUWeb/HTML.ashx?id=249; saegersven/robocup23.
- **Intersections/dead ends** — green 25 mm markers; side color sensors; no marker = straight;
  both = 180° turn. Canonical: RCJA "Turn on Green" (robocupjunior.org.au 2021 PDF);
  pradhyuman511 pivot mechanics; RCJA hints-and-tips (CoG point).
- **Ramps ≤25° / seesaws <20°** — motor-load detection (no current sense on L298N → use
  slowdown/encoder), creep onto seesaw, CoG over drive wheels, wait for tilt balance before
  proceeding. RCJA hints-and-tips; IITA skill repo.
- **Obstacles (≥15 cm, heavy/fixed)** — Leave line → wall/offset-follow around → reacquire the
  SAME line. NewPing non-blocking `ping_timer()` pattern (docs.arduino.cc/libraries/newping).
- **Victims** — discriminate by *physical properties*, not vision (lighting noise, §3.11):
  reflectivity probe (silver vs black vs white) + **conductivity probe** (live is electrically
  conductive; dead is not) = camera-free, mega-friendly, anti-fake-victim. TCS3200 cal notes
  (zbotic.in); RCJ forum threads #67, #5326, #3018, #5011.
- **Evacuation zone** — silver entrance tape / black exit tape (same reflectivity trick);
  wall-follow to perimeter points; colored (red/green) evacuation triangles; then resume line.
- **Architecture** — single non-blocking enum/switch FSM with millis() timers, shared sensor
  scheduler; no blocking delay. References: yash-saini-nx/pid-line-follower-robot (12-state
  FSM), Meryemsoltaniiii/Line-Follower-Robot.

## Constraint & Risk Survey

- **Mega limits:** 8KB SRAM; timer budget — Servo→Timer5, NewPing non-blocking→Timer2, motor
  PWM→Timer1/3/4, never Timer0. Allocate before wiring. (forum.arduino.cc, Servo GitHub #64)
- **QTR-8A at 11.2 mm:** above recommended ~6 mm operating range; contrast deteriorates with
  height — HIGH severity/certainty. Lower mounting or validate empirically. (pololu docs)
- **L298N:** common-ground mandatory (classic "nothing works"); 5V regulator jumper beyond
  ~12V; 4 motors parallel-paired OK on 2 channels. (omartronics, SSE)
- **Rules constraints:** autonomous-only (§4.1.1); no pre-mapping/dead-reckoning from field
  layout (§4.1.3, §5.2.5, §5.3.9 — line must be primary reference, randomization mid-run);
  single binary start button + at most one power switch (§4.2.8) with a declared LoP procedure;
  8 min = calibration + run, no code changes after run start (§5.3); source submitted (§4.4.7);
  Technical Challenge = reprogram-under-time-pressure, no HW changes (§6.2). →
  Parameterized thresholds, modular single-compile layout, cold-restart from stored calibration.
- **Environment:** variable lighting + camera flash (§3.11.4, LED light strips in evac zone),
  magnetic interference (§3.11.3), 25 cm bridges / 3 mm steps / ±10% tolerances. → skirt
  visible-color sensors (TCS3200 has no IR-cut; prefer TCS34725), keep IR line sensing for
  tracking, de='bounce IMU handling.

## Existing Test Coverage

None. No test framework, no unit tests, no simulation. `PWM Test` is a manual serial bench
harness (uses the same wrong pin constants).

## Dependencies

- Installed/used: Pololu QTRSensors (qtr.setTypeAnalog / readLineBlack).
- To add (licensed, Mega-supported): NewPing (HC-SR04), Adafruit TCS34725 (color), MPU6050
  (IMU, poll raw registers — skip DMP/FIFO), built-in Servo (mind Timer5), Wire (400 kHz),
  Adafruit_APDS9960 or SparkFun_APDS9960 (green markers) + TCA9548A mux channel-select helper.
- Mechanical: existing 3D-printable chassis (Base.stl/Robocup.f3d) + victim transport mechanism.

## Relevant Ledger Entries

### Applicable Insights
- None (no institutional memory yet — first Heist in this repo).

### Applicable Negative Constraints
- None recorded (none exist). Laundering should seed the Ledger with this build's lessons.

## Risks and Unknowns

1. Sensor ride-height (11.2 mm) vs QTR-8A spec — must be resolved empirically/modally before
   the contract's line-following milestone is credible.
2. Exact physical mounting space for the Nicla Vision, ultrasonic, IMU, and a victim mechanism
   on the existing chassis — mechanical unknowns; CoG rules for ramps/seesaws must be respected;
   the Nicla is 22.86 × 22.86 mm with a mounting footprint to preserve (camera + ToF viewports).
3. Victim handling (pick-up, classification, deposit to red/green triangle) is the hardest
   sub-problem and least covered by verified open prior art.
4. **Nicla Vision integration** (NEW): 3.3 V ↔ 5 V logic levels, boot/ready sequencing with the
   single-button LoP restart, and I2C link reliability under motor noise — all unknowns to be
   proved with a bench link-up test early (P0'——prototype the Mega⇄Nicla I²C link).
5. 2026 rules are final today but "an updated version will be released later" — feature/config
   deltas must be cheap (config-driven thresholds + FSM parameters).
6. No mechanical/electrical design references verified for the specific "BIO motor" gearboxes
   (stall current for ramp boost) and L298N max current headroom at 4 motors.
7. APDS-9960 pair specifics: fixed I2C address 0x39 → TCA9548A mux (channel-select per read
   adds latency/admin on the one Wire bus); 3.3 V logic on a 5 V Mega bus; RGB mode is passive
   → green/white contrast varies with angle/lighting (calibrate during the 10 s sweep); the
   proximity IR mode could see the QTR-8A's own IR or silver victims — keep disabled unless
   validated.

## Recommended Scope

Suggested Heist boundaries (code-first, mechanically assisted):

**P0 — Core under-runner (must land first, unblocks everything):**
1. Apply the documented LEFT/RIGHT pin remap to `Robocup.ino` + `PWM Test`; verify `150,150`
   drives straight (§7 checklist). Fix `Kp` to positive, small (0.05–0.15), `Kd` 0; fix the
   inverted line-lost predicate; keep forward-only? → confirm pivot capability needed.

**P1 — Line-tracing mission core:**
2. Motor layer refactor (bidirectional, constrain at one place, straight-line trimming).
3. Non-blocking enum FSM + millis scheduler; single-button start; LoP restart procedure
   (state persists across power cut from stored calibration).
4. Gap crossing + line reacquisition.
5. Intersection/dead-end with green markers (2 × APDS-9960 on the Mega, I2C via TCA9548A)
   → turn/straight/180°.
6. Ramp + seesaw handling (load-based, config-driven).
7. Obstacle detection (**Nicla Vision ToF** / optional HC-SR04) + navigate-around + reacquire
   same line.
8. Goal detection (red tape via **Nicla Vision**) → stop 5 s → exit bonus.

**P1' — I2C bring-up spikes (do first, unblock P1):**
Mega⇄Nicla I2C/UART link-up with 3.3 V↔5 V handling; message protocol; boot/ready sequencing.
Plus the APDS-9960 ×2 + TCA9548A mux bring-up on the same bus (channel select/deselect per
read). Calibration-time link check working before P2 depends on it.

**P2 — Evacuation zone + victims:**
9. Silver entrance / black exit tape detection (reflectivity + **Nicla Vision**) + zone FSM.
10. Victim search (**Nicla Vision** camera + reflectivity/conductivity probes on Mega) +
    ignore fakes.
11. Victim transport + deposit mechanism (servo arm) to red (dead) / green (live) triangles
    (**Nicla Vision** red/green triangle detection + heading feedback).
12. Wall/zone navigation & triangle detection (multicolor TCS34725).

**P3 — Competition-readiness:**
13. In-field fast calibration within 8-min clock; cold restart w/o re-mapping; Technical
    Challenge-ready modular config; source submission hygiene; docs delivered (HARDWARE,
    wiring additions, TDP/Poster/Video/BOM tracked as separate workstream).

**Out of scope for the code contract:** poster/video/TDP production (separate artifacts),
actual hardware purchase, physical mounting changes beyond what's needed to attach sensors
(coordinate with mechanical crew).