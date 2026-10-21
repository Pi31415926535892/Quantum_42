# Team Quantum 42 — Hardware Specifications

RoboCup Junior Rescue Line 2026 platform. **One robot, one configuration.**

> **Read this first — this file exists to stop pin hallucinations.**
>
> 1. **Every claim carries an evidence tag.**
>    - **CODE** — read directly from the firmware repo's `Robocup.ino.ino` / headers;
>      the constant name is given.
>    - **BENCH** — measured or observed on the physical robot, with the date.
>    - **UNVERIFIED** — believed but not measured. Treat it as a hypothesis, not a fact.
> 2. **Authority order.** The physical robot is the ground truth → this file records it
>    → the firmware constants must match it. If code and this file disagree, **stop,
>    measure, and fix one**. Never pick the convenient version. Run
>    `scripts/check-hardware.py` after any pin change; it fails loudly on drift.
> 3. **No legacy designs.** Configurations that no longer ship are not described here.
>    The removed in-repo design and bench sketches are in git history (snapshot
>    commit `3450fff`); do not treat their pins as current.
> 4. **Adding hardware?** Add its pin, its evidence tag, and a checklist item in the
>    same change. A pin with no owner is how the next person guesses.
>
> Firmware source of truth: `~/Documents/Arduino/Robocup.ino/Robocup.ino.ino`
> (and the headers beside it). This document never silently overrides the code.

## 1. Bill of Materials

| Component | Part | Qty | Notes / evidence |
|-----------|------|-----|------------------|
| Controller | Arduino Mega 2560 Pro (ATmega2560) | 1 | 16 analog inputs; 8 are used by the line bar |
| Motor driver | L298N dual H-bridge module | 1 | Ch A = RIGHT pair, Ch B = LEFT pair — **BENCH 2026-10-18** |
| Motors | BIO DC gear motor | 4 | Two per channel, paralleled; no encoder inputs in the sketch |
| Line sensor | Pololu QTR-8A analog reflectance bar | 1 | 8 channels; emitters always on; ~11.2 mm ride height — **BENCH** |
| Marker sensors | APDS-9960 RGB / proximity | 2 | Both at I²C `0x39`; LEFT on hardware I²C, RIGHT on software I²C — **CODE** |
| LED strip | WS2812B, 12 pixels | 1 | Data D35; pixels 0–7 display, 8–11 illumination — **CODE** |
| Coprocessor | Arduino Nicla Vision | 1 | Camera / ToF / IMU. **Not linked by the current sketch** — see §7 |
| Power | Battery → L298N 12 V; L298N 5 V logic rail | 1 | Battery chemistry and capacity are **not recorded** — see §9 |

**Not used by the current sketch** (do not infer pins for them from old commits):
TCA9548A I²C mux, bumpers, reflectivity/conductivity probes, servos.
If any of these return, give it a pin row here and code that reads it.

## 2. Complete Pin Map

### 2.1 Drivetrain — L298N

| Function | MCU pin | Firmware constant | Evidence |
|----------|---------|-------------------|----------|
| LEFT channel input 1 | D2 | `LEFT_IN1 = 2` | **CODE**; forward direction **BENCH 2026-10-18** |
| LEFT channel input 2 | D3 | `LEFT_IN2 = 3` | **CODE** |
| RIGHT channel input 1 | D4 | `RIGHT_IN1 = 4` | **CODE**; forward direction **BENCH 2026-10-18** |
| RIGHT channel input 2 | D5 | `RIGHT_IN2 = 5` | **CODE** |
| LEFT enable (ENB) | — | not driven | Must be held HIGH — **UNVERIFIED** |
| RIGHT enable (ENA) | — | not driven | Must be held HIGH — **UNVERIFIED** |

The sketch PWM-modulates the four **IN** lines (`analogWrite` on D2–D5). It does not
configure ENA/ENB. On most L298N modules the enable inputs are held high by the onboard
5 V-EN jumper; if those jumpers are missing, **the wheels will never turn**. Check this
before diagnosing anything else.

### 2.2 Line bar — QTR-8A

Logical index 0 is the **leftmost** sensor when looking forward; index 7 is the rightmost.

| Logical index | 0 (left) | 1 | 2 | 3 | 4 | 5 | 6 | 7 (right) |
|---------------|----------|---|---|---|---|---|---|-----------|
| MCU pin | A14 | A12 | A10 | A8 | A6 | A4 | A2 | A0 |

- **CODE**: `sensorPins[8] = { A14, A12, A10, A8, A6, A4, A2, A0 }` in `Robocup.ino.ino`.
- **BENCH 2026-10-20 — net flip = 1**: the harness is flipped, so OUT1 (leftmost) lands
  on A14 and OUT8 (rightmost) on A0. With the straight-through table the robot steered
  **away** from the line, the wheels fought and the motors whined without following.
  That symptom is the field test for this table: if it returns, check the mapping with
  `j` before changing anything else (§8).

### 2.3 Colour / marker sensors — 2 × APDS-9960

Both sensors answer at address `0x39`; they use **separate buses instead of a mux**.

| Function | MCU pin | Firmware | Evidence |
|----------|---------|----------|----------|
| LEFT APDS SDA | D20 | hardware `Wire` | **CODE** |
| LEFT APDS SCL | D21 | hardware `Wire` | **CODE** |
| RIGHT APDS SDA | D30 | `SoftWire softWire(30, 31)` | **CODE** |
| RIGHT APDS SCL | D31 | `SoftWire softWire(30, 31)` | **CODE** |

- The APDS-9960 is a **3.3 V** part. Level shifting / pull-ups: **UNVERIFIED** on the
  current harness — inspect the breakout before adding anything to the bus.
- Register setup in code: RGBC mode, `ATIME = 0xFC` (≈10 ms), `CONTROL = 0x01` (4× gain).
- There is **no TCA9548A mux** in the current build. The old mux section was removed;
  re-adding a mux means changing both the sketch and this file.

### 2.4 LED strip — WS2812B

| Function | MCU pin | Firmware | Evidence |
|----------|---------|----------|----------|
| Data | D35 | `Adafruit_NeoPixel strip(12, 35, NEO_GRB + NEO_KHZ800)` | **CODE** |
| Power | 5 V | — | Source not recorded — **UNVERIFIED** |
| Ground | common ground | — | **BENCH** (required for data) |

- 12 pixels: 0–7 are the display (PID error dot, green marks), 8–11 are always-on white
  illumination for the colour sensors.
- Brightness levels: display `40`, illumination `127` (0–255) — **CODE**.
- Keep the illuminators on during any colour/line calibration; they are part of the run
  condition, not a bench convenience.

### 2.5 Serial and reserved pins

| Function | MCU pin | Evidence |
|----------|---------|----------|
| USB serial (bench console + log dump) | USB, **115200 baud** | **CODE**: `Serial.begin(115200)` |
| StartRun button — start / pause / resume and calibration start | D41 | **CODE**: `START_BUTTON_PIN = 41`; wiring convention **UNVERIFIED** |
| Calibration mode button — skip line ↔ green dot | D39 | **CODE**: `CAL_SKIP_BUTTON_PIN = 39`; wiring convention **UNVERIFIED** |
| Calibration button — enter/exit menu, repeat last sweep | D43 | **CODE**: `CAL_BUTTON_PIN = 43`; wiring convention **UNVERIFIED** |
| Nicla link | **RX3 = D15** (Mega receive) / **TX3 = D14** (Mega transmit) + BSS138 shifter | **ASCII ToF readout is live in the main sketch**; binary protocol not implemented — see §7 |

All three buttons are momentary, wired leg-to-**GND**, read with the internal
pull-up (`INPUT_PULLUP`, pressed = **LOW**). Debounce is 25 ms. D41 starts only
on a 500 ms hold; a tap pauses or resumes instantly. D43 opens and closes the
calibration menu on a 3000 ms hold, and only from Idle, AwaitingCalibration or
Paused — it is inert during a run. Inside the menu D41 **presses** to start the
selected sweep, D39 skips between line (blue) and green-dot (green), a short D43
tap repeats the last sweep, and a 3000 ms D43 hold exits. Verify the convention
at the bench: the boot banner prints
`BTN D41=<0|1> D39=<0|1> D43=<0|1> pressed=LOW (INPUT_PULLUP)`, and a press must
read `0`.
If the leads run beside motor wiring, fit 4.7–10 kΩ to +5 V and 100 nF to GND at
each pin — the internal pull-up is weak and this is **UNVERIFIED** until measured.

Do not reuse D2–D5, D14–D15, D20–D21, D30–D31, D35, D41, D39, D43 or
A0/A2/A4/A6/A8/A10/A12/A14 without updating this table and the sketch together.
D18/D19 (Serial1) were freed on 2026-10-22 when the Nicla link moved to Serial3.

### 2.6 EEPROM Map

<!-- eeprom-cal 0 127 -->

| Region | Addresses | Holds |
|---|---|---|
| Calibration slot A | **0–63** | magic `"Q42C"`, version, flank flags, sequence, line min/max, green thresholds, CRC |
| Calibration slot B | **64–127** | the other slot; writes alternate so a torn write leaves one loadable |
| Journal | **disabled** | not written; before re-enabling, move it to 128 and read the note in `Journal.h` |

The calibration region owns 0–127. The journal is disabled for now — its code is
kept, but nothing offers, pumps or dumps it. A re-enable that forgets to move the
journal would overwrite the calibration magic, which reads as **corrupt** and
refuses to start, never as garbage thresholds.

### 2.7 LED Diagnostics

| State | Display (0–7) | Pattern |
|---|---|---|
| Boot, calibrated | all green, or all **red** if a colour flank used defaults | 3 × 120 ms |
| Idle | pixel 0 green heartbeat | 100 ms every 2 s |
| No calibration | all **amber** | missing 300/300, corrupt 120/130 |
| Start refused (no calibration) | all **red** | 300 ms |
| Calibration menu — line mode | all **blue** | static |
| Calibration menu — green-dot mode | all **green** | static |
| Calibrating line | one **blue** dot walking | continuous |
| Calibrating green dot | one **green** dot walking | continuous |
| Calibration saved | all green | 3 × 120 ms |
| Calibration usable, not saved | all **amber** | 3 × 120 ms |
| Calibration failed | all **red** | 3 × 950 ms |
| Paused | pixels 3 and 4 white | static |

Pixels 8–11 are the colour sensors' illuminators: they stay lit in **every** frame.

## 3. Drivetrain Wiring


```
   ARDUINO MEGA                         L298N                        MOTORS
   D2  ──────────────────────────────►  IN3  (Ch B)  ── OUT3/OUT4 ──►  LEFT pair
   D3  ──────────────────────────────►  IN4  (Ch B)
   D4  ──────────────────────────────►  IN1  (Ch A)  ── OUT1/OUT2 ──►  RIGHT pair
   D5  ──────────────────────────────►  IN2  (Ch A)
   ENB ── held HIGH (not from MCU) ───► ENB
   ENA ── held HIGH (not from MCU) ───► ENA
   GND ──────────────────────────────►  GND ───────────────────────────  common ground
```

- **Forward on a channel** = input 1 HIGH/PWM, input 2 LOW
  (`IN1=1/IN2=0` on Ch A, `IN3=1/IN4=0` on Ch B).
- **Brake** = both inputs HIGH (`stopMotors()` does exactly this). Both LOW is a coast —
  the robot keeps rolling, which is how junctions were previously missed.
- The L298N silkscreen and the robot's sides are **crossed**: Ch A (OUT1/OUT2) drives
  the physical RIGHT wheels, Ch B (OUT3/OUT4) the physical LEFT. The firmware names
  follow the robot, not the silkscreen. Do not "fix" this by renaming.
- Bench keys: `1`/`2` = left wheel forward/reverse, `3`/`4` = right wheel forward/reverse,
  `w` = both forward, `s` = stop.

## 4. Line Bar — QTR-8A Wiring

```
   ARDUINO MEGA                         QTR-8A
   A0  ──────────────────────────────►  OUT8 (rightmost)
   A2  ──────────────────────────────►  OUT7
   A4  ──────────────────────────────►  OUT6
   A6  ──────────────────────────────►  OUT5
   A8  ──────────────────────────────►  OUT4
   A10 ──────────────────────────────►  OUT3
   A12 ──────────────────────────────►  OUT2
   A14 ──────────────────────────────►  OUT1 (leftmost)
   5 V ──────────────────────────────►  VCC
   GND ──────────────────────────────►  GND
                                        LEDON: NOT CONNECTED
```

- **Flipped (net flip = 1, BENCH 2026-10-20)**: OUT1 (leftmost) → A14 … OUT8 (rightmost)
  → A0. The firmware table (`A14 … A0`) matches this. The straight-through table made the
  robot steer away from the line and the motors whine; do not restore it without a bench
  check.
- `LEDON` is not connected — correct for the analog QTR-8A, whose emitters run continuously.
- **Ride height ~11.2 mm** — near the top of the QTR-8A's contrast range. Black/white
  readings are closer together than ideal, so the calibration sweep is mandatory and
  the base speed may need to stay low.
- Calibration is split into two selectable sweeps: the **line** sweep (10 s over white +
  black, blue LED) and the **green-dot** sweep (10 s over green + white + black, green LED).
  The line sweep must run first because the persisted blob needs its references; the green
  sweep refreshes only the colour thresholds. `k` re-runs the line sweep. Sweeping only one
  surface produces a degenerate `min ≈ max` calibration.
- The sketch reads the bar with plain `analogRead`; there is **no QTRSensors library** in
  this firmware.

## 5. LED Strip

- 12 × WS2812B on **D35**, GRB order, 800 kHz.
- Pixels 0–7: PID error dot (white), green marks at the outer pixels when a marker is seen,
  whole display green during a marker turn, red failure cue.
- Pixels 8–11: always-on white illumination over the colour sensors.
- Power source and current budget are **not recorded**. Do not assume the Mega's 5 V
  regulator can carry the strip; meter it before changing brightness or pixel count.

## 6. Marker Detection Notes

- Rule geometry: 25 mm green squares beside the line. Green on one side arms a turn that
  way; green on both sides means dead end / U-turn; none means straight.
- The illuminators (pixels 8–11) light the colour sensors, so they are part of the
  optical path — do not switch them off to "help" detection.
- Thresholds are **derived on demand** by the green-dot mode of the calibration menu; a failed
  green-dot sweep keeps the previous thresholds (or falls back to the hand-tuned constants)
  and lights the red failure cue.
- Bench readout: `g` prints `GRN L=… R=… conf LR`; `c` clears the held peak samples.
- Mounting height/geometry is **not recorded** — do not quote a number until it is measured.

## 7. Power, Ground, and Nicla

- **Motor supply:** battery → L298N `12 V` terminal (drives the four BIO motors). The
  L298N accepts 5–35 V; **the pack's actual chemistry/voltage is unrecorded** — see §9.
- **Logic supply:** L298N `5 V` output → QTR-8A VCC and the logic rail. On the bench the
  Mega is powered over USB instead; on the field the L298N rail powers it. **BENCH.**
- **Common ground** across Mega, L298N, QTR-8A, APDS-9960s, LED strip and battery is
  mandatory. A missing ground shows up as buzzing motors or a dead bus, not a clean failure.
- **Battery chemistry, capacity and loaded voltage are not recorded** — measure and write
  them here before quoting runtime or tuning pivots.

### Nicla Vision power — Option A (PROVISIONAL, UNVERIFIED)

**Decision (Don, 2026-10-01): Option A is locked provisionally, pending the P0 EMI
measurement (motors running → Nicla rail).**

- **Option A (current):** Nicla powered from the shared battery through a dedicated
  **ferrite + LC filtered** branch into its 3.3 V LDO, with common ground.
- **Option B (auto-promotes only if P0 EMI fails):** dedicated Li-Po feeding the Nicla
  directly; the Nicla then survives a main-rail cut with stale state.
- 2.4 GHz radios are compile-time disabled for the match window.
- ⚠️ **UNVERIFIED** until the P0 metered run. Record the measurement and the decision here.
- **Link status:** the main sketch services **Serial3** on every loop tick. It reads the
  Nicla's **ASCII** ToF fields (`{dist}` padded to four characters, comma separated,
  115200 8N1) through `NiclaToF.h` and prints `TOF <mm>` at 5 Hz. Physical wiring: Mega
  **RX3 = D15** ← Nicla TX, Mega **TX3 = D14** → Nicla RX, through a BSS138 level shifter
  (see `docs/mega-nicla-protocol.md`). The ratified **binary protocol is not implemented
  in the main sketch** yet; the ASCII path is the seam where it will land. The Mega → Nicla
  direction is wired but unused until then.

## 8. Pre-Race Verification Checklist

Run these on the bench before any run. Each item names the evidence it produces.

- [ ] **Enable jumpers:** ENA and ENB are HIGH (module jumpers fitted or wired to 5 V).
      Without them `1`–`4`/`w` do nothing.
- [ ] **Wheels:** `1`/`2` spin the LEFT wheel forward/reverse; `3`/`4` the RIGHT;
      `w` drives all four forward; `s` brakes. Wrong side ⇒ stop and fix the wiring.
- [ ] **Pivots:** `l` and `r` pivot in place, and their direction matches the marker
      convention (green on the left → left turn). Record the observed direction.
- [ ] **Live readout:** serial streams the eight QTR channels (`raw` + `blk`) and
      `pos` at **20 Hz** by default; every **fifth** line also carries both APDS flows
      (`LA`/`RA` = r/g/b/clear + green verdict). The motors keep running. Send `s` first
      for a static read, or `g`/`j` for the dedicated stopped readouts.
- [ ] **Line bar mapping:** use `j`, or the `raw`/`blk` columns in the default QTR
      stream. With the line under the physical leftmost sensor, logical index 0 must read
      dark; with the line under the rightmost, index 7 must read dark. Steering must move
      toward the line, not away.
- [ ] **Line calibration:** hold D43 3 s to open the menu (blue), then press D41. Sweep the
      bar over white **and** black for the full 10 s. Check the printed `MIN/MAX` per sensor:
      all eight ranges must be usable, not `min ≈ max`. Hold D43 3 s again to leave.
- [ ] **Green-dot calibration:** from the menu, press D39 (display turns green), then D41.
      Sweep both flanks over the green squares plus white and black for 10 s. The printed
      `CAL L/R … OK` lines and the green result cue confirm it.
- [ ] **Button control:** a short D43 tap in the menu repeats the last sweep; D39 changes the
      mode without leaving; D41 press starts; D43 hold leaves to Idle (or AwaitingCalibration
      if no usable blob exists).
- [ ] **Markers:** `g`, or the `LA`/`RA` fields on every fifth QTR line, one green square
      under each flank. `conf` must show the correct side, and the grey floor / black tape
      must not confirm.
- [ ] **Ground:** continuity between Mega GND, L298N GND, QTR GND and battery negative.
- [ ] **Nicla rail:** P0 EMI measurement with the motors running; record it in §7.
- [ ] **Record the date and result** next to the item. An unrecorded pass is not evidence.

## 9. Known Unknowns — Do Not Assume

| # | Unknown | How to close it |
|---|---------|-----------------|
| 1 | ENA/ENB actually held high | Inspect the L298N jumpers / measure EN voltage; wheels turning on `w` is the proof |
| 2 | Line bar logical-to-physical mapping | `j` readout with a line under each end (see §8) |
| 3 | Battery chemistry, capacity, loaded voltage | Read the pack label; meter it under motor load; record here |
| 4 | Mega supply path on the field | Trace the 5 V wire from L298N to the Mega; record the connector |
| 5 | Nicla rail EMI / brownout under motor switching | P0 metered run; Option B only on failure |
| 6 | Nicla link | ASCII ToF readout is live on Serial3 RX3 = D15; binary protocol not implemented. Implement the protocol, then soak |
| 7 | APDS-9960 pull-ups / level shifting | Inspect the breakouts; scope SDA/SCL if reads are flaky |
| 8 | Marker sensor mounting height | Measure; record before quoting detection geometry |
| 9 | LED strip 5 V source and current budget | Meter the strip; do not assume the Mega rail can carry it |
| 10 | QTR contrast at 11.2 mm | Read the calibration `MIN/MAX`; if the spread is small, lower the bar |

## Change Log

| Date | Change | Evidence |
|------|--------|----------|
| 2026-10-21 | Calibration is now a button menu (D43 3 s): line (blue) / green-dot (green), D39 skips, D41 starts, short D43 repeats, long D43 exits. D39 added | **CODE** + host tests |
| 2026-10-20 | Line bar **flipped (net flip = 1)**: table `A14 … A0`. The straight-through table caused inverted steering, fighting wheels and motor whine | **BENCH** |
| 2026-10-20 | Legacy sketches and the in-repo design removed; this document rebuilt with evidence tags | code + git history |
| 2026-10-17 | Motor inputs remapped to D2–D5; ENA/ENB no longer driven by the sketch | **CODE** |
| 2026-10-18 | L298N verified: `w` drove all four wheels forward, `l`/`r` pivots matched the marker convention | **BENCH** |
