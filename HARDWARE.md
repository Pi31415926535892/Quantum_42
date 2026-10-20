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
TCA9548A I²C mux, bumpers, reflectivity/conductivity probes, servos, start button.
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
- **UNVERIFIED — reported 2026-10-20**: connector flipped — OUT1 (leftmost) now lands on
  A14, OUT8 (rightmost) on A0. The logical order is unchanged, so the pin table reads
  backwards. The firmware already mirrors this; the bench check is still pending (§8).

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
| Nicla link, planned | D18 (TX1) / D19 (RX1) + BSS138 shifter | **UNVERIFIED / not implemented** — see §7 |

Do not reuse D2–D5, D18–D21, D30–D31, D35 or A0/A2/A4/A6/A8/A10/A12/A14 without
updating this table and the sketch together.

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

- **Connector flipped 2026-10-20** — the table above is the current wiring. The firmware
  table was reversed to match (`A14 … A0`); logical index 0 is still the leftmost sensor.
- `LEDON` is not connected — correct for the analog QTR-8A, whose emitters run continuously.
- **Ride height ~11.2 mm** — near the top of the QTR-8A's contrast range. Black/white
  readings are closer together than ideal, so the calibration sweep is mandatory and
  the base speed may need to stay low.
- Calibration: 10 s boot sweep over **white + black + green**; `k` re-runs it. Sweeping
  only one surface produces a degenerate `min ≈ max` calibration.
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
- Thresholds are **derived at boot** from the same sweep as the line calibration; a failed
  colour calibration falls back to the hand-tuned constants and lights the red failure cue.
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
- The **link is not implemented**: the current sketch has no `Serial1` code. The planned
  wiring is Mega TX1 = D18 → Nicla RX, Mega RX1 = D19 ← Nicla TX, through a BSS138
  level shifter (see `docs/mega-nicla-protocol.md`). Do not assume the Nicla is talking
  to the Mega until that code exists and the link is bench-soaked.

## 8. Pre-Race Verification Checklist

Run these on the bench before any run. Each item names the evidence it produces.

- [ ] **Enable jumpers:** ENA and ENB are HIGH (module jumpers fitted or wired to 5 V).
      Without them `1`–`4`/`w` do nothing.
- [ ] **Wheels:** `1`/`2` spin the LEFT wheel forward/reverse; `3`/`4` the RIGHT;
      `w` drives all four forward; `s` brakes. Wrong side ⇒ stop and fix the wiring.
- [ ] **Pivots:** `l` and `r` pivot in place, and their direction matches the marker
      convention (green on the left → left turn). Record the observed direction.
- [ ] **Line bar after the flip:** use `j`. With the line under the physical leftmost
      sensor, logical index 0 must read dark; with the line under the rightmost, index 7
      must read dark. Steering must move toward the line, not away.
- [ ] **Line calibration:** `k`, sweeping the bar over white **and** black for the full
      10 s. Check the printed `MIN/MAX` per sensor: all eight ranges must be usable, not
      `min ≈ max`.
- [ ] **Markers:** `g`, one green square under each flank. `conf` must show the correct
      side, and the grey floor / black tape must not confirm.
- [ ] **Ground:** continuity between Mega GND, L298N GND, QTR GND and battery negative.
- [ ] **Nicla rail:** P0 EMI measurement with the motors running; record it in §7.
- [ ] **Record the date and result** next to the item. An unrecorded pass is not evidence.

## 9. Known Unknowns — Do Not Assume

| # | Unknown | How to close it |
|---|---------|-----------------|
| 1 | ENA/ENB actually held high | Inspect the L298N jumpers / measure EN voltage; wheels turning on `w` is the proof |
| 2 | Line bar logical-to-physical mapping after the flip | `j` readout with a line under each end (see §8) |
| 3 | Battery chemistry, capacity, loaded voltage | Read the pack label; meter it under motor load; record here |
| 4 | Mega supply path on the field | Trace the 5 V wire from L298N to the Mega; record the connector |
| 5 | Nicla rail EMI / brownout under motor switching | P0 metered run; Option B only on failure |
| 6 | Nicla link | Not implemented in the sketch; implement `Serial1` + shifter, then soak |
| 7 | APDS-9960 pull-ups / level shifting | Inspect the breakouts; scope SDA/SCL if reads are flaky |
| 8 | Marker sensor mounting height | Measure; record before quoting detection geometry |
| 9 | LED strip 5 V source and current budget | Meter the strip; do not assume the Mega rail can carry it |
| 10 | QTR contrast at 11.2 mm | Read the calibration `MIN/MAX`; if the spread is small, lower the bar |

## Change Log

| Date | Change | Evidence |
|------|--------|----------|
| 2026-10-20 | Line-bar connector flipped: pin table reversed (`A14 … A0`); legacy sketches and the in-repo design removed; this document rewritten with evidence tags | reported; bench pending |
| 2026-10-17 | Motor inputs remapped to D2–D5; ENA/ENB no longer driven by the sketch | **CODE** |
| 2026-10-18 | L298N verified: `w` drove all four wheels forward, `l`/`r` pivots matched the marker convention | **BENCH** |
