# Team Quantum 42 — Hardware Specifications

RoboCup Junior line-follower-with-rescue platform.

## 1. Bill of Materials

| Component | Part | Qty | Notes |
|-----------|------|-----|-------|
| Controller | Arduino Mega 2560 Pro (ATmega2560) | 1 | 8+ analog inputs required (A0–A14) |
| Motor driver | L298N dual H-bridge | 1 | Channels A and B; 2 motors per channel |
| Motors | BIO DC gear motors (4 total) | 4 | **Left pair on Ch B**, right pair on Ch A |
| Line sensor | Pololu QTR-8A (analog reflectance, 8 channels) | 1 | IR LED emitters always on in analog mode |
| Marker sensors | APDS-9960 reflective color/proximity (I2C) | 2 | Detect 25 mm green intersection markers; **fixed I2C addr 0x39** → I2C mux required (§9) |
| I2C mux | TCA9548A 8-channel | 1 | Gives each APDS-9960 its own bus channel (addr 0x70 default) |
| Power | See §5 | — | Logic supply + motor supply + common ground |

## 2. Microcontroller ↔ L298N Wiring (Drivetrain)

Wired by hand (ground truth — matches `Robocup.ino` / `PWM Test` pin constants only partially, see §6):

| MCU pin | L298N terminal | Channel | Physical role | Code constant (current) | PWM? |
|---------|----------------|---------|---------------|-------------------------|------|
| D7 | ENA | Ch A | RIGHT enable | `LEFT_PWM` = 7 ⚠ | Yes |
| D6 | ENB | Ch B | LEFT enable | `RIGHT_PWM` = 6 ⚠ | Yes |
| D3 | IN1 | Ch A | RIGHT input 1 | `LEFT_IN1` = 3 ⚠ | No |
| D5 | IN2 | Ch A | RIGHT input 2 | `LEFT_IN2` = 5 ⚠ | No |
| D4 | IN3 | Ch B | LEFT input 1 | `RIGHT_IN2` = 4 ⚠ | No |
| D2 | IN4 | Ch B | LEFT input 2 | `RIGHT_IN1` = 2 ⚠ | No |

⚠ = code constant name does NOT match the physical wheel it actually drives
(see §6).

Motor grouping:
- **LEFT drive** → L298N channel **B** outputs (OUT3/OUT4), 2 BIO motors wired in parallel.
- **RIGHT drive** → L298N channel **A** outputs (OUT1/OUT2), 2 BIO motors wired in parallel.

L298N forward convention: channel is forward when input-1 = HIGH and input-2 = LOW
(`IN1=1/IN2=0` on Ch A, `IN3=1/IN4=0` on Ch B).

## 3. System Wiring Diagram (ASCII)

Overview:

```
  ┌─────────────┐   ┌─────────────┐   ┌──────────────────┐
  │   ARDUINO   │   │    L298N    │   │  BIO MOTOR PAIR  │
  │ MEGA 2560   │◄─►│             │──►│  Ch A / RIGHT    │
  │     PRO     │   │             │   ├──────────────────┤
  │             │   │             │──►│  Ch B / LEFT     │
  │             │   └─────────────┘   └──────────────────┘
  │             │   ┌─────────────┐
  │             │◄─►│   QTR-8A    │   IR reflectance, ~11.2 mm above floor
  └─────────────┘   └─────────────┘
```

Drivetrain — Mega ⇄ L298N ⇄ motors:

```
  ┌─────────────────────────┐       ┌──────────────────────────────┐
  │      ARDUINO MEGA       │       │            L298N             │
  │          PRO            │       │                              │
  │  D7 (PWM) ────────────►├─ ENA ─►│  Ch A (physical RIGHT):      │
  │  D3      ────────────►├─ IN1 ─►│  ENA │ IN1 │ IN2             │
  │  D5      ────────────►├─ IN2 ─►│  OUT1 ──► RIGHT BIO x2       │
  │                         │       │  OUT2 ──► RIGHT BIO x2       │
  │  D6 (PWM) ────────────►├─ ENB ─►│                              │
  │  D4      ────────────►├─ IN3 ─►│  Ch B (physical LEFT):        │
  │  D2      ────────────►├─ IN4 ─►│  ENB │ IN3 │ IN4              │
  │                         │       │  OUT3 ──► LEFT BIO x2        │
  │                         │       │  OUT4 ──► LEFT BIO x2        │
  │  GND ────────────────►├─ GND ─►│  GND                          │
  └─────────────────────────┘       └──────────────────────────────┘
```

Sensor — Mega ⇄ QTR-8A:

```
  ┌─────────────────────────┐       ┌──────────────────────────────┐
  │      ARDUINO MEGA       │       │           QTR-8A             │
  │          PRO            │       │  (IR emitters always on)     │
  │  A0  ──────────────────►├──────►│ OUT8 (rightmost)             │
  │  A2  ──────────────────►├──────►│ OUT7                         │
  │  A4  ──────────────────►├──────►│ OUT6                         │
  │  A6  ──────────────────►├──────►│ OUT5                         │
  │  A8  ──────────────────►├──────►│ OUT4                         │
  │  A10 ──────────────────►├──────►│ OUT3                         │
  │  A12 ──────────────────►├──────►│ OUT2                         │
  │  A14 ──────────────────►├──────►│ OUT1 (leftmost)              │
  │  5V  ──────────────────►├──────►│ VCC                          │
  │  GND ──────────────────►├──────►│ GND                          │
  └─────────────────────────┘       │ LEDON: NOT CONNECTED         │
                                    └──────────────────────────────┘
```

Power:

```
  Battery 7–12 V ──► L298N 12 V pin (motor supply, drives BIO motors)
  5 V logic supply ──► L298N 5 V pin ──► QTR-8A VCC
  GND: Mega + L298N + QTR-8A + battery ── ALL TIED TOGETHER.
       (Missing common ground = motors buzz or nothing moves.)
```

## 4. Microcontroller ↔ QTR-8A Wiring (Sensor)

**The bar's connector was flipped on 2026-10-20** (Don, bench rewiring): OUT1
(leftmost) now lands on A14 and OUT8 (rightmost) on A0. The logical order is
unchanged — index 0 is still the leftmost sensor — so the firmware's pin table
reads the flattened analog row backwards.

| QTR-8A output | Position | MCU pin |
|---------------|----------|---------|
| OUT1 (leftmost) | weight 0 | A14 |
| OUT2 | | A12 |
| OUT3 | | A10 |
| OUT4 | | A8 |
| OUT5 | | A6 |
| OUT6 | | A4 |
| OUT7 | | A2 |
| OUT8 (rightmost) | weight 7000 | A0 |

- `LEDON`: **not connected** — correct. The QTR-8A analog variant runs its IR emitters continuously; there is no emitter control pin to drive.
- `VCC`: connect to 5 V (logic). `GND`: common ground with controller and driver.
- **Sensing height: ~11.2 mm above the floor.** Working but near the upper end of the
  QTR-8A's contrast range — black/white readings get closer together, so the calibration
  sweep over both colors (§7) is essential, and expect to tune `BASE_SPEED` lower.

Configuration in code (`Robocup.ino.ino`, logical order left → right):

```cpp
const int sensorPins[8] = {
  A14, A12, A10, A8,
  A6, A4, A2, A0
};
```

## 5. Power & Ground

- L298N logic supply (VS logic / 5 V rail) powers the driver logic and MUST share a common **GND** with the Mega.
- Motor supply (VS, typically 7–12 V) powers the BIO motors through the L298N.
- Common ground between: Mega, L298N, QTR-8A, and the motor supply. Missing ground = motors buzzing/slow or nothing moving.

### Nicla power — Option A (PROVISIONAL, UNVERIFIED)

**Decision (Don, 2026-10-01): Option A is locked provisionally, pending the P0 EMI
measurement (motors running → Nicla rail).**

- **Option A (chosen now):** Nicla Vision powered from the shared battery through a dedicated
  **ferrite + LC filtered** branch into its 3.3 V LDO. Must share common ground with the rest of
  the system. This is the contract FR-023 primary design.
- **Option B (auto-promotes ONLY if P0 EMI fails):** dedicated Li-Po feeding the Nicla directly.
  Under Option B the Nicla survives a main-rail cut with stale state and is a distinct,
  documented restart path.
- 2.4 GHz radios compile-time disabled for the match window.
- ⚠ **UNVERIFIED until the P0 metered run (motors running → Nicla rail).** If the run shows the
  filtered shared rail failing the Nicla's supply/brownout tolerance, flip to Option B and record
  the measurement + decision here.

## 6. Known Wiring Discrepancy — LEFT/RIGHT constants swapped (HIGH PRIORITY)

Physical fact: **the LEFT motor pair is wired to channel B, the RIGHT pair to channel A.**

Code constants, however, assign every `LEFT*` constant to Ch A pins and every `RIGHT*`
constant to Ch B pins:

| Code constant (current) | Pin | Actual terminal | Actually drives |
|-------------------------|-----|-----------------|-----------------|
| `LEFT_PWM` = 7          | D7  | ENA (Ch A)      | RIGHT wheels ⚠ |
| `LEFT_IN1` = 3          | D3  | IN1 (Ch A)      | RIGHT wheels ⚠ |
| `LEFT_IN2` = 5          | D5  | IN2 (Ch A)      | RIGHT wheels ⚠ |
| `RIGHT_PWM` = 6         | D6  | ENB (Ch B)      | LEFT wheels ⚠ |
| `RIGHT_IN2` = 4         | D4  | IN3 (Ch B)      | LEFT wheels ⚠ |
| `RIGHT_IN1` = 2         | D2  | IN4 (Ch B)      | LEFT wheels ⚠ |

Two consequences:

1. **PWM channels are swapped.** In `setMotor()`, `analogWrite(LEFT_PWM…)` sends `leftSpeed`
   to Ch A = the physical RIGHT wheels, and `analogWrite(RIGHT_PWM…)` sends `rightSpeed`
   to Ch B = the physical LEFT wheels. The two channels drive crosswise.
2. **Ch B (left pair) is driven in reverse.** L298N forward on Ch B is `IN3=1/IN4=0`.
   The code drives `IN4=D2 HIGH` and `IN3=D4 LOW` → `IN3=0/IN4=1` = reverse. So the left
   pair spins BACKWARD. Combined with (1), an equal `150,150` command spins the robot in
   place instead of driving forward.

Fix (code, recommended) — swap the constants in both `Robocup.ino` and `PWM Test` so names
match the physical wheels and each channel is driven forward:

```cpp
const uint8_t LEFT_IN1  = 4;   // IN3  (Ch B input 1) — physical LEFT
const uint8_t LEFT_IN2  = 2;   // IN4  (Ch B input 2)
const uint8_t LEFT_PWM  = 6;   // ENB  (Ch B enable)

const uint8_t RIGHT_IN1 = 3;   // IN1  (Ch A input 1) — physical RIGHT
const uint8_t RIGHT_IN2 = 5;   // IN2  (Ch A input 2)
const uint8_t RIGHT_PWM = 7;   // ENA  (Ch A enable)
```

After re-mapping, `setMotor()` drives `LEFT_IN1=4 HIGH / LEFT_IN2=2 LOW`
(`IN3=1/IN4=0` = Ch B forward) and `RIGHT_IN1=3 HIGH / RIGHT_IN2=5 LOW`
(`IN1=1/IN2=0` = Ch A forward).

Confirm polarity with §7 before running line following. If one side still spins
backward after the re-map, swap that channel's two IN constants (or its two motor leads).

Note: with the constants fixed to their physical channels, the standard convention applies
again — line on the right (`error > 0`) must make the LEFT wheel faster, which requires a
**positive** `Kp`. Current `Kp = -10` is the wrong sign for the corrected wiring.

## 7. Pre-Race Verification Checklist

Run the `PWM Test` sketch over Serial at 115200 baud.

- [ ] `150,150` — robot drives **straight forward**. If it turns/spins → §6 not fully applied (channels crossed and/or one side reversed).
- [ ] `150,-150` — robot pivots **in place**, and confirm which way: left-wheel forward = clockwise.
- [ ] `-150,150` — pivot opposite direction.
- [ ] `150,50` — robot arcs; confirm the arc direction matches which wheel has the higher speed.
- [ ] Record the "wheel A faster ⇒ physical turn direction" result; it determines the
      correct `Kp` sign in `Robocup.ino`.
- [ ] During calibration, physically sweep the sensors over **both black and white** for
      the full 10 s window.
- [ ] Confirm the track is a **black line on white** (`readLineBlack` is correct).

## 8. Tuning Reference (as shipped)

| Constant | Value | Note |
|----------|-------|------|
| `POSITION_CENTER` | 3500 | Center of 0–7000 sensor range |
| `BASE_SPEED` | 130 | Start tuning lower (~80) |
| `MIN_SPEED` / `MAX_SPEED` | 45 / 200 | PWM bounds |
| `MAX_CORRECTION` | 65 | Steering cap |
| `Kp` | -10 | Wrong sign for corrected wiring (must be positive) + magnitude ~500× too large (§6/§7) |
| `Ki` | 0.0 | Disabled |
| `Kd` | -0.015 | Keep at 0 until P-term tracks |
| `LINE_THRESHOLD` | 150 | Line-lost detection (calibrated 0–1000 scale) |

## 9. Green-Marker Sensors — APDS-9960 × 2 (I2C via TCA9548A)

Detect the **25 mm green intersection/dead-end markers** that sit beside the black line
(2026 rules): green on one side → turn that way, green on both sides → dead end (U-turn),
none → straight. Two downward-facing APDS-9960 units (one each side of the line) on the Mega's
I2C bus through a TCA9548A mux — each APDS-9960 answers only at address `0x39`, so two cannot
share one bus directly.

| Item | Qty | Notes |
|------|-----|-------|
| APDS-9960 breakout (Adafruit or SparkFun) | 2 | RGB + proximity + gesture; 3.3 V logic |
| TCA9548A mux | 1 | default addr `0x70`; **CH0 = LEFT sensor, CH1 = RIGHT sensor** |

Wiring:

```
  ┌─────────────────────────┐   ┌──────────────────────────────────────────────┐
  │      ARDUINO MEGA       │   │                I2C — 3.3 V logic               │
  │          PRO            │   │                                                │
  │  SDA ──── D20 ──────────┼──►├── TCA9548A (mux, addr 0x70)                   │
  │  SCL ──── D21 ──────────┼──►├──   ├─ CH0 ──► APDS-9960 #1  = LEFT side       │
  │                          │   ├──   ├─ CH1 ──► APDS-9960 #2  = RIGHT side     │
  │  3.3V ──────────────────►├──►├── VCC (3.3 V)                                  │
  │  GND ───────────────────►├──►├── GND                                          │
  └─────────────────────────┘   └──────────────────────────────────────────────┘
```

- The APDS-9960s and the mux run at **3.3 V**. I2C is open-drain: pull SDA/SCL to 3.3 V (Mega
  HIGH threshold ≈ 3.0 V, so 3.3 V highs register; if flaky, insert a BSS138 bidirectional
  level shifter on SDA/SCL).
- Read pattern: select the mux channel → read that sensor → deselect (`0x00`). One sensor at a
  time; the `tcaselect()` helper (Adafruit mux guide) is the template.
- Mount ~1–2 cm off the floor, just beside the line where a 25 mm marker would sit while the
  QTR-8A tracks the line. RGB mode is passive → calibrate the green/white threshold during the
  10 s sweep; keep the proximity IR LED disabled unless validated (it can see the QTR's own IR).
- Code constants (sketch): `TCA_ADDR 0x70`, `MARKER_LEFT_CH = 0`, `MARKER_RIGHT_CH = 1`.
- Ground: all tied together per §5.