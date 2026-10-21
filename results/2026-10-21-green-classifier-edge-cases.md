---
date: 2026-10-21
subject: green classifier edge-case probe
status: analysis only — no fixes applied
heist: button-controls
---

# Green Classifier — Edge-Case Findings

Probe compiled against the real `GreenMarker.h` / `ColourCalibration.h` and the
default (hand-tuned) thresholds. Simulators:
`$TMP/green_edge_sim.cpp`, `$TMP/green_edge_sim2.cpp` (host `g++`, read-only).

Current rule, restated:

- `c < minClear` → TooDark
- `g < minLevel (20)` → not green
- `g < b` → not green
- `g <= r` → not green
- `(g-r)*100 >= redPct(40) * g` → base gate
- green if `g >= 40` and `g-b >= blueMargin(4)` (blue edge), or
  `(g-r)*100 >= redStrongPct(61) * g` (strong-red bar)

## Findings, highest severity first

### F1 — HIGH: one sample decides, and the poll rate amplifies every false positive

`pollMarkers()` opens a settle on a single green reading, 33 times a second.
Probability of at least one false stop in a 1 s window:

| per-sample false rate | P(false stop in 1 s) |
|---|---|
| 0.1 % | 3.2 % |
| 1 % | 28 % |
| 5 % | 82 % |
| 10 % | 97 % |

Over a two-minute run a 1 % per-sample rate is effectively certain to stop the
robot. This is the mechanism behind the observed "going crazy", not a single bad
threshold.

### F2 — HIGH: a marker must cover ~90 % of the flank to be seen at all

Mixing a real marker with the grey floor:

| marker share | bright marker 26/84/78 | dim marker 12/33/32 |
|---|---|---|
| 50 % | not green | not green |
| 70 % | not green | not green |
| 80 % | not green | not green |
| 90 % | **GREEN** | not green |
| 100 % | **GREEN** | **GREEN** |

A marker straddling the centre (neither flank covered) or sitting half under one
flank is invisible. There is no positional or shape check to fall back on.

### F3 — HIGH: chartreuse / yellow-green is green, and survives the settle

| surface (g=80) | deficit | verdict |
|---|---|---|
| chartreuse `44/80/48` | 45 % | **GREEN** |
| yellow-green `52/80/48` | 35 % | not green |
| yellow `66/80/40` | 17 % | not green |
| olive `60/80/40` | 25 % | not green |

Because 45 % clears the 40 % base gate and `g-b = 32` clears the blue edge, a
static chartreuse object is green on **every** sample. The settle's average then
also passes, so it does not just cost a stop — it can **arm a wrong turn** at the
next junction. This is the only finding that can change the robot's direction
rather than interrupt it.

### F4 — MEDIUM: a red chassis reflection masks a genuine marker

Real marker `26/84/78` plus an additive red bounce:

| added red | reading | verdict |
|---|---|---|
| +0 | 26/84/78 | GREEN |
| +20 | 46/86/80 | GREEN |
| +30 | 56/87/81 | **not green** |
| +50 | 76/89/83 | not green |

A red-painted chassis, a red object or a strong red reflection can push a real
marker below the gate: a **missed** turn, not a false one.

### F5 — MEDIUM: a weak sweep can loosen the base gate

The strong-red bar is floored at 61, but `redPct` is derived as
`0.6 × greenPct` and has no floor.

| thresholds | reading `57/80/76` (28 % deficit) | verdict |
|---|---|---|
| defaults (redPct 40) | | not green |
| weak sweep (redPct 30) | | **GREEN** |
| very weak sweep (redPct 25) | | **GREEN** |

So a sweep whose green reference reads 40–50 % deficit derives a base gate low
enough to admit olive/chartreuse surfaces. The earlier clamp fixed only half the
problem.

### F6 — MEDIUM: the decision flips with brightness at `g = 40`

Same colour ratios (`r = 0.50 g`, `b = 0.90 g`) at different brightness:

| g | verdict |
|---|---|
| 34 | not green |
| 38 | not green |
| 40 | **GREEN** |
| 46 | **GREEN** |

The absolute 4-count blue margin plus the 40-count brightness floor make the
verdict depend on distance, coverage and illumination even when the surface did
not change. A marker dimming under the robot can cross this boundary and flicker.

### F7 — MEDIUM: a blue-channel bias of 8 counts kills detection

Real marker `26/84/78` with blue nudged up:

| blue | verdict |
|---|---|
| 78–84 (+0…+6) | GREEN |
| 86 (+8) | **not green** |
| 90 (+12) | not green |

`g < b` is a hard reject. Any blue-ish cast, blue reflection or channel offset of
8 counts loses the marker entirely. A real teal/blue-green marker (`20/80/82`,
75 % deficit) can never be green.

### F8 — LOW: dim genuine markers flap, though they are eventually seen

True marker ratios under ±4 uniform noise:

| green level | samples green |
|---|---|
| 20 | 41 % |
| 30 | 67 % |
| 40 | 75 % |
| 60 | 92 % |
| 80 | 96 % |

Near `minLevel` the classifier is coin-flip; at normal brightness it is reliable.

### F9 — LOW: the clear floor can reject a partially covered marker

With a derived `minClear = 71`, a marker `20/80/78` is **TOO DARK** at c = 40–70
and green only at c ≥ 71. A partially covered or geometrically occluded marker
shrinks the clear channel and is rejected before colour is considered.

### F10 — INFO: invariants that hold

- Adding red never creates green (0 violations over 0…200 counts).
- Adding blue never creates green (0 violations).
- Default thresholds reproduce `looksGreen()` exactly (0 disagreements over the
  full `0…150` cube).
- Black tape `18/39/46`, grey floor `97/124/134`, white line and warm grey are
  all rejected; ±4 noise does not lift them over the bar.

## Bug list (analysis only)

| # | Finding | Class | Can it change direction? |
|---|---|---|---|
| 1 | Single-sample trigger × 33 Hz | design/interaction | no, but frequent stops |
| 2 | ≥90 % marker coverage needed | sensitivity | no, missed turns |
| 3 | Chartreuse at ≥40 % deficit | gate hole | **yes — wrong turn** |
| 4 | Red bounce masks real marker | colour-only limit | no, missed turn |
| 5 | `redPct` not floored after a weak sweep | gate hole | **yes — wrong turn** | **fixed** `b9ffa54` |
| 6 | Brightness discontinuity at g=40 | design | no, flapping |
| 7 | Blue bias ≥8 counts | hard gate | no, missed marker |
| 8 | Dim marker flapping | noise | no |
| 9 | `minClear` vs partial coverage | calibration | no, missed marker |

## What was not tested

- Real APDS-9960 channel responses (the probe treats RGBC as independent).
- Specular highlights on a glossy marker.
- Two markers under one flank at once.
- The 400 ms ignore window racing a second marker at speed.
