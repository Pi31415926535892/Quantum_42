# Field Mock-Up Guide & Rehearsal Protocol

Bench document for the Quantum 42 Rescue Line robot: everything needed to build a cheap surrogate
arena in an afternoon, run the per-element test matrix against it, and rehearse the timed pre-run
and full-mission loop.

**Read this first.** The mock is not a simulator and it is not the venue. It answers three
questions only: does each reflex sub-state fire on a physical trigger and recover; can the
pre-run sequence finish inside the 120 s gate; can the full mission finish inside the 8-minute
clock with zero unplanned touches. If a prop is easier to build than the real thing but removes
the *difficulty* under test, it is a bad prop — correctness of proportion beats visual fidelity
everywhere.

**Rules-vs-mock rule:** numbers in this document are mock-controlled and may be tuned. Numbers
that belong to the rules — line width, marker size, gap limits, mission clock — are quoted from
the rules and must not be "improved" on the mock.

## 0. Materials and budget

| Item | Qty | Used for |
|------|-----|----------|
| Poster board / foam-core sheets (A2 or A1) | 3 | floor, walls, target board |
| Black electrical tape, 19 mm (¾ in) | 3 rolls | line, exit tape |
| Green tape or green paper, cut to 25 mm dots | ~10 | junction + dead-end markers |
| Red tape / red index card | 1 | goal patch, red triangle |
| Silver tape **or** aluminium foil tape | 2 rolls | silver victims, evac entry tape |
| Matte black tape (non-shiny) | 1 roll | dead victim shells, fake overcoat |
| Clear packing tape + hot glue gun | 1 | dielectric overcoat, off-centre ballast |
| Softwood plank ~1.2 m × 15 cm × 2 cm | 2 | ramp, seesaw beam |
| PVC pipe 20 mm × 40 mm (or a dowel) | 1 | seesaw pivot |
| Cereal box / foam block (~20 cm cube) | 2 | obstacle stand-in, ramp prop |
| Orange traffic cone (or a cardboard cone) | 1 | angled obstacle stand-in |
| Ping-pong balls | 6 | 3 victims + 2 fakes + 1 spare |
| M6 bolt + 2 washers, or a 10 g coin | 2 | off-centre CoM ballast |
| Cable ties | 10 | tape to planks, wall joints |
| Marker pen; protractor or phone inclinometer | 1 | geometry verification |

Total ≈ $10–20 (≈ ₹800–1500) — a fraction of one sensor's cost. Where an item is hard to get,
the substitute is named inline in the prop note that needs it.

---

## 1. Arena layout

**Overall floor extent, element spacing and arena proportions are per the rules floor plan.** No
arena dimensions are recorded in this repository, so they are deliberately *not* restated here.
Build on three A2 sheets taped edge-to-edge (≈ 1.2 m × 0.9 m of usable floor), enough for one
full loop plus the evac rectangle. Scale up by adding sheets — the line and element sizes below
are the only things that must not scale.

```
  LEGEND  ▓ black tape (line)   ● green 25 mm marker   ░ silver tape
          ▲ red/green triangle  ○ victim   ┄ gap       = wall

  COURSE SPINE (A) — one pass top to bottom          EVAC BLOCK (B) — beside A
  ┌──────────────────────────┐                       ┌─────────────────────┐
  │ A1  START                │                       │ B1 ▲ E7 goal patch  │
  │ A2  ▓▓▓▓▓▓ ▓▓▓▓▓▓ ▓▓▓     │                       │      5 s hold       │
  │ A3  ▓▓▓┄┄┄┄┄▄┄▄┄▄▄▓▓▓     │                       │ B2 ▲ cone  ▲ box    │
  │ A4  ●─╗  ●─╝  ╔═●═╗      │                       │    ╔═══════════╗  ▓ │
  │     ▓▓ ║ ▓▓▓▓ ▓║ ▓▓▓      │                       │    ║ ░ entry ░ ║  ▓ │
  │ A5  ▓▓▓ ║ ▓▓┄┄┄┄▄▄▄ ║ ▓    │                       │    ║ ○ ○ ○     ║  ▓ │
  │ A6  ▓▓▓┄ ║ ▓▓▓▓▓▓▓▓▓▓ ║ ▓    │                       │    ║ ▲red ▲grn ║  ▓ │
  │ A7  ▲seesaw ◄┄┄┄┄┄┄┄┄ ║ ▓    │                       │    ╚═══════════╝  ▓ │
  │ A8  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓    │                       │         ▓ exit ────┘ │
  │ A9  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓   │                       └─────────────────────┘
  └──────────────────────────┘
  A3 gaps 5/10/20 cm   A4 junctions + dead-end stub (both sides green → 180°)
  A5 ramp 22° peak     A7 seesaw ≤15° pivot   B2 obstacles: cone angled, box head-on
```

**Lighting rule.** Hang the same two LED desk lamps over the mock for every session and record
the rig in the run log. Move them mid-session once per rehearsal, to prove the mission does not
depend on a fixed light field.

---

## 2. Surrogate props, element by element

### E0 — Calibration target board (build first, use every session)

One A4 poster board: black square, white square, red, green, silver and a black-tape patch, each
≥ 6 cm, in two rows. This is what the 10 s sweep and the per-arena colour calibration aim at, and
it doubles as the reflectivity-probe bench jig. Mount it on a chair or box at the height that
puts the line array at its installed ride height, and mark its position in pen so every session
is repeatable. Keep the black and white squares **the same size and orientation** every time —
the contrast-health gate compares them, and a moving target causes false rejections.

### E1 — Black line with gaps

Black electrical tape, ¾ in (19 mm), on the white floor. Cut segments and butt them end-to-end so
the joins are invisible; weight the joins with a strip of clear tape.

- Gaps **5 cm, 10 cm and 20 cm**, in that order along the run. 5 cm is the minimum straight-in
  approach; 20 cm is the rules maximum — omit it and the hardest case is never rehearsed.
- Approach each gap dead straight, with a short guide stub before it. The reflex is specified
  for straight-in entry only, so a diagonal entry is a mis-test, not a robot failure.
- Worn or chalky tape loses contrast at the 11.2 mm ride height. Keep one sealed spare roll; a
  contrast drop shows up first as a gap failure and will be mis-diagnosed as a code fault.

### E2 — Straight-through and turn junctions, one green marker each

Green 25 mm dots cut from green tape or paper, stuck **beside** the line (never on it) at the
mount point of a side sensor 1–2 cm above the floor, spaced ≥ 30 cm apart so one marker cannot
bleed into the other sensor's field.

- **E2a** marker on the **right** of the line, robot must turn **right** (turn toward the green
  side, per contract FR-008 / HARDWARE §9). **E2b** marker on the **left**, robot must turn
  **left**. **E2c** a plain straight junction with **no** marker, robot
  must continue straight — the false-positive guard, without which a real detection cannot be
  told from noise.

  > Direction rule: green on one side → turn **that way** (left/right follow the marked side).
  > This doc originally had it inverted; corrected to match the signed contract.

### E3 — Dead-end stub, two green markers

A 20 cm black-tape stub branching off the main line and ending in the open, with a green dot on
**both** sides at the branch point. The only prop that must produce a 180° U-turn. Build it on
wide-open floor so the pivot has clearance and a radar sweep there reads an *open* profile, not a
wall — keep ≥ 30 cm clear of the evac wall.

### E4 — Ramp (peak tilt 20–25°)

Softwood plank, black line taped up the centre, propped on a non-slip block to a measured peak.

- Set the peak to **22°** with a protractor or phone inclinometer and **write the measured value
  on the plank**. A shallow ramp reading below 20° will be classified as a seesaw.
- A ramp may turn, so build **two**: E4a straight (line continues over the crest) and E4b with
  the line bending near the top. E4b catches a seesaw-tuned controller; E4a alone will happily
  pass a controller that is really just "slow down on tilt".
- Cable-tie the tape to the plank — a lifting tape corner on a 22° incline is the most common
  cause of a bogus gap test.

### E5 — Seesaw (peak tilt < 20°)

A second plank on a 20 mm PVC pipe or dowel pivot, taped down the middle, with **physical
end-stops that limit the tilt**. Set the stops for a **15°** maximum — that margin is the whole
reason this prop is built with stops rather than free.

- **Ramp vs seesaw differentiation note.** The discriminator is a gyro-pitch band: tilt below
  20° reads as *seesaw*, 20–25° reads as *ramp*, above 25° reads as *none*. The bands are only
  5° wide and sit adjacent, so the mock geometry is what keeps them apart: seesaw peak 15°
  (5° of margin below the boundary), ramp peak 22° (2° above the band floor, 3° below its
  ceiling). Measure both with the inclinometer at build time and write the numbers on the props.
  - *Fails safe:* a seesaw overshooting past 20° reads as a *ramp*, so the robot keeps QTR
    guidance — degraded, not dangerous. A prop fault, not a code fault.
  - *Fails dangerous:* a ramp shallower than 20° reads as a *seesaw* and is handed to the
    gyro-guided slow straight. On a bending ramp (E4b) that means no line guidance at all.
    Hence the measured 22° and the second, bending variant.
- Rebuild the stops and re-measure after any plank swap. Timber is not a precision instrument,
  and this is the cheapest element in the whole mock to get silently wrong.

### E6 — Obstacle stand-ins (ToF + bumper)

**E6a cone** — orange traffic cone (or cardboard cone) met at roughly 30–45° to heading.
**E6b box** — cereal box or foam cube, ~20 cm, dead ahead on the line. Height matters for both:
tall enough to break the bumper (≈ 8–10 cm), short enough that the ToF still sees it — a normal
traffic cone is ideal, a sugar cube is not.

Behaviour under test is identical for both: the ToF detects at the configured range and biases
the pass; the two soft bumper buttons are the never-fail backstop and the **pressed side
dictates the go-around direction**. Topple the cone by hand mid-test to force the bumper path and
prove the arc and the reacquisition both work with the ToF silent.

### E7 — Goal: red tape patch, 5 s hold

A red patch of tape or red card ≥ 10 cm across, at the end of a straight run. Place it so a range
return is available — against a vertical cardboard wall, or at the end of a short box-lined
corridor. The goal decision is cross-validated, so a goal with no possible return is a mock that
cannot exercise the rule at all. Keep that wall short enough not to read as an enclosed radar
profile, and keep ≥ 60 cm between the goal and the evac wall so the two do not contaminate each
other. The 5 s hold is the scored behaviour: stopwatch it on the first two rehearsals, then trust
the onboard timer once it has been seen to hold twice.

### E8 — Evac zone: silver-tape enclosure, black-tape exit

A ~60 × 60 cm floor rectangle surrounded by a **low vertical wall** cut from poster board,
~12 cm tall, cable-tied joints, wall height above the ToF mount height and joints tight
enough that no sphere can roll out mid-drill.

- **Entry side:** a band of silver tape along the inside edge of that wall.
- **Exit side:** black tape along the inside edge of the opposite wall, with the black line
  running out of it so the robot has a line to reacquire.
- **The wall is not optional decoration.** The re-localisation radar classifies surroundings
  as *enclosed* or *open* from a range profile: enclosed needs ≥ 6 contiguous close bins —
  120° of continuous arc — with a close echo defined as ≤ 1200 mm. A flat tape rectangle on
  bare floor produces no close bins and always reads *open*. Conversely, keep the open regions
  of the course clear enough that no more than 4 close bins appear at all. Build the wall, then
  verify with a live radar sweep before trusting any re-localisation drill.

### E9 — Victims: weighted spheres

Ping-pong balls, each with a small mass fixed to **one side** so the centre of mass is
off-centre and the sphere rolls when nudged. Hot-glue a bolt, washer or coin to one quadrant
and tape it down. Four are needed for the mission; two more are fakes.

| # | Body | Conductive? | Expected verdict |
|---|------|-------------|------------------|
| V1 | silver — bare aluminium/foil tape wrap | yes | **live** → green triangle |
| V2 | silver — bare aluminium/foil tape wrap | yes | **live** → green triangle |
| V3 | black — matte black tape over the ball | no | **dead** → red triangle |
| V4 (fake) | silver outside, sealed non-conductive | no | **fake → ignore** |
| V5 (fake) | matte black outside, foil inside | yes | **fake → ignore** |

**How to make them conductive-testable.** The two probes read different things: reflectivity
reads the *outside surface*, conductivity reads *any metal the probe tips can touch*. So the build
rule follows directly — **a sphere is conductive if there is a continuous metal path between the
two probe contact points, and non-conductive if there is not.** *Conductive silver (V1, V2):*
foil-tape wrap, burnished smooth, generous overlap — continuity is obvious and needs no special
care. *Non-conductive silver (V4):* identical foil wrap, then **coat the whole surface in hot
glue and finish with matte packing tape** — reflectivity is unchanged, still bright silver, but
every metal surface now sits under a dielectric film. This is the realistic fake, and the case
that punishes a vision-only classifier. *Non-conductive black (V3):* matte black tape over the
bare plastic ball, no metal at all. *Conductive black (V5):* foil-wrapped ball under matte black
tape over the whole exterior — reflectivity says black, conductivity says live; they disagree,
which is the second fake branch.

**Bench-test every sphere before it goes on the floor.** Touch the two probe contacts with a
multimeter: conductive targets well under 10 Ω, non-conductive over 1 MΩ. Write the measured
resistance on the sphere in pen. An accidentally-conductive victim silently reclassifies and the
fault surfaces three runs later as an unexplained deposit.

Place victims off-centre in the zone, ≥ 15 cm clear of the walls and of each other, at heights
where both probes reach the same sphere without the chassis touching the wall. Two at one end,
one at the far end is a good default. Keep the layout **identical for every rehearsal** so a
changed deposit result always means a changed robot.

---

## 3. Pre-run timing protocol (≤ 120 s wall-clock)

One rehearsed sequence, run identically at the qualifier and on the mock. The last column is the
failure action if a step overruns.

| # | Step | Budget | Cumulative | Watch for | Overrun action |
|---|------|--------|------------|-----------|----------------|
| 1 | Single button pressed; main switch on; both rails up | 4 s | 4 s | battery lead seated, Nicla branch filter LED | re-seat battery, re-time — nothing is degraded yet |
| 2 | Mega boot: config load, pin self-test, diagnostic dump | 6 s | 10 s | full parameter dump reaches the serial monitor | abort; a failed boot inside the clock is a lost run |
| 3 | Nicla cold boot + QSPI cal-cache load → `READY` | 4 s | 14 s | `READY` inside the hard 5 s timeout; `cal_cached` flag | timeout → error LED → degrade to line-only and log it |
| 4 | Link handshake: `MODE_SET` ack + 3 heartbeats each way, CRC counters zero | 6 s | 20 s | both directions, not just Mega→Nicla | re-sync; after 3 bad frames expect a logged recovery |
| 5 | Mandatory 10 s black+white sweep over E0 + green baseline + contrast-health gate | 10 s | 30 s | contrast pass/fail verdict, per-sensor midpoints | retry ×2, then last-good values with logged degradation, or abort per venue decision |
| 6 | Per-arena colour calibration on Nicla: `CAL_START` ch 0 (goal red, triangle red/green, silver + black tapes) | 75 s | 105 s | `CAL_COMPLETE` status 0 | `CAL_ABORT` and fall back to the QSPI-cached cal — this is what keeps the total inside 120 s |
| 7 | Verdict print, `RUN_START` to Nicla, 8-minute clock starts | 5 s | **110 s** | clock starts only on the confirmed start, never on the button | — |

**Committed total 110 s, reserve 10 s, ceiling 120 s.** Two branches deliberately sit
*outside* this budget, and the run log must record which one was taken:

- **Sweep rejection (retry ×2, up to +20 s).** It cannot fit inside 120 s alongside a fresh
  colour calibration, so on a second failed sweep the colour calibration is aborted and the
  last-good values are used with a logged degradation. The venue decision is between that and
  aborting the run outright — make it before the qualifier, not during it.
- **Cold-cache colour calibration.** The 75 s budget is the fresh case. The ≤ 90 s per-arena
  ceiling is a hard software limit; the budget sits below it on purpose, so there is room to
  finish the handshake and the start inside the same 120 s.

### Per-step checklist (print one per session)

```
PRE-RUN  ☐ 1  battery on + single button          t=___s   (budget 4)
         ☐ 2  Mega parameter dump complete        t=___s   (budget 6, cum 10)
         ☐ 3  Nicla READY, cal_cached=1           t=___s   (budget 4, cum 14)
         ☐ 4  heartbeat both ways, bad_frames=0   t=___s   (budget 6, cum 20)
         ☐ 5  10 s sweep, contrast gate = PASS    t=___s   (budget 10, cum 30)
         ☐ 6  CAL_COMPLETE status 0               t=___s   (budget 75, cum 105)
         ☐ 7  RUN_START sent, clock armed         t=___s   (budget 5, cum 110)
RESULT   total pre-run ______ s  (ceiling 120)    branch taken: good-cal / cached-cal / degraded
```

### Full-run rehearsal sessions

An 8-minute session run as a group, three runs deep with a 10-minute gap between runs: **5 min**
timing steps 1–7 twice, cold, until two consecutive clean ≤ 120 s; **8 min** for the first
supervised full mission with the operator touching nothing; **24 min** for the reset plus the
second and third pass, which is where consistency is measured rather than a single lucky run;
**10 min** debrief to fill the run log and list every deviation. Between runs, power fully off,
rest the battery 2 minutes, and **do not** re-tune inside a session — tune between sessions or
the clock measures the operator instead of the robot.

A "clean" run means: pre-run inside 120 s, mission complete inside 8 minutes, **zero unplanned
touches**. Two consecutive clean runs back to back is the freeze gate; one good run is not
evidence of anything.

---

## 4. Per-element test matrix

| # | Element | How to verify | Expected observable | Acceptance | Gate |
|---|---------|---------------|---------------------|-----------|------|
| E0 | Calibration target | Sweep + colour cal with robot on the E0 jig | Per-sensor adaptive midpoints, contrast-health verdict, QSPI cal stored | Contrast gate passes at the installed ride height; EEPROM round-trips; rejection path exercised once to a logged decision | M2 |
| E1 | Line + gaps (5/10/20 cm) | Straight-in run at reduced base speed, three passes per gap | Line-loss class fires, last-good PD output held, reacquisition on the far segment | Robot bridges all three; after reacquisition the centroid re-centres within 500 counts of centre; no timer-abort on a bridgeable gap | M3 |
| E2 | Junction, one green marker | E2a / E2b / E2c, five passes each | Correct side turn, or straight-through on the no-marker junction | Direction correct on 4 of 5 passes minimum; E2c false-positive ≤ 1 in 5; every turn decision confirmed on two separate reads | M2 (detection) + M3 (turn) |
| E3 | Dead-end stub, two greens | Three approaches, clear floor | 180° U-turn executed, robot lands on the main line | U-turn completes without line loss; robot is following again within 2 s of the pivot finishing | M3 |
| E4 | Ramp 22°, straight and bending | E4a then E4b, five passes each | Terrain class reads *ramp*; QTR guidance stays active through the climb | Class = ramp on 5 of 5 (measured pitch logged); E4b still tracks through the bend; no marker misread at the crest | M3 |
| E5 | Seesaw ≤ 15° | Five crossings, mid-crossing nudge to force a tilt | Terrain class reads *seesaw*; gyro-guided slow straight or a declared fallback | Class = seesaw on 5 of 5; no line loss; measured peak tilt recorded and below 20° | M3 |
| E6 | Obstacles (cone, box) | Dead-ahead box, then angled cone, then hand-toppled cone to force the bumper | ToF bias-away, bumper arc away from the pressed side, same line reacquired | Both senses agree or the bumper overrides in the right direction; robot returns to the **same** line within 3 s | M3 |
| E7 | Goal red patch | Three approaches, timed hold | Vision **and** range both fire; stop and hold | Hold ≥ 5.0 s on all three; cross-validation rejects a goal visible to only one sensor | M3 |
| E8 | Evac zone | Enter on silver tape, deposit, exit on black tape | Entry needs blob **and** bright-class agreement; exit on black tape | Entry and exit both detected; every zone state bounded; exit reached on all three runs | M4 |
| E9 | Victims | All five spheres, one at a time, on the probe jig and then on the floor | Reflectivity and conductivity verdicts logged per sphere | V1/V2 live, V3 dead, V4/V5 ignored — 5 of 5 correct; deposits on the correct triangle; accounting written **before** each release | M4 |
| L | Loss of power | Mid-run cut on the black tape section, three times | Restart, re-orient, resume | Re-localised and following again within 12 s, every time; ≤ 1 landmark error; works with radar assist and again with radar forced silent | M4 |
| P | Mechanism | Bench cycle, 20 repetitions | Contain, transport, release, home, every time | Decision recorded **before** the gate closes; enable/disable frozen on that evidence | M4 |

Run each row as its own short session. A row that only ever passes as part of a full run has not
been tested — you cannot tell which element broke.

## 5. Loss-of-power drill and the re-localisation bench test

### The drill

1. Run the mission and cut main power at a **known** point — the black tape straight after the
   seesaw, with two markers already passed and nothing deposited yet.
2. Restore power and press the **single** start button. Nothing else pressed, reset, or
   configured — this is the only permitted restart action. The clock below starts the moment the
   rail comes back; record when normal line following resumes.
3. Compare the landmarks actually passed against the persisted accounting. **Error = 1 landmark
   is acceptable; error ≥ 2 is a failure**, however fast the resume was.
4. Repeat three times, then a fourth with the radar forced silent, to prove the search works on
   line sensing alone.

### The 12 s budget

| Window | Elapsed | Activity |
|--------|---------|----------|
| 0.0 – 0.5 s | 0.5 s | power restored, single button pressed |
| 0.5 – 1.5 s | 1.0 s | Mega deterministic boot, watchdog armed, accounting read |
| 1.5 – 5.5 s | 4.0 s | Nicla cold boot + QSPI cal-cache load → `READY` (hard timeout 5.0 s, then error LED and degrade to line-only) |
| 5.5 – 8.5 s | 3.0 s | bounded line re-acquisition search — the primary mechanism, coarse all-white → all-dark |
| 8.5 – 11.5 s | 3.0 s | re-centred and back in line following, confirmed by the first stable centroid |
| **total** | **11.5 s** | 0.5 s reserve against the 12 s bound |

The 360° radar sweep (18 sectors at 20°, roughly 8 s at 45°/s) runs **concurrently** with
the re-acquisition search from the 5.5 s mark. It is an assist, not a gate: its sector result
biases the search direction if the search is still running, and it finishes after the line has
been reacquired. If the sweep is ever put on the critical path, boot time plus sweep time exceeds
12 s and the bound is unachievable — that ordering is a decision to confirm on the bench, not
an assumption to make here.

**Bench check before trusting any drill:** stand the robot in the open, run a radar sweep and
confirm it reads *open*; stand it inside the evac rectangle and confirm it reads *enclosed*. If
the rectangle reads *open*, the E8 walls are too low, too far, or gapped — fix the prop, not
the classifier.

## 6. Mechanism bench test (gate verdict)

The mechanism sits behind a single isolated interface with five states — idle, deploy,
contain, release, home — and one servo. The verdict is **enable or disable**, decided on
bench evidence, decided once, with no second attempt and no alternate design. Robot off the
floor, on the bench:

1. **Containment.** Twenty consecutive cycles. Place a sphere with its ballast facing the cup,
   run idle → deploy → contain. Pass only if the sphere is inside the cup and stayed there
   through the transport move. **Target: 15 of 20.**
2. **Transport.** With a contained sphere: drive 1 m, turn 90°, drive 1 m, turn 90°. Pass if
   the sphere is still contained. **Target: 15 of 20, counted with step 1.**
3. **Release.** Positioned at a triangle, release. Pass if the sphere is on the floor, clear
   of the chassis footprint, and not re-captured on the next move. **Target: 15 of 20.**
4. **Home.** From every state, including mid-cycle interruption, home must return inside its
   bounded time. Any state that can hang is an automatic fail. **Target: 20 of 20.**
5. **Accounting order.** On five releases, confirm the deposit is written to persistent
   storage *before* the sphere is released. A power cut immediately after a release must never
   produce a double or a lost deposit. **Target: 5 of 5.**

Record raw counts, not a verdict — `contain X/20, transport X/20, release X/20, home X/20,
ordering 5/5 → ENABLED | DISABLED`, plus the date. Then freeze it. **Disabled is an acceptable
outcome, not a failure:** the no-capture lane — locate, classify, decide, report — earns its
points on its own and never blocks the zone exit or the exit bonus. Decide on the numbers, not
on the disappointment.

## 7. Run log

One row per full run, copied into a notebook — not a laptop-only file. `cal branch` is
`fresh` / `cached` / `degraded`. `touches` counts any hand on the robot, however brief — one
touch means the run is not clean whatever the clock says. Review the log at the start of every
session: if the same cell fails twice, that element is not a tuning problem, it is a design
decision that has to be made and frozen.

```
date | run | pre-run s | cal branch | gap | markers | dead-end | ramp | seesaw |
     | obst | goal | evac entry | victims | deposit | LoP s | LoP err | total s | touches
-----+-----+----------+-----------+------+---------+----------+------+--------+--------+
     |     |          |           |      |         |          |      |        |        |      |      |
     |     |          |           |      |         |          |      |        |        |      |      |
```

## 8. Open build decisions and known traps

1. **The evac zone needs a wall, not a tape rectangle.** The re-localisation radar's
   enclosed/open test is a range-profile test; a flat taped rectangle on bare floor is
   invisible to it. Highest-consequence item on this page.
2. **The two tilt bands are 5° wide and adjacent.** Every decision about E4 and E5 is really
   a decision about 5° of margin. Measure, write the number on the prop, re-measure after any
   plank swap.
3. **Tape age is a sensor risk masquerading as a prop risk.** Lost contrast at the installed
   ride height presents as a gap, marker or seesaw fault. Date the rolls, keep a sealed spare.
4. **Victim conductivity is a bench measurement, never an assumption.** Multimeter, write the
   resistance on the sphere, re-check after any handling.
5. **Marker false positives need a negative control.** E2c, the no-marker straight junction,
   exists only to make false positives measurable. Building E2 without E2c gives a test that
   cannot fail.
6. **Restart is one button.** If a drill ever needs a second press, a jumper or a reflash to
   work, both the rules compliance and the drill are broken.
7. **Record the layout, then change nothing.** A moved victim or a rotated prop turns the next
   result into a puzzle instead of a measurement.
