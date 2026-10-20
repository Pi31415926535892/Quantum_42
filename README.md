# The Official Repo for Team Quantum 42

RoboCup Junior Rescue Line 2026 platform — Arduino Mega 2560 + Nicla Vision, line-following
with rescue (goal-stop, evac zone, victim classification), built under the Gangsta Agents
Spec-Driven Development framework.

**Status (2026-09-22):** Heist `rcj-rescue-line-2026` is in **The Hit** (execution) phase.
See `docs/gangsta/rcj-rescue-line-2026/checkpoints/` for the resume checkpoint, and
`docs/gangsta/rcj-rescue-line-2026/plans/2026-09-22-execution-plan.md` for the Work Package
breakdown and milestones (Don deadline: robot fully up and running by **Oct 15**, tuning
Oct 15–22, qualifier ≈ Oct 22).

## Layout

| Path | What it is |
|------|-----------|
| `src/rescue_robot/` | Mega firmware modules (pure-C++14 logic in `.h`/headers + AVR-only glue in `.ino`) |
| `nicla/` | Nicla Vision MicroPython/OpenMV scripts (`tof.py`, `imu.py`; `config/link/vision/main` in progress) |
| `test/` | Host-side test harness — `run_tests.sh` compiles+runs `*_test.cpp` (g++ -std=c++14) and `*_test.py` (python3 unittest); `dryrun.ino` is the AVR overlay skeleton |
| `docs/mega-nicla-protocol.md` | Byte-level Serial1 (D18/D19 + BSS138) link spec between Mega and Nicla |
| `docs/gangsta/rcj-rescue-line-2026/` | Heist artifacts: dossier, grilling, contract (signed), execution plan, checkpoints |
| `HARDWARE.md` | Hardware ground truth — wiring, power, calibration sweep, pre-race checklist |
| `Robocup.ino`, `PWM Test` | Legacy broken sketches (LEFT/RIGHT swapped, wrong Kp/line-lost); remap pending `WP-022` |
| `RoboCupBot/` | Chassis CAD (`Base.stl`, `Robocup.f3d`) |

## Build & test

```sh
./test/run_tests.sh        # host tests: g++ C++14 + python3 unittest; must exit 0
```

The signed Contract forbids reproducing Gangsta-internal spec identifiers (`FR-xxx`,
`NFR-xxx`, `WP-xxx`) in source, tests, comments, or documentation — Heist identifiers live
only in `docs/gangsta/`.

Working branch: `heist/rcj-rescue-line-2026`. Do not merge to `main` until Laundering.