# The Official Repo for Team Quantum 42

RoboCup Junior Rescue Line 2026 platform — Arduino Mega 2560 + Nicla Vision, line-following
with rescue (goal-stop, evac zone, victim classification), built under the Gangsta Agents
Spec-Driven Development framework.

**Status (2026-10-20):** The active Arduino Mega 2560 firmware lives in the
separate **firmware repo** (`~/Documents/Arduino/Robocup.ino`) — see `REPOS.md`.
The in-repo firmware design from heist `rcj-rescue-line-2026` was excised on
2026-10-20 and remains recoverable from git history (snapshot commit `3450fff`);
the heist artifacts stay in `docs/gangsta/`.

## Layout

| Path | What it is |
|------|-----------|
| `nicla/` | Nicla Vision MicroPython/OpenMV scripts (`tof.py`, `imu.py`, `config/link/vision/main`) |
| `test/` | Host-side Python tests for `nicla/` (`*_test.py`, run by `run_tests.sh`) |
| `docs/mega-nicla-protocol.md` | Byte-level Serial1 (D18/D19 + BSS138) link spec between Mega and Nicla |
| `docs/gangsta/rcj-rescue-line-2026/` | Heist artifacts: dossier, grilling, contract (signed), execution plan, checkpoints |
| `HARDWARE.md` | Hardware ground truth — wiring, power, calibration sweep, pre-race checklist |
| `firmware/` | Pinned read-only submodule of the firmware repo (edit the Arduino folder, not this) |
| `RoboCupBot/` | Chassis CAD (`Base.stl`, `Robocup.f3d`) |
| `results/` | Bench/run logs |

## Build & test

```sh
./test/run_tests.sh        # Nicla host tests (python3 unittest); must exit 0
```

Mega firmware (separate repo):

```sh
cd ~/Documents/Arduino/Robocup.ino
./test/run_tests.sh                               # host tests (g++ C++14)
arduino-cli compile --fqbn arduino:avr:mega .     # sketch build
```

The signed Contract forbids reproducing Gangsta-internal spec identifiers (`FR-xxx`,
`NFR-xxx`, `WP-xxx`) in source, tests, comments, or documentation — Heist identifiers live
only in `docs/gangsta/`.

Working branch: `heist/rcj-rescue-line-2026`. Do not merge to `main` until Laundering.