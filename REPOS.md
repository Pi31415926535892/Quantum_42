# Repository Map — where the code actually lives

This project is spread across **two separate Git repositories**. They are
deliberately independent — neither is a copy of the other, and they do **not**
share history. Read this file before you go looking for "the latest code".

> **Short version**
> The **newest, actively-developed robot firmware** lives in the **firmware
> repo** at `~/Documents/Arduino/Robocup.ino`.
> **This** repo (`Quantum_42`) holds specs, docs, hardware notes, CAD, results
> and an earlier in-repo firmware design. If you are hunting for working robot
> code, it is almost certainly the firmware repo.

---

## 1. Project repo (this one)

| | |
|---|---|
| **Path** | `~/Documents/Quantum_42` |
| **Remote (origin)** | `https://github.com/Pi31415926535892/Quantum_42.git` — **public** |
| **GitHub owner** | `Pi31415926535892` |
| **Default branch** | `main` |
| **Working branch** | `heist/rcj-rescue-line-2026` |
| **Holds** | Gangsta heist artifacts (`docs/gangsta/`), hardware ground truth (`HARDWARE.md`), Nicla Vision scripts (`nicla/`) with their host tests (`test/`), chassis CAD (`RoboCupBot/`), and bench/run logs (`results/`). **No Mega firmware code** — that lives in the firmware repo. |

## 2. Firmware repo (active development)

| | |
|---|---|
| **Path** | `~/Documents/Arduino/Robocup.ino` |
| **Remote (origin)** | `git@github.com:BismarckYamato/Quantum_42-Firmware.git` — **private** (SSH) |
| **GitHub owner** | `BismarckYamato` |
| **Default branch** | `main` |
| **Holds** | **The active Arduino Mega 2560 firmware.** Newest commits and the current robot code (`Robocup.ino.ino`, `ColourCalibration.h`, `MarkerSettle.h`, `LedDisplay.h`, …) live here. |

The firmware repo is also registered in **this** repo as a **git submodule** at
`firmware/`, pinned to a specific commit — see [§5](#5-submodule-firmware).

> **Note on accounts.** The two repos are owned by two different GitHub
> accounts: `Pi31415926535892` (project) and `BismarckYamato` (firmware).
> `BismarckYamato` is the account authenticated on this machine. On 2026-10-22
> the project repo pushed successfully over SSH (the HTTPS remote is rewritten),
> so **both repos are writable from here**. Do not rename repos or switch
> remotes to "fix" access.

---

## 3. Firmware lives in exactly one repo — the firmware repo

There used to be three firmware-like locations; the two stale ones in this repo
(`src/rescue_robot/` and the legacy `Robocup.ino` / `PWM Test` sketches) were
**removed on 2026-10-20** because they duplicated the active firmware. They
remain recoverable from git history (snapshot commit `3450fff`). Only two
locations exist now, and only one is writable:

| Location | What it is | Status |
|---|---|---|
| `~/Documents/Arduino/Robocup.ino` (**firmware repo**) | The real, actively-developed firmware | ✅ **CURRENT — develop here** |
| `firmware/` (**this repo**, submodule) | A pinned read-only reference to the firmware repo at a known commit | 📌 Pin only — edit the Arduino folder, not here |

## 4. How to tell which folder is newest

Run the helper:

```sh
scripts/repo-status.sh
```

It prints, side by side, each repo's branch, HEAD, last commit date, working-tree
status, remote, and how the submodule pin compares to the firmware repo's HEAD.

Manually, the fastest check is:

```sh
git -C ~/Documents/Quantum_42            log -1 --format='project : %h %ci %s'
git -C ~/Documents/Arduino/Robocup.ino   log -1 --format='firmware: %h %ci %s'
```

Whichever shows the **more recent date** is the more recent code. Historically
that has been the **firmware repo**.

## 5. Submodule (`firmware/`)

The submodule records *exactly* which firmware commit this project snapshot
corresponds to.

```sh
# see the pinned commit
git submodule status firmware

# after pushing new firmware, re-point the pin to the firmware repo's latest
scripts/sync-firmware.sh
```

Because the firmware's working copy lives in the Arduino sketchbook, the
`firmware/` checkout here will **lag** unless you re-run the sync script. That is
expected: `firmware/` is a reference, not where you edit.

## 6. Everyday workflow

1. **Write firmware** in `~/Documents/Arduino/Robocup.ino`.
2. Commit and push there (`git push` → `Quantum_42-Firmware`, private). This is the
   **only** remote this machine can push to.
3. Back in this repo, update the pin: `scripts/sync-firmware.sh`.
4. Commit any project-side docs/results here on the `heist/rcj-rescue-line-2026`
   branch and push (the remote is SSH-rewritten and writable as of 2026-10-22).

---

_Last updated: 2026-10-20. If you change where code lives, update this file._
