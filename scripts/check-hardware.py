#!/usr/bin/env python3
"""Drift alarm: HARDWARE.md vs the firmware repo's pin constants.

HARDWARE.md is only useful if it cannot silently disagree with the sketch.
This script reads the firmware constants (the source of truth for pins) and
checks the claims HARDWARE.md makes about them. It also refuses the stale
designs that were excised on 2026-10-20 (I2C mux, QTRSensors library,
Serial1 link, legacy sketches).

Usage:
    scripts/check-hardware.py
    FIRMWARE_DIR=~/Documents/Arduino/Robocup.ino scripts/check-hardware.py

Exit 0 = the document and the firmware agree. Exit 1 = drift; fix one of them.
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HW = ROOT / "HARDWARE.md"
FW = Path(os.environ.get("FIRMWARE_DIR", Path.home() / "Documents/Arduino/Robocup.ino"))

failures: list[str] = []
checks = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global checks
    checks += 1
    if ok:
        print(f"PASS {label}")
    else:
        print(f"FAIL {label}{(': ' + detail) if detail else ''}")
        failures.append(label)


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"FAIL cannot read {path}: {exc}")
        raise SystemExit(1)


def constant(source: str, name: str) -> str | None:
    m = re.search(rf"const\s+int\s+{name}\s*=\s*(\d+)\s*;", source)
    return m.group(1) if m else None


def main() -> int:
    hw = read(HW)
    sketch = read(FW / "Robocup.ino.ino")
    led = read(FW / "LedDisplay.h")

    # --- Drivetrain -----------------------------------------------------
    for name, pin in (
        ("LEFT_IN1", "2"),
        ("LEFT_IN2", "3"),
        ("RIGHT_IN1", "4"),
        ("RIGHT_IN2", "5"),
    ):
        got = constant(sketch, name)
        check(f"firmware {name} == D{pin}", got == pin, got or "missing")
        check(f"HARDWARE.md documents D{pin} for {name}", f"| D{pin} |" in hw)

    check(
        "sketch does not drive ENA/ENB (they must be jumpered high)",
        not re.search(r"\bENA\b|\bENB\b", sketch),
    )

    # --- Line bar -------------------------------------------------------
    m = re.search(r"sensorPins\[8\]\s*=\s*\{(.*?)\};", sketch, re.S)
    pins = re.findall(r"A\d+", m.group(1)) if m else []
    expected = ["A0", "A2", "A4", "A6", "A8", "A10", "A12", "A14"]
    check("firmware line bar is left-to-right A0..A14", pins == expected, ",".join(pins))
    for pin in expected:
        check(f"HARDWARE.md lists line pin {pin}", f"| {pin} |" in hw)

    # --- Colour sensors -------------------------------------------------
    m = re.search(r"SoftWire\s+softWire\((\d+),\s*(\d+)\)", sketch)
    check("RIGHT APDS SoftWire == D30/D31", bool(m and m.groups() == ("30", "31")))
    check("HARDWARE.md documents the SoftWire bus", "D30" in hw and "D31" in hw)
    check("HARDWARE.md documents the hardware I2C bus", "D20" in hw and "D21" in hw)
    check("sketch has no I2C mux", not re.search(r"TCA|tcaselect", sketch))

    # --- LED strip ------------------------------------------------------
    check("sketch uses 12 strip pixels", "STRIP_PIXELS = 12" in led)
    check("strip data pin is D35", "q42::STRIP_PIXELS, 35" in sketch)
    check("HARDWARE.md documents D35", "D35" in hw)

    # --- Console / Nicla ------------------------------------------------
    check("sketch console is 115200", "Serial.begin(115200)" in sketch)
    check("sketch has no Serial1 link yet", "Serial1" not in sketch)
    check("HARDWARE.md says the Nicla link is not implemented", "not implemented" in hw)

    # --- No resurrected removed designs ---------------------------------
    check("sketch does not use the QTRSensors library", "QTRSensors" not in sketch)
    check("HARDWARE.md has no removed src/ path", "src/rescue_robot" not in hw)
    check("HARDWARE.md has no removed PWM Test sketch", "PWM Test" not in hw)
    check("HARDWARE.md carries no spec ids", not re.search(r"\b(FR|NFR|WP)-\d", hw))

    print(f"\n{checks - len(failures)}/{checks} checks pass")
    if failures:
        print("DRIFT: fix HARDWARE.md or the firmware (do not leave them disagreeing)")
        return 1
    print("OK: HARDWARE.md and the firmware pin map agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
