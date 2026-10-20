---
heist: rcj-rescue-line-2026
phase: the-grilling
status: completed
date: 2026-09-21
next-action: Proceed to The Sit-Down (Don approval pending)
artifacts:
  - Reconnaissance Dossier: docs/gangsta/rcj-rescue-line-2026/recon/2026-09-20-recon-dossier.md
note: Grilling Conclusions passed in-context to The Sit-Down for inclusion in the Contract.
---

# Grilling Checkpoint — RCJ Rescue Line 2026

Single-pass debate complete: Proposer → Devils-Advocate (CHALLENGE) → Don Phase D (partial
agreement, 5 decisions) → Synthesizer (0 unresolved CRITICAL objections).

## Grilling Conclusions (summary held in-context)

- **Idea Verdict:** SOUND (with refinements; initial verdict CHALLENGE, resolved in Phase A).
  Refinements: qualifier-first; full mission required to advance; improvised-only practice
  field; QTR-8A ride height locked at 11.2 mm; victim mechanism open w/ dated revisit gate.
- **Key Decisions:** Serial1 D18/D19 + BSS138 for Mega⇄Nicla; centroid PD as the line error
  source (no cowling); ToF + 2-button front bumper for obstacles (no HC-SR04); bounded boot +
  360° ToF radar sweep LoP re-localization; FSM as PROGMEM transition table; CONFIG/CAL/TUNING
  split; victim mechanism isolated behind victim.h w/ Q−14 gate + no-capture lane; Nicla on
  shared battery via ferrite+LC (Option A, B auto-promotes on P0 EMI fail); Hybrid LoP
  bookkeeping (victims/deposits persist, nav re-derived); seesaw = verify-then-gyro-straight,
  ramp = keep QTR guidance (line not assumed straight).
- **Rejected Alternatives:** I2C host link (bus contention); HC-SR04; light cowling; 4-state
  coarse line error; full deterministic cold-restore / re-baseline-fresh bookkeeping; SD card
  on the deck; Option B as primary Nicla power.
- **Unresolved Objections:** 0 CRITICAL; residual risks accepted per decision log (EMI on the
  UART link, APDS-9960 lighting contrast, off-center-CoM sphere capture, 11.2 mm contrast).
- **Termination Reason:** single-pass complete.