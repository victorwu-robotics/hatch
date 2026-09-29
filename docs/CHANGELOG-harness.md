# Hatch Harness Changelog

Every change from `v0.1.0-trunk-working` forward. One entry per harness task.
This file is the human-readable record of intent. Git history tells you
*what* changed; this file tells you *why*.

---

## 2026-09-29 — hatch/fix-robot-loaded-payload

**Fixed:** ROBOT_LOADED event payload key mismatch.
`robot_manager` published `'model'`; `main_window` read `'kinematic_model'`.
Sliders did not appear after loading a robot.

**Files:** `core/robot_manager.py`

**Principles:** #2, #6

**Smoke test:** ur10_nominal.urdf loads, sliders present.
my_arm.urdf.xacro loads, sliders present.

---

## 2026-09-29 — hatch/constitution-and-changelog

**Added:** `docs/CONSTITUTION.md` and this changelog.
The constitution is the operational contract for Hatch, distilled from
`philosophy.md` and `architecture.md`, with rules derived from real
bugs encountered during the 2026-09 repair session.

**Files:** `docs/CONSTITUTION.md`, `docs/CHANGELOG-harness.md`

**Principles:** all ten (this file defines them)

**Smoke test:** not applicable (documentation only)