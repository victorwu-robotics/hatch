# Hatch Handoff

**Purpose:** This file is the entry point for a fresh session (human or AI).
Paste it as the first message. It says where the project is, what was
decided, what is in progress, and what comes next.

**Keep it up to date.** Update before closing any session.

---

## 1. Project state

Hatch is at `v0.1.0-trunk-working` plus two harness commits:
- `b49a18b` — post-merge repair (all `-X theirs` merge wounds fixed)
- `44553d0` — constitution and changelog added

Trunk: `main`. Both `main` and `FR-arms` point at the same commit.
Old branches preserved as `archive/*` and `backup/*`.

**What works:** Hatch launches; loads plain URDF and xacro scenes;
renders meshes; joint sliders appear and move the model.

**What does not yet work:** IK solver is the old `URIKSolver` /
`OffsetWristIKSolver`, which has a `base_offset` mismatch that
silently falls back to a broken state for UR robots. This is being
replaced by a `UnifiedIKSolver` (see section 3).

**Known minor issues:** `docs/CONSTITUTION.md` has unbalanced
markdown fences from Part 7 onward (cosmetic; fix opportunistically).

---

## 2. Standing decisions

These are settled. Do not re-litigate without a reason.

- **`HATCH_PACKAGE_PATH` is additive to `~/hatch/assets/`.**
  The default assets are always searched; the env var adds more.
  (Reading 1 from the 2026-09-28 decision.)
- **Default assets (Tier 1):** three tested arms + RealSense D435.
  Three arms not yet named in the constitution — TBD.
- **IK solver input:** flange pose in true root frame.
  Not TCP pose. Tool chain is the controller's responsibility.
- **IK selection:** by continuity of branch (shoulder/elbow/wrist),
  not by proximity. Proximity is a fallback.
- **Spherical θ₁:** the two shoulder solutions are φ and φ + π,
  not φ ± arcsin(d₄/r). The limit is singular as d₄ → 0.
- **Forearm length:** distance from J3 axis to P5.
  Not `axis_gap(2)` — these differ for offset wrists.
- **Event payloads:** keys are a contract; document in
  `event_types.py`. No silent early returns from handlers.

---

## 3. Current task: unified IK solver

The old solvers (`URIKSolver`, `OffsetWristIKSolver`) are being
replaced by a single `UnifiedIKSolver`, derived from first principles
in `docs/inverse_kinematics.md`.

**The derivation** (Parts I–VI plus Part VII and two appendices) is
written and has been corrected four times:
1. Spherical θ₁ case is φ and φ + π (singular limit of offset case)
2. Forearm length is J3-to-P5, not axis_gap(2)
3. Selection is by continuity of branch, not proximity
4. Terminology: "wrist offset" = J4 offset, not J5 offset

**The plan, in order:**
- Step A: inspect `geometric_extraction.py` and `sph_theta_1_test.py`
  to see what already exists and what needs to change.
- Step B: integrate the four corrections into
  `docs/inverse_kinematics.md` as a single coherent document.
- Step C: align `geometric_extraction.py` if it has any of the
  earlier errors (forearm length, tool_offset to flange not TCP,
  J5 offset heuristic).
- Step D: write `core/kinematics/unified_ik_solver.py` matching
  Part VII step by step.
- Step E: write `tests/test_unified_ik.py` reproducing the worked
  Elfin example and validating on UR10, FR5, Elfin.
- Step F: delete `URIKSolver` and `OffsetWristIKSolver`.

**Currently stuck at:** Step A. Need to see
`geometric_extraction.py` and `sph_theta_1_test.py` to know
what is already correct and what needs changing.

---

## 4. Open questions

- Is `tool_mount_link` in `KinematicModel` exactly the flange,
  or something else? Determines how extraction computes `d₆`.
- Does `sph_theta_1_test.py` test the corrected spherical formula
  (φ and φ + π) or the old one (φ only)?
- Which three arms are the "tested and working" defaults?
  (UR10, one FR variant, one more — E15_Pro?)
- Does FR5 have a nonzero J4 wrist offset, and what is its value?

---

## 5. Where everything lives

- `docs/CONSTITUTION.md` — the operational contract.
- `docs/CHANGELOG-harness.md` — every harness task, why it happened.
- `docs/inverse_kinematics.md` — the unified IK derivation.
- `docs/HANDOFF.md` — this file.
- `docs/philosophy.md`, `docs/architecture.md` — the long-form
  philosophical and architectural documents.
- `git branch -a` — lists all branches including archives.

---

## 6. How to start a fresh session

1. Paste this file as the first message.
2. Say what you want to do next.
3. The new AI reads the constitution and this handoff, and we begin.

Do not re-explain the project from scratch. The files contain
everything. Trust the files.

---

The handoff I drafted earlier needs two additions:

1. A "next session's first task" line:

Next session's first task: Integrate the four corrections into docs/inverse_kinematics.md. Before writing, paste and read: the current docs/inverse_kinematics.md, core/kinematics/geometric_extraction.py, and core/kinematics/sph_theta_1_test.py. Then rewrite the affected sections as one coherent document.

2. A note that the current inverse_kinematics.md is not yet corrected:

Warning: The current docs/inverse_kinematics.md on disk is the pre-correction version. The four corrections exist as a separate document in the prior session's transcript, not yet written into the file. Do not treat the current file as authoritative until the integration task is done.

That second note is critical. Without it, a fresh session might read the file, see Part IV §8 saying "the code must add the φ + π solution," and take it as correct. The handoff must say explicitly that the file is known to be pre-correction.