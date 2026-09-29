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
in `docs/unified_IK.md`.

**Two documents exist, and they are different:**

- `docs/inverse_kinematics.md` — the **old** spherical-wrist IK
  derivation for the Elfin arm. It stays on disk as a reference
  during development. It will be deleted when the unified solver
  is implemented and tested.

- `docs/unified_IK.md` — the **new** unified derivation. It is
  the target specification for `UnifiedIKSolver`.

**State of `docs/unified_IK.md`:**

Parts I–VI are the **pre-correction** derivation. Part VII and the
two appendices are the **corrected** version. The four corrections
have NOT yet been integrated into Parts I–VI. A status note at the
top of the file says so. The file is internally inconsistent until
integration is done.

**The four corrections that need to be integrated:**

1. **Spherical θ₁ (Part IV §8).** The two shoulder solutions for
   a spherical wrist are `φ` and `φ + π` by construction. This is
   a *singular limit* of the offset case as `d₄ → 0`, not a
   special case bolted onto the offset formula. Revise §8 so it
   presents the two-case construction directly.

2. **Forearm length (Part VI §3).** The forearm length `a₃` is
   the distance from J3's axis to P5. It is **not** `axis_gap(2)`.
   For a spherical wrist they coincide; for an offset wrist they
   differ. Revise §3.

3. **Selection (Part VI §11).** Selection is by **continuity of
   branch** (shoulder/elbow/wrist tags matching the current pose),
   not by proximity. Proximity is a fallback. Revise §11.

4. **Terminology (Part V §2).** The quantity that varies between
   robots is the **J4 wrist offset** — the perpendicular distance
   from J4's axis to P5. The J5 offset is zero by construction.
   Use "J4 wrist offset" or "wrist_offset" consistently.

Also delete the stray `# Confirmed — Writing Now` fragment if it
still exists between Part VI and Part VII. (It may already have
been removed.)

**The plan, in order:**

- Step A: inspect `core/kinematics/geometric_extraction.py` and
  `core/kinematics/sph_theta_1_test.py`. Confirm whether they
  already use the corrected quantities, or whether they need
  alignment with the corrections.
- Step B: integrate the four corrections into `docs/unified_IK.md`,
  producing one coherent document.
- Step C: align `geometric_extraction.py` if it has any of the
  earlier errors (forearm length, `tool_offset` measured to TCP
  instead of flange, J5 offset heuristic).
- Step D: write `core/kinematics/unified_ik_solver.py` matching
  Part VII step by step.
- Step E: write `tests/test_unified_ik.py` reproducing the worked
  Elfin example and validating on UR10, FR5, Elfin.
- Step F: delete `URIKSolver` and `OffsetWristIKSolver`. Delete
  `docs/inverse_kinematics.md` and rename `docs/unified_IK.md` to
  `docs/inverse_kinematics.md`.

**Currently stuck at:** Step A. Need to see the two files to know
what already exists and what needs changing.

---

## 4. Open questions

- Is `tool_mount_link` in `KinematicModel` exactly the flange, or
  something else? Determines how extraction computes the flange
  offset `d₆`.
- Does `sph_theta_1_test.py` test the corrected spherical formula
  (φ and φ + π) or the old one (φ only)?
- Which three arms are the "tested and working" defaults?
  Candidates in `assets/robots/`: `ur_description` (UR10),
  `Farino` (FR3/FR5/FR10), `E15_Pro` (Elfin?), `bunker_pro_description`.
- Does FR5 have a nonzero J4 wrist offset, and what is its value?
- The old `docs/inverse_kinematics.md` — how does its content
  relate to the new unified derivation? Does it have anything
  worth preserving as a pedagogical "simple case" section, or is
  it fully superseded?

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