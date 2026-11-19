---
layout: default
title: Hatch Handoff 3 — Unified IK Complete, What Comes Next
---

# Hatch Handoff 3 — Unified IK complete; the spine is the next frontier

**Date:** 2026-11-18
**Trunk:** `main` @ `eae6401`
**Supersedes:** `HANDOFF-2.md` (its two "unresolved" problems are solved; its
acceptance target is unreachable — see §2 and §6)

Read this first. It is written so a fresh session can continue without
re-deriving anything, and so a compacted conversation can be corrected against
ground truth rather than against recollection.

**The one-command gate:** `python verify_ik.py` (needs only numpy). If that
prints `ALL CHECKS PASSED`, the kinematics kernel is sound.

---

## 1. What is built and verified

| Component | File | State |
|---|---|---|
| Unified analytical IK | `core/kinematics/unified_ik_solver.py` | complete, verified |
| TCP/world adapter | `core/kinematics/ik_solver.py` | complete, verified |
| Axis geometry extraction | `core/kinematics/geometric_extraction.py` | A-target rework (`072f62a`) |
| URDF model + FK | `core/kinematics/kinematic_model.py` | unchanged |
| Test suites | `core/kinematics/test_unified_ik_solver.py`, `test_ik_solver.py` | 119 pass, 1 skip |
| Standalone verifier | `verify_ik.py` (repo root) | 31 checks, all pass |

One solver covers offset and spherical wrists by the same derivation. **The
spherical wrist is not a special case — it is the value at δ = 0**, where a term
vanishes and a direction becomes available later. There are no wrist-type labels
and no per-arm branches anywhere in the solver.

### The minimal working set

`core/kinematics/` needs exactly five files to drive the Cartesian panel:

```
__init__.py                 (empty, keep it)
geometric_extraction.py
kinematic_model.py
unified_ik_solver.py
ik_solver.py                ← the adapter; deleting this disables Cartesian control
```

Everything else formerly in that folder is dead code (§8). Verified by deleting
all but these five and re-running all three arms.

---

## 2. HANDOFF-2's two open problems are closed

### Problem A — θ₅

**Solved:** `cos θ₅ = dot(â₆, â₄)`, and `â₅ = ±normalize(â₄ × â₆)`. That sign is
the wrist branch; δ is a positive length and carries no sign of its own.

HANDOFF-2 proposed deriving θ₅ from a lateral offset `lat = dot(P6, n̂)`. **That
argument is circular.** θ₁ was solved precisely to make `dot(P5, n̂) = h`, so
`lat = h + d₆·dot(â₆, n̂)` and the expression collapses identically to
`dot(â₆, n̂)`. Its reachability guard `|lat − σρ| ≤ d₆` reduces to
`|dot(â₆, n̂)| ≤ 1`, which can never fail. A guard that cannot guard is not a
guard. Do not resurrect it.

### Problem B — the position of J4 / the arm chain's target

**Solved:**

```
A = P5 − ρ·â₄ − δ·â₅          P5 = P6 − d₆·â₆   (d₆ signed)
```

`A` is the wrist root, defined by **projection**, never by a line/plane
intersection. The arm chain targets **A, never Q**. `|A − Q| = δ`.

HANDOFF-2 proposed `A = P05_x·cosθ₁ + P05_y·sinθ₁ − d4`. That is the DH route,
which the project forbids, and it does not generalise to a roll wrist.

**Do not use "J4 axis ∩ arm plane" to find A.** That divides by `â₄·n̂`, which is
~3e−6 on Elfin and produces garbage (ρ ≈ 0.57 m). Projection is the only correct
operation, and it is why the spherical case falls out for free.

---

## 3. The derivation in brief

Full treatment: **`docs/unified_IK.md`** (corrected; Parts 5–9 and Appendices A–D).

Three lengths, mutually independent — conflating them was a recurring error:

| symbol | meaning | UR10 | FR5 | Elfin |
|---|---|---|---|---|
| `ρ_arm` | J1 axis → arm plane | ~0 | ~0 | ~0 |
| `ρ` | arm plane → B | 0.163941 | 0.102099 | ~0 |
| `δ` | B → P5 | 0.115700 | 0.102000 | **0** |

Other measured constants (true-root frames):

| | h (signed) | d₆ (signed) | L2 = `axis_gap(2)` | L3 = `elbow_to_wrist` | \|â₄·n̂\| |
|---|---|---|---|---|---|
| UR10 | +0.163941 | **+0.092200** | 0.612000 | 0.572300 | 1.0 |
| FR5 | +0.102099 | +0.100000 | 0.425000 | 0.395010 | 1.0 |
| Elfin | ~+1e−6 | +0.200000 | 0.730000 | 0.570000 | 3e−6 |

`n̂₀ = (0, −1, ~0)` and `az₂ = −90°` on **all three** arms. Elfin has
`â₂·â₃ = −1` (anti-parallel); the signed-angle primitive handles this with no
special case.

### The availability rule — order is forced, not chosen

A direction is available once the joints that fix it are known. That, not a
wrist-type flag, determines the solving order:

| J4 | δ | order |
|---|---|---|
| normal to arm plane | > 0 | θ₁ → wrist → A → arm  (UR10, FR5) |
| normal to arm plane | = 0 | θ₁ → A → arm → wrist |
| in the arm plane (roll) | = 0 | θ₁ → A → arm → wrist  (Elfin) |
| in the arm plane (roll) | > 0 | **circular dependency → refuse** |

### The single angle primitive

Every joint angle is obtained the same way:

```
θᵢ = signed_angle(reference → target about âᵢ) − cᵢ
```

No Euler extraction, no rotation-matrix inversion, no DH table, no numerical
iteration. `c₅` and `c₆` are measured at the URDF zero pose (`c₆ = +π/2` on UR10
and FR5, **−π/2 on Elfin** — hardcoding π/2 would silently mis-orient Elfin).
`c₄` is zero by construction (§4).

---

## 4. Five traps found by implementing the derivation

These cost real debugging time. They are now fixed in code and folded back into
`unified_IK.md`, but they are the places a future rewrite would go wrong.

1. **θ₁ needs the `n₀z·P5_z` term.** The exact constraint is
   `cos(θ₁ + az₂ − α) = (h − n₀z·P5_z) / (s·r)` with `s = |(n₀ₓ, n₀ᵧ)|`.
   FR5 and Elfin tilt J2 out of horizontal by 3.7e−6 rad. Dropping the term cost
   **7.6e−4 rad** in θ₁ near the shoulder singularity — and since the arm plane
   normal *is* `Rz(θ₁)·n̂₀`, that error rotates the whole plane into every
   downstream joint. It was the largest single error source found.

2. **θ₄'s reference must be â₅ itself at θ₄ = 0.** Not the parent link's X̂
   (wrong by exactly π/2 on UR10), and not the forearm direction f̂ (**degenerate
   on Elfin**, where â₄ lies along the forearm so f̂ ∥ â₄). Reading â₅⁰ needs one
   chain placement and no constant.

3. **Signed d₆ is +0.0922 for UR10**, not −0.0922. The sign follows *which link
   the model calls the flange*. `get_flange()` returns `wrist_3_link`; walking to
   the end of the fixed chain (`tool0`) flips Z. Measure it per model as
   `dot(P_flange − P5, â₆)`. Note `ArmGeometry.flange_offset` and
   `wrist_normal_offset` are **unsigned norms** and cannot supply signs.

4. **The singularity threshold must come from the model's own tilt.** A fixed
   `|â₄ × â₆| < 1e−7` never fires on FR5 or Elfin (their axes are tilted 3.7e−6),
   so a genuinely singular wrist looks resolvable and the solver returns an
   unflagged, arbitrary answer. Use
   `singular_tol = max(1e−7, 10 × model_tilt)`. Found by a failing test, not by
   reasoning. **A direction cannot be resolved more finely than the model defines
   it.**

5. **At the wrist singularity, a pitch wrist's â₅ depends on the arm angles too.**
   A is then a *circle* of radius δ, not a point. Placing the chain with θ₂=θ₃=0
   to read â₅ picks the wrong member of the family. Continuity must rotate the
   whole reference configuration's â₅ into the current arm plane.

---

## 5. Frames: the contract that must not be blurred

| question | API | frame | tool |
|---|---|---|---|
| kinematics layer | `model.get_flange_pose(q)` | **true root** | none |
| application layer | `model.forward_kinematics(q)` | **world** | composed |

`unified_ik_solver` speaks flange/true-root. `ik_solver.IKSolver` is the adapter
that speaks TCP/world: it removes the world transform and the tool transform,
maps joint ordering **by name** (never by position), and selects a branch by
continuity.

**The tool must come off before the solver sees the pose.** A bent welding torch
puts the TCP off J6's axis, and `P6 − d₆â₆` applied to a TCP is simply wrong.

The adapter refuses a model whose `tool_mount_link` differs from `get_flange()`,
because the conversion assumes they coincide. They do coincide on all three arms.

`true_base == world` on all three arms at load, so the Cartesian panel's
world→base conversion is currently a no-op. It will not be if a robot is
repositioned in the scene — worth remembering when that becomes possible.

---

## 6. Verification, and the honest accuracy limit

```bash
python verify_ik.py                    # 400 poses/arm, ~5 s
python verify_ik.py --poses 2000 --show-branches
python verify_ik.py --urdf path/to/arm.urdf --name MyArm
python -m pytest core/kinematics/test_unified_ik_solver.py \
                 core/kinematics/test_ik_solver.py -q
```

Measured on both the development sandbox and the maintainer's machine
(numpy 2.2.6/2.4.6, Python 3.11/3.12) with **bit-identical results**:

| | poses | branches | worst position | worst orientation |
|---|---|---|---|---|
| UR10 | 400/400 | 2822 | **1.9e−11 m** | 3.65e−8 rad |
| FR5 | 400/400 | 2766 | 7.35e−7 m | 7.35e−6 rad |
| Elfin | 400/400 | 3200 | 5.50e−6 m | 7.34e−6 rad |

### Residual = URDF axis tilt × pose condition number

The solver is exact. The residual is the model's own inconsistency amplified by
geometry. Bucketing by `cond = max(1/sinβ, 1/r, 1/|sinθ₅|, 1/|sinθ₃|)`:

| arm | model tilt | cond<10 | cond<100 | cond<1000 | cond≥1000 |
|---|---|---|---|---|---|
| UR10 | 2.1e−10 | 1.9e−9 | 9.2e−9 | 2.9e−8 | 1.3e−7 |
| FR5 | 3.7e−6 | 3.4e−5 | 1.2e−4 | 5.6e−4 | 5.7e−4 |
| Elfin | 3.7e−6 | 1.3e−4 | 1.2e−3 | 3.2e−3 | 2.3e−2 |

Two consequences, both important:

- **Joint-vector comparison is the wrong acceptance test.** Near a singularity
  the pose is well determined but the joint vector is not. Test the *pose*.
- **HANDOFF-2's target of `max residual < 1e−6` for all three arms is
  unreachable** on FR5 and Elfin without cleaning their URDF axis definitions.
  Chasing it means hunting a defect that does not exist.

### Singularities and refusals are tested, not assumed

Wrist (θ₅ = 0, ±π), elbow (full stretch), shoulder (r = |h|), unreachable →
`None`, limits after one 2π wrap, non-mutation, and a **200-step Cartesian jog**
asserting configuration and limits hold throughout. Plus three refusal cases.

**`q = 0` is a wrist singularity on all three arms** (`â₄·â₆ = +1`). Never use it
as the general test case; keep it as a labelled singularity test.

---

## 7. Refusal policy — no numerical fallback, ever

An arm outside the supported class raises `UnsupportedArmError`, naming the
violated condition and its measured value. `robot_manager` logs it at ERROR and
records `ik_unavailable_reason`. Nothing is substituted.

Refused: shoulder offset ρ_arm ≠ 0 (PUMA 560 class — needs common tangents to two
circles, up to 16 solutions); roll wrist with δ > 0 (the dependency loop);
non-orthogonal wrist; flange Z not aligned with J6; h not constant across poses.

**Why the numerical fallback was removed — measured, not asserted:**

| test | old `NumericalIKSolver` returned |
|---|---|
| Elfin, zero seed | `q₃ = 57.55 rad` against a ±2.61 limit |
| target 3 m out of reach | a joint vector with **3.01 m** error, and **not `None`** |

The first is the right answer unwrapped and unchecked. The second was reported to
the caller as *success*. The objection is not "numerical is imprecise" — it is
**no failure signal and no limit enforcement**.

Also measured: the old dispatcher was **already dead on all three arms**. Its
wrist discrimination read `origin_xyz[0]` of the last three joints — on UR10 that
is −0.5723, the *forearm length*, not a wrist offset. Both analytical branches
raised `TypeError` (constructor drift), a bare `except Exception` swallowed them,
and every arm silently fell through to the numerical solver.

---

## 8. Superseded — do not follow these

| artifact | status |
|---|---|
| `docs/HANDOFF-2.md` | superseded by this file; its §3 problems are solved, its §5 target unreachable |
| `docs/inverse_kinematics.md` | teaches **spherical-wrist-only** IK; contradicts the unified solver. Kept for pedagogy only |
### 8.1 Deleted — commit `6be51c4`, 13 files, 3,411 lines

Gone from the tree; recoverable from history only. Do not resurrect any of them.

| artifact | why it went |
|---|---|
| `core/kinematics/numerical_ik_solver.py` | refused; see §7. Unsafe, not merely imprecise |
| `core/kinematics/ur_ik_solver.py` | arm-specific, hardcoded DH |
| `core/kinematics/offset_wrist_ik_solver.py` | wrapper over the above |
| `core/kinematics/analytical_ik_solver.py` | Elfin-specific, hardcoded ZYZ Euler |
| `core/kinematics/solver_v2.py`, `unified_ik_solver_backup.py` | pre-correction derivation |
| `core/kinematics/dh_extract.py` | superseded by `geometric_extraction.py` |
| `core/kinematics/ik_test.py`, `sph_theta_1_test.py` | pre-existing collection errors (broken import; obsolete `package_dirs=` kwarg) |
| `core/kinematics/fk_test.py` | DH-comparison diagnostic; its only caller was a commented-out block |
| `tests/test_analytical_ik.py`, `phi_0_diag.py`, `test_solver_v2.py` | diagnostics for the abolished solvers; would have become broken importers |

Verified with all 13 removed: `verify_ik.py` 31 checks pass, 119 tests pass,
adapter imports clean. `core/kinematics/` is now **7 files** — the 5-file working
set (§1) plus the two test modules.

The only live-code reference was `robot_manager` → `fk_test`, and it sat inside a
`'''…'''` string, confirmed by AST rather than by grep. That block was labelled
*"Remove once the unified solver is in place"*; it has now been removed, with the
executable AST verified unchanged.

### 8.2 Still on disk, still broken

`check_solver.py` and `mini_solver_check.py` (repo root) were written against an
older API and **crash if run**: they call `get_true_root()` on an `ArmGeometry`,
but that method belongs to `KinematicModel` (`kinematic_model.py:390`), so they
fail with `AttributeError` before reaching their next obsolete access,
`solver.forearm_to_root`. Left in place pending a decision; they are diagnostics,
not part of any import graph.

**Naming trap:** `ur_ik_solver.py` and `ik_solver.py` were easy to confuse, and
restoring the wrong one produced a silent `IK failed (local IK (error))` in the
Cartesian panel. The first is now deleted, but the trap survives in history — if
you ever `git checkout` an old commit's kinematics folder, check which one you
got. The correct adapter contains `UnifiedIKSolver`. As of `32295be` that failure
mode is no longer silent: `SimulatedRobot` logs the exception type and traceback.

---

## 9. What comes next — Hatch beyond IK

IK was stage 5 of the spine-first build order. It is done. The larger gaps are
untouched, and they are where the founding use case lives.

### F1 (highest priority) — the founding use case still has no component

Principle #7 says *"Movements as Models — trajectories, commands and goals are
data, not side effects."* But there is **no path, trajectory, waypoint or
feedrate type anywhere**. Between the IK solver and the command events sits a
void exactly where the welding seam should live.

Proposed `core/motion/`, pure data, no Qt, no VTK, no robot reference:

| Type | Meaning |
|---|---|
| `Pose` | position + rotation vector; one representation, human-readable |
| `CartesianPath` | authored geometry, **arc-length parameterised**, no time in it |
| `Feedrate` | speed intent (mm/s), accel limits, blend/stop policy |
| `Trajectory` | derived: time-stamped joint samples + the pose they realise |
| `PathValidation` | derived: the report saying *why this path is or is not safe to run* |

**Geometry is authored, time is derived, execution is a consumer.** That is what
makes a movement "anticipated, monitored and replayed" literally true.

### F2 — configuration continuity: one third done

Today's work delivered part of it:

1. ✅ **`IKSolution` carries branch tags** (`shoulder`, `elbow`, `wrist`) plus
   singularity flags and FK errors. ❌ **Still missing: a conditioning measure**
   (distance to singularity) on the solution.
2. ⚠️ **Continuation** — `solve_closest` *prefers* the current configuration
   (100× penalty per tag change) but does not **require** it. Along a path it
   must be a hard constraint, not a preference.
3. ❌ **Pre-execution validation** — walking a whole trajectory as data and
   rejecting it for branch flips, joint-speed violations, or near-singularity
   passage, **before anything moves**. Needs `core/motion/` first.

This is the mechanism that makes "intelligence the user didn't ask for"
structurally impossible rather than merely discouraged. The 200-step jog test is
a down payment on it, not a substitute.

### Remaining findings

- **F3** — true-root detection by "first moving joint" contradicts Principle #4
  (*Everything in URDF*). Still open in `kinematic_model.py`.
- **F4** — redundant world → relative → world round trip.
- **F5** — "constant speed" is not free even with rotation vectors; needs
  arc-length parameterisation (belongs to F1).
- **F6** — the REAL-mode gate checks IK accuracy but not **state continuity**.
- **F7** — Principle #2 needs an honest boundary or it condemns its own
  implementation.
- **F8** — tests and configuration are cheaper now than later; the spine is the
  most testable and most load-bearing part.

### Build order, updated

| # | Deliverable | State |
|---|---|---|
| 0 | skeleton, pytest, numpy/scipy | done |
| 1 | `Pose` + rotation-vector math | not started |
| 2 | `StateChannel` + `EventType` | exists; needs tests |
| 3 | `TransformRegistry` | exists; needs tests |
| 4 | `URDFPreprocessor` + `KinematicModel` | exists; F3 open |
| 5 | **analytical IK, 8 branches, flags, continuation** | **✅ done, unified** |
| 6 | `core/motion/`: Path → Trajectory + validation | **next** |
| 7 | `SimulatedRobot` + handlers, headless end-to-end | partial |
| 8 | `KinematicDisplay` + `VisualizerEngine` | exists |
| 9 | `ur_rtde_bridge` + REAL gate | needs hardware; F6 open |

Stage 6 is the recommendation: it is the founding use case, it is pure data, and
it is fully testable headless.

### Two structural proposals, not yet decided

- **A Qt-free `Session`** owning the services, so the user owns `main()`.
  Currently `ui/main_window.py` defines `main()` and constructs `QApplication`,
  which blocks headless applications. This is the precondition for calling Hatch
  a *platform* rather than an application.
- **Record/replay** of the `StateChannel` event stream. `StateChannel` already
  has a 100-event ring buffer with `enable_history=False` by default and no
  recorder. This is the cheapest ROS-parity win (rosbag equivalent) and makes
  "anticipated, monitored, replayed" demonstrable.

---

## 10. Environment

The kinematics kernel needs **only numpy** — `KinematicModel` is numpy + stdlib,
and `verify_ik.py` runs on nothing else. No scipy, no ROS, no PackageResolver
required for plain URDFs.

`requirements.txt` is currently a full `pip freeze` (90 pinned packages including
open3d, vtk, PyQt5, ur_rtde, pyorbbecsdk2, dash, Flask, jupyter). Splitting it into
core / ui / hardware layers would let a fresh environment boot the kernel in
seconds. Not done yet.

**The test suite needs `pytest`, and `requirements.txt` does not list it** — not
among the 90 pins either. Installing from that file therefore leaves you unable to
run the tests. The two modules are pytest-style functions with fixtures, so
`python -m unittest` cannot collect them (it reports import errors, not tests).

```bash
pip install pytest numpy      # the kernel + tests need nothing else
python verify_ik.py                                                   # 31 checks
python -m pytest core/kinematics/test_unified_ik_solver.py \
                 core/kinematics/test_ik_solver.py -q     # 119 passed, 1 skipped
```

Run from the repo root. `numpy` is pinned to 1.26.4 but 2.2.6 is verified working
on Python 3.12 — the pin is not a constraint the kernel actually needs.

Sandbox note: workspace files persist, but `.git`, `.venv` and installed packages
do **not**. If imports fail, re-run `setup-env.sh`; if git commands fail, re-clone.
The repo on GitHub is the anchor, not the sandbox.

---

## 11. Restoring deleted files, and working agreements

### 11.0 If you deleted something you needed

The five files in §1 are the minimum. The two test modules are **not** imported by
the application, so deleting them breaks nothing at runtime and the loss is silent
until you next try to run the tests. `verify_ik.py` lives at the repo root, not in
`core/kinematics/`, and is easy to miss for the same reason.

Restore specific paths from the last known-good commit — this works whether or not
the deletion was committed:

```bash
git checkout eae6401 -- core/kinematics/test_unified_ik_solver.py \
                        core/kinematics/test_ik_solver.py \
                        verify_ik.py
```

Do **not** restore the whole directory (`git checkout eae6401 -- core/kinematics/`):
that also brings back the dead solvers listed in §8.

To see what else is missing compared with that commit:

```bash
git diff --diff-filter=D --name-only eae6401
```

### 11.1 Working agreements

These were established by argument and measurement, and should not be
re-litigated without new evidence.

1. **Verify numerically against the URDFs; never assert.** Every claim in this
   file was measured. Several plausible-sounding claims (including two of the
   maintainer's and three of the assistant's) were disproved that way.
2. **No wrist-type labels.** Frame it as a distance that may be zero.
3. **Refuse out-of-class robots loudly**, naming the violated condition and its
   value. Never approximate, never fall back silently.
4. **URDF-driven.** No hardcoded DH, no hardcoded Euler sequence, no per-arm
   constants.
5. **The triangle's right angle is at B**, not at P5. (Measured: the angle at P5
   is 54.8°.) This was a genuine trap, fallen into once.
6. **A bare `except Exception` that discards a traceback is a defect.** It hid a
   dead dispatcher for months and cost two debugging rounds on a filename mix-up.
7. **Never use `q = 0` as the general test case** — it is singular on all three
   arms.

---

## 12. Open items

- [ ] Delete the dead solvers (§8), pending maintainer confirmation
- [ ] Add a conditioning measure to `IKSolution` (F2.1)
- [ ] Make continuation a hard constraint along paths (F2.2)
- [ ] `core/motion/` — F1, the founding use case
- [ ] Decide the headless `Session` question
- [ ] Split `requirements.txt` into core / ui / hardware
- [ ] F3: true-root detection consistent with Principle #4
- [ ] Log the traceback in `simulated_robot.move_pose` (2-line fix, drafted)
- [ ] No verification against physical hardware or a controller's own IK yet
