# Cartesian Jog Control — Design Note

**Status: IMPLEMENTED.** Semantics agreed with the maintainer, then built.
Drafted against `main @ 9c8797f`; the panel rewrite lands on top of `6353827`.
Run by the maintainer against the visualizer, and covered by
`ui/panels/test_cartesian_control_panel.py` (58 tests, headless).
The findings below describe the **previous** panel and are kept as the record of
why the change was made.

Scope: `ui/panels/cartesian_control_panel.py` and the two contracts it touches
(`CARTESIAN_COMMAND`, `RobotInterface.move_pose`). Not a motion-planning document —
no paths, trajectories or feedrates are defined here.

---

## 1. Why this is needed

Findings from reading the panel, each verified by running its own methods:

- **Base and Tool are different mechanisms.** Base treats the sliders as absolute
  extrinsic-XYZ Euler coordinates (`as_euler('xyz')`, so `R = Rz·Ry·Rx`). Tool
  applies incremental rotations and springs back. One set of controls, two meanings.
- **The Base tooltip is false as a description of behaviour.** It says *"Absolute
  Euler angles around fixed base axes"*. Measured at (20°, 35°, 50°): dragging RX
  rotates about **Tool X**, RZ about **Base Z**, and RY about **neither** — an
  intermediate axis that swings with RZ.
- **RY's range is wrong for the chart.** The slider spans ±180°; the middle angle of
  an extrinsic-XYZ chart spans only ±90°. Dragging RY to 120° reads back as RY=60°
  with RX and RZ flipped to 180°.
- **Gimbal lock at RY=90°.** scipy forces the third angle to zero: (10°, 90°, 20°)
  reads back as (−10°, 90°, 0°), and RX/RZ then act about the same axis.
- In both cases **the pose is preserved**. This is not pose corruption and not an IK
  fault. The orientation is correct; the numbers jump and the axis labels lie.
- **`move_pose(pose, frame)` ignores `frame` entirely** — accepted, documented, never
  read. Its docstring says `"base" or "world"`; the panel sends `"base"/"tool"`.
- **Latent frame mismatch.** The panel builds the target in **base** coordinates;
  `solve_ik_for_tcp` expects **world**. Base coincides with world on all three
  current arms, so it is invisible today.

The IK solver is not implicated: it reproduces commanded poses to ~1e−11 m.

---

## 2. Decisions taken

1. Position and orientation remain **separate groups**.
2. Each group has its **own frame selector**, switched independently.
3. **Position**: Base = absolute values; Tool = incremental deltas.
4. **Orientation**: incremental deltas in **both** frames.
5. Labels become `along X/Y/Z` and `about X/Y/Z`.
6. Orientation is reported as a **rotation vector**, with RPY secondary.
   **No quaternions** — Constitution Part 7.

---

## 3. Definitions

- **TCP** — origin of the tool frame. `get_tcp_pose()` returns `mount_pose @ tool_transform`.
- **Base frame** — the robot's true kinematic root. Fixed in the world.
- **Tool frame** — the TCP frame. Its axes move with the robot.
- **Rotation vector** — `axis × angle`, e.g. `[1.57, 0, 0]` is 90° about X.
- `R₀`, `p₀` — orientation and TCP position **at the moment a drag begins**.
- `A(δ)` — rotation by δ about one coordinate axis.

---

## 4. The four modes

| Group | Base frame | Tool frame |
|---|---|---|
| **Position** | absolute value slider; stays where put | delta jog; springs back to centre |
| **Orientation** | delta jog; springs back | delta jog; springs back |

Four independent combinations. Three of the four spring back.

---

## 5. Exact semantics

| Mode | Rule |
|---|---|
| Position / Base | `p = slider value` (base coords); `R` untouched |
| Position / Tool | `p = p₀ + R₀ · δ`; `R` untouched |
| Orientation / Base | `R = A(δ) · R₀`; `p` untouched |
| Orientation / Tool | `R = R₀ · A(δ)`; `p` untouched |

**Anchor every jog at press.** `R₀`/`p₀` are captured when the drag starts and the
slider reads 0. The pose is computed from the anchor and the current delta, not by
accumulating successive event deltas. This makes a drag exactly reversible and
immune to event quantisation.

**Rotation is about the TCP origin**, so an orientation jog never translates the TCP.

---

## 6. Invariants — these become the acceptance tests

Measured on a representative pose (RPY 20/35/50, TCP 0.4/0.1/0.6):

| # | Invariant | Measured |
|---|---|---|
| I1 | Orientation jog leaves TCP position unchanged | exact |
| I2 | Tool position jog leaves orientation unchanged | exact |
| I3 | Tool position jog direction equals `R₀[:,k]`, magnitude equals `|δ|` | 20.0000 mm for 20 mm |
| I4 | Base orientation jog axis is exactly `e_k` | `[1,0,0]` |
| I5 | Tool orientation jog axis is exactly `R₀[:,k]` | `[0.527, 0.628, −0.574]` |
| I6 | Returning a jog slider to centre restores the pose at press | 1e−15 |
| I7 | TCP position expressed in its own frame is zero | 5e−17 |
| I8 | Base position jog and orientation jog **commute** | 0.000 mm |
| I9 | Tool position jog and orientation jog **do not commute** | 2.856 / 3.486 mm |

**I9 is a property, not a defect.** "Along tool X" points somewhere new once the tool
has been reoriented, so in Position/Tool mode the *order* of the user's jogs is part
of the result. This must be documented and shown, not engineered away. Base-frame
translation is immune because base axes do not move.

Joint-vector comparison is the wrong test, as with IK: **assert the pose.**

---

## 7. Ranges derived from the loaded URDF

The current `position_limits = (-1.5, 1.5)` is hard-coded and robot-independent.
Maximum planar reach `L2 + L3 + |d6|`, from the verifier:

| Arm | L2 | L3 | d6 | Reach | Current ±1.5 m |
|---|---|---|---|---|---|
| UR10 | 0.6120 | 0.5723 | +0.0922 | **1.277 m** | too large |
| FR5 | 0.4250 | 0.39501 | +0.1000 | **0.920 m** | 63% beyond reach |
| Elfin | 0.7300 | 0.5700 | +0.2000 | **1.500 m** | coincidentally right |

On FR5 most of the slider travel commands unreachable poses.

Rules:
- Absolute Base position range derived from the URDF, not a constant.
- Jog full-scale a fraction of that reach (default suggestion 10%), **user-adjustable**
  via the existing step control.
- Orientation jog full-scale a chosen angle per drag (default suggestion 30°),
  user-adjustable. Not ±180°, which is an absolute range, not a jog range.

---

## 8. Clipping and refusal

Available space is the intersection of the **project workspace** and **what the robot
can reach**. But:

**Robot reach is not a box.** It is roughly an annulus, with a dead zone near the base
column, and reachability depends on orientation as well as position. A point inside
the robot's bounding box can still be unreachable. So this cannot be computed as the
minimum of two boxes.

Therefore:
- **Clip to the project box** — a genuine box, cheap and exact.
- **Let IK refusal handle reachability.** That machinery exists and is loud.
- Do not approximate the reachable set as a box; it would give false confidence.

The project workspace is new state that does not exist anywhere yet. Constitution
Part 7 forbids *"configuration file formats before pain points are proven"*, so keep it
as in-session UI entry until persistence is genuinely needed. It is the natural seed
of `PathValidation` when `core/motion/` is eventually built.

---

## 9. Failure policy — the risk this design introduces

**Jogs hide state.** After a drag the slider reads 0, so it carries no information
about where the target is. Today `target_pose` absorbs every change; if IK refuses, an
error is published but the target keeps the unreachable value. With an absolute slider
the user can see that stuck value. **With a jog, the failed delta becomes invisible**
and target diverges from reality silently. `_sliders_moved` resets only on
`ROBOT_LOADED`, so it will not self-heal.

Required, not optional:
1. A **persistent target readout**, visible in every mode.
2. A **defined refusal policy** — either roll the target back to the last achieved pose,
   or make the divergence unmistakable in the readout. *Decision outstanding.*

This is the one respect in which jogs are genuinely worse than value sliders, and it
must be paid for explicitly.

---

## 10. Readout

- Show **target** and **current** separately. The existing group shows current only.
- Orientation as a **rotation vector** (primary), RPY secondary and labelled
  `extrinsic XYZ`.
- No quaternions anywhere in the public API. scipy may use them internally —
  philosophy.md permits them "hidden internally".
- Caveat: RPY still jumps at RY = ±90° even as a display, because the chart is
  degenerate there. Harmless, but the rotation vector is the stable truth.

A jog *is* a rotation vector, so readout and control speak the same language.

---

## 11. UI details

- Labels: `along X/Y/Z`, `about X/Y/Z`. Group headers name the frame and the mode,
  e.g. *"Position — Tool (incremental)"*.
- Jog sliders: centre detent tick, `Δ` prefix on the value label, distinct groove colour.
  Value sliders: no centre mark.
- Set `pageStep` to the step size on jog sliders so a groove click nudges one step
  instead of jumping a page.
- **Wheel recentring needs an idle timer.** Verified on Qt 5.15.14 with signal counts:

  | Action | sliderPressed | sliderReleased | valueChanged |
  |---|---|---|---|
  | Wheel notch | 0 | **0** | **1** |
  | Handle press + release | 1 | **1** | 0 |

  The wheel moves the arm through `valueChanged` → handler → 100 ms debounce → publish.
  It never emits `sliderReleased`, so mouse-release spring-back does not cover it.

---

## 12. Frame-switch transitions

- **Tool → Base**: populate sliders from the current target.
- **Base → Tool**: recentre to 0. This **does not change the pose** — only the pending
  delta zeroes. Keeping the old slider position would display a delta that was never
  applied, give asymmetric travel, and be discarded at the first release anyway. The
  absolute orientation remains visible in the readout, which is where that knowledge
  belongs.

---

## 13. Fate of existing methods

| Method | Disposition |
|---|---|
| `_extract_euler_angles_base` | keep, **readout only** — out of the slider path |
| `_apply_euler_angles_base` | **remove** — this is the chart that causes the symptoms |
| `_apply_tool_incremental_rotation` | generalise to both frames (pre/post multiply) |
| `_on_orientation_slider_changed` | one jog path, no frame branch in the maths |
| `_on_orientation_step` | jog in both frames |
| `_on_slider_released` | spring back all jog sliders, plus wheel idle timer |
| `_on_frame_changed` | one per group |
| `_update_orientation_slider`, `_update_all_orientation_sliders` | drive the readout, not the sliders |
| `_angle_slider_to_value`, `_angle_value_to_slider` | map jog scale, not ±π |
| position handlers | absolute in Base, jog in Tool |
| `_publish_command` | pose is already resolved to base; the `frame` string is unused downstream — keep for logging or drop. *Decision outstanding.* |
| `_sliders_moved` | revisit together with the refusal policy |

Net effect: the folding, the gimbal-lock jumps and the RY range mismatch are **deleted**,
not patched.

---

## 14. Out of scope

Absolute numeric orientation entry; persistence of the project workspace; paths,
trajectories, feedrates; perception-derived geometry; `core/motion/`.

---

## 15. Open decisions

Three of the four were settled by the implementation; the values below are what
the code does, not a proposal.

1. ~~Refusal policy~~ — **loud indicator, no rollback.** A refusal sets the status
   line and colours the target readout; the target is never silently rewound. (§9)
2. ~~Default jog full-scale~~ — position **10% of the URDF-derived reach**,
   orientation **±30°**. Both are user-adjustable. (§7)
3. **Still open:** keep or drop the unused `frame` argument in `CARTESIAN_COMMAND` /
   `move_pose`. It is still accepted and still ignored. (§13)
4. ~~Wheel idle timeout~~ — **500 ms** before the handle recentres. (§11)

Also still open, and not addressed by this change: the panel builds **base**-frame
poses while `solve_ik_for_tcp` expects **world**. They coincide on all three arms
today, so the defect is latent (§13).

---

## 16. Compliance

- Constitution Part 7: no quaternions in the public API — rotation vectors only.
- Constitution Part 7: no configuration file format before the pain point is proven.
- Principle #0: single robot, single process. Nothing here introduces coordination.
- Verification discipline: assert poses, not joint vectors; every claim measured.
