# HANDOFF-4 — pointer handoff, 2026-11-17

## 0. How to use this file

This is a handoff, not a report. It asserts as few facts as possible on
purpose. Every fact worth having is in a file in this repository; this file
says which file, and what is still undecided.

**Rule for whoever continues: read before claiming.** Every wrong claim in the
session that produced this file came from recall. Every correction came from
opening the file. Do not trust a summary of this repo — including this one.

## 1. State of the repository

In `main`: IK and Cartesian jog complete; `move_pose` frame fix; 48 driver
tests; scene fix; xacro false-warning fix; panel readout fix; IK-source label
removed; three factual doc corrections. Test inventory measured then:
`core/kinematics` 120 + `drivers` 48 + `ui/panels` 61 = 229. With PyQt5
228 passed / 1 skipped; without it 226 passed / 3 skipped. Bare `pytest` at the
root does not collect — scope it to those three directories.

Never transferred, existing only in the assistant's sandbox: a long asset
boundary design note and a proposal file. Their live content is carried in
sections 5 to 7 below, so nothing depends on that sandbox, which has lost its
git history seven times.

**Nothing in sections 5 to 7 is applied. No principle numbering has changed.**

## 2. Read in this order

| # | File | Why |
|---|---|---|
| 1 | `docs/CONSTITUTION.md` | the refusals; Part 3.5 is the kinematic boundary |
| 2 | `docs/philosophy.md` | the why — Origin Story, Prologue, Appendix A |
| 3 | `docs/architecture.md` | the derivation, Need then Principle then Component |
| 4 | this file, sections 5-7 | the live task and the open decisions |
| 5 | `docs/unified_IK.md` | the IK derivation, corrected; the earlier version was wrong |
| 6 | `docs/sensing_funnel_design.md` | the measurement funnel and its accuracy budgets |

`docs/architecture_v1.md` carries a superseded banner. Do not take facts from it.

## 3. Standing corrections — do not flatten these

Stated by the user, several against the assistant's recommendation. Not open
for re-derivation.

- Safety is the **first** need of a robot system. A static mechanism cannot
  produce danger; a moving arm can. Returning the moving part to static as
  fast as possible is the primary protective act, so a stop is mandatory.
- **Correctness is not safety.** Correctness belongs to the command, safety to
  the situation. Do not imply the controller guarantees safety either; its
  functions reduce risk, they do not remove it.
- Hatch must admit it cannot guarantee safety, and supply tools that reduce harm.
- Principles are numbered **in the order the derivation discovers them**, as
  the Needs appear in `architecture.md`. Ordering by dependency was rejected
  as a decree.
- The right angle of triangle ABC is at **B**, not at P5. Disproved and accepted.
- P4 = A = the wrist root, defined geometrically, origin-independent.
- J4 parallel to J6 is the wrist singularity. Do **not** name two wrist types;
  present it as a distance that may be zero.
- **No quaternions** in the public API. Hidden internal use permitted.
- **No numerical IK solver.** Refused.
- **No timestamps, no tf.** Single process, single memory space: no message
  passing, no serialization.
- Headless *operation* rejected — preview is a safety gate. A headless-capable
  *kernel* is fine.
- Pure Python is a **means, not an end**. The end is no C++ environment setup.
- One arm per scene is about **assets**, not robots. Reword, do not delete.
- Asset boundary: the arm ships a raw URDF with no attachments. The arm is the
  file owning the 6-revolute chain; the flange is that file's chain end;
  anything spliced from another file is an attachment. Each asset's URDF is
  authoritative about itself. Tool transforms come from supplier data or
  calibration, never from inference.
- The `bunker_to_ur10` mount `rpy="0 0 ${pi}"` is correct. Keep it.
- Do not modify `my_orbbec.urdf`.
- A warning that stands between the user and their goal will be bypassed; the
  design is at fault, not the user.
- Keep strategy and market reasoning out of the public repo.
- Do not push without explicit authorization. The user pushes from their PC.
- The user does their own testing and wants results on their own PC.
- Discussion before implementation. Do not push toward coding.
- Base is static but may be offset from world. At most one arm per scene; a
  second is refused. Many attachments, each with its own TCP; the active TCP
  switches by task phase.

## 4. Delivery — four channels are dead, do not retry

Confirmed failures: `.patch` download; the file viewer (Bad Gateway); the HTTP
preview proxy (502); editor paste (auto-indent joins statements into a
SyntaxError). **Only an inline terminal heredoc works.** If a patch must carry
markdown code fences, build them programmatically; never nest triple
backticks inside a pasted heredoc — it truncates the paste and leaves the
shell at a continuation prompt. The multiple-choice question widget blocks
the user's input; ask in plain prose.

## 5. The live task — the two documents

### 5.1 The alleged contradiction is a truncation, in opposite directions

`philosophy.md:208` says every component exists because a **principle**
demanded it. `architecture.md:731` says each component was demanded by a
**need**. `architecture.md:11` states the whole chain correctly.

The chain is need then principle then component. Philosophy states the second
link; architecture's closing line states the first and last and drops the
middle. Neither is false. Philosophy's reads as a decree only because it makes
the principle the origin instead of the middle. One clause repairs it.

Chronology is not the test of a derivation. A derivation is a logical order,
not a diary. The test: **can I name a need the beginner has before knowing the
principle, and independently of it?** If not, it is a decree whatever number
it carries.

### 5.2 The layout testifies against itself

A derived principle sits under the Need that demands it. Four sit under none:

| Principle | Line | What precedes it |
|---|---|---|
| #0 Individuals Before Groups | 78 | a Component, not a Need |
| #8 Pure Python | 261 | another Principle |
| #1 Single Process | 386 | a Component |
| #10 One Robot Per Session | 426 | a Component |

Also: #6 shares #2's Need verbatim, so it is not independently derived; and #7
appears after the Component that already answered its Need. Five principles
are clean: #4, #5, #3, #2, #9.

The four orphans are exactly the four the semantic test called decrees. Two
independent tests agree, so this is not a matter of taste.

### 5.3 The needs column, in a beginner's words

Numbering is the proposed discovery order, safety first. The verdict judges
the **need as currently written** in `architecture.md:611`, not the principle.

| New | Principle | The need as the beginner has it | As written | Verdict |
|---|---|---|---|---|
| 0 | The Platform Is Not a Safety Controller | the arm must not hurt anyone, and I must be able to stop it | absent | new |
| 1 | Everything in URDF | I must describe what is in front of me | describe the scene | derived |
| 2 | Individuals Before Groups | there is one arm in front of me and I want to understand it | one robot, one session | circular |
| 3 | Space = TransformRegistry | where is the tip, and where is the camera, right now | know where everything is | derived |
| 4 | Visualizer as Mind-Prying Tool | I must see what the robot is, not what I assume | see the robot's true state | derived |
| 5 | Movements as Models | I must see what the arm will do before it does it | commands as data | mis-stated |
| 6 | Nothing to Compile | I must be able to read and change every part myself | rapid development | **decree** |
| 7 | Event-Driven, No Polling | when the robot moves the picture must move unasked | decoupled communication | mis-stated |
| 8 | Time = StateChannel | I must be able to find out what happened, in order | components must communicate | duplicate of 7 |
| 9 | Single Process, Single Memory Space | what I see must be the robot's state, not a copy | no serialization overhead | mis-stated |
| 10 | UI Separate from Services | the controls must not be a second place decisions are made | controls without coupling | mis-stated |
| 11 | One Arm Per Scene | I must never be unsure which arm and which tool I command | clean boundaries | **decree** |

Every principle survives. The defect is the column, not the architecture.
Title of #6 still unapproved. Safety at #0 displaces Individuals Before Groups
from #0 to #2 in `philosophy.md`; the alternative is to state Individuals as
the premise before the numbering, since the prologue already asks to control
**a** robot, singular. Undecided.

### 5.4 One question per document

- `philosophy.md` — why does this exist, and why should I trust it?
- `architecture.md` — what must exist, and in what order does the need arise?
- `CONSTITUTION.md` — what do we refuse?

Test for any paragraph: which question does it answer? If it answers another
document's question, it moves. Measured against that test, **174 of
philosophy's 474 lines answer architecture's question**: The Ten Principles
(214-260, 47), The URDF is the Scene and The True Kinematic Root (262-309, 48),
Future Extension: Dynamic Objects (311-342, 32), Appendix B Known Gaps
(414-460, 47). The last three already exist in architecture.

This is deletion and redirection, not re-authoring. The genuinely *why* prose —
Origin Story, Prologue, Appendix A, about 250 lines — is untouched. Do not
rewrite these documents from scratch; that was considered and rejected.

### 5.5 Sequence of philosophy.md

Narrative order is reasoned order here: a reader cannot accept a refusal
before feeling the wall that produced it.

Keep, in order: Origin Story; Prologue: On Understanding; Core Philosophy;
Appendix A: On Formalism; one honesty paragraph pointing at architecture's
limitations; one Closing.

Remove or move: The Ten Principles (replace with the chain and a pointer);
The URDF is the Scene (to architecture, its two tables to the constitution as
rules); Future Extension (already in architecture Part Seven); Appendix B
(facts about code belong in one list, which then cannot contradict itself).

The bridge near line 197 says the experiences "shaped ten principles". The
count is stale and the list is leaving; it should hand the reader to
architecture. The document currently ends three times, at 178, 410 and 461.

**Safety is already in the Origin Story, unnamed.** The Descartes episode —
two correct IK solutions, both reaching the seam, the arm passing through the
workpiece between them, "the planner avoided a collision that didn't exist by
creating a collision that did" — is the need for #0, felt first-hand. It is
told as insubordination. It is also harm. Naming it there makes #0 the user's
own first wall rather than an import from a standards document.

### 5.6 Sequence of architecture.md

1. Need #0 before Part One. Every other need assumes the user survives it.
2. **A principle discovered early may demand components late.** #0 is stated
   first; its components sit where they belong — the mode gate in Part Three,
   already written as "Key safety property", a reachable stop, the controller
   watchdog. Early principle, late component is not inconsistency.
3. Part Six, the Render Loop, folds into Part Two: its need is no-polling
   applied to rendering, not a new discovery.
4. Derivation, then a divider, then reference. The principles summary is the
   conclusion of the argument and belongs immediately after the last Part,
   not among the lookup tables.
5. **Open, not decided:** where is decoupling first felt? Part Four derives it
   after Part Three has already used events. The need arises when a second
   consumer of robot state appears. Either move Communication earlier, or
   keep it and name the need at that moment. Both are reasoned. The user chooses.

### 5.7 Contradiction and duplication register

| # | Item | Where | Resolution |
|---|---|---|---|
| 1 | **live contradiction:** "no mechanism to override URDF transforms" vs "a mechanism does exist: `set_tool_transform`" | arch 550 vs phil 450 | apply both together |
| 2 | #5 carries a clause the other copy lacks: dynamic frames vs callbacks | phil 238 / arch 184 | one home |
| 3 | #6 "Timestamps preserve sequence" — false, dispatch is inline | **both**, arch 362 | correct once |
| 4 | Closing quote and "does one thing well" sentence identical | phil 461 / arch 724 | different endings |
| 5 | The True Kinematic Root explained twice | phil 288 / arch 167 | architecture |
| 6 | Dynamic Objects extension point twice | phil 311 / arch 509 | architecture |
| 7 | Limitations listed twice | phil App B / arch Part Eight | architecture |
| 8 | "The Ten Principles", stale count, will be twelve | phil 214, arch 611, const 14 | "The Principles" |
| 9 | "See philosophy.md for the origin of each" — origins are in architecture | const 17 | repoint |
| 10 | Directory tree lists `drivers/robot_arm/`; no such directory | arch 672 | files are in `drivers/` |
| 11 | Philosophy ends three times | 178, 410, 461 | one closing |
| 12 | "Calibrated transforms (future)" vs an existing mechanism | phil 286 vs 450 | two mechanisms, conflated |

Item 1 exists because the philosophy correction was applied while its
architecture pair was not. **A cross-document pair must be applied
atomically**, exactly like the renumbering: 11 headings, 18 number references
in the service and component tables, and the summary, all at once.

The exact replacement for architecture's Sensor Calibration paragraph, to be
applied together with the philosophy text already in main:

> Camera extrinsics are assumed to match the URDF; there is no hand-eye
> calibration and no way to refine them.
>
> For the tool, the override mechanism exists — `set_tool_transform`, honoured
> by the IK adapter and tested — but nothing produces the value: no calibration
> procedure, no persistence, no UI. For applications requiring precise spatial
> accuracy, pre-calibrate your URDF transforms before loading.
>
> The designed approach is recorded in `sensing_funnel_design.md`.

## 6. Open code items, ranked

1. **Before any real motion.** Three docstrings call `stopScript()` an
   "Emergency stop": `ur_rtde_bridge.py:404`, `real_robot.py:158`,
   `base_robot_arm.py:96`. The safety principle forbids exactly this confusion.
2. **There is no robot stop control in the UI.** `RobotManager.stop_robot()`
   has zero callers repository-wide; the only Stop buttons stop cameras.
   `SimulatedRobot.stop()` is a no-op and sim motion is instantaneous, which is
   why the gap has never been felt. The stop must exist before the first
   continuous motion, not after.
3. `drivers/real_robot.py:127` has the same frame defect that was fixed in the
   simulated driver. **Do not copy the fix** — the UR controller's base-frame
   convention is not established.
4. `#6 Time = StateChannel` claims timestamps preserve sequence. `publish`
   dispatches synchronously inline, so call order preserves sequence, and
   `time.time()` can step backwards.
5. `_on_mode_switched` REAL-branch readout defect in the Cartesian panel.
6. Revisit the IK-source indicator now that the label is removed.
7. Controller-side mechanisms Hatch does not use, documented in ur_rtde, not
   measured here and not safety-rated: `setWatchdog(min_frequency)` shuts down
   control if not kicked, so a hung Hatch still stops the arm;
   `triggerProtectiveStop()`; `isPoseWithinSafetyLimits()`.

## 7. Open decisions and the measurements they need

Settled: D7, D9. Open: D1-D6, D8, D10, D11, D12. D4 is blocked on D11 and D12.
The full boundary note was never transferred; these are the measured facts the
decisions rest on. Re-measure before relying on them.

- **The TCP slot is never filled.** `tool_mount_link` is `wrist_3_link`;
  `get_tool_transform()` is identity; `set_tool_transform` is called nowhere in
  application code, only by `core/kinematics/test_ik_solver.py:149`. `my_arm` declares
  `torch_tip` at 382.001 mm and 1.056 degrees from the flange, ignored. Tip
  travel is 6.667 mm per degree of flange rotation, against a 0.2 mm groove
  tolerance. Adding the link alone accomplishes nothing; filling the slot is
  the change that matters. This is what makes D4 concrete.
- **Raw URDF past the last moving joint:** UR10 `ft_frame`, `flange`, `tool0`
  all at +0.000 m; FR5 `tool_Link` +0.100 m with a mesh; Elfin `elfin_end_link`
  +0.200 m with a mesh. The flange walk breaks unless a link has exactly one
  child, so UR10 is safe only because it ships two children. A coaxial 250 mm
  tool is silently absorbed; a tilted tool is refused.
- **A compliant `xacro:include` is indistinguishable from an inline hack.**
  Expanded links carry only a name; provenance is known at
  `utils/xacro_expander.py:320-322` and discarded. The prototype rule — the arm
  is the file owning the 6R chain — fixes all three failures and leaves the
  bare arm bit-identical.
- **An attachment asset must not declare its own root joint.** `base_link` is
  declared by three assets; `camera_link` gets two parents, putting the camera
  1048.25 mm from the flange against a declared 210.99 mm.
- **TCP switching costs:** torch 381.4, scanner 395.4, camera 205.0 mm from
  `wrist_3`. Wrong TCP costs 63.8 / 181.1 / 205.0 mm and 179 / 90 / 90 degrees
  of approach axis. `d435_link` to `d435_depth_optical_frame` is 0.000 mm and
  120 degrees.
- **Static base offset works exactly:** 8 branches, 9.705e-09 mm across four
  mounts; the world-to-base transform is q-independent, 0.000e+00 drift over
  200 random q. This is why the frame conversion needs no timestamp. UR10 ships
  `world` with its fixing joint commented out, so offsetting `world` alone is
  silently ignored.
- **Sensing funnel:** A about 1 m and 10 mm; B about 40 mm and 0.2 mm; C arc on.
  Measure between passes with the arc off. Capture settles
  `(points, q_at_capture, T_flange_optical)` at capture time. A stale pose at
  50 mm/s over 40 ms is 2.00 mm. D435 RMS 1.85 / 3.77 / 8.49 / 15.09 mm at
  0.7 / 1.0 / 1.5 / 2.0 m. Pivot calibration with 0.2 mm noise: 3, 4, 6 poses
  give 0.4460 / 0.1828 / 0.1669 mm.
- **IK core, unchanged:** worst pose error 1.9e-11 / 7.35e-7 / 5.50e-6 m for
  UR10 / FR5 / Elfin; reach 1.2765 / 0.92001 / 1.5000 m; `c6` is +pi/2 for UR10
  and FR5, -pi/2 for Elfin. **Test the pose, not joint vectors.**

## 8. First question to settle when work resumes

The needs column in 5.3, and the one-question test in 5.4. If those are right,
the sequences follow from them and nothing is left to invent.
