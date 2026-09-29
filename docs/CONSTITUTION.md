# Hatch Constitution

> *"A platform is not defined by what it can do. It is defined by what it
> will not do — and why."*

This file is the operational contract for Hatch. It is short on purpose.
It is allowed to be terse. It is not allowed to be wrong.

Every commit that changes behavior must check this file first. If the change
affects a rule in this file, the rule must be updated in the same commit.

---

## Part 1 — The Ten Principles

Each principle was discovered from a need, not chosen from convention.
See `philosophy.md` for the origin of each.

| # | Principle | In one sentence |
|---|-----------|-----------------|
| 0 | Individuals Before Groups | One robot per session. Multi-robot is composition, not core. |
| 1 | Single Process, Single Memory Space | No serialization, no distributed nodes, no network overhead. |
| 2 | Event-Driven, No Polling | Components communicate via events. No busy-waiting loops. |
| 3 | Visualizer as Mind-Prying Tool | The 3D view reads state; it does not control. |
| 4 | Everything in URDF | The URDF is the single source of truth for the scene. |
| 5 | Space = TransformRegistry | All relative poses in one place, lazily evaluated. |
| 6 | Time = StateChannel | All events in one place, publish/subscribe, timestamped. |
| 7 | Movements as Models | Trajectories and commands are data, not side effects. |
| 8 | Pure Python | No C++ except VTK bindings. Qt only in UI and driver bridges. |
| 9 | UI Separate from Services | UI publishes and subscribes. It holds no business logic. |
| 10 | One Robot Per Session | To change robots, restart the application. |

---

## Part 2 — The Purist's Question

Before adding any dependency, formalism, or abstraction, ask:

1. Do I understand this fully, or only how to use it?
2. Is there a simpler representation that is more human-understandable?
3. Am I solving a problem I actually have, or someone else's problem?
4. If this disappeared tomorrow, could I rebuild it from first principles?

If any answer is "no," the addition does not belong in Hatch.

---

## Part 3 — Repository Structure
hatch/
├── core/ Services: no Qt, no UI, no visualization.
│ ├── kinematics/ URDF parsing, FK, IK.
│ └── world_state/ TransformRegistry, StateChannel, EventTypes.
├── drivers/ Robot hardware bridges and simulation.
├── displays/ VTK visualizations. Read-only observers of state.
├── viz/ VTK render window, camera, grid. The render engine.
├── ui/ Qt widgets. Presentation only. Publishes events.
├── utils/ Shared utilities: PackageResolver, XacroExpander.
├── assets/ Default shipped assets (see Part 4).
├── docs/ All documentation, including this file.
└── tests/ Automated tests (when they exist).


**Dependency rule:**
- `ui/` may import from `core/`, `displays/`, `drivers/`, `utils/`.
- `displays/` may import from `core/`, `viz/`, `utils/`.
- `drivers/` may import from `core/`, `utils/`.
- `core/` may import from `utils/` only. Never from `ui/`, `displays/`, `drivers/`.
- `utils/` may not import from `core/` or anything above it.

If you need an exception, the exception is a design smell. Document it here.

### Event payload contracts

Every `EventType` in `core/world_state/event_types.py` must document its
payload schema as a comment next to the constant. Example:

    ROBOT_LOADED = "robot_loaded"
    # Payload: { asset_id: str, urdf_path: str, kinematic_model: KinematicModel }

Publishers must emit exactly these keys. Subscribers must read exactly these
keys. No string literals for keys; use the documented names.

---

## Part 4 — Asset Resolution

### The default assets location

`~/hatch/assets/` is where Hatch's own curated assets live. It is **always
searched**, regardless of `HATCH_PACKAGE_PATH`.

### The user override

`HATCH_PACKAGE_PATH` is a colon-separated (Unix) or semicolon-separated
(Windows) list of directories to search **in addition to** `~/hatch/assets/`.

Resolution order:
1. `~/hatch/assets/` (always)
2. Each directory in `HATCH_PACKAGE_PATH`, in order.

If `HATCH_PACKAGE_PATH` is unset, only `~/hatch/assets/` is searched.

### The URI form

Hatch accepts both `package://<name>/...` and `package://<category>/<name>/...`.
The resolver searches recursively and finds packages by name.

URI form is a consequence of where the user pointed the resolver, not a rule
Hatch imposes.

### The default assets (Tier 1)

Curated, tested, and guaranteed to load:
- Robots: UR10, FR arms, [third arm TBD]
- Sensors: Intel RealSense D435

Everything else in `assets/` is either:
- **Experimental** — present, may not work.
- **Example** — used by docs, not shipped.

---

## Part 5 — Naming Conventions

### Files
- Python modules: `snake_case.py`
- Markdown docs: `snake_case.md` in `docs/`
- URDF files: `<robot_name>.urdf` for singles, `<robot_name>.urdf.xacro` for
  composed scenes.

### Frames
- `<asset_id>_<link_name>` for registered frames. Example:
  `ur10_nominal_base_link`.

### Commits

`<scope>: <one-line summary>`

`<why this change was needed>`
- Principles: #`<n>`, #<`n>`
- Docs: `<files updated>`
- Smoke test: `<what was loaded, what passed>`


The `Smoke test:` line is mandatory for any commit that changes code.

---

## Part 6 — The Smoke Test

Before any commit that touches code, run:

```bash
python -c "import core.robot_manager; import core.kinematics.kinematic_model; print('imports ok')"
python -m ui.main_window
```

The smoke test must verify:

The import check prints imports ok.

python -m ui.main_window launches without a stack trace.

Loading assets/robots/ur_description/urdf/ur10_nominal.urdf completes
and the joint sliders appear in the Motion Control dock.

Loading assets/scenes/my_arm.urdf.xacro completes and the joint
sliders appear.

No ERROR or WARNING lines appear in the console, except the known
URIKSolver base_offset warning (tracked separately).

If any of these fail, the commit is not ready.

This is the single rule that catches the most bugs. Every merge wound in the
2026-09 session was caught by this test.

Part 7 — Forbidden in This Repository
Quaternions in the public API. Rotation vectors only.

Polling loops anywhere in core/.

Configuration file formats before pain points are proven.

Multi-robot coordination (Principle #0).

Motion planning (unless added as an extension, per philosophy.md).

Silent fallbacks. Every fallback logs at WARNING or higher.

Silent early returns from event handlers. If a required payload key is
missing, log a WARNING with the event name and the missing key.

String literals for event payload keys in publisher or subscriber code.
Keys are part of the event contract and must be documented in
event_types.py.

Committing code that references an undefined name.

Committing a function signature change without updating every call site.

Part 8 — Required
Every source file has a header docstring stating its purpose and the
principle(s) it serves.

Every new public function has a /docs/api entry (once /docs/api exists).

Every change to behavior updates this file if a rule is affected.

Every commit updates CHANGELOG-harness.md.

Every fallback logs why it fell back, with enough context to identify the
specific input.

Part 9 — Known Gaps
Hatch is honest about what it does not do. These are tracked, not forgotten:

Error handling: no severity levels, no structured recovery.

Configuration management: no config file format yet.

Sensor calibration: no hand-eye, no TCP, no intrinsics.

Automated tests: minimal, not organized under a test runner.

Robot topology: serial chains only; no parallel or closed-loop robots.

URI prefix handling: package://robots/... works, but the prefix is a
historical artifact; may be normalized in the future.

IK solver: URIKSolver.__init__ keyword mismatch (base_offset) tracked
as the next harness task.

Part 10 — How This Document Changes
This document is not sacred. It changes when a rule is proven wrong.

When you change this document:

Change one rule at a time.

Explain why in the commit message.

Update the affected code and docs in the same commit.

A constitution that is not enforced is worse than no constitution — it
teaches the reader to ignore rules. Enforce or delete.

Part 11 — Worked Examples
Example 1: Fixing the ROBOT_LOADED payload mismatch (2026-09-29)
Symptom: After loading a robot, joint sliders did not appear.
The 3D view rendered the robot correctly.

Diagnosis:

core/robot_manager.py published ROBOT_LOADED with key 'model'.

ui/main_window.py:_on_robot_loaded read key 'kinematic_model'.

The subscriber's guard if model is None: return skipped
MotionContainer creation without logging anything.

The mismatch was invisible because nothing failed; the handler
just returned early.

Fix: Change the publisher to emit 'kinematic_model' (matching
the semantic name used by the subscriber and the class KinematicModel).

Principle served: #2 (Event-Driven), #6 (Time = StateChannel).

Rule derived: Event payload keys are a contract. Document them in
event_types.py. Forbid silent early returns from event handlers.

Hatch (孵) 🐣 — built to understand, built to see.


