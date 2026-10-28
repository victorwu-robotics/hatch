# Hatch Foundation Patches

Five amendments in the document's own voice. Each gives the insertion point,
the text, and one line of reasoning. Review before pasting — the threading
table's third row needs confirmation against the actual driver code.

---

## Patch A — philosophy.md
**Where:** Under "Core Philosophy", as a new subsection immediately after the
quoted definition ("A robot platform should be understood...").

### Scope: Above the Real-Time Layer

Hatch commands trajectories. It does not close servo loops.

The servo loop — the millisecond-scale feedback that regulates torque, reads
encoders, and guarantees stability — always lives in the robot's own
controller. Hatch sits above it: it computes where the robot should go and
hands that intent to hardware that already knows how to go there.

This is not a temporary limitation waiting to be engineered away. It follows
from the platform's own principles. A single-process, pure-Python system is
the wrong tool for hard real-time control, and Principle #1 does not ask it to
be one. "Single process, single memory space" is a statement about data
structures, not about deadlines.

What Hatch must guarantee is different, and harder in its own way: that the
intent it sends is correct, understood, and visible.

Therefore there is no torque control, no force control, no impedance control
in Hatch. Not as a missing feature — as a boundary. These will not be added
by wrapping a library the author cannot yet derive from first principles.
(The author does not yet understand them. Hatch refuses to pretend.)

**Why:** states the boundary the principles imply, so no reader attempts to
close fast loops through the platform.

---

## Patch B — architecture.md
**Where:** Part Four, immediately after the StateChannel component diagram.

### The Threading Contract

Hatch is single-process, not single-threaded. Three threads exist, each for
one reason:

| Thread | Role | What it may do |
|---|---|---|
| Main (UI) thread | Qt event loop, event delivery, StateHandler, render timer | Mutate models and registries. Everything else waits on it. |
| Driver receive thread | Blocks on RTDE/socket reads | Translate hardware packets into events and enqueue them. Nothing else. |
| Driver command thread | Owns connection lifecycle, sends commands | Send the bytes CommandHandler asked for. Nothing else. |

*(If the driver implementation merges receive and command into one thread,
the table says two. The rules below do not change.)*

Four rules follow:

1. **One mutator.** Only StateHandler, running on the main thread, updates
   KinematicModel and TransformRegistry. No callback on any other thread may
   touch them.
2. **Publishers enqueue; subscribers never block producers.** A driver thread
   publishes by handing the event to the channel's queue. Delivery to
   subscribers happens on the main thread. The receive loop never waits on a
   subscriber.
3. **Driver threads are translators, not participants.** They convert bytes
   to events and events to bytes. They hold no business state and read no
   models.
4. **Extensions inherit the same rules.** If an extension needs a thread,
   that thread may only publish events. State changes go through the channel
   like everyone else's.

This section exists because "single process, single memory space" is about
data, not scheduling. Shared memory without a stated discipline is not
simplicity — it is an accident with a timer.

**Why:** "single owner of state updates" was only half the contract; this
states the threading half that makes it true.

---

## Patch C — philosophy.md
**Where:** Amend Principle #4; then new subsection under "The URDF is the
Scene".

**Amendment to Principle #4** — append:

The URDF is **nominal**. A manufacturer URDF describes the series, not your
arm. It is the shared starting point — testable, comparable, portable — and
it is expected to be wrong in small ways for any specific unit.

Calibration is **deployment, not runtime**. Hatch corrects the URDF for the
specific arm before deployment: a documented calibration procedure records
measured deviations (link lengths, joint zero offsets, tool and sensor
mounts) and generates a corrected URDF for that arm. The nominal file stays
untouched. The corrected file carries its provenance and is versioned per
arm. One truth per arm, still one format, one parser.

### Calibration: Correcting the Nominal

The procedure has three steps:

1. **Measure.** Follow the documented procedure for the quantities the
   application depends on. Record everything, including what was not
   measured and why.
2. **Generate.** Calibration output is a parameter file, not a hand edit.
   The corrected URDF is the same xacro instantiated with measured values —
   the preprocessor turns it into the flattened URDF that is loaded.
3. **Deploy.** Load the corrected file. The nominal file remains the
   reference for the series; the corrected file is the truth for this arm.

What URDF cannot express — for example, joint zero calibration that lives in
the robot controller — is documented in the procedure's output, not forced
into the format.

Runtime transform overrides (a registry superseding URDF frames for a
session) remain a designed extension point, not the current mechanism. If
that component arrives, the URDF stays the authored truth and the registry
stays the runtime truth. Until then: calibrate, generate, deploy.

*(Mirror one line into architecture.md Part One, after Principle #4: "The
URDF is the authored truth for the series; the corrected URDF generated at
calibration is the deployed truth for the arm.")*

**Why:** makes the deployment-time correction decision explicit and
repeatable, and turns the xacro preprocessor into the calibration delivery
mechanism instead of a second, hand-edited file.

---

## Patch D — architecture.md
**Where:** Part Three, after the mode transition list.

### Fault Transitions

- `SIMULATE_REAL_IK --CONNECTION_LOST--> SIMULATE_LOCAL`: the virtual robot
  holds its last state; the connection panel shows red. No real motion was
  possible in this mode, so nothing must be undone.
- `REAL --CONNECTION_LOST--> SIMULATE_LOCAL with position sync`: the real
  controller's watchdog stops the hardware. Hatch syncs the virtual robot to
  the last received state and refuses further commands until reconnection.
  The user returns to REAL only through SIMULATE_REAL_IK, as always — the
  gate is not bypassed by an accident.

A lost connection in REAL is not an error dialog. It is a mode transition
with a defined safe endpoint.

*(Add both edges to the state diagram.)*

**Why:** an undefined transition in a safety-relevant state machine is a bug
waiting for a network hiccup.

---

## Patch E — architecture.md
**Where:** Part One, end of the URDFPreprocessor component block.

The preprocessor implements a growing subset of xacro as the URDF takes on
scene-description duty. The supported and refused features are listed in
`docs/xacro_support.md`, which is updated when the subset changes — a
refusal here should be a decision, not an accident of what has been
implemented so far.

**Why:** a growing macro language is a growing formalism; the subset must be
visible to be questioned, per Appendix A.

---

## Deferred (not patched now)

Error handling: agreed, and "at the right time" is fine. The one cheap
interim step worth taking before the full machinery exists — stop swallowing
RTDE exceptions; publish them unconditionally — is a few lines and can land
any session. The principle-level sentence ("the driver never swallows an
error; worst case is a safe stop plus a loud event") should be added in the
same pass as the machinery, not before.
