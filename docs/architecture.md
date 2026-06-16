# Hatch (孵) Architecture Document

## A Derived Architecture

> *"I need to control a robot. What must exist for that to happen?"*

This document does not describe a system that was designed in advance.
It traces a chain of needs — each one demanding the next — until a
complete platform emerges.

Every component in Hatch exists because a need required it. Every principle
was discovered, not decreed. This is what it means to be a **derived architecture**.

---

## Part One: The Scene

### Need: I must describe my robot and its world

Before anything moves, before anything is visualized, before any control
panel exists — there must be a description of what is in the scene.

A robot is not one thing. It is a chain of links connected by joints.
It carries tools. It sits on a table or a mobile base. It has sensors
mounted to its wrist. All of these things exist in spatial relationship
to each other.

We need a format that can describe:
- Links (rigid bodies with geometry)
- Joints (connections between links, some fixed, some moving)
- Their positions relative to each other
- Their visual appearance (meshes)

The URDF format does this. It is the robotics community's standard for
robot description. We adopt it — not because ROS uses it, but because
it solves the need.

### Principle #4: Everything in URDF

All components — robots, sensors, tools, fixtures, tables, AGVs — are
described by URDF. The URDF is the single source of truth for the entire
scene. There is no separate world file, no launch file, no external
configuration for how things are positioned. Fixed joints from `world`
position every object. One format, one parser, one truth.

### Need: My scene has many parts from different sources

A robot arm comes from one manufacturer. A laser scanner from another.
A UGV base from a third. Each has its own meshes, its own coordinate
frames, its own URDF definition. I need to compose them into one scene.

The xacro format provides composition: include files, define variables,
instantiate macros. We don't need all of xacro — just enough to assemble
a scene from modular parts.

### Component: URDFPreprocessor

```
User provides .urdf or .xacro file
    ↓
URDFPreprocessor resolves <xacro:include> files
    ↓
Resolves package:// paths to find mesh files
    ↓
Substitutes ${variables}
    ↓
Expands <xacro:macro> calls
    ↓
Outputs plain URDF XML
    ↓
KinematicModel parses it
```

The preprocessor is invisible to the user. They load any `.urdf` or `.xacro`
file, and Hatch figures out the rest. No separate build step. No external tool.

### Principle #0: Individuals Before Groups

A team of robots is only as reliable as each individual. Hatch focuses on
**one robot, one session**. Multi-robot coordination is composition, not core.

### Need: My scene needs to refer to mesh files portably

The URDF references mesh files for visualization. These files live alongside
the URDF in a package structure. The paths must work on any machine, not just
the author's.

The `package://` URI scheme solves this: `package://package_name/relative/path`.
Hatch searches for `package_name` in configured directories and follows the
relative path inside it.

The user organizes their files following ROS package conventions:

```
~/hatch/assets/
├── scenes/my_scene/urdf/scene.urdf    ← The file loaded in Hatch
├── robots/ur10/urdf/ur10.urdf         ← Included by scene.urdf
├── robots/ur10/meshes/base_link.stl   ← Referenced by ur10.urdf
├── sensors/keyence/...                ← Included by scene.urdf
└── ugv/bunker/...                     ← Included by scene.urdf
```

---

## Part Two: Understanding the Robot

### Need: I must know where everything is

The URDF describes links and joints. But joints move. At any moment, I need
to know: where is the TCP relative to the world? Where is the camera relative
to the robot base? Where is link_3 relative to link_1?

This is the transform problem. Every robotics framework must solve it.

### Component: KinematicModel

```
URDF file
    ↓
KinematicModel parses links and joints
    ↓
Detects the true kinematic root
    ↓
Computes forward kinematics on state changes
    ↓
Provides transforms for every link in world coordinates
```

The model is pure data. No visualization. No control logic. It answers
one question: given joint angles, where is everything?

### Why ElementTree?

Hatch parses URDF using only Python's built-in `xml.etree.ElementTree`. It does
not depend on any external URDF parsing library — not `urdfdom`, not
`urdf_parser_py`, not `yourdfpy`. Just the standard library.

This is not an oversight. It is a deliberate choice with three justifications:

**Zero version conflicts.** URDF parsing libraries have complex dependency
chains and frequently break across Python or ROS versions. ElementTree has
been stable in the Python standard library for decades. It will still work
exactly the same way ten years from now.

**One less moving part.** Every external library is a commitment — to its
release cycle, its API changes, its own dependencies. Hatch already requires
NumPy, VTK, and SciPy. Adding a URDF parser for a format that is fundamentally
XML is unnecessary weight.

**Understanding what we depend on.** If ElementTree misparses a URDF, the
problem is in the XML, not in a library whose internals we haven't read.
Hatch's URDF parsing is straightforward XML traversal — a few hundred lines
that anyone can read and understand completely. There is no hidden behavior.
There is no magic.

The tradeoff is that Hatch does not support every edge case of the full URDF
specification. But the URDF files produced by robot manufacturers and
ROS-Industrial packages use a practical subset that ElementTree handles
without issue. If a URDF genuinely requires a full spec-compliant parser,
it can be pre-processed externally before loading into Hatch.

This choice reflects the same principle that governs every dependency in
Hatch: *"If this dependency disappeared tomorrow, could I rebuild it from
first principles?"* For ElementTree, the answer is yes — in an afternoon.
For a dedicated URDF library with years of accumulated edge-case handling,
the answer would be no.

### The True Kinematic Root

Not all URDFs use `base_link` as the kinematic root. Universal Robots
insert a `base_inertia` link with a 180° fixed joint between `base_link`
and the first moving joint. Using `base_link` for kinematics produces
wrong results.

`KinematicModel` detects the true root automatically: find the first joint
of type `revolute`, `continuous`, or `prismatic`. Its parent link is the
kinematic root. All forward kinematics are computed from this frame.

### Need: Transforms must be computed efficiently

A robot with six joints has dozens of links when you include fixed offsets,
sensor mounts, and tool frames. Recomputing all transforms on every query
is wasteful. Recomputing them on a timer is polling.

### Principle #5: Space = TransformRegistry

All relative poses in one place. Lazy evaluation — transforms computed only
when requested. Cache invalidation on change. Callbacks notify interested
parties when transforms change.

### Component: TransformRegistry

```
KinematicModel computes link transforms in world frame
    ↓
StateHandler converts to parent-relative transforms
    ↓
TransformRegistry stores them with lazy caching
    ↓
On query: compute world transform by walking up the tree
    ↓
On update: invalidate cache for the frame and all descendants
    ↓
Callbacks notify KinematicDisplay to re-render
```

### Need: The scene must become visible

I have a kinematic model. I have transforms. I have mesh files. I need
to see the robot in 3D — its links at their current positions, moving
as joints change.

### Principle #3: Visualizer as Mind-Prying Tool

Visualization is a service that reads, not controls. The 3D view is a
window into the robot's internal state — not a separate simulation.

### Component: KinematicDisplay + VisualizerEngine

```
KinematicModel provides link transforms and mesh paths
    ↓
MeshLoader loads mesh files into VTK PolyData
    ↓
KinematicDisplay creates VTK actors for each link
    ↓
TransformRegistry callbacks update actor positions
    ↓
VisualizerEngine renders at 60Hz when dirty
```

The display does not control anything. It observes the TransformRegistry
and reflects what it sees. The engine's render loop sleeps when nothing
moves.

---

## Part Three: Making the Robot Move

### Need: I must command the robot to move

Seeing the robot is not enough. I need to move its joints. I need to move
its TCP to a specific position. I need to do this whether the robot is
real hardware or a simulation.

### Component: RobotInterface, SimulatedRobot, RealRobot

```
CommandHandler receives a command
    ↓
Routes to active robot (SimulatedRobot or RealRobot)
    ↓
Robot executes the command:
    SimulatedRobot: solve IK locally, update internal state
    RealRobot: send command via RTDE to hardware
    ↓
Robot publishes ROBOT_STATE with new joint positions
```

### Principle #7: Movements as Models

Trajectories, commands, and goals are data, not side effects. Movements
can be anticipated, monitored, and replayed.

### Principle #8: Pure Python

No C++ extensions except VTK bindings. Rapid development. Safe memory
management. Qt is permitted only in the UI layer and for hardware driver
signal bridging — never in core services.

### Need: The model must stay synchronized with the robot

When the robot moves, the kinematic model must update. The transform
registry must update. The display must update. This must happen exactly
once per state change, not multiple times from different paths.

### Component: StateHandler

```
Robot publishes ROBOT_STATE
    ↓
StateHandler receives it (the ONLY subscriber that modifies state)
    ↓
Updates KinematicModel with new joint positions
    ↓
KinematicModel recomputes forward kinematics
    ↓
StateHandler updates TransformRegistry with new transforms
    ↓
TransformRegistry callbacks fire
    ↓
KinematicDisplay sets _needs_render = True
    ↓
VisualizerEngine renders on next timer tick
```

A single owner of state updates. No duplicate registrations. No missed
updates. No race conditions.

---

## Part Four: Communication

### Need: Components must talk without knowing about each other

The joint control panel should not import the robot driver. The robot
driver should not import the 3D display. Components must communicate
without being coupled.

### Principle #2: Event-Driven, No Polling

Components communicate via events. No `while` loops waiting for data.
No busy-waiting. No periodic checks.

### Principle #6: Time = StateChannel

All events in one place. Publish/subscribe with history. Timestamps
preserve sequence. Decoupled communication.

### Component: StateChannel

```
JointControlPanel publishes JOINT_COMMAND
    ↓
StateChannel delivers to all subscribers
    ↓
CommandHandler receives, routes to robot
    ↓
Robot publishes ROBOT_STATE
    ↓
StateChannel delivers to all subscribers
    ↓
StateHandler updates model (one subscriber)
    ↓
JointControlPanel updates sliders (another subscriber)
    ↓
CartesianControlPanel updates display (another subscriber)
    ↓
KinematicDisplay updates via TransformRegistry callbacks (indirect)
```

### Principle #1: Single Process, Single Memory Space

No serialization between components. Direct data access. No network
overhead. No distributed complexity. Events carry Python objects —
no message definitions, no code generation, no serialization.

---

## Part Five: The User Interface

### Need: I need controls to interact with the robot

Joints need sliders. Cartesian movement needs position controls.
Connection to hardware needs IP input and status display. These
are presentation concerns — they should not contain business logic.

### Principle #9: UI Separate from Services

UI components publish events. They do not call managers directly
except for commands (user-initiated actions). They do not hold
business logic. They do not update models or registries. They are
pure presentation.

### Component: UI Panels

```
JointControlPanel:
    User drags slider → publishes JOINT_COMMAND
    Subscribes to ROBOT_STATE → updates slider positions

CartesianControlPanel:
    User drags slider → publishes CARTESIAN_COMMAND
    Subscribes to ROBOT_STATE → updates current TCP display

RobotConnectionPanel:
    User clicks Connect → calls RobotManager.connect_robot()
    Subscribes to CONNECTION_ESTABLISHED → shows green status
    Subscribes to CONNECTION_LOST → shows red status
    Subscribes to MODE_SWITCHED → updates mode display
```

### Principle #10: One Robot Per Session

The platform manages one robot at a time. To work with a different
robot, restart the application. Clean boundary. No complex cleanup.

### Need: Everything must be wired together

The services, the displays, the UI panels — they need to be created
and connected. One place must own this responsibility without doing
the work itself.

### Component: MainWindow

```
MainWindow.__init__:
    Create TransformRegistry
    Create StateChannel
    Create VisualizerEngine
    Create MeshLoader
    Create RobotManager
    Create SimulatedRobot, RealRobot
    Create CommandHandler
    Create CameraManager
    Create UIBuilder
    Subscribe to ROBOT_LOADED → create MotionContainer
    Subscribe to ERROR_OCCURRED → show dialog
```

`MainWindow` orchestrates. It does not update models, modify transforms,
or handle commands. It creates components and connects them. Then it
steps back.

---

## Part Six: The Render Loop

### Need: The 3D view must update smoothly without polling

When the robot moves, the display must re-render. When nothing moves,
the CPU must sleep. A render loop that runs at fixed intervals and
recomputes everything is wasteful.

### Component: VisualizerEngine Render Timer

```
QTimer fires at 60Hz
    ↓
Check _needs_render flag on each display
    ↓
If no display needs rendering: return immediately (CPU sleeps)
    ↓
If any display needs rendering: call Render()
    ↓
Clear all _needs_render flags
```

This is not polling. The timer checks a boolean — a single memory read
per display. The flag is set only by TransformRegistry callbacks, which
fire only when transforms actually change. When the robot is stationary,
nothing happens. The CPU enters low-power states between timer ticks.

*(See Appendix A for the full defense of event-driven architecture.)*

---

## Part Seven: Extension Points

### Need: I must add my own functionality

The platform cannot anticipate every use case. Users need to add
logging, safety monitoring, custom control strategies, sensor
processing. The same APIs that built the built-in panels must be
available to extensions.

### Public APIs

| API | Purpose |
|-----|---------|
| `StateChannel.subscribe()` | React to robot state, connection events, errors |
| `StateChannel.publish()` | Send commands, report detections, trigger actions |
| `TransformRegistry.get_transform()` | Query spatial relationships |
| `TransformRegistry.register_callback()` | React to transform changes |
| `EventType` enum | All events the system understands |

Extensions follow the same principles as built-in components:
observe, don't control; publish, don't call; clean up after yourself.

### Dynamic Objects (Future)

The `TransformRegistry` supports `FrameStatus.DYNAMIC` — frames whose
transforms change during operation. Currently this serves robot joints.
In future versions, it will also serve runtime-discovered objects from
cameras and sensors.

This is a designed extension point, not current capability. The
`FrameStatus.DYNAMIC` value and the callback system exist. The
perception pipeline does not.

---

## Part Eight: Limitations

Hatch is an honest platform. These areas are not yet addressed.

### Robot Topology

Hatch assumes a serial kinematic chain. The arm chain detection, inverse 
kinematics solver, and transform registration all expect a single sequence 
of links from base to tool. Parallel robots, branching chains, and closed 
loops are not supported in the current version.

### Error Handling

Errors are published as `ERROR_OCCURRED` events and displayed in dialogs.
There is no error severity classification, no structured recovery path,
and some driver-level errors are silently caught. Users should monitor
console output during development.

### Configuration Management

Values like RTDE frequency, grid size, and render FPS are hardcoded as
defaults. There is no configuration file, no persistence between sessions,
and no way to override defaults without modifying source. Configuration
will be addressed when pain points emerge during regular use.

### Sensor Calibration

Camera extrinsics are assumed to match the URDF. There is no hand-eye
calibration, no TCP calibration, and no mechanism to override URDF
transforms with calibrated values. For applications requiring precise
spatial accuracy, pre-calibrate your URDF transforms before loading.

### Testing

No automated tests exist. The `TransformRegistry`, `StateChannel`, and
`KinematicModel` are particularly testable — they have clear inputs
and outputs with no external dependencies. Tests will be added as the
platform stabilizes.

---

## The Ten Principles (Summary)

| # | Principle | Discovered From |
|---|-----------|-----------------|
| 0 | Individuals Before Groups | Need: one robot, one session |
| 1 | Single Process, Single Memory Space | Need: no serialization overhead |
| 2 | Event-Driven, No Polling | Need: decoupled communication |
| 3 | Visualizer as Mind-Prying Tool | Need: see the robot's true state |
| 4 | Everything in URDF | Need: describe the scene |
| 5 | Space = TransformRegistry | Need: know where everything is |
| 6 | Time = StateChannel | Need: components must communicate |
| 7 | Movements as Models | Need: commands as data |
| 8 | Pure Python | Need: rapid development |
| 9 | UI Separate from Services | Need: controls without coupling |
| 10 | One Robot Per Session | Need: clean boundaries |

Each principle was not chosen. It was demanded by a need that arose
during the derivation. This is what makes the architecture honest.

---

## The Full Event Flow

```
User moves slider
    ↓
UI Panel publishes COMMAND event
    ↓
StateChannel distributes event
    ↓
CommandHandler receives, routes to active robot
    ↓
Robot executes command, publishes ROBOT_STATE
    ↓
StateHandler receives ROBOT_STATE (single owner)
    ↓
StateHandler updates KinematicModel → recomputes FK
    ↓
StateHandler updates TransformRegistry → new link transforms
    ↓
TransformRegistry notifies callbacks
    ↓
KinematicDisplay sets _needs_render = True
    ↓
VisualizerEngine 60Hz timer checks flag → renders if dirty
    ↓
ROBOT_STATE also received by UI panels
    ↓
UI panels update slider positions and value labels
```

**No direct calls between UI and model. No polling. Single render path.
Single owner of state updates.**

---

## Directory Structure

```
hatch/
├── core/
│   ├── urdf_preprocessor.py    # Scene composition from .xacro files
│   ├── mesh_loader.py           # Pure mesh loading service
│   ├── robot_manager.py         # Robot lifecycle (no Qt)
│   ├── command_handler.py       # Command routing
│   ├── state_handler.py         # Single owner of model + registry updates
│   ├── mode.py                  # Mode enum
│   ├── kinematics/
│   │   ├── kinematic_model.py   # URDF parsing, FK, true root detection
│   │   ├── ik_solver.py         # IK solver wrapper with base compensation
│   │   └── ur_ik_solver.py      # Parameterized analytical IK for UR robots
│   └── world_state/
│       ├── transform_registry.py
│       ├── state_channel.py
│       └── event_types.py
│
├── drivers/
│   └── robot_arm/
│       ├── robot_interface.py   # Plain ABC (no Qt)
│       ├── base_robot_arm.py    # Driver-internal ABC (no Qt)
│       ├── simulated_robot.py   # Pure Python simulation
│       ├── real_robot.py        # Hardware bridge (Qt for signal handling)
│       └── ur_rtde_bridge.py    # RTDE driver (Qt signal holder)
│
├── displays/
│   └── kinematic_display.py     # VTK visualization
│
├── viz/
│   └── visualizer_engine.py     # VTK render window, grid, camera
│
├── ui/
│   ├── main_window.py           # Application entry point
│   ├── ui_builder.py            # Menu and dock construction
│   ├── menus/
│   └── panels/
│
├── assets/
│   ├── scenes/                  # Scene-defining URDF files
│   ├── robots/                  # Robot URDFs and meshes
│   ├── sensors/                 # Sensor URDFs and meshes
│   ├── ugv/                     # Mobile base URDFs and meshes
│   └── tools/                   # End-effector URDFs and meshes
│
└── docs/
    ├── architecture.md           # This document
    └── user_guide.md             # Getting started guide
```

---

## Appendices

### Appendix A: On Event-Driven Architecture and the Main Loop

*(Content from original Appendix A)*

### Appendix B: On Formalism and Understanding

*(Content from original Appendix B)*

### Appendix C: Event Types Reference

*(Content from original Event Types Reference section)*

---

## Closing

> *"A platform is not defined by what it can do. It is defined by what it will not do — and why."*

Hatch does one thing well: **make a single robot's mind transparent,
efficient, and easy to program.**

It was not designed. It was derived — each component demanded by a need,
each principle discovered in the process. This document traces that chain.

From this foundation, everything else grows.

---

*Document version 3.0*
*Hatch (孵) Architecture Foundation*
*Restructured as a derivation chain. User needs drive component creation.
Each principle maps to the need that demanded it.*
```

