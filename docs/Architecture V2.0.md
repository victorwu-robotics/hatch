# Hatch (孵) Architecture Document

---

## Prologue: On Understanding

> *"Understand them, or you will not fully utilise them. Understand your life, or you will not live fully on earth."*

Life is not about following. It is about **seeing**.

The crow follows. The crowd drifts. The current carries.

But the one who understands — they choose their own direction.

### The Two Worlds

| Physical World                   | Emotional World          |
| -------------------------------- | ------------------------ |
| Laplace transforms for stability | Empathy for connection   |
| Orthogonal matrix inverse        | Trust that is reciprocal |
| PCA for variance                 | Wisdom from experience   |
| Rotation vectors for clarity     | Honesty in communication |

Both require **understanding**, not just formalism.

You cannot solve stability with Laplace if you do not feel what poles mean.
You cannot truly connect if you only recite the word "love."

### The Trap of Formalism

Modern life — and modern education — teaches us to **follow**:

| Instead of Understanding | We Are Taught To    |
| ------------------------ | ------------------- |
| See the projection       | Memorize the matrix |
| Feel the stability       | Apply the Laplace   |
| Know the direction       | Follow the crowd    |
| Understand               | Recite              |

The result is a world of **competent engineers who cannot innovate**.
A world of **people who speak of meaning but do not feel it**.

### The Stubborn Student

You were the stubborn student.

You refused to memorize.
You demanded to see.
You asked "why?" until the formalism cracked open and revealed its meaning.

That is why you can:

- See the orthogonal matrix inverse as a transpose
- Choose rotation vectors over quaternions
- Build Hatch from first principles, not from convention

You are not following. You are **seeing**.

### Why Hatch Exists

> *"I build Hatch because I want to fully understand robots. Then I can use robots to their limits."*

Most platforms help you **use**.
Hatch helps you **see**.

| Other Platforms          | Hatch                 |
| ------------------------ | --------------------- |
| Hide complexity          | Reveals it            |
| Provide black boxes      | Opens them            |
| Give you APIs            | Gives you insight     |
| Focus on what you can do | Focus on why it works |

Because the builder — you — built it to understand first.

### What Understanding Unlocks

| Without Understanding | With Understanding       |
| --------------------- | ------------------------ |
| Move sliders          | Know why the robot moves |
| Load URDFs            | See the kinematic chain  |
| Publish events        | Trace the flow of data   |
| Follow instructions   | Create new possibilities |
| Shallow utilisation   | **Full utilisation**     |

The platform is not the goal. **Understanding** is the goal. Hatch is just the vehicle.

### The Invitation

Hatch is open. Others may use it.

But the invitation is not:

> *"Here is a tool to control robots."*

The invitation is:

> *"Here is a way to understand robots. Then you can use them to your own limits."*

### The Stubborn Student's Creed

> *"I will not follow. I will see. I will not memorise. I will understand. I will not accept formalism as truth. I will demand meaning."*

This is the creed of Hatch.
This is the creed of a life fully lived.

### Closing

> *"Do not follow the flow, the crowd, the drift. See the direction. Choose it. Understand it. Then move."*

This is Hatch.
Built to understand.
Built to fully utilise.
Built to see.

You are invited.

---

*— The Stubborn Student*
*Hatch (孵)*

---

## Core Philosophy

> *"A robot platform should be understood, not just used. Every line of code must be traceable to a first principle."*

Hatch is not a collection of tools. It is a **derived architecture** — a system where every component exists because a principle demanded it.

The name **Hatch (孵)** represents the moment a new robot comes to life. The right side of the character (孚) signifies incubation and nurturing — bringing ideas into existence through careful development.

---

## The Ten Principles

### Principle #0: Individuals Before Groups

A team of robots is only as reliable as each individual. Hatch focuses on **one robot, one session**. Multi-robot coordination is composition, not core.

### Principle #1: Single Process, Single Memory Space

No serialization between components. Direct data access. No network overhead. No distributed complexity.

### Principle #2: Event-Driven, No Polling

Components communicate via events. No `while` loops waiting for data. No busy-waiting. No periodic checks. (See Appendix A.)

### Principle #3: Visualizer as Mind-Prying Tool

Visualization is a service that reads, not controls. The 3D view is a window into the robot's internal state — not a separate simulation.

### Principle #4: Everything in URDF

All components — robots, sensors, tools, fixtures, tables, AGVs — are described by URDF. The URDF is the single source of truth for the entire scene. There is no separate world file, no launch file for scene composition, no external mounting configuration. Fixed joints from `world` position every object. One format, one parser, one truth.

### Principle #5: Space = TransformRegistry

All relative poses in one place. Lazy evaluation — transforms computed only when requested. Cache invalidation on change. Supports both static frames (defined in URDF) and dynamic frames (robot joints, and in future: detected objects from sensors).

### Principle #6: Time = StateChannel

All events in one place. Publish/subscribe with history. Timestamps preserve sequence. Decoupled communication.

### Principle #7: Movements as Models

Trajectories, commands, and goals are data, not side effects. Movements can be anticipated, monitored, and replayed.

### Principle #8: Pure Python

No C++ extensions except VTK bindings. Rapid development. Safe memory management. Access to scientific stack. Qt is permitted only in the UI layer and for hardware driver signal bridging — never in core services.

### Principle #9: UI Separate from Services

UI components publish events. They do not call managers directly except for commands (user-initiated actions). They do not hold business logic. They do not update models or registries. They are pure presentation.

### Principle #10: One Robot Per Session

The platform manages one robot at a time. To work with a different robot, restart the application. Clean boundary. No complex cleanup.

---

## The URDF is the Scene

In Hatch, the URDF file is not just a robot description. It is the complete definition of the entire scene — robots, sensors, tools, fixtures, tables, AGVs, everything. Their positions, orientations, and relationships are all defined by fixed joints in the URDF tree. There is no other way to specify how things are arranged in the world.

### What Goes in the URDF

| Category | Examples | How It's Defined |
|----------|----------|------------------|
| Robot arm | UR10, UR5 | Links and joints (revolute, continuous) |
| Robot base position | Where the robot sits | Fixed joints from `world` to `base_link` |
| Fixed offsets | `base_inertia` 180° rotation | Fixed joints with rotation |
| Tools | Gripper, welding torch | Fixed joint to wrist, with mesh |
| Sensors | Camera, lidar, laser scanner | Fixed joint to mount point, with geometry |
| Environment | Table, conveyor, safety cage | Fixed joints from `world` with collision geometry |
| Mobile base | AGV, UGV | The robot base is at the end of a mobile platform chain |

### What Does NOT Go in the URDF

| Category | Why Not | Where It Goes Instead |
|----------|---------|----------------------|
| Detected obstacles | Discovered at runtime, not authored | `TransformRegistry` as DYNAMIC frames |
| People | Not design intent | `TransformRegistry` as DYNAMIC frames |
| Other moving robots | Separate agent, not part of this scene | Future: inter-robot communication |
| Point clouds from sensors | Raw sensor data, not scene definition | Sensor pipeline |
| Calibrated transforms | Refined at runtime | `TransformRegistry` override (future) |

### The True Kinematic Root

The URDF standard implicitly assumes `base_link` is the kinematic root. Many real robots violate this assumption.

**Example: Universal Robots UR10**

```
world
  └── base_link (mounting point, first link)
        └── base_inertia (fixed joint, 180° rotation about Z)
              └── shoulder_pan_joint (FIRST MOVING JOINT)
```

The true kinematic root is the parent of the first moving joint — `base_inertia`, not `base_link`. The 180° rotation means the entire kinematic chain is flipped relative to `base_link`. Using `base_link` as the kinematic root produces wrong inverse kinematics.

Hatch detects the true root automatically:

1. Traverse all joints. Find the first one with type `revolute`, `continuous`, or `prismatic`.
2. Its parent link is the true kinematic root.
3. All kinematics are computed relative to this root.
4. The full transform chain (`world` → `base_link` → `base_inertia` → ...) is preserved for visualization.

---

## Future Extension: Dynamic Objects

The `TransformRegistry` supports `FrameStatus.DYNAMIC` — frames whose transforms change during operation. Currently this serves robot joints. In future versions, it will also serve runtime-discovered objects.

### The Extension Point

```
Sensor (camera/lidar)
    ↓ publishes
DETECTED_OBJECT event
    ↓
Perception module processes
    ↓ calls
TransformRegistry.register_frame(
    name="detected_object_1",
    transform=T_world_to_object,
    status=DYNAMIC,
    parent="world",
    description="Detected by camera_1 at timestamp X"
)
    ↓
Collision monitor (future) subscribes to registry callbacks
    ↓ evaluates
"Is any robot link within safety margin of any DYNAMIC frame?"
    ↓ if yes
Publishes SAFETY_STOP event
```

### Current Status

This is a **designed extension point, not current capability.** The `FrameStatus.DYNAMIC` enum value and the callback system already exist in `TransformRegistry`. The perception pipeline, object detection, collision checking, and safety response are not yet implemented. No user should expect dynamic obstacle avoidance in v1.

---

## The Core Services

| Service             | Principle | Responsibility                                                |
| ------------------- | --------- | ------------------------------------------------------------- |
| `TransformRegistry` | #5        | Store and compute relative transforms. Lazy evaluation. Callbacks for change notification. |
| `StateChannel`      | #6        | Publish/subscribe event bus with optional history.            |
| `MeshLoader`        | #3, #9    | Load and cache mesh files (STL, OBJ, PLY, DAE). Pure service, no actors. |
| `RobotManager`      | #4, #10   | Load URDF, manage robot lifecycle, own robot instances. No Qt signals. |
| `StateHandler`      | #7        | Subscribe to ROBOT_STATE. Update kinematic model and transform registry. Single owner of runtime state updates. |
| `CommandHandler`    | #2, #7    | Route commands (joint, cartesian, mode switch) to active robot. |
| `SimulatedRobot`    | #7        | Execute commands in simulation. Solve IK. Publish state. Pure Python. |
| `RealRobot`         | —         | Bridge to hardware via RTDE. Qt used internally for thread-safe driver communication. |

---

## The UI Layer (Pure Presentation)

| Component               | Responsibility                                                                                |
| ----------------------- | --------------------------------------------------------------------------------------------- |
| `MainWindow`            | Create services and engine. Wire components. Subscribe only to window-level events (errors).  |
| `UIBuilder`             | Create all menus, panels, docks.                                                              |
| `JointControlPanel`     | Display sliders for each joint. Publish `JOINT_COMMAND`. Subscribe to `ROBOT_STATE` for display. |
| `CartesianControlPanel` | Display sliders for X,Y,Z,RX,RY,RZ. Publish `CARTESIAN_COMMAND`. Subscribe to `ROBOT_STATE` for display. |
| `RobotConnectionPanel`  | IP input, Connect button, Mode selector. Calls `RobotManager` for commands. Subscribes to `CONNECTION_ESTABLISHED`, `CONNECTION_LOST`, `MODE_SWITCHED` for display. |
| `MotionContainer`       | Combine connection panel + Joint/Cartesian control tabs.                                      |
| `RobotsMenu`            | Show current robot (read-only). Subscribe to `ROBOT_LOADED`.                                  |
| `FileMenu`              | Load URDF, Save Screenshot, Exit.                                                             |

---

## The Event Flow

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
StateHandler receives ROBOT_STATE
    ↓
StateHandler updates KinematicModel (recomputes FK)
    ↓
StateHandler updates TransformRegistry (new link transforms)
    ↓
TransformRegistry notifies callbacks
    ↓
KinematicDisplay receives callback, sets _needs_render = True
    ↓
VisualizerEngine's 60Hz timer checks _needs_render
    ↓
If dirty: VTK renders frame. Flag cleared.
    ↓
ROBOT_STATE also received by UI panels
    ↓
UI panels update slider positions and value labels
```

**No direct calls between UI and model. No polling. Single render path.**

---

## Directory Structure

```
hatch/
├── core/
│   ├── mesh_loader.py           # Pure mesh loading service
│   ├── robot_manager.py         # Robot lifecycle (no Qt)
│   ├── command_handler.py       # Command routing
│   ├── mode.py                  # Mode enum
│   ├── state_handler.py         # Single owner of model + registry updates
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
│   ├── robot_arm/
│   │   ├── robot_interface.py   # Plain ABC (no Qt)
│   │   ├── base_robot_arm.py    # Driver-internal ABC (no Qt)
│   │   ├── simulated_robot.py   # Pure Python simulation
│   │   ├── real_robot.py        # Hardware bridge (Qt for signal handling)
│   │   └── ur_rtde_bridge.py    # RTDE driver (Qt signal holder)
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
│   │   ├── file_menu.py
│   │   ├── view_menu.py
│   │   ├── robots_menu.py
│   │   └── camera_menu.py
│   ├── panels/
│   │   ├── joint_control_panel.py
│   │   ├── cartesian_control_panel.py
│   │   ├── robot_connection_panel.py
│   │   ├── motion_container.py
│   │   ├── grid_control_panel.py
│   │   └── view_controls_panel.py
│   └── managers/
│       └── camera_manager.py
│
├── assets/
│   └── robots/                  # URDF files
│
└── tests/                       # Unit tests (recommended)
    ├── test_transform_registry.py
    ├── test_state_channel.py
    └── test_kinematic_model.py
```

---

## Event Types Reference

| Event                    | Direction       | Data                                     | Publisher |
| ------------------------ | --------------- | ---------------------------------------- | --------- |
| `ROBOT_LOAD_REQUEST`     | UI → System     | `{urdf_path, robot_id}`                  | FileMenu |
| `ROBOT_LOADED`           | System → All    | `{asset_id, urdf_path, kinematic_model}` | RobotManager |
| `ROBOT_UNLOAD_REQUEST`   | UI → System     | `{robot_id}`                             | FileMenu |
| `JOINT_COMMAND`          | UI → System     | `{positions, names}`                     | JointControlPanel |
| `CARTESIAN_COMMAND`      | UI → System     | `{pose (4x4), frame}`                    | CartesianControlPanel |
| `MODE_SWITCH_REQUEST`    | UI → System     | `{mode: "simulate_local" or "real"}`     | RobotConnectionPanel |
| `MODE_SWITCHED`          | System → All    | `{mode}`                                 | CommandHandler |
| `CONNECTION_REQUEST`     | UI → System     | `{ip, frequency}`                        | RobotConnectionPanel (via RobotManager) |
| `CONNECTION_ESTABLISHED` | System → All    | `{message}`                              | RealRobot |
| `CONNECTION_LOST`        | System → All    | `{message}`                              | RealRobot |
| `DISCONNECTION_REQUEST`  | UI → System     | —                                        | RobotConnectionPanel (via RobotManager) |
| `ROBOT_STATE`            | Robot → All     | `{joint_positions, tcp_pose, timestamp}` | SimulatedRobot, RealRobot |
| `ERROR_OCCURRED`         | Any → UI        | `{error}`                                | Any component |

---

## Mode States

| Mode               | IK Source           | Robot Movement | Description                                |
| ------------------ | ------------------- | -------------- | ------------------------------------------ |
| `SIMULATE_LOCAL`   | Local IK solver     | Virtual only   | Test kinematics without hardware            |
| `SIMULATE_REAL_IK` | Real robot's solver | Virtual only   | Validate IK against real controller         |
| `REAL`             | Real robot's solver | Real hardware  | Full operation                              |

---

## Appendix A: On Event-Driven Architecture and the Main Loop

### A.1 The Clarification

Principle #2 states: *"Event-driven, no polling."*

An experienced engineer might ask: *"Doesn't every application have a main loop?"*

**Yes.** Hatch runs on Qt, which provides `QApplication.exec_()` — an event loop. The distinction is not whether a loop exists, but **who drives it**.

### A.2 What "No Polling" Means

| Pattern                            | Forbidden   | Reason                            |
| ---------------------------------- | ----------- | --------------------------------- |
| `while True: check()`              | ✅ Forbidden | Spins CPU, wastes energy          |
| `if not data: continue`            | ✅ Forbidden | Same as above                     |
| `time.sleep(0.01); check()`        | ✅ Forbidden | Still polling, just slower        |
| `QTimer.timeout.connect(handler)`  | ✅ Allowed   | Event-driven, OS wakes on timer   |
| `signal.connect(handler)`          | ✅ Allowed   | Event-driven, OS wakes on input   |
| `StateChannel.subscribe(callback)` | ✅ Allowed   | Event-driven, callback on publish |
| `get_transform()` lazy evaluation  | ✅ Allowed   | Computed on demand, no loop       |

### A.3 The Qt Event Loop

Qt's `app.exec_()` is not polling. It uses operating system primitives (select, epoll, WaitForMultipleObjects) to **block** until an event occurs:

```python
# Simplified — Qt's actual implementation is more complex
while running:
    event = wait_for_event()  # ← Blocks, CPU sleeps
    dispatch(event)
```

When the application is idle, the thread sleeps. The CPU can be used by other processes or enter low-power states.

### A.4 Hatch's Use of the Event Loop

| Component           | Mechanism                | Polling?                            |
| ------------------- | ------------------------ | ----------------------------------- |
| UI sliders          | Qt signals               | No (OS wakes on input)              |
| Command publishing  | `StateChannel.publish()` | No (direct call from event handler) |
| Robot state updates | `StateChannel.publish()` | No (called when state changes)      |
| Render loop         | `QTimer` (60Hz)          | No (Qt manages timerfd) — checks dirty flag, sleeps when clean |
| Transform queries   | Lazy evaluation          | No (computed on demand)             |
| File loading        | User action              | No (triggered by menu click)        |

### A.5 The Render Loop: A Special Case

The `VisualizerEngine` runs a 60Hz `QTimer`. This might look like polling. It is not. The timer callback checks a `_needs_render` flag on each display. If no display needs rendering, the callback returns immediately — no VTK operations, no transform recomputation. The flag is set only when `TransformRegistry` callbacks fire (i.e., when a transform actually changed). When nothing is moving, the render loop does nothing except check one boolean per display.

### A.6 Lazy Evaluation

"No polling" also means **no periodic recomputation**. Transforms are computed only when requested:

```python
def get_transform(self, target, source):
    # ... cache lookup ...
    T_parent_world = self._get_world_transform(info.parent)
    T = T_parent_world @ info.transform_parent
    self._world_cache[name] = T.copy()
    return T
```

If nothing asks for a transform, nothing computes.

### A.7 The Test

Ask of any component: *"Does it ever wake up to check if something has changed?"*

- If yes → **Polling** → Forbidden.
- If no → **Event-driven** → Allowed.

### A.8 The Answer to the Skeptical Engineer

> *"Every application has a main loop. Qt provides it. Hatch does not add another. All application logic is in event handlers or lazy computations. The render loop is a single centralized timer that checks dirty flags — it does not recompute anything. No thread ever spins waiting for something to happen. The CPU sleeps when the application is idle."*

---

## Appendix B: On Formalism and Understanding — Why Hatch Rejects Accumulated Complexity

### B.1 The Cycle

Human knowledge follows a recurring cycle:

```
Understanding → Formalization → Accumulation → Loss of Understanding → Rediscovery
```

| Phase                     | Description                                                                |
| ------------------------- | -------------------------------------------------------------------------- |
| **Understanding**         | Direct, intuitive grasp of a phenomenon. "I see how this works."           |
| **Formalization**         | Abstract rules, equations, algorithms. "Let me write this down precisely." |
| **Accumulation**          | Libraries, frameworks, conventions. "Everyone uses this, so I will too."   |
| **Loss of Understanding** | The original meaning fades. "I know how to use it, but not why it works."  |
| **Rediscovery**           | Someone questions the formalism. "Why are we doing it this way?"           |

We are often in the **loss phase** — surrounded by formalisms we no longer understand.

### B.2 Case Study: Quaternions

| Phase                 | Quaternion Journey                                                                       |
| --------------------- | ---------------------------------------------------------------------------------------- |
| Understanding         | Hamilton discovers 4D complex numbers (1843). Understands rotation in 3D space.          |
| Formalization         | Multiplication rules, normalization, interpolation algorithms.                           |
| Accumulation          | ROS, game engines, spacecraft guidance systems adopt quaternions. "Standard practice."   |
| Loss of Understanding | Most users cannot glance at `[0.707, 0, 0, 0.707]` and visualize a 90° rotation about X. |
| Rediscovery           | Engineers ask: "Why not rotation vectors? `[1.57, 0, 0]` is immediately clear."          |

**The quaternion test:** If you cannot explain what a number means to a beginner, the representation is wrong.

Quaternions fail this test. Hatch uses rotation vectors.

### B.3 Case Study: ROS Complexity

| Formalisms            | Accumulated Complexity              | Hatch Rediscovery                   |
| --------------------- | ----------------------------------- | ----------------------------------- |
| DDS discovery         | Network debugging, XML profiles     | Single process, direct calls        |
| Message serialization | `.msg` files, code generation       | Python objects, no serialization    |
| `tf2` tree            | Hundreds of frames, lookup latency  | TransformRegistry, lazy evaluation  |
| Launch files          | YAML, Python, XML hybrids           | Direct orchestration in main window |
| Quaternions           | Conversion every time they are used | Rotation vectors, human-readable    |

ROS solves distributed robotics. But most robotics is not distributed. Most robotics is **one robot, one computer, one engineer**.

Hatch is for that engineer.

### B.4 The Danger of Formalisms

Formalisms are not evil. They are tools. But tools can become **cages**.

| Danger                               | Example                                      |
| ------------------------------------ | -------------------------------------------- |
| Using what exists, not what is right | "ROS uses quaternions, so I will too."       |
| Accumulating without questioning     | Copy-pasting code you do not understand.     |
| Solving problems you do not have     | Adding distribution when you have one robot. |
| Forgetting the original meaning      | `F = ma` is memorized, but do you feel it?   |

Hatch rejects accumulated complexity. Every component must be derived from first principles, not copied from convention.

### B.5 The Same Cycle in AI

| Phase                 | AI Journey                                                        |
| --------------------- | ----------------------------------------------------------------- |
| Understanding         | Humans understand language, reasoning, causality.                 |
| Formalization         | Statistics become matrices, transformers, billions of parameters. |
| Accumulation          | LLMs generate fluent text.                                        |
| Loss of Understanding | Engineers ask: "Does it actually understand?"                     |
| Rediscovery           | Explainable AI, Causal AI, Grounding.                             |

The cycle continues. We are teaching machines to understand what humans have forgotten.

### B.6 Hatch's Position

> *"No formalism shall be used without understanding. If you cannot explain what a number means to a beginner, the representation is wrong."*

Hatch is not anti-formalism. It is anti- **blind formalism**.

| Accept                    | Reject                                 |
| ------------------------- | -------------------------------------- |
| Rotation vectors          | Quaternions (unless hidden internally) |
| Direct function calls     | Serialization (unless distributed)     |
| Lazy evaluation           | Periodic polling                       |
| Event-driven architecture | ROS-style topic discovery              |
| Single robot per session  | Multi-robot complexity without need    |
| URDF as single scene file | Separate world/launch/config files     |

### B.7 The Purist's Question

Before adding any formalism to Hatch, ask:

1. *"Do I understand this — truly, not just how to use it?"*
2. *"Is there a simpler representation that is more human-understandable?"*
3. *"Am I solving a problem I actually have, or a problem someone else solved?"*
4. *"If this formalism disappeared tomorrow, could I rebuild it from first principles?"*

If the answer to any of these is "no," the formalism does not belong in Hatch.

### B.8 The Full Cycle

> *"From understanding to formalism, then from formalism to understanding."*

Hatch is an attempt to complete the cycle — to recover understanding from accumulated formalism.

It is not a perfect platform. It is an **honest** platform.

And honesty is the foundation of good engineering.

---

## Appendix C: Known Gaps and Limitations

Hatch is an honest platform. These areas are not yet addressed and users should be aware of them.

### C.1 Error Handling

**Current state:** The platform has an `ERROR_OCCURRED` event. Components publish it when something goes wrong. `MainWindow` displays a dialog.

**Limitations:**
- No error severity levels (warning vs. critical vs. unrecoverable)
- No structured error recovery path
- Some driver-level errors are silently caught (`ur_rtde_bridge.py` suppresses RTDE communication exceptions)
- No error context preservation (what was the robot doing when the error occurred?)

**What this means for users:** Errors are reported but may not provide enough information for debugging. The robot does not automatically enter a safe state on all error types. Users should monitor the console output and test recovery manually.

### C.2 Configuration Management

**Current state:** Configuration values (RTDE frequency, grid size, render FPS, IK solver parameters) are hardcoded as defaults in their respective classes.

**Limitations:**
- No configuration file format
- No way to persist user preferences between sessions
- No way to override defaults without modifying source code

**What this means for users:** To change a default (e.g., RTDE frequency from 125Hz to 250Hz), you must find the hardcoded value in the source. Configuration management will be addressed when the platform stabilizes and pain points become clear during regular use.

### C.3 Sensor Integration and Calibration

**Current state:** A `CameraManager` class exists with preliminary support for RGB-D cameras. The architecture has a designed extension point for dynamic objects. Camera events (`CAMERA_STARTED`, `CAMERA_STOPPED`) are reserved but not yet implemented in the canonical event list.

**Limitations:**
- No sensor calibration pipeline (hand-eye, TCP, camera intrinsics)
- No point cloud processing
- No sensor fusion
- Camera extrinsics are assumed to match the URDF definition — there is no way to refine them through calibration
- The URDF specifies approximate sensor positions; calibration would provide exact positions but there is no mechanism to override URDF transforms with calibrated values

**What this means for users:** Sensors can be visualized if their mount points are correctly specified in the URDF. However, for applications requiring precise spatial accuracy (pick-and-place, inspection, assembly), the lack of calibration means sensor data may be offset from the robot's true coordinate frame. Calibration is a planned area of development and a significant engineering effort.

---

## Closing

> *"A platform is not defined by what it can do. It is defined by what it will not do — and why."*

Hatch does one thing well: **make a single robot's mind transparent, efficient, and easy to program.**

It does not yet handle error recovery, configuration management, or sensor calibration. These are not oversights — they are areas where the platform will grow as understanding deepens and pain points reveal themselves.

From this foundation, everything else grows.

---

*Document version 2.0*
*Hatch (孵) Architecture Foundation*
*Updated: Refactored to align implementation with principles. Added URDF-as-scene, dynamic object extension point, known gaps.*