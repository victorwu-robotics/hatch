# Hatch (孵) 🐣 Philosophy

## Origin Story
**In the beginning**, I knew nothing about robots and robotics. My thinking was very simple.

I didn't even know what a **pose** was. I thought: when I know the coordinate I want the robot tip to move to, I just give it that coordinate, and it moves there. This should be very simple.

Then I discovered that nothing like that exists.

After digging a little deeper, I realized there is **orientation** on top of **position** — the concept of a **pose**. Later, I understood that things were not as simple as I thought — because of **inverse kinematics**. I had been thinking in terms of **forward kinematics**. I never knew there was such a thing as **IK**, nor the complexity and difficulty of finding joint angles.

It was a voyage of learning without a mentor.

Every stage was a surprise. Nobody warned me. No mentor guided me. I **learned by hitting walls**.

For my welding cell, the seam was a known curve on a steel plate. I needed
the robot to trace it precisely, at a **constant speed**, with the torch at a
**fixed angle**. I didn't need a **planner** to find a path through unknown space.
I needed a **driver** to execute the path I had already defined.

I was lonely. I didn't know who to turn to or where to look for answers.

Fortunately, the **Internet** already existed. I could think of reasonable questions, search with reasonable terms, and hope to find clues.

When I found **ROS**, I was so happy. Someone had built something. There were drivers. There was a community. There were answers.

But the happiness didn't last.

---

ROS was solving a different problem. It was built for warehouses **full of robots and computers**. It had **distributed nodes**, **message serialization**, **launch files**, and — most painfully for me — **motion planning**.

For an **engineering project**, motion planning is unnecessary. My welding environment was known. The seam was fixed. The robot moves from A to B to C along a path I defined. There are no surprises.

But ROS included motion planning because they wanted to build a robot that could move through any unforeseeable space. Their **Descartes** package tried to **find paths in Cartesian space**. I wasted a great deal of time trying to use it, without knowing that a six-axis arm can reach the same pose in up to eight different configurations. **Shoulder left or right**. **Elbow up or down**. **Wrist flipped or not**.

Descartes, trying to be helpful, might choose one configuration for waypoint 1 and a different configuration for waypoint 2. The robot, moving between them, swings its elbow through space — potentially through the workpiece, through a fixture, through a person.

The planner avoided **a collision that didn't exist** by creating **a collision that did**.

---

And **MoveIt** — the standard ROS interface for robot arms — was built around motion
planning. Every tutorial, every example, every API path assumed I wanted to plan.
I couldn't find a way to use its driver and IK solver without the planner coming
along. Whether it was technically possible or not, the architecture didn't make
it possible *for me* — and that's what matters when you're a user trying to get
work done.

A good architecture **separates** what the user might **not need** from what they **must
have**. **The driver is essential**. **The IK solver is essential**. Motion planning is
**optional** — and should be. Hatch keeps them separate. **If you want motion planning,
you add it as an extension**. It never forces itself on you.

---

This taught me a fundamental lesson: A tool that makes decisions the user didn't ask for is not intelligent — it is insubordinate. If the user defines a path, the robot should follow that path. If the user wants a specific configuration, the robot should use that configuration. Intelligence the user didn't ask for is not intelligence — it is interference.

But this does not mean motion planning and collision avoidance have no value. There are situations where we genuinely need them.

When a humanoid robot enters a home, reaching for a cup on a cluttered table, it needs collision avoidance. But it also needs to know: which configuration am I in? If I change configurations to avoid this obstacle, where will my elbow go? Will I knock over the vase behind me?

This requires understanding. This is why I — the Stubborn Student — place so much emphasis on understanding.

Hatch is not against motion planning. Hatch is against **blind** motion planning.

---

This platform exists so that the next person's journey need not be so lonely.

For a welding cell, you don't need motion planning. You need a teach pendant and a device driver. Hatch gives you that.

For a humanoid in a home, you will need motion planning. But you will need to understand configurations, inverse kinematics, and what your planner is actually doing first. Hatch teaches you that understanding.

Hatch is not against automation. It is against automation whose behavior the user does not understand. The Stubborn Student's creed applies at every scale: from a single weld seam to a humanoid in a kitchen.

---

— The Stubborn Student
孵 (Hatch)


---

## Prologue: On Understanding

> *"Understand them, or you will not fully utilise them. Understand your life, or you will not live fully on earth."*

Life is not about following. It is about **seeing**.

The crow follows. The crowd drifts. The current carries.

But the one who understands — they choose their own direction.

The physical world and the emotional world both require understanding — not just formalism.

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
*Hatch (孵) 🐣*

---

These experiences — the loneliness of learning without a mentor, the
disappointment of tools that solved problems I didn't have, the danger
of automation that acts without understanding — shaped ten principles.
Each one was discovered through failure, not decreed from a design
document. Together they form the foundation of Hatch.

---

## Core Philosophy

> *"A robot platform should be understood, not just used. Every line of code must be traceable to a first principle."*

Hatch is not a collection of tools. It is a **derived architecture** — a system where every component exists because a principle demanded it.

The name **Hatch (孵) 🐣** represents the moment a new robot comes to life. The right side of the character (孚) signifies incubation and nurturing — bringing ideas into existence through careful development.

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

## Appendix A: On Formalism and Understanding — Why Hatch Rejects Accumulated Complexity

### A.1 The Cycle

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

### A.2 The Danger of Formalisms

Formalisms are not evil. They are tools. But tools can become **cages**.

| Danger                               | Example                                      |
| ------------------------------------ | -------------------------------------------- |
| Using what exists, not what is right | "ROS uses quaternions, so I will too."       |
| Accumulating without questioning     | Copy-pasting code you do not understand.     |
| Solving problems you do not have     | Adding distribution when you have one robot. |
| Forgetting the original meaning      | `F = ma` is memorized, but do you feel it?   |

Hatch rejects accumulated complexity. Every component must be derived from first principles, not copied from convention.

### A.3 Hatch's Position

> *"No formalism shall be used without understanding. If you cannot explain what a number means to a beginner, the representation is inappropriate for that context."*

Hatch is not anti-formalism. It is anti- **blind formalism**.

| Accept                    | Reject                                 |
| ------------------------- | -------------------------------------- |
| Rotation vectors          | Quaternions (unless hidden internally) |
| Direct function calls     | Serialization (unless distributed)     |
| Lazy evaluation           | Periodic polling                       |
| Event-driven architecture | ROS-style topic discovery              |
| Single robot per session  | Multi-robot complexity without need    |
| URDF as single scene file | Separate world/launch/config files     |

### A.4 The Purist's Question

Before adding any formalism to Hatch, ask:

1. *"Do I understand this — truly, not just how to use it?"*
2. *"Is there a simpler representation that is more human-understandable?"*
3. *"Am I solving a problem I actually have, or a problem someone else solved?"*
4. *"If this formalism disappeared tomorrow, could I rebuild it from first principles?"*

If the answer to any of these is "no," the formalism does not belong in Hatch.

### A.5 The Full Cycle

> *"From understanding to formalism, then from formalism to understanding."*

Hatch is an attempt to complete the cycle — to recover understanding from accumulated formalism.

It is not a perfect platform. It is an **honest** platform.

And honesty is the foundation of good engineering.

---

## Appendix B: Known Gaps and Limitations

Hatch is an honest platform. These areas are not yet addressed and users should be aware of them.

### B.1 Error Handling

**Current state:** The platform has an `ERROR_OCCURRED` event. Components publish it when something goes wrong. `MainWindow` displays a dialog.

**Limitations:**
- No error severity levels (warning vs. critical vs. unrecoverable)
- No structured error recovery path
- Some driver-level errors are silently caught (`ur_rtde_bridge.py` suppresses RTDE communication exceptions)
- No error context preservation (what was the robot doing when the error occurred?)

**What this means for users:** Errors are reported but may not provide enough information for debugging. The robot does not automatically enter a safe state on all error types. Users should monitor the console output and test recovery manually.

### B.2 Configuration Management

**Current state:** Configuration values (RTDE frequency, grid size, render FPS, IK solver parameters) are hardcoded as defaults in their respective classes.

**Limitations:**
- No configuration file format
- No way to persist user preferences between sessions
- No way to override defaults without modifying source code

**What this means for users:** To change a default (e.g., RTDE frequency from 125Hz to 250Hz), you must find the hardcoded value in the source. Configuration management will be addressed when the platform stabilizes and pain points become clear during regular use.

### B.3 Sensor Integration and Calibration

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

The Stubborn Student

---

*Hatch (孵) 🐣 — built to understand, built to see.*