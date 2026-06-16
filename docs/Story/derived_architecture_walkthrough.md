# Hatch: A Derived Architecture — The Walkthrough

*This document is an archived walkthrough of how Hatch's architecture was derived,
step by step, from first principles. It was written during development as a way to
trace the chain of reasoning behind every component. For the canonical reference,
see [the Architecture document](../architecture.md). For the philosophical
foundation, see [the Philosophy document](../philosophy.md).*

---

## Prologue: Why Build Another Robot Platform?

I needed to weld. Electric arc welding — a robot arm holding a torch, tracing seams on steel plates. I started with ROS and C++, as everyone did. It worked, eventually. But the code was hard to write, harder to debug, and hardest to change. Every small modification rippled through launch files, message definitions, and build configurations. I was spending more time fighting the platform than solving the welding problem.

Then I found Python. The same logic that took hundreds of lines in C++ took dozens in Python. The robot moved. The torch traced. But ROS was still underneath — a distributed system designed for warehouses full of robots and computers, when all I had was one robot and one laptop. I was carrying a backpack full of tools I never used, and they were heavy.

So I asked: what do I actually need?

I need to describe a robot. I need to see it on screen. I need to move its joints, position its tool, and watch it respond. I need to connect to real hardware when it's time to weld, and simulate when I'm testing. That's six needs. Not sixty. Not six hundred.

Every line of code in Hatch exists because one of those needs demanded it. Nothing was added because another platform does it. Nothing was kept because it's conventional. This document traces each need to the component it created, and each component to the principle it revealed.

Hatch is not a collection of tools. It is a chain of reasoning. This is that chain.

---

## Part One: Seeing the Robot

I cannot debug what I cannot see. A robot is a physical thing — it has shape, it occupies space, it moves. If I send a command and the robot moves wrong, I need to see what happened. Not read log messages. Not inspect joint angles in a terminal. I need to see the arm, the torch, the camera, the scanner — all of them moving together in space, exactly as they would on the factory floor.

This is not a luxury. It is a prerequisite for understanding. Without vision, I am programming blind. With vision, the robot's behavior becomes obvious. The torch dipped because the wrist rotated unexpectedly. The scanner missed the groove because the arm approached from the wrong angle. The point cloud is sparse because the camera is too far from the surface. These are not things I deduce from numbers. They are things I see.

So the first component Hatch needed was a **visualizer**. A 3D view that shows every link of the robot, every sensor mounted to its wrist, every tool it holds. Not a separate simulation — a window into the robot's internal state. The visualizer does not control anything. It observes. It reflects whatever the kinematic model tells it is true. If the model says the elbow is at 45 degrees, the visualizer shows the elbow at 45 degrees. If the model is wrong, the visualizer shows the wrong thing — and I know the model needs fixing.

This is the principle: **Visualizer as Mind-Prying Tool.** The 3D view is not the robot. It is what the robot thinks it is. The gap between them is where debugging happens.

But to show the robot, the visualizer needs something to show. It needs the robot's shape — the meshes, the geometry, the physical form of every link. And it needs to know where each piece goes. That requires a description of the robot. That requires a URDF.

---

## Part Two: Describing the Robot

A robot is a chain of rigid bodies connected by joints. Every industrial robot — from a small UR5 to a massive KUKA — follows this pattern. A base. A shoulder. An upper arm. A forearm. A wrist. At the end, a tool.

To put a robot on screen, I need to tell the computer exactly what those parts look like and how they connect. The robot's manufacturer already did this work. They created a file — a URDF — that lists every link, every joint, and every dimension. URDF stands for Unified Robot Description Format. It is the industry standard. Every major robot brand provides one.

### The URDF File

The first time I opened a URDF file, I saw things I understood and things I didn't. The links and joints made immediate sense. A `<link>` is a rigid piece of the robot — the base, the upper arm, the wrist. A `<joint>` connects two links and tells how they move relative to each other. Revolute joints rotate. Fixed joints don't move. Prismatic joints slide. That much was comfortable.

But there were strange things too. `<xacro>` blocks that looked like code but weren't quite code. `<collision>` tags that seemed to duplicate what `<visual>` already said. File references ending in `.stl` and `.dae` that I had never seen before. And `<parent>` and `<child>` elements that defined a family tree I didn't yet understand.

Each of these strange things exists for a reason. They are not accidents of the format. They solve real problems.

### The Parent-Child Chain

A joint connects a parent link to a child link. The parent is the link closer to the base. The child is the link further out. This creates a chain:

```
world → base_link → shoulder_link → upper_arm_link → forearm_link → wrist_1_link → wrist_2_link → wrist_3_link
```

When the shoulder joint rotates, the upper arm, forearm, and everything beyond it rotates too. The parent-child relationship captures this: a child moves with its parent, plus whatever movement its own joint adds. This simple rule — repeated for every joint — is all the kinematic model needs to compute where every link is at any moment.

### What is Xacro?

Xacro is a preprocessor. It is not part of the URDF standard — it is a convenience layer on top. A robot with many sensors and tools can have a URDF file that is thousands of lines long, with repeated blocks for similar components. Xacro lets you write that structure once and reuse it.

Hatch includes its own xacro preprocessor, just enough to handle the features that matter: including other files, defining variables, and instantiating macros. If you already have a URDF that works in ROS, it will work in Hatch. If you are building one from scratch, you can use plain URDF without xacro at all. The preprocessor is invisible — you load a `.xacro` file, and Hatch produces a clean URDF before parsing it.

### Meshes: STL and DAE

A link is a rigid body, but the URDF does not describe its shape in numbers. It points to a mesh file — a 3D model of the link. Two formats dominate: STL and DAE (Collada).

STL is the simplest. It lists triangles — three points, then another three, then another three — until the entire surface of the part is described. It contains no color, no material, no scene structure. Just triangles. STL files are small, fast to load, and universally supported. Every CAD program can export them.

DAE (Collada) is richer. It can contain color, texture, and multiple objects arranged in a scene. A single DAE file can hold an entire assembly of parts with their relative positions baked into a scene graph. This richness comes with complexity. A DAE file must be interpreted correctly — its scene transforms must be applied, or the mesh appears in the wrong place at the wrong size. This exact problem would later teach us something important about mesh loading.

### Visual and Collision

Every link can have two representations: a visual mesh (what you see on screen) and a collision mesh (what the robot checks for impacts). They are often the same file, but they serve different purposes. The visual mesh can be detailed and beautiful. The collision mesh should be simple — a box or a cylinder — because collision checking is computationally expensive. Hatch uses the visual mesh for display. Collision detection is a future feature, but the URDF structure is ready for it.

---

## Part Three: Placing Things in Space

The URDF describes the robot. But a description is not a position. A link is a rigid body with a shape, but it has no location until something computes where it sits at this moment, with these joint angles. The visualizer needs to know where to draw every piece. The user needs to query the distance between the TCP and the workpiece. The IK solver needs to know where the wrist center is relative to the base.

So Hatch needed two things: a **kinematic model** that computes positions from joint angles, and a **transform registry** that stores those positions and keeps them up to date as the robot moves.

### The Actor and the Space

VTK, the visualization library Hatch uses, thinks in terms of **actors**. An actor is a thing that appears on screen — a link, a tool, a sensor housing. Each actor has a mesh (its shape) and a transform (its position and orientation in the 3D scene). To place an actor correctly, you set its transform to the world position of the link it represents.

But the robot has many links. They move. Their positions change every time a joint angle changes. Keeping track of every link's world position — and updating every actor when something moves — is the central spatial problem Hatch must solve.

### The Kinematic Model: Computing Where Things Are

The kinematic model answers one question: given all six joint angles, where is every link?

It starts from the URDF. Each joint has an origin — a fixed offset from its parent link. A revolute joint adds a rotation about its axis. A prismatic joint adds a translation. The model walks the parent-child chain from the base to the tip, multiplying transforms:
```
T_world_to_link = T_world_to_parent @ T_parent_to_child @ T_joint_motion
```


At zero angles, every link has a known world position. Change joint 2 by 0.1 radians, and the model recomputes. Every link from joint 2 outward gets a new world transform. The computation is pure trigonometry — no iteration, no approximation. Forward kinematics is the easy direction.

The kinematic model also detects the **true root** — the parent of the first moving joint. Some URDFs place fixed joints between the world and the first moving joint (a pedestal, a mounting plate, a 180-degree rotation hack). The model walks backward through those fixed joints to find where the kinematic chain actually begins. This matters for IK, where using the wrong root produces systematically wrong solutions.

### The Transform Registry: Remembering Where Things Are

Computing transforms is one thing. Keeping them organized is another. A six-joint robot with sensors and tools has dozens of frames — the base, six links, a camera, a scanner, a torch holder, optical reference frames. Each frame has a parent-child relationship. Each frame's world position depends on its parent's world position.

The transform registry stores every frame in a tree:
```
world
└── base_link
└── shoulder_link
└── upper_arm_link
└── forearm_link
└── wrist_1_link
└── wrist_2_link
└── wrist_3_link
├── torch
├── camera_link
│ └── camera_depth_frame
└── scanner_optical_frame
└── scanner_frame
```


Each frame stores its transform **relative to its parent** — not relative to the world. This is the key insight. When joint 2 rotates, only `shoulder_link` and its descendants change. The registry invalidates the cached world transforms for those frames. Everything else stays cached. When something queries the world position of the scanner, the registry walks up the tree — `scanner_frame` → `scanner_optical_frame` → `wrist_3_link` → ... → `world` — multiplying parent-relative transforms along the way. The result is cached until the next invalidation.

This is lazy evaluation. If nobody asks for a frame's world position, it is never computed. If a frame hasn't moved since the last query, the cached answer is returned instantly. The registry is event-driven: it only recomputes when something changes, and only for the frames that are actually needed.

This is the principle: **Space = TransformRegistry.** All relative poses in one place. Lazy evaluation. Cache invalidation on change. No polling. No periodic recomputation.

### The Single Owner of Updates

The kinematic model computes transforms. The transform registry stores them. But who decides when to update? If multiple components call `update_frame()` at different times, the registry's cache could reflect inconsistent states.

Hatch has exactly one component that updates the model and registry in response to robot motion: the **StateHandler**. It subscribes to `ROBOT_STATE` events. When a new state arrives, it updates the kinematic model with the new joint angles, then updates every frame in the registry with the model's new link transforms. No other component touches the model or the registry during operation.

The initial registration happens once, when the robot is loaded. The runtime updates happen only through StateHandler. This single-owner pattern prevents the duplicate updates and inconsistent states that plague systems where every component can modify the shared spatial data.

---

## Part Four: Making the Robot Move

The visualizer shows the robot at rest. The kinematic model knows where every link is. The transform registry keeps those positions organized. Now I need to make the robot move.

### The First Motion: Joint Commands

A robot moves when you change its joint angles. Joint 1 rotates the base. Joint 2 tilts the shoulder. Joint 3 bends the elbow. Together, they position the wrist. Joints 4, 5, and 6 orient the tool.

The simplest control is direct: give each joint a target angle, and let the robot go there. This is **joint control**. A slider for each joint. Move the slider, the robot moves. No math required. No coordinates to calculate. Just six numbers.

But even this simple action requires a chain of events. The slider cannot call the robot directly — that would couple the UI to the hardware. The robot cannot update the display directly — that would couple the hardware to the UI. Components must communicate without knowing about each other.

### The Event System: StateChannel

Hatch's answer is the **StateChannel** — a publish-subscribe event bus at the center of everything. Components publish events. Other components subscribe to events they care about. No component knows who receives its events or who sent the events it receives.

The flow for joint control:
```
User drags slider
↓
JointControlPanel publishes JOINT_COMMAND
↓
StateChannel delivers to all subscribers
↓
CommandHandler receives JOINT_COMMAND
↓
CommandHandler routes to the active robot (simulated or real)
↓
Robot moves to the commanded angles
↓
Robot publishes ROBOT_STATE with its new joint positions
↓
StateChannel delivers ROBOT_STATE to all subscribers
↓
StateHandler updates the kinematic model and transform registry
↓
KinematicDisplay receives registry callbacks, updates VTK actors
↓
VisualizerEngine's render timer checks _needs_render flag
↓
If dirty: render new frame
```

Seven steps. No direct calls between UI and model. No polling. Each component does one thing. The StateChannel is the only thing that connects them.

This is the principle: **Time = StateChannel.** All events flow through here. Timestamps preserve sequence. History is available for debugging. Decoupled communication.

And the principle: **Event-Driven, No Polling.** No component sits in a loop waiting for data. No component periodically checks if something changed. Events wake up the components that need to respond. When nothing is happening, the CPU sleeps.

### The Command Handler: One Router for All Commands

If every UI panel published commands directly to the robot, and the robot could be either simulated or real, every panel would need to know which robot is active. That logic would be scattered across the codebase.

The **CommandHandler** centralizes routing. It subscribes to `JOINT_COMMAND` and `CARTESIAN_COMMAND`. It knows which robot is active — `SimulatedRobot` or `RealRobot`. It forwards every command to the right place. When the user switches modes, only CommandHandler changes. The panels never know the difference.

### Simulation and Reality: The Same Interface

A `SimulatedRobot` and a `RealRobot` look identical to the rest of the system. Both implement the same interface: `move_joints()`, `move_pose()`, `get_state()`, `connect()`, `disconnect()`. Both publish `ROBOT_STATE` when they move.

The `SimulatedRobot` updates its internal joint angles, solves forward kinematics, and publishes the result. No hardware needed. The `RealRobot` sends the command over RTDE to the physical controller, waits for the robot to move, fetches the actual state, and publishes that. The rest of the platform cannot tell which one is active.

This is the principle: **Movements as Models.** Trajectories, commands, and goals are data. Whether they go to a simulated arm or a real one is an implementation detail.

### The Mode Switch

The user chooses Simulate or Real from a dropdown. `RobotConnectionPanel` calls `RobotManager.set_mode()`. `RobotManager` publishes `MODE_SWITCH_REQUEST`. `CommandHandler` receives it, changes the active robot, and publishes `MODE_SWITCHED`. The UI panels update their indicators. The next command goes to the new active robot.

When switching from Simulate to Real, the real robot's current position is fetched, and the virtual robot snaps to match. The sliders sync once to the real robot's state, then become input-only — they command, the robot follows, the visualizer confirms. No feedback loop. No redundant updates.

---

## Part Five: The Event and Command Flow

If you understand how a joint slider moves the robot, you understand Hatch. Everything else — Cartesian control, mode switching, real hardware — is built on this same pattern. This section traces a single joint command from the moment your finger touches the slider to the moment the robot stops moving and the display updates.

### The Cast of Components

Before tracing the flow, we need to know who is involved:

**The UI Layer** (what you see and touch):
- `JointControlPanel` — six sliders, one per joint. Publishes `JOINT_COMMAND`.
- `CartesianControlPanel` — six sliders for X, Y, Z, RX, RY, RZ. Publishes `CARTESIAN_COMMAND`.

**The Core Services** (the logic that connects everything):
- `StateChannel` — the event bus. Every component publishes here. Every component subscribes here.
- `CommandHandler` — the router. Subscribes to commands. Knows which robot is active.
- `StateHandler` — the synchronizer. Subscribes to `ROBOT_STATE`. Updates model and registry.
- `RobotManager` — the lifecycle manager. Loads URDFs, creates robots, handles connect/disconnect.

**The Robots** (the things that actually move):
- `SimulatedRobot` — a virtual arm. Receives commands, computes FK, publishes `ROBOT_STATE`.
- `RealRobot` — a physical arm. Receives commands over RTDE, publishes `ROBOT_STATE`.

**The Display** (what reflects the robot's state):
- `KinematicDisplay` — VTK actors for every link. Subscribes to registry callbacks.
- `VisualizerEngine` — the render window. 60Hz timer. Renders only when dirty.

### The Flow: Joint Control in Simulate Mode

**Step 1: User drags a slider.** The slider's `valueChanged` signal fires. `JointControlPanel._on_slider_changed` runs. It reads the new slider position, converts it to a joint angle in radians, and publishes a `JOINT_COMMAND` event.

**Step 2: StateChannel delivers the event.** `StateChannel.publish()` creates an `Event` object with the data, source, and a timestamp. It delivers this event to every subscriber of `EventType.JOINT_COMMAND`.

**Step 3: CommandHandler routes the command.** `CommandHandler._on_joint_command` runs. It reads the positions from the event data and calls `self._active_robot.move_joints(positions)`. In Simulate mode, `_active_robot` is the `SimulatedRobot`.

**Step 4: SimulatedRobot executes the command.** `SimulatedRobot.move_joints()` stores the new joint angles, computes forward kinematics, and publishes a `ROBOT_STATE` event.

**Step 5: StateHandler updates the model.** `StateHandler._on_robot_state` runs. It calls `self._model.update_state(joint_positions)`. The kinematic model recomputes every link's world transform. Then `StateHandler` updates every frame in the `TransformRegistry`.

**Step 6: TransformRegistry notifies the display.** When `TransformRegistry.update_frame()` is called, it fires its registered callbacks. `KinematicDisplay._on_transform_updated` runs for each changed frame. It updates the corresponding VTK actor and sets `_needs_render = True`.

**Step 7: VisualizerEngine renders.** The `VisualizerEngine` has a `QTimer` running at 60Hz. On each tick, it checks `_needs_render` on every display. If any display is dirty, it calls `Render()`. If nothing is dirty, it does nothing — the CPU sleeps.

**The flow is complete.** Every component did exactly one thing. No component called another directly. No component polled for changes.

### The Flow: Joint Control in Real Mode

The flow in Real mode is identical in structure, with one critical difference at Step 4: `RealRobot.move_joints()` sends the joint angles to the physical robot controller via RTDE. The controller moves the motors. `RealRobot` then fetches the **actual** joint positions from the controller — which may differ slightly from the commanded positions. It publishes a `ROBOT_STATE` event with the actual positions.

**The driver fetches state only after sending a command.** It does not stream state continuously. This is event-driven: the event is "a command was sent." The response is "fetch the new state." Between commands, nothing happens. The CPU is idle.

### What Goes Wrong: The Feedback Loop

The most persistent bug in Hatch's development was the **slider feedback loop**. In Real mode, after sending a command, the real robot publishes its actual state. If the `JointControlPanel` receives this state and updates its sliders, updating a slider fires `valueChanged`, which publishes a new `JOINT_COMMAND` — an infinite loop.

The fix: the sliders sync to the robot's state exactly once after a mode switch. After that, they ignore `ROBOT_STATE` events. The slider shows what the user commanded, not what the robot actually is. The visualizer shows what the robot actually is. The separation between input (slider) and display (visualizer) breaks the loop.

This bug taught us something fundamental: **input devices should not be display devices.** A slider that both commands the robot and reflects the robot's state is a feedback loop waiting to happen.

---

## Part Six: Cartesian Control and Inverse Kinematics

Joint control is direct: move joint 3 to 0.8 radians, and the elbow bends. Cartesian control is indirect: put the TCP at (0.5, 0.2, 0.8) with the torch pointing down. The user specifies where the tool should be. The platform must figure out what joint angles will put it there.

This is **inverse kinematics** — the hard direction. Forward kinematics always has exactly one answer. Inverse kinematics can have zero, one, or many answers.

### The Decoupling: Position and Orientation

A six-joint robot with a spherical wrist has a crucial property: the last three joints control orientation without affecting position. Their axes intersect at a single point — the wrist center. This means the problem splits cleanly:

1. **Position the wrist center** using joints 1, 2, and 3. A 3-DOF positioning problem solvable with trigonometry.
2. **Orient the tool** using joints 4, 5, and 6. A pure orientation problem.

This decoupling is the foundation of analytical inverse kinematics.

### The Arm Triangle

Joints 1, 2, and 3 form a planar mechanism. Joint 1 rotates the entire arm about the base's vertical axis. Joints 2 and 3 move within that vertical plane. From a top-down view, Joint 1 is simply the horizontal angle to the wrist center. Once the arm plane is aimed, the problem collapses to two dimensions: a triangle formed by the shoulder, the elbow, and the wrist center. The Law of Cosines gives the elbow angle. The shoulder angle follows.

This yields four solutions: shoulder left or right, elbow up or down.

### The Wrist Orientation

With the wrist center positioned, the remaining task is to orient the tool. The target rotation is decomposed into three wrist joint angles. For a standard spherical wrist, this is a Z-Y-Z Euler angle extraction from a rotation matrix.

The wrist contributes 2 solutions (flip or no-flip), bringing the total to 8 configurations.

For the full derivation, see the [Inverse Kinematics in Hatch](../inverse_kinematics.md) document.

---

## Part Seven: Mode Switching and Real Hardware

A robot platform that only simulates is a toy. A robot platform that only controls hardware is a black box. Hatch does both, and the transition between them must be seamless.

### The Three Modes

- **SIMULATE_LOCAL:** Use Hatch's own IK solver. Works offline. Always available.
- **SIMULATE_REAL_IK:** Use the real robot controller's IK solver. Requires a connection.
- **REAL:** Commands go to hardware. The real robot moves.

When the real robot is connected, Hatch automatically upgrades from SIMULATE_LOCAL to SIMULATE_REAL_IK. When the connection drops, it reverts silently.

### The Slider Sync Problem

When switching to Real mode, the sliders must sync to the real robot's actual position. But subscribing to `ROBOT_STATE` for continuous updates creates a feedback loop. The solution: sliders sync exactly once on mode switch by directly querying the real robot's state. After that, they ignore `ROBOT_STATE` events entirely.

### The Driver Interface

Every robot brand has a different communication protocol. Hatch isolates this behind a common interface. `SimulatedRobot` and `RealRobot` both implement it. The rest of the platform never knows which one is active.

### The RTDE Driver

The UR RTDE driver embodies the event-driven philosophy at the hardware level. It does not stream state continuously. It fetches state once after each command. For path following, it opens a scoped receive stream that exists only for the duration of the path. Between commands and between paths, nothing happens.

---

## Part Eight: The Visualizer and the Render Loop

The visualizer is the window into the robot's mind. It does not control anything. It observes. Whatever the kinematic model says is true, the visualizer shows.

### The Render Loop

The `VisualizerEngine` runs a `QTimer` at 60Hz. On each tick, it checks `_needs_render` on every registered display. If any display is dirty, it calls `Render()`. If no display is dirty, it does nothing. The CPU sleeps between ticks.

This is not polling. Polling would recompute transforms or re-render on every tick regardless of need. The render loop checks a single boolean per display — a memory read, not a computation. The flag is set only by `TransformRegistry` callbacks, which fire only when transforms actually change.

### The Joint Frame Display

An optional overlay shows the coordinate frame at every joint and at the TCP. Each frame is drawn as small RGB axes. The TCP frame is drawn in magenta to distinguish it. These frames reveal the kinematic structure that the URDF describes: which joints are coincident, which axes are orthogonal, where the wrist center truly sits. They are the bridge between the abstract URDF and the physical robot.

---

## Part Nine: The URDF Preprocessor and Scene Composition

A robot does not exist in isolation. It sits on a table. A camera is bolted to its wrist. A laser scanner peers down at the workpiece. All of these things must be described and positioned relative to each other. The URDF is the single source of truth for the entire scene.

### Why a Preprocessor?

Xacro lets you define a component once and reuse it. A laser scanner is defined in its own file. The main scene file includes it with a single line. Hatch's preprocessor supports the five features that matter: include, property, variable substitution, macro definition, and macro instantiation.

### The package:// Resolution

URDF files reference mesh files using the `package://` URI scheme. Hatch searches for packages in configured directories. The convention is ROS-standard: place your packages in `~/hatch/assets/` and everything resolves automatically.

### The Scene URDF

Everything is in the URDF. No separate world file. No launch file. No external configuration. Fixed joints from `world` position every object. One format. One parser. One truth.

This is the principle: **Everything in URDF.**

---

*This walkthrough was written during the development of Hatch as a way to trace the chain of reasoning behind every architectural decision. It is preserved here as a historical record. For the current architecture, see the [Architecture document](../architecture.md).*