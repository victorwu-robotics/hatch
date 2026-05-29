## Part Five: The Event and Command Flow

If you understand how a joint slider moves the robot, you understand Hatch. Everything else — Cartesian control, mode switching, real hardware — is built on this same pattern. This section traces a single joint command from the moment your finger touches the slider to the moment the robot stops moving and the display updates.

### The Cast of Components

Before tracing the flow, we need to know who is involved:

**The UI Layer** (what you see and touch):
- `JointControlPanel` — six sliders, one per joint. Publishes `JOINT_COMMAND`. Subscribes to `ROBOT_STATE` only on mode switch to sync sliders.
- `CartesianControlPanel` — six sliders for X, Y, Z, RX, RY, RZ. Publishes `CARTESIAN_COMMAND`.

**The Core Services** (the logic that connects everything):
- `StateChannel` — the event bus. Every component publishes here. Every component subscribes here. Nothing else connects components.
- `CommandHandler` — the router. Subscribes to `JOINT_COMMAND` and `CARTESIAN_COMMAND`. Knows which robot is active. Forwards commands to the right place.
- `StateHandler` — the synchronizer. Subscribes to `ROBOT_STATE`. Updates the kinematic model and transform registry. The single owner of spatial state during operation.
- `RobotManager` — the lifecycle manager. Loads URDFs, creates robots, handles connect/disconnect.

**The Robots** (the things that actually move):
- `SimulatedRobot` — a virtual arm. Receives commands, computes forward kinematics, publishes `ROBOT_STATE`.
- `RealRobot` — a physical arm. Receives commands over RTDE, sends them to hardware, reads back actual positions, publishes `ROBOT_STATE`.

**The Display** (what reflects the robot's state):
- `KinematicDisplay` — VTK actors for every link. Subscribes to `TransformRegistry` callbacks. Sets `_needs_render = True` when transforms change.
- `VisualizerEngine` — the render window. Runs a 60Hz timer. Checks `_needs_render` on each display. Renders only when dirty.

### The Flow: Joint Control in Simulate Mode

Here is the complete chain, step by step:

**Step 1: User drags a slider**

The slider's `valueChanged` signal fires. `JointControlPanel._on_slider_changed` runs. It reads the new slider position, converts it to a joint angle in radians, and publishes a `JOINT_COMMAND` event:

```
Event: JOINT_COMMAND
Data: {positions: [0.0, -0.5, 0.8, 0.0, 0.0, 0.0], names: ["shoulder_pan", ...]}
Source: "joint_control_panel"
```

**Step 2: StateChannel delivers the event**

`StateChannel.publish()` creates an `Event` object with the data, source, and a timestamp. It delivers this event to every subscriber of `EventType.JOINT_COMMAND`.

**Step 3: CommandHandler routes the command**

`CommandHandler` subscribed to `JOINT_COMMAND` during initialization. Its `_on_joint_command` method runs. It reads the positions from the event data and calls:

```python
self._active_robot.move_joints(positions)
```

In Simulate mode, `_active_robot` is the `SimulatedRobot`.

**Step 4: SimulatedRobot executes the command**

`SimulatedRobot.move_joints()` stores the new joint angles, computes forward kinematics (where is every link now?), and publishes a `ROBOT_STATE` event:

```
Event: ROBOT_STATE
Data: {joint_positions: [...], tcp_pose: [...], timestamp: ..., source: "simulated_robot"}
Source: "simulated_robot"
```

**Step 5: StateHandler updates the model**

`StateHandler` subscribed to `ROBOT_STATE` during initialization. Its `_on_robot_state` method runs. It calls:

```python
self._model.update_state(joint_positions)
```

The kinematic model recomputes every link's world transform. Then `StateHandler` calls `_update_transform_registry()`, which updates every frame in the `TransformRegistry` with the new parent-relative transforms.

**Step 6: TransformRegistry notifies the display**

When `TransformRegistry.update_frame()` is called, it fires its registered callbacks. `KinematicDisplay` registered a callback during initialization. `_on_transform_updated` runs for each changed frame. It reads the new world transform, updates the corresponding VTK actor, and sets `_needs_render = True`.

**Step 7: VisualizerEngine renders**

The `VisualizerEngine` has a `QTimer` running at 60Hz. On each tick, it checks `_needs_render` on every display. If any display is dirty, it calls `Render()` on the VTK window. If nothing is dirty, it does nothing — the CPU sleeps.

**The flow is complete.** The slider moved. The command was published. The simulated robot executed it. The state was published. The model was updated. The display reflected the change. Every component did exactly one thing. No component called another directly. No component polled for changes.

### The Flow: Joint Control in Real Mode

The flow in Real mode is identical in structure, with one critical difference at Step 4:

**Step 4 (Real): RealRobot executes the command**

`RealRobot.move_joints()` sends the joint angles to the physical robot controller via RTDE. The controller moves the motors. `RealRobot` then fetches the **actual** joint positions from the controller — which may differ slightly from the commanded positions due to physics, calibration, or the robot still being in motion. It publishes a `ROBOT_STATE` event with the actual positions.

```
Event: ROBOT_STATE
Data: {joint_positions: [actual positions from hardware], ..., source: "real_robot"}
Source: "real_robot"
```

Steps 5-7 are identical. The virtual robot reflects the real robot's actual state, not the commanded state.

**The driver fetches state only after sending a command.** It does not stream state continuously. This is event-driven per Principle #2: the event is "a command was sent." The response is "fetch the new state." Between commands, nothing happens. The CPU is idle.

### The Mode Switch

When the user switches from Simulate to Real:

1. `RobotConnectionPanel` calls `RobotManager.set_mode("real")`
2. `RobotManager` publishes `MODE_SWITCH_REQUEST`
3. `CommandHandler` receives it. It checks that the real robot is connected. It sets `_active_robot = self._real_robot`. It sets `_current_mode = Mode.REAL`. It publishes `MODE_SWITCHED`.
4. `JointControlPanel` receives `MODE_SWITCHED`. It resets a flag that tells it to sync sliders to the next `ROBOT_STATE`.
5. `RealRobot` publishes a `ROBOT_STATE` with the current hardware position.
6. `StateHandler` updates the model. The virtual robot snaps to the real robot's pose.
7. `JointControlPanel` receives the `ROBOT_STATE`, syncs its sliders once, and sets the flag to stop further syncs.

After this, joint commands go to the real robot. The sliders are input-only. The visualizer reflects the real robot's state. The feedback loop is impossible because the sliders do not update from state after the initial sync.

### What Goes Wrong: The Feedback Loop

The most persistent bug in Hatch's development was the **slider feedback loop**. Here is what happened and why:

In Real mode, after sending a command, the real robot publishes its actual state. The `JointControlPanel` receives this state and updates its sliders to match. But updating a slider fires `valueChanged`, which publishes a new `JOINT_COMMAND`. This command goes to the real robot, which moves slightly (or doesn't, because it's already there), publishes a new state, which updates the sliders again, which fires another command... an infinite loop.

The fix: the sliders sync to the robot's state exactly once after a mode switch. After that, they ignore `ROBOT_STATE` events. The slider shows what the user commanded, not what the robot actually is. The visualizer shows what the robot actually is. The separation between input (slider) and display (visualizer) breaks the loop.

This bug taught us something fundamental: **input devices should not be display devices.** A slider that both commands the robot and reflects the robot's state is a feedback loop waiting to happen. The visualizer is the display. The sliders are the input. They serve different purposes. They should not be coupled.

### The Principles in Action

This flow embodies four principles:

- **Principle #2: Event-Driven, No Polling.** Every action starts with an event. No component sits in a loop waiting. The render timer checks a dirty flag — a single boolean — not recomputing transforms.
- **Principle #6: Time = StateChannel.** All events flow through one bus. Components are decoupled. You can add a logger, a safety monitor, or a new control panel without modifying existing code.
- **Principle #7: Movements as Models.** The command is data. The state is data. Whether the robot is simulated or real is an implementation detail behind a common interface.
- **Principle #9: UI Separate from Services.** The slider publishes events. It does not call the robot. The robot publishes state. It does not update the UI. The StateChannel is the only connection.

