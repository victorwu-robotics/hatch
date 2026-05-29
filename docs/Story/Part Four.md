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

This is Principle #6: **Time = StateChannel.** All events flow through here. Timestamps preserve sequence. History is available for debugging. Decoupled communication.

And Principle #2: **Event-Driven, No Polling.** No component sits in a loop waiting for data. No component periodically checks if something changed. Events wake up the components that need to respond. When nothing is happening, the CPU sleeps.

### The Command Handler: One Router for All Commands

If every UI panel published commands directly to the robot, and the robot could be either simulated or real, every panel would need to know which robot is active. That logic would be scattered across the codebase.

The **CommandHandler** centralizes routing. It subscribes to `JOINT_COMMAND` and `CARTESIAN_COMMAND`. It knows which robot is active — `SimulatedRobot` or `RealRobot`. It forwards every command to the right place. When the user switches modes, only CommandHandler changes. The panels never know the difference.

### Simulation and Reality: The Same Interface

A `SimulatedRobot` and a `RealRobot` look identical to the rest of the system. Both implement the same interface: `move_joints()`, `move_pose()`, `get_state()`, `connect()`, `disconnect()`. Both publish `ROBOT_STATE` when they move.

The `SimulatedRobot` updates its internal joint angles, solves forward kinematics, and publishes the result. No hardware needed. The `RealRobot` sends the command over RTDE to the physical controller, waits for the robot to move, fetches the actual state, and publishes that. The rest of the platform cannot tell which one is active.

This is Principle #7: **Movements as Models.** Trajectories, commands, and goals are data. Whether they go to a simulated arm or a real one is an implementation detail.

### The Mode Switch

The user chooses Simulate or Real from a dropdown. `RobotConnectionPanel` calls `RobotManager.set_mode()`. `RobotManager` publishes `MODE_SWITCH_REQUEST`. `CommandHandler` receives it, changes the active robot, and publishes `MODE_SWITCHED`. The UI panels update their indicators. The next command goes to the new active robot.

When switching from Simulate to Real, the real robot's current position is fetched, and the virtual robot snaps to match. The sliders sync once to the real robot's state, then become input-only — they command, the robot follows, the visualizer confirms. No feedback loop. No redundant updates.

