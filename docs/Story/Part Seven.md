## Part Seven: Mode Switching and Real Hardware

A robot platform that only simulates is a toy. A robot platform that only controls hardware is a black box. Hatch does both, and the transition between them must be seamless. The user should be able to test a trajectory in simulation, connect to the real robot, run the same trajectory, and see the same result. The platform should handle the differences transparently.

### The Three Modes

Hatch has three operating modes, but the user only sees two:

**Simulate** — the virtual robot moves. No hardware required. This is the default mode when Hatch starts. The user can load a URDF, move joints, test Cartesian poses, and verify trajectories. The IK solver runs locally. The visualizer shows the result.

**Real** — the physical robot moves. Requires a network connection to the robot controller. Commands go to hardware. The real robot's state is fetched and displayed on the virtual robot. The user sees what the hardware is actually doing.

Behind the scenes, Simulate mode has two sub-modes for IK source:

- **SIMULATE_LOCAL:** Use Hatch's own IK solver. Works offline. Always available.
- **SIMULATE_REAL_IK:** Use the real robot controller's IK solver. Requires a connection. Gives more accurate results for UR robots because the controller uses factory-calibrated parameters.

When the real robot is connected, Hatch automatically upgrades from SIMULATE_LOCAL to SIMULATE_REAL_IK. The user doesn't need to know this happened — the Cartesian panel shows "IK: Real robot solver" as a quiet indication. When the connection drops, it reverts silently.

### The Mode Switch Flow

Switching from Simulate to Real is a coordinated sequence:

1. User selects "Real" from the dropdown in `RobotConnectionPanel`
2. `RobotConnectionPanel` calls `RobotManager.set_mode("real")`
3. `RobotManager` checks that the real robot is connected. If not, it publishes an error and aborts.
4. `RobotManager` publishes `MODE_SWITCH_REQUEST`
5. `CommandHandler` receives it. It sets `_active_robot = self._real_robot`. It sets `_current_mode = Mode.REAL`. It publishes `MODE_SWITCHED`.
6. `JointControlPanel` receives `MODE_SWITCHED`. It fetches the real robot's current joint positions and syncs its sliders once.
7. `CartesianControlPanel` receives `MODE_SWITCHED`. It fetches the real robot's current TCP pose and syncs its target sliders once.
8. The virtual robot snaps to the real robot's actual pose.
9. Subsequent commands go to the real robot. The virtual robot tracks the real robot's state.

Switching back to Simulate:

1. User selects "Simulate" from the dropdown
2. `CommandHandler` sets `_active_robot = self._simulated_robot`
3. The virtual robot stays at the last known position — it does not snap back to zero
4. Subsequent commands go to the simulated robot only

### The Slider Sync Problem

The most persistent bug in Hatch's development was the slider feedback loop during mode switching. Here is the problem and its solution:

**The problem:** When switching to Real mode, the sliders must sync to the real robot's actual position. But if the sliders subscribe to `ROBOT_STATE` events, every state update from the real robot changes the slider values, which fires `valueChanged`, which publishes a new `JOINT_COMMAND`, which moves the robot, which publishes a new state — an infinite loop.

**The solution:** Sliders sync exactly once on mode switch, by directly querying the real robot's state. After that, they ignore `ROBOT_STATE` events entirely. The slider is an input device, not a display. The visualizer is the display.

```python
# In _on_mode_switched:
if mode == "real":
    state = self.robot_manager._real_robot.get_state()
    positions = state.get('joint_positions')
    if positions:
        self._update_ui_from_positions(positions)
```

This is a one-time fetch, not a subscription. The sliders reflect the real robot's position at the moment of the switch. After that, the user moves the sliders, the robot follows, and the visualizer confirms. No loop. No feedback.

### The Connection Lifecycle

Connecting to a real robot is a separate flow from mode switching:

1. User enters the robot's IP address and clicks "Connect"
2. `RobotConnectionPanel` calls `RobotManager.connect_robot(ip)`
3. `RobotManager` delegates to `RealRobot.connect(ip)`
4. `RealRobot` delegates to `URRobotDriver.connect(ip)`
5. The driver establishes an RTDE connection to the controller
6. On success, the driver publishes `CONNECTION_ESTABLISHED`
7. `CommandHandler` receives it. If in Simulate mode, it upgrades the IK source from local to real.
8. The connection status indicator turns green. The "Connect" button becomes "Disconnect."

Disconnecting follows the reverse path. The driver closes the RTDE connection and publishes `CONNECTION_LOST`. `CommandHandler` downgrades the IK source. The status indicator turns red.

The connection and mode are independent. You can be connected to the real robot but stay in Simulate mode — the virtual robot moves, but the IK uses the real controller's solver for better accuracy. You can be in Real mode without a connection — the platform will refuse to switch and show an error.

### The Driver Interface

Every robot brand has a different communication protocol. Hatch isolates this behind a common interface:

```python
class RobotInterface(ABC):
    def move_joints(self, positions) -> bool
    def move_pose(self, pose, frame) -> bool
    def get_state(self) -> Dict
    def is_connected(self) -> bool
    def connect(self, ip, **kwargs) -> bool
    def disconnect(self) -> None
    def stop(self) -> None
```

`SimulatedRobot` and `RealRobot` both implement this interface. The rest of the platform — `CommandHandler`, `StateHandler`, the UI panels — never know which one is active. They call the same methods. They receive the same events. The interface is the contract.

This is Principle #7: **Movements as Models.** The command is data. The state is data. Whether the robot is silicon or simulation is an implementation detail.

### The RTDE Driver

The UR RTDE driver deserves special attention because it embodies the event-driven philosophy at the hardware level.

The driver does not stream state continuously. It fetches state once after each command. When `send_joint_command` is called, the driver sends the angles via `moveJ` (which blocks until the robot reaches the target), then calls `getActualQ()` to read the final position, and publishes a `ROBOT_STATE` event. Between commands, nothing happens. The driver is idle. The CPU is idle.

For path following — where the robot must trace a continuous trajectory — the driver will support a scoped receive stream: open a stream when the path begins, fetch state at ~30Hz during the motion, close the stream when the path ends. The stream is scoped to the path execution event. It is not permanent. It does not poll. It starts and stops with the task that needs it.

This is Principle #2 in action: **Event-Driven, No Polling.** The event is "a command was sent" or "a path was started." The response is fetching state. When the event ends, the fetching stops.

