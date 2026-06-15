# Path Execution in Hatch: The UR RTDE Driver

## The Problem

A welding robot must trace a seam — not move to a single point and stop, but follow a continuous path of dozens or hundreds of target poses. The visualizer must track the real robot's motion in real time, showing the torch moving along the seam as it welds. The user must see what the robot is doing, not just where it ended up.

This requires:

1. Sending a sequence of move commands without blocking between each one
2. Receiving the robot's actual state continuously during the motion
3. Displaying that state on the virtual robot in real time
4. Stopping the state stream when the path is complete

The naive approach — a permanent receive loop streaming state at 125Hz — floods the GUI and burns CPU even when the robot is idle. The URx library attempted this with `is_program_running()` polling, but relied on timing heuristics that proved unreliable.

## The Hatch Approach: Scoped Execution

Hatch does not stream state continuously. It fetches state only when the robot is moving. The path execution is a single method call, triggered by an event (a user button or a planner publishing `PATH_READY`). The method internally manages the receive stream for the duration of the path.

**External view (what the platform sees):**

```python
# Triggered by an event — a single call
real_robot.execute_path(pose_list)
```

**Internal implementation (what the driver does):**

1. Open a receive stream (start fetching state at ~30Hz)
2. For all poses except the last: `moveL(pose, asynchronous=True)`
3. For the final pose: `moveL(pose)` — blocking, waits for completion
4. Close the receive stream
5. Return

Each state update during the stream publishes a `ROBOT_STATE` event. `StateHandler` updates the kinematic model. `KinematicDisplay` updates the VTK actors. The visualizer tracks the real robot's motion in real time. When the path completes, the stream closes. No more state updates. The platform returns to idle.

**Why this is event-driven, not polling:**

The receive stream is scoped to the path execution event. It starts when the path begins and stops when the path ends. Between paths, nothing streams. The CPU is idle. The GUI is responsive. No component outside the driver knows or cares that a stream existed — they only see `ROBOT_STATE` events, which arrive at 30Hz during motion and stop when motion stops.

This is the same pattern as `send_joint_command`: a single method call that blocks internally while the robot moves, fetches state when done, and returns. The platform sees an event-driven interface. The driver owns the complexity of talking to hardware.

## Guidance for Other Robot Brands

Every industrial robot has a different communication protocol:

| Brand | Protocol | Command Style |
|-------|----------|---------------|
| Universal Robots | RTDE | Blocking or asynchronous moves |
| KUKA | KRL via EthernetKRL | Submit program, poll status |
| ABB | RAPID via EGM/RI | Streaming position commands |
| Fanuc | Karel via socket | Submit TP program, poll registers |

Each driver must implement `execute_path()` using whatever the protocol provides. The principle is the same regardless of brand:

1. The driver owns the communication pattern — the platform doesn't care how it works
2. State is published as `ROBOT_STATE` events — the platform doesn't know if they came from a stream or a single fetch
3. The stream is scoped to the path — never permanent, never polling
4. When the path ends, the driver stops fetching state — the platform returns to idle

A KUKA driver might poll a status register at 10Hz during path execution. An ABB driver might read the position from the EGM stream. A Fanuc driver might wait for a "program complete" register to change. The implementation differs. The interface is the same: `execute_path(pose_list)` → the platform receives `ROBOT_STATE` events → the visualizer tracks the motion → the stream stops when done.

## Historical Note: The URx Failure

The URx library (pre-2016) attempted path following by sending asynchronous move commands and polling `is_program_running()` in a loop. This method relied on timing heuristics — checking whether enough time had passed to assume the program finished. It was unreliable because network latency, robot speed, and controller load all affected the timing. When it broke, the community abandoned URx in favor of `ur-rtde`, which provides direct access to the controller's actual state.

Hatch's RTDE driver uses the controller's native state reporting, not timing heuristics. The `robot_status` field in the RTDE protocol tells you definitively whether the robot is moving, idle, or in error. No guesswork. No timing assumptions. Just the controller's own state, fetched when needed, scoped to the task at hand.

