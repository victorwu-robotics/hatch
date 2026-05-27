Here's the document.

---

# The RTDE Driver: A Case Study in Event-Driven Architecture

## Why Hatch's Principle #2 Matters

---

## Part One: The Problem

Hatch needed to control a real UR10 robot. The robot communicates via RTDE (Real-Time Data Exchange) — a protocol that sends joint positions, velocities, and TCP pose over a TCP connection. A Python library (`ur-rtde`) provides the interface.

The original `URRobotDriver` had a method `_update_state_from_rtde()` that called `getActualQ()` to fetch the latest joint positions from the robot. This method was called exactly once — during `connect()`, to verify the connection was working. After that, it was never called again.

When the user moved a joint slider, `send_joint_command()` sent the new angles to the robot via `moveJ()`. The real robot moved. But nobody asked the robot where it was now. The `_update_state_from_rtde()` method sat unused. The `RealRobot` never published a `ROBOT_STATE` event. The virtual robot in the visualizer stayed frozen at its initial position.

**The user moved the sliders. The real robot obeyed. The display never updated.**

The problem was not that the robot failed to move. It moved. The problem was that Hatch never asked it where it went. You must fetch state from the robot. If you don't ask, you don't know.

---

## Part Two: The Wrong Fix

The obvious fix: call `_update_state_from_rtde()` repeatedly. The robot streams data at 125Hz, so we should read it at 125Hz. A receive loop was added:

```python
def receive_loop():
    while not stop:
        if connected:
            _update_state_from_rtde()  # blocks until data arrives
```

This worked. The virtual robot synced to the real robot. The display updated. But everything became sluggish. The GUI froze. Sliders wouldn't move. Zooming took seconds.

The loop was "event-driven" in name only. `getActualQ()` blocks at the OS level until data arrives — no CPU is burned waiting. But when data arrives (125 times per second, every 8 milliseconds), the following chain reaction fires:

1. `state_signal` emitted via Qt signal
2. `RealRobot._on_driver_state` publishes `ROBOT_STATE` event
3. `StateHandler` updates the kinematic model (forward kinematics on all links)
4. `StateHandler` updates the `TransformRegistry` (23+ frames recomputed)
5. `TransformRegistry` fires callbacks to `KinematicDisplay`
6. `KinematicDisplay._on_transform_updated` updates VTK actors for all links
7. `KinematicDisplay` sets `_needs_render = True`
8. `VisualizerEngine`'s 60Hz timer renders the frame

That's hundreds of Python function calls, NumPy matrix multiplications, and VTK pipeline updates — every 8 milliseconds — whether the robot moved or not. Most of the time, the robot is stationary. The joint angles haven't changed. But the entire pipeline runs anyway.

**The loop didn't burn CPU waiting for data. It burned CPU processing data that hadn't changed.**

This is the hidden trap of continuous streaming. The I/O is efficient (blocking reads). The processing is wasteful (recomputing everything on every read, even when nothing changed). The GUI drowns not in data, but in redundant computation triggered by data.

---

## Part Three: The Event-Driven Insight

The robot does not move by itself. It moves because Hatch sent it a command. The user dragged a slider. `JOINT_COMMAND` was published. `CommandHandler` called `send_joint_command()`. The robot moved.

That is the event. The user acted. The robot responded.

Between commands, the robot is stationary. There is no event. Nothing happens. Nothing should happen. The CPU should sleep. The GUI should be idle. The display should show the last known state — which hasn't changed.

**The true event-driven approach:**

1. User moves slider → `JOINT_COMMAND` event
2. `CommandHandler` sends command to robot
3. Robot moves
4. **Fetch state once:** call `_update_state_from_rtde()`
5. Publish `ROBOT_STATE` with new joint positions
6. `StateHandler` updates model and registry (once)
7. Display renders the new pose (once)
8. **Done. Nothing else happens until the next command.**

No loop. No streaming. No redundant processing. The driver fetches state exactly once per command, right after telling the robot to move. Between commands, the driver does nothing. The thread is idle. The CPU is available.

This is what Principle #2 means: **Event-Driven, No Polling.** The event is the command. The response is the state fetch. There is nothing in between. No periodic checks. No "is there new data?" No "has anything changed?" If nothing happened, nothing runs.

---

## Part Four: The Implementation

The change is three lines added to the driver:

```python
def send_joint_command(self, positions):
    success = self._rtde_c.moveJ(positions, 0.5, 0.5)
    if success:
        time.sleep(0.05)              # Brief pause for robot to respond
        self._update_state_from_rtde() # Fetch new state once
    return success
```

Same for `send_cartesian_command`. That's it. The receive loop is deleted. The threading is removed. The throttle is unnecessary. The driver is simpler than either the original (which never updated) or the looped version (which updated too much).

---

## Part Five: The Lesson

This is not just a bug fix. It is a vivid illustration of why the Ten Principles exist and why they are tested against reality.

**Principle #2: Event-Driven, No Polling** was violated twice — first by omission (never fetching state), then by excess (fetching state continuously). The first violation broke functionality. The second broke performance. Both came from not thinking clearly about what the event actually is.

The event is not "data arrived on the RTDE socket." That is an implementation detail of the transport layer. The event is "the user commanded the robot to move." State should be fetched in response to that event — not on a timer, not in a loop, not continuously.

A driver that fetches state once per command is simpler, faster, and more correct than one that streams continuously. It uses less CPU, floods the GUI less, and aligns perfectly with the event-driven architecture.

The continuous streaming pattern — reading sensor data in a loop and pushing it to subscribers — is a ROS convention. It works for ROS because ROS nodes are separate processes communicating over DDS, and each node can process data at its own rate. But Hatch is a single process. Every "event" triggers a cascade of in-process computation. Streaming data at 125Hz into a single-process application is not event-driven — it's a denial-of-service attack on your own GUI.

**Ask of any component: "Does it ever do work when nothing has changed?" If yes, it is not event-driven.**

The RTDE receive loop did work when nothing had changed — 125 times per second. The fix is to do work only when something has changed: when a command was sent. That is event-driven. That is Hatch.