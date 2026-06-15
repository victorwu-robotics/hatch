# Integrating Hardware with Hatch

This document covers everything you need to add a robot, camera, or sensor to Hatch.
It begins with the core driver pattern — event-driven, no polling — and builds
through reference implementations, camera integration, and a real-world case study
of reverse-engineering a proprietary sensor.

---

# Part I: The Event-Driven Driver Pattern

## 1.1 Why Not Stream Continuously?

Hatch's Principle is: **Event-Driven, No Polling.** This is not a slogan. It is a
hard rule discovered through failure.

Ask of any component: *"Does it ever do work when nothing has changed?"* If yes,
it is not event-driven.

Most robotics platforms stream data continuously. ROS nodes publish at fixed rates.
Drivers read sensors in loops. This works when components are separate processes
on separate machines. But Hatch is a single process. Every "event" triggers a
cascade of in-process computation. Streaming data at 125Hz into a single-process
application is not event-driven — it's a denial-of-service attack on your own GUI.

The event-driven alternative: fetch state only when something changed. A command was
sent. A path was started. A sensor was triggered. When the event ends, the fetching
stops. The CPU returns to idle.

## 1.2 The RTDE Case Study

This is the story of how we learned Principle #2 the hard way — by breaking Hatch,
then fixing it, then understanding why the fix was correct.

### The Original Problem

Hatch needed to control a real UR10 robot via RTDE (Real-Time Data Exchange).
A Python library (`ur-rtde`) provides the interface. The original driver had a
method `_update_state_from_rtde()` that fetched the latest joint positions. It was
called exactly once — during `connect()`, to verify the connection. After that,
it was never called again.

The user moved a slider. The real robot moved. But nobody asked the robot where
it was now. The virtual robot stayed frozen at its initial position.

**The problem was not that the robot failed to move. It moved. The problem was
that Hatch never asked it where it went.**

### The Wrong Fix

The obvious fix: call `_update_state_from_rtde()` in a continuous loop at 125Hz.
The robot streams data, so we should read it.

```python
def receive_loop():
    while not stop:
        if connected:
            _update_state_from_rtde()  # blocks until data arrives
```

This worked. The virtual robot synced to the real robot. The display updated.
But everything became sluggish. The GUI froze. Sliders wouldn't move.

The loop didn't burn CPU waiting for data — `getActualQ()` blocks at the OS level.
But when data arrived (125 times per second, every 8 milliseconds), the following
chain reaction fired:

1. Qt signal emitted
2. `ROBOT_STATE` event published
3. `StateHandler` updates kinematic model (forward kinematics on all links)
4. `StateHandler` updates `TransformRegistry` (23+ frames recomputed)
5. `TransformRegistry` fires callbacks to `KinematicDisplay`
6. `KinematicDisplay` updates VTK actors for all links
7. `KinematicDisplay` sets `_needs_render = True`
8. `VisualizerEngine` renders the frame

Hundreds of Python function calls, NumPy matrix multiplications, and VTK pipeline
updates — every 8 milliseconds — whether the robot moved or not.

**The loop burned CPU processing data that hadn't changed.**

### The Event-Driven Insight

The robot does not move by itself. It moves because Hatch sent it a command.
The user dragged a slider. `JOINT_COMMAND` was published. The robot moved.

That is the event. The user acted. The robot responded.

Between commands, the robot is stationary. There is no event. Nothing happens.
Nothing should happen. The CPU should sleep.

**The true event-driven approach:**

1. User moves slider → `JOINT_COMMAND` event
2. `CommandHandler` sends command to robot
3. Robot moves
4. **Fetch state once** — call `_update_state_from_rtde()`
5. Publish `ROBOT_STATE` with new joint positions
6. `StateHandler` updates model and registry (once)
7. Display renders the new pose (once)
8. **Done. Nothing else happens until the next command.**

No loop. No streaming. No redundant processing.

### The Lesson

The event is not "data arrived on the RTDE socket." That is an implementation
detail of the transport layer. The event is "the user commanded the robot to move."
State should be fetched in response to that event — not on a timer, not in a loop,
not continuously.

This is Principle #2 in action: **Event-Driven, No Polling.**

---

## 1.3 The Command-Response Pattern (Reference)

The RTDE driver implements a blocking command-response pattern. State is published
only when it changes — after each command completes.

### Joint Command

```python
def send_joint_command(self, positions):
    """Move robot to target joint positions. Blocks until complete."""
    success = self._rtde_c.moveJ(positions, speed=0.5, acceleration=0.5)
    if success:
        time.sleep(0.05)              # Brief pause for controller
        self._update_state_from_rtde() # Fetch new state once
    return success
```

### Cartesian Command

```python
def send_cartesian_command(self, pose_list):
    """Move robot to target Cartesian pose. Blocks until complete."""
    success = self._rtde_c.moveL(pose_list, speed=0.5, acceleration=0.5)
    if success:
        time.sleep(0.05)
        self._update_state_from_rtde()
    return success
```

### The `ROBOT_STATE` Event

After each command, the driver publishes:

```python
self._channel.publish(
    EventType.ROBOT_STATE,
    data={
        'joint_positions': final_q,
        'tcp_pose': final_tcp,
        'timestamp': time.time()
    },
    source="real_robot"
)
```

---

## 1.4 Path Execution: Scoped Streaming

For path following — where the robot must trace a continuous trajectory of dozens
or hundreds of poses — a single blocking call per point would be too slow. The
visualizer must track the robot's motion in real time.

Hatch uses **scoped streaming**: a temporary receive stream that exists only for
the duration of the path.

### The Pattern

```python
def execute_path(self, pose_list):
    """Execute a path of poses with real-time state updates."""
    # 1. Open receive stream (start fetching state at ~30Hz)
    self._rtde_r.start_streaming()

    # 2. Send all poses except the last asynchronously
    for pose in pose_list[:-1]:
        self._rtde_c.moveL(pose, speed, acc, async=True)

    # 3. Send final pose — blocking, waits for completion
    self._rtde_c.moveL(pose_list[-1], speed, acc)

    # 4. Close receive stream
    self._rtde_r.stop_streaming()
```

During the stream, each state update publishes a `ROBOT_STATE` event. The
visualizer tracks the real robot's motion. When the path completes, the stream
closes. No more state updates. The platform returns to idle.

### Why This Is Still Event-Driven

The receive stream is scoped to the path execution event. It starts when the path
begins and stops when the path ends. Between paths, nothing streams. No component
outside the driver knows a stream existed — they only see `ROBOT_STATE` events,
which arrive at ~30Hz during motion and stop when motion stops.

### Guidance for Other Robot Brands

Every industrial robot has a different protocol. The principle is the same:

| Brand | Protocol | Approach |
|-------|----------|----------|
| Universal Robots | RTDE | Blocking or asynchronous moves |
| KUKA | KRL via EthernetKRL | Submit program, poll status register during path |
| ABB | RAPID via EGM/RI | Streaming position commands |
| Fanuc | Karel via socket | Submit TP program, poll registers |

Each driver must implement `execute_path()` using whatever the protocol provides.
The platform doesn't care how it works — it only sees `ROBOT_STATE` events.

---

# Part II: Adding a Camera

## 2.1 Camera Pipeline Architecture

Hatch supports RGB-D cameras (Orbbec Gemini 335, Intel RealSense D435) as
point cloud sources. The pipeline follows the event-driven architecture:

```
CameraDriver.capture_raw_pointcloud()
    ↓ raw (N,3) points in optical frame
PointCloudProcessor.process_frame()
    ↓ ROI clipping, world transform
PointCloudRenderer.update_point_cloud()
    ↓ zero-copy VTK update, sets _needs_render = True
VisualizerEngine (60Hz timer)
    ↓ checks _needs_render, calls Render()
Screen
```

Everything runs in the main thread — no separate threads, no signal queues.

## 2.2 Setting Up a Camera in Your URDF

Include the camera's URDF and mount it to your robot. For an Orbbec Gemini 335:

```xml
<!-- Include the camera definition -->
<xacro:include filename="$(find orbbec_camera)/urdf/gemini_335_336.urdf.xacro"/>

<!-- Mount the camera to the robot wrist -->
<joint name="wrist_to_camera" type="fixed">
  <parent link="wrist_3_link"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

Hatch automatically detects the depth optical frame by scanning link names
for `depth_optical_frame`. The detected frame is used to query the
`TransformRegistry` for the camera's world transform. As the robot moves,
the point cloud follows correctly.

## 2.3 Performance and Tuning

At 640×360 resolution, the Orbbec Gemini 335 produces ~210,000 points per frame:

| Stage | Time |
|-------|------|
| Capture | ~9ms |
| Process (ROI + transform) | ~8ms |
| Render (VTK) | ~38ms |
| **Total** | **~55ms (~18 FPS)** |

**For higher frame rates, downsample:**

```python
if len(points) > 50000:
    indices = np.random.choice(len(points), 50000, replace=False)
    points = points[indices]
    colors = colors[indices]
```

**Point size tuning:**

```python
self.actor.GetProperty().SetPointSize(2)  # Default
self.actor.GetProperty().SetPointSize(3)  # Larger for better visibility
```

## 2.4 Adding a New Camera Type

1. Create a driver file in `drivers/camera/` inheriting from `BaseCameraDriver`
2. Implement `start_streaming()`, `capture_raw_pointcloud()`, `stop_streaming()`
3. Register the camera in `CameraManager.available_cameras`
4. Add resolution presets in `CameraManager.camera_resolutions`

The processor and renderer work with any camera that produces (N,3) float32
points and (N,3) uint8 colors — no changes needed.

---

# Part III: Case Study — The Keyence Laser Scanner

## 3.1 The Challenge

The Keyence LJ-V7200 laser scanner has no public protocol documentation.
Keyence distributes their driver as a Windows DLL. To use it on Ubuntu with
Python, we had to reverse-engineer the TCP protocol from a ROS-Industrial C++
driver and trial-and-error testing.

This case study records what we got wrong, what we learned, and the principles
that emerged.

## 3.2 Four Mistakes We Made

### Mistake 1: Continuous Streaming vs. On-Demand Requests

**What we tried:** Open a TCP connection and read profiles in a continuous loop.
The scanner blasts data at over 1000 profiles per second. We tried to keep up
with a `while True` loop.

**Why it failed:** The scanner's firehose overwhelmed the OS TCP buffer. When
the buffer filled, the scanner dropped the connection. We also tried draining
the buffer with non-blocking reads, but new data arrived faster than we could
drain it, freezing the GUI.

**The correct approach:** On-demand request-response. Open the connection once.
Send a trigger only when you need a profile. Read exactly one response. The
scanner is silent between requests.

### Mistake 2: The Double-Line Artifact (20-Bit Unpacking)

**What we saw:** The scanner profile appeared as two separate parallel lines
instead of one continuous surface. Z values alternated between two numbers.

**The cause:** The Keyence packs two 20-bit signed depth values into 5 bytes.
The middle byte is shared — its high nibble belongs to point 0, its low nibble
to point 1. Our unpacking misassigned these nibbles.

**The fix:**

```python
p0 = ((b2 & 0x0F) << 16) | (b1 << 8) | b0
p1 = (b4 << 12) | (b3 << 4) | (b2 >> 4)
```

### Mistake 3: The Phantom 0.26208 Depth

**What we saw:** The profile contained a depth value of exactly 0.26208 meters,
even when the scanner was pointed at empty space.

**The cause:** The Keyence uses `-524285` as an error code for out-of-range
pixels. Our unit conversion multiplied this by a scaling factor and flipped
the sign, producing `+0.26208`. We were displaying the scanner's error flag
as if it were a real measurement.

**The physics check:** The LJ-V7200 has a measurement range of 100mm ±20mm.
A value of 262mm is physically impossible — more than double the sensor's range.

**The fix:**

```python
INVALID_LOWER_BOUND = -524280
valid_mask = (raw_z > INVALID_LOWER_BOUND) & (raw_z != 0)
```

### Mistake 4: The 60-Byte ACK Confusion

**What we saw:** When using the 00 00 00 00 single-profile command, the
scanner responded with exactly 60 bytes instead of the expected 2000+ byte
profile.

**The cause:** The 60-byte response was a Command Acknowledgment (ACK) —
the scanner confirming it received the trigger. We were treating the ACK as
profile data. The 00 00 00 00 command is not a profile request — it is an
initialization command that switches the controller from continuous
streaming mode to blocking mode. It only needs to be sent once at the start
of a session.

**The resolution:** After sending 00 00 00 00 once to initialize blocking
mode, all subsequent 01 01 00 00 requests return a full profile immediately
and the scanner waits silently between triggers. No separate fetch step needed.

## 3.3 The Final Architecture

```
Application calls capture_profiles(count=200, interval=0.1)
    ↓
Driver opens TCP socket (once)
    ↓
Send initialization: 00 00 00 00 (switches controller to blocking mode)
    ↓
For each profile:
    sendall(01 01 00 00)    # Now blocking — returns one profile
    recv(4) → response size
    recv(size) → profile data
    unpack 20-bit → raw Z values
    filter invalid points (-524285)
    convert to meters
    flip to optical convention
    return (points, colors)
    sleep(interval)
    ↓
Driver closes TCP socket
```

**Key constants:**
- `KEYENCE_FUNDAMENTAL_LENGTH_UNIT = 1e-8` (0.01 µm per step)
- `KEYENCE_INVALID_LOWER_BOUND = -524280`
- Trigger command ends with `01 01 00 00` (streaming mode, single response)
- Response header: 84 bytes, profile data follows
- 800 points per profile for the LJ-V7200

## 3.4 Lessons for Sensor Integration

1. **Don't assume continuous streaming is the only mode.** Many industrial
   sensors support on-demand request-response but don't document it publicly.

2. **Error codes can look like real data after unit conversion.** Always
   identify and filter error flags before scaling. Check physical limits.

3. **Persistent connections are better than per-request connections.**
   Keep the socket open for the duration of the task.

4. **Sleep between requests, not during reads.** The socket should be read
   as soon as data arrives. Pacing happens between requests.

5. **Reverse-engineering requires multiple sources.** The ROS-Industrial C++
   driver gave us the protocol structure. Physical testing with a target
   object confirmed the results. No single source had the complete answer.

6. **When the sensor has no public documentation, every value is suspect
   until verified against physical reality.** The `0.26208` was mathematically
   correct but physically impossible. The double line was algorithmically valid
   but geometrically wrong. Only testing against a real object could distinguish
   correct behavior from plausible-looking errors.

---

## Summary

The event-driven pattern is not just for robot arms. It applies to every piece
of hardware Hatch integrates. The question is always the same:

> *"Does this component ever do work when nothing has changed?"*

If the answer is yes, it is not event-driven. The RTDE driver learned this
by breaking the GUI. The Keyence driver learned it by drowning in a firehose
of profiles. Both arrived at the same pattern: persistent connection, on-demand
requests, idle between events.

This is Principle #2 in hardware. It is the same principle that governs the
UI, the render loop, and the transform registry. One rule, applied everywhere.

---

*This document combines the Event-Driven Drivers case study, Non-Polling RTDE
Driver reference, Path Execution guide, Camera Integration guide, and Keyence
Scanner reverse-engineering chronicle into a single hardware integration guide.*
```

---

This is the full `integrating_hardware.md`. It preserves all the content from the five source documents, organized under a clear three-part structure. The RTDE story leads because it's the strongest narrative and establishes the pattern that the camera and Keyence sections reinforce.

Do you want to adjust anything, or shall we move on to the next merge — `technical_notes.md`?