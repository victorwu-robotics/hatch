# Hatch (孵) User Guide

## First Steps: Creating Your Scene URDF

Hatch needs one thing to start: a URDF file that describes your robot and its
environment. This file is the single source of truth for everything in the scene —
robot arms, sensors, tools, tables, UGV bases, and their positions relative to
each other.

### What Hatch Supports

Hatch works with **serial robot arms** — a single chain of links connected by 
joints, from a base to a tool endpoint. This covers most industrial robots 
(UR, KUKA, Han's, Fanuc, etc.).

Hatch does not currently support:
- Parallel robots (Stewart platforms, delta robots)
- Branching chains (two-arm robots, humanoids)
- Closed kinematic loops (four-bar linkages)

If your robot is a serial arm, Hatch will detect its kinematic chain 
automatically from the URDF.

### The URDF is the Scene

In Hatch, the URDF file is not just a robot description. It is the complete
definition of the entire scene. There is no separate world file, no launch file,
no external configuration for how things are positioned.

Everything is placed using fixed joints from the `world` link:

```
world
  ├── table (fixed joint)
  ├── robot_base (fixed joint)
  │     └── robot arm links (revolute joints)
  │           └── camera (fixed joint)
  │           └── tool (fixed joint)
  └── UGV base (fixed joint)
```

### Organizing Your Files

Hatch follows the ROS convention for package layout. Each component (robot,
sensor, tool, UGV) lives in its own package directory:

```
~/hatch/assets/
├── scenes/
│   └── my_scene/
│       ├── urdf/
│       │   └── my_scene.urdf     ← The file you load in Hatch
│       └── meshes/
│           └── table.stl
├── robots/
│   └── ur10/
│       ├── urdf/
│       │   └── ur10.urdf
│       └── meshes/
│           ├── base_link.stl
│           └── ...
├── sensors/
│   └── keyence/
│       ├── urdf/
│       │   └── keyence.urdf
│       └── meshes/
│           └── scanner.stl
├── ugv/
│   └── bunker/
│       ├── urdf/
│       │   └── bunker.urdf
│       └── meshes/
│           └── ...
└── tools/
    └── gripper/
        ├── urdf/
        │   └── gripper.urdf
        └── meshes/
            └── ...
```

### Writing Your Scene URDF

There are two ways to create your scene URDF:

#### Option 1: Plain URDF (Simple Scenes)

For simple scenes, write a plain URDF file that references meshes using
`package://` paths:

```xml
<?xml version="1.0"?>
<robot name="my_scene">

  <!-- World origin -->
  <link name="world"/>

  <!-- A table in the scene -->
  <link name="table">
    <visual>
      <geometry>
        <mesh filename="package://my_scene/meshes/table.stl"/>
      </geometry>
    </visual>
  </link>

  <joint name="table_to_world" type="fixed">
    <parent link="world"/>
    <child link="table"/>
    <origin xyz="1.0 0.5 0" rpy="0 0 0"/>
  </joint>

  <!-- Robot arm (include from another file) -->
  <link name="ur10_base_link"/>  <!-- Placeholder, detailed in ur10.urdf -->

  <joint name="robot_to_world" type="fixed">
    <parent link="world"/>
    <child link="ur10_base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- ... more components ... -->

</robot>
```

#### Option 2: Xacro (Complex Scenes with Many Components)

For scenes with many components, Hatch includes a preprocessor that supports
a subset of xacro syntax. This lets you compose your scene from modular files:

```xml
<?xml version="1.0"?>
<robot name="my_scene" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="world"/>

  <!-- Define reusable values -->
  <xacro:property name="table_height" value="0.75"/>
  <xacro:property name="robot_x" value="0.5"/>

  <!-- Include component URDFs -->
  <xacro:include filename="package://my_scene/urdf/table.urdf"/>
  <xacro:include filename="package://ur10/urdf/ur10.urdf"/>
  <xacro:include filename="package://keyence/urdf/keyence.urdf"/>
  <xacro:include filename="package://bunker/urdf/bunker.urdf"/>

  <!-- Position the robot on the table -->
  <joint name="robot_mount" type="fixed">
    <parent link="table_top"/>
    <child link="ur10_base_link"/>
    <origin xyz="${robot_x} 0 ${table_height}" rpy="0 0 0"/>
  </joint>

  <!-- Mount the scanner on the robot wrist -->
  <joint name="scanner_mount" type="fixed">
    <parent link="ur10_wrist_3_link"/>
    <child link="keyence_base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

**Supported xacro features:**

| Feature | Example |
|---------|---------|
| Include files | `<xacro:include filename="package://..."/>` |
| Define variables | `<xacro:property name="x" value="0.5"/>` |
| Use variables | `${x}` in any attribute |
| Define macros | `<xacro:macro name="box" params="size color">...</xacro:macro>` |
| Call macros | `<xacro:box size="0.1" color="0.5 0.5 0.5"/>` |

**Not supported (use full xacro if needed):**

- Python expressions in `${}`
- Conditional blocks (`<xacro:if>`, `<xacro:unless>`)
- ROS-specific features

If your scene requires these features, install xacro separately
(`pip install xacro`) and pre-process your file before loading it in Hatch.

### Mesh Files and package:// Paths

All mesh references must use the `package://` URI scheme:

```xml
<!-- Correct: -->
<mesh filename="package://ur10/meshes/base_link.stl"/>

<!-- Wrong: -->
<mesh filename="/home/user/my_meshes/base_link.stl"/>
<mesh filename="file:///home/user/my_meshes/base_link.stl"/>
<mesh filename="../meshes/base_link.stl"/>
```

The `package://` scheme tells Hatch: "find a directory named `ur10` in one of the
search paths, then look for `meshes/base_link.stl` inside it."

Hatch searches for packages in these locations (in order):

1. The directory containing the URDF file you loaded
2. The parent and grandparent directories of the URDF file
3. `~/hatch/assets/` and its subdirectories (`robots/`, `sensors/`, `ugv/`, `tools/`, `scenes/`)

If you keep your packages in `~/hatch/assets/`, everything resolves automatically.

### The True Kinematic Root

Some robots (like Universal Robots UR10) have a fixed joint with a 180° rotation
between `base_link` and the first moving joint. Hatch automatically detects the
**true kinematic root** — the parent of the first moving joint — for correct
inverse kinematics.

You don't need to do anything special. If your IK results look wrong (robot appears
flipped or rotated), Hatch has already handled this. If you encounter a robot where
the detection fails, please report it.

### Loading Your Scene in Hatch

1. Start Hatch: `python -m ui.main_window`
2. Click **File → Load URDF**
3. Select your `.urdf` or `.xacro` file
4. The scene appears in the 3D view

Hatch processes `.xacro` files automatically — no separate preprocessing step needed.

---

## Controlling Your Robot

Once your scene is loaded, the Motion Control panel appears on the right side
of the window. It has three sections:

### Robot Connection

At the top of the Motion Control panel, you can:

- **Select Mode**: Choose from three operating modes.
  - **Simulate (Local IK)**: A virtual robot moves in the 3D view. No hardware
    required. Uses Hatch's built-in IK solver. Safe for testing.
  - **Simulate (Real IK)**: Uses the real robot controller's IK solver for more
    accurate results, but only moves the virtual robot. Requires a connection.
  - **Real**: Connects to physical hardware. Commands move the real robot.
    Requires network connection to the robot controller.

- **Connect to Hardware**: Enter the robot's IP address and click Connect.
  The RTDE frequency defaults to 125 Hz — this works for most UR robots.
  Once connected, the status indicator turns green. Hatch automatically
  upgrades the IK source from local to real robot solver.

- **Disconnect**: Safely closes the connection to the robot.

### Joint Control

The Joint Control tab shows a slider for each joint in the robot arm.

**How it works:**
1. You move a slider → Hatch publishes a joint command
2. The command goes to the active robot (simulated or real)
3. The robot moves to the commanded position
4. The robot publishes its new state
5. The 3D view updates to reflect the actual position

**Important: Sliders are input devices, not state displays.** The sliders show
your commanded position — what you asked the robot to do. The 3D view shows
what the robot is actually doing. If you need to see the real robot's current
joint angles, they sync to the sliders once when you connect to hardware or
switch to Real mode. After that, the sliders operate independently. This
prevents feedback loops and keeps you in control.

**Tips:**
- **Home Position**: Returns all joints to their neutral (zero) position
- **Zero All**: Sets all joints to zero if within limits, or the closest valid angle
- **Mouse wheel**: Hover over a slider and scroll for fine adjustment
- **Real mode**: Joint labels show "(actual)" to remind you that the hardware is moving

### Cartesian Control

The Cartesian Control tab lets you move the robot's **Tool Center Point (TCP)**
— the point in space where a tool would attach to the robot's wrist. Hatch
automatically detects which link serves as the mounting point by finding the 
last link in the kinematic chain.

**How it works:**
1. You move a slider → Hatch computes the target TCP pose
2. Hatch solves inverse kinematics to find joint angles for that pose
3. The joint command is sent to the active robot
4. The robot moves and publishes its new state
5. The current TCP display updates

**Current TCP Display**: Shows where the TCP actually is, computed from the
robot's current joint positions. This may differ from the target if:
- The IK solver couldn't find an exact solution
- The real robot hasn't reached the target yet
- Joint limits prevent reaching the target

**Step Size**: Controls how much each slider notch changes the pose.
- **1mm**: Fine positioning (0.001 m per step)
- **1cm**: Coarse positioning (0.01 m per step)
- **1°**: Orientation adjustment (about 0.017 rad per step)

**Reset to Current**: Sets the target sliders to match the current TCP pose.
Useful when you want to make a small adjustment from the current position.

### Operating Modes

| Mode | IK Solver | Robot Moves | Use Case |
|------|-----------|-------------|----------|
| Simulate (Local IK) | Hatch's built-in solver | Virtual only | Testing trajectories offline, learning the interface |
| Simulate (Real IK) | Real robot's controller | Virtual only | Validating IK against real controller before moving hardware |
| Real | Real robot's controller | Physical hardware | Production use |

**Switching between modes:** The robot must be connected to use Real IK or
Real mode. When switching to Real mode, the virtual robot snaps to the real
robot's current position, and the joint sliders sync once to show where the
hardware actually is. After that, sliders return to showing your commands.

**Switching back to Simulate:** The virtual robot stays at the last known
position. You can continue testing without the hardware.

---

## Understanding the 3D View

### Camera Controls

- **Mouse drag (left button)**: Rotate the view
- **Mouse drag (middle button)**: Pan the view
- **Mouse scroll**: Zoom in/out
- **Preset views**: Use the View menu or toolbar buttons for Top, Front, Side,
  and Isometric views. These preserve your current zoom distance.
- **Zoom to Fit (Ctrl+F)**: Frames all objects in the view

### Grid Settings

The ground grid helps with spatial orientation:

- **Grid Size**: Adjust from 10mm (fine) to 1.0m (coarse)
- **Grid Color**: Choose from presets or pick a custom color
- **Grid Controls panel**: Access from View → Grid Settings → Show Grid Controls

### What You See

- **Robot links**: Rendered as 3D meshes from the URDF files
- **Robot position**: Updated in real time as joints move
- **TCP indicator**: The tool center point, auto-detected from the URDF
- **Grid**: Ground reference plane at z=0
- **Axes indicator**: Red=X, Green=Y, Blue=Z (bottom-left corner)

---

## Understanding the Architecture

Hatch is built on clear principles. Understanding them will help you use it
to its full potential:

### Everything is an Event

Sliders don't directly move robots. They publish events. The robot publishes
its state. The display listens for state changes. This means:

- You can add your own components that publish or subscribe to events
- State is always traceable — you know exactly what happened and when
- Nothing polls or busy-waits — the CPU sleeps when idle

### Space = TransformRegistry

All positions and orientations live in one place. When the robot moves,
its transforms are updated. The display updates automatically. You can query
the transform between any two frames at any time:

```python
# In your own code:
T = transform_registry.get_transform("ur10_tcp", "world")
```

### The URDF is the Scene

Everything visible in the 3D view comes from the URDF file you loaded.
There is no separate configuration for robot position, tool offset, or
sensor mounting. It's all in the URDF. This means:

- Share one file to reproduce a complete scene
- Version control your entire setup
- No hidden state — what you see is what the URDF defines

---

## Adding Cameras and Sensors

Hatch supports RGB-D cameras (Orbbec Gemini 335, Intel RealSense D435) and
laser scanners (Keyence LJ-V7200) as point cloud sources.

### Setting Up a Camera

Include the camera in your scene URDF and mount it to the robot:

```xml
<xacro:include filename="$(find orbbec_camera)/urdf/gemini_335_336.urdf.xacro"/>

<joint name="wrist_to_camera" type="fixed">
  <parent link="wrist_3_link"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

Hatch automatically detects the camera's depth optical frame. When the robot
moves, the point cloud follows correctly.

### Camera Control Panel

- **Start/Stop** — begin or end camera streaming
- **Camera Type** — switch between available cameras
- **Resolution** — select from supported resolutions
- **ROI Settings** — clip point cloud to a 3D bounding box
- **Transform to World** — toggle world-frame transformation

For full details, see the [Hardware Integration guide](integrating_hardware.md).

---

## Extending Hatch

Hatch is designed to be extended. Every component you see — the joint sliders,
the 3D view, the connection panel — was built using the same public APIs
available to you.

### The Event System: How Everything Communicates

Hatch components never call each other directly. They publish events to
`StateChannel`, and other components subscribe to events they care about.

```
Slider moved → JOINT_COMMAND published → CommandHandler receives → Robot moves
                                                                    ↓
Robot state changes → ROBOT_STATE published → StateHandler updates model
                                             → KinematicDisplay updates 3D view
```

To add your own component, you subscribe to the events you need and publish
events when something changes.

### Example: A Simple Position Logger

Create `my_logger.py` anywhere on your Python path:

```python
"""A simple TCP position logger for Hatch."""

import numpy as np
from scipy.spatial.transform import Rotation as R
from core.world_state.event_types import EventType


class TCPLogger:
    """
    Logs TCP position to a file whenever the robot moves.

    Subscribes to ROBOT_STATE events. Pure observer — does not
    modify the robot or the scene.
    """

    def __init__(self, state_channel, transform_registry, asset_id, filepath="tcp_log.csv"):
        self._channel = state_channel
        self._registry = transform_registry
        self._asset_id = asset_id
        self._filepath = filepath

        # Initialize the log file with a header
        with open(self._filepath, 'w') as f:
            f.write("timestamp,x,y,z,rx,ry,rz\n")

        # Subscribe to robot state updates
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        """Called every time the robot state changes."""
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return  # Frame not registered yet

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        timestamp = event.data.get('timestamp', 0)

        # Convert rotation matrix to rotation vector (human-readable)
        rotvec = R.from_matrix(T[:3, :3]).as_rotvec()

        # Append to log file
        with open(self._filepath, 'a') as f:
            f.write(f"{timestamp:.3f},{x:.4f},{y:.4f},{z:.4f},"
                    f"{rotvec[0]:.4f},{rotvec[1]:.4f},{rotvec[2]:.4f}\n")
```

Wire it into Hatch by adding a few lines to `MainWindow.__init__`:

```python
self.tcp_logger = TCPLogger(
    state_channel=self.state_channel,
    transform_registry=self.transform_registry,
    asset_id=asset_id
)
```

### Example: A Safety Zone Monitor

```python
"""A safety zone monitor that warns when TCP enters restricted areas."""

import numpy as np
from core.world_state.event_types import EventType


class SafetyZoneMonitor:
    """
    Monitors TCP position and publishes warnings when it enters defined zones.
    """

    def __init__(self, state_channel, transform_registry, asset_id):
        self._channel = state_channel
        self._registry = transform_registry
        self._asset_id = asset_id

        # Define safety zones: {name: (x_min, x_max, y_min, y_max, z_min, z_max)}
        self._zones = {
            "table_surface": (0.5, 1.5, -0.5, 0.5, 0.6, 0.8),
            "camera_area": (-0.2, 0.2, 0.8, 1.2, 0.0, 0.5),
        }

        self._active_zones = set()
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return

        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        for zone_name, (xmin, xmax, ymin, ymax, zmin, zmax) in self._zones.items():
            inside = (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax)

            if inside and zone_name not in self._active_zones:
                self._active_zones.add(zone_name)
                self._channel.publish(
                    EventType.ERROR_OCCURRED,
                    data={
                        'error': f"TCP entered zone: {zone_name}",
                        'severity': 'warning',
                        'tcp_position': [x, y, z]
                    },
                    source="safety_zone_monitor"
                )
            elif not inside and zone_name in self._active_zones:
                self._active_zones.discard(zone_name)
```

### The APIs Available to You

**StateChannel** — Publish and subscribe to events:

```python
# Subscribe
channel.subscribe(EventType.ROBOT_STATE, my_callback)

# Publish
channel.publish(EventType.ERROR_OCCURRED, data={...}, source="my_component")

# Unsubscribe when done
channel.unsubscribe(EventType.ROBOT_STATE, my_callback)
```

**TransformRegistry** — Query spatial relationships:

```python
# Get transform between any two frames
T = registry.get_transform("ur10_tcp", "world")
position = T[:3, 3]
rotation = T[:3, :3]

# Be notified when transforms change
def on_transform_changed(frame_name, transform):
    print(f"{frame_name} moved to {transform[:3, 3]}")

registry.register_callback(on_transform_changed)
```

**Event Types** — All events your component can subscribe to or publish:

| Subscribe to | When you want to know |
|-------------|----------------------|
| `ROBOT_STATE` | Robot joint positions or TCP pose changed |
| `ROBOT_LOADED` | A new robot was loaded |
| `MODE_SWITCHED` | User switched between operating modes |
| `CONNECTION_ESTABLISHED` | Connected to hardware |
| `CONNECTION_LOST` | Connection to hardware dropped |
| `ERROR_OCCURRED` | Something went wrong |

| Publish | When you want to tell the system |
|---------|----------------------------------|
| `ERROR_OCCURRED` | Your component detected a problem |
| `JOINT_COMMAND` | You want to move the robot (advanced) |
| `CARTESIAN_COMMAND` | You want to move the TCP (advanced) |

### Design Principles for Extensions

**Be an observer, not a controller.** Subscribe to events to know what's
happening. Don't modify the kinematic model or transform registry directly —
those are owned by `StateHandler`.

**Publish, don't call.** If your component detects something worth sharing,
publish an event. Don't call another component's methods directly.

**Clean up after yourself.** Unsubscribe when your component is destroyed:

```python
def cleanup(self):
    self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
```

**Use rotation vectors, not quaternions.** Hatch's convention is rotation
vectors (`[rx, ry, rz]`) because they are human-readable. Use
`scipy.spatial.transform.Rotation` for conversions.

---

## Troubleshooting

### My robot appears as red cubes instead of meshes

The mesh files couldn't be found. Check:
1. Are your meshes in the correct package directory? (e.g., `~/hatch/assets/robots/ur10/meshes/`)
2. Does the URDF use `package://` paths? (not `file://` or relative paths)
3. Is the package name in the `package://` path correct?

### IK fails or the robot moves to the wrong position

The kinematic root might not be detected correctly. Hatch detects the
true root automatically, but some robot URDFs have unusual structures.
Check the console for "True root" messages on load.

### Connection to real robot fails

- Verify the robot's IP address is correct and reachable
- Check that the RTDE port (default 50002) is open
- Ensure no other program is connected to the robot

### The 3D view is slow or stuttering

- Reduce the render load: close other 3D applications
- Larger mesh files take longer to load initially but don't affect animation speed
- The render loop runs at 60 FPS and only renders when something changes

### My sliders don't match the robot's position

This is by design. Sliders show your commanded position, not the robot's
actual state. The 3D view shows where the robot really is. Sliders sync
to the robot once when you connect or switch to Real mode, then operate
independently. This prevents feedback loops.

---

## Further Reading

| Document | What It Covers |
|----------|---------------|
| [Architecture](architecture.md) | The derived architecture — every component, every principle, and why they exist |
| [Philosophy](philosophy.md) | Why Hatch exists — the Stubborn Student, the refusal to follow |
| [Inverse Kinematics in Hatch](inverse_kinematics.md) | An intuitive guide to 6-DOF IK with worked example and implementation reference |
| [Integrating Hardware](integrating_hardware.md) | Robot drivers, cameras, sensors — the event-driven pattern and case studies |
| [Technical Notes](technical_notes.md) | Kinematic model vs. transform registry, URDF root detection, mesh loading |

---

*This guide covers Hatch v1.0. Contributions and questions welcome.*
