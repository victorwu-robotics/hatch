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

*Next: Controlling Your Robot*
*(Joint Control, Cartesian Control, Simulation vs. Real modes)*

## Controlling Your Robot

Once your scene is loaded, the Motion Control panel appears on the right side
of the window. It has three sections:

### Robot Connection

At the top of the Motion Control panel, you can:

- **Select Mode**: Choose between Simulate and Real.
  - **Simulate**: A virtual robot moves in the 3D view. No hardware required.
    Uses Hatch's built-in IK solver. Safe for testing.
  - **Real**: Connects to physical hardware. Commands move the real robot.
    Requires network connection to the robot controller.

- **Connect to Hardware**: Enter the robot's IP address and click Connect.
  The RTDE frequency defaults to 125 Hz — this works for most UR robots.
  Once connected, the status indicator turns green.

- **Disconnect**: Safely closes the connection to the robot.

### Joint Control

The Joint Control tab shows a slider for each joint in the robot arm.

**How it works:**
1. You move a slider → Hatch publishes a joint command
2. The command goes to the active robot (simulated or real)
3. The robot moves to the commanded position
4. The robot publishes its new state
5. The sliders and 3D view update to reflect the actual position

**Tips:**
- **Home Position**: Returns all joints to their neutral (zero) position
- **Zero All**: Sets all joints to zero if within limits, or the closest valid angle
- **Mouse wheel**: Hover over a slider and scroll for fine adjustment
- **Real mode**: Joint labels show "(actual)" to remind you that the hardware is moving

### Cartesian Control

The Cartesian Control tab lets you move the robot's **Tool Center Point (TCP)**
 - the point in space where a tool would attach to the robot's wrist. Hatch
 automatically detects which link serves as the mounting point by finding the 
 last link in the kinematic chain. For UR robots this is `wrist_3_link`; for
 other robots it is detected from the URDF.

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

### Simulation vs. Real Modes

| Mode | IK Solver | Robot Moves | Use Case |
|------|-----------|-------------|----------|
| Simulate | Hatch's built-in solver | Virtual only | Testing trajectories, learning the interface |
| Real | Real robot's controller | Physical hardware | Production use |

**Switching from Simulate to Real**: The robot must be connected. If the real
robot is at a different position than the simulation, the virtual robot will
snap to the real robot's position.

**Switching from Real to Simulate**: The virtual robot stays at the last known
real robot position. You can continue testing without the hardware.

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
- **TCP indicator**: The tool center point is at the wrist_3_link by default
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
- Try increasing the frequency if commands are slow

### The 3D view is slow or stuttering

- Reduce the render load: close other 3D applications
- Larger mesh files take longer to load initially but don't affect animation speed
- The render loop runs at 60 FPS and only renders when something changes

---

*Next: Extending Hatch — Adding Your Own Components*

## Extending Hatch

Hatch is designed to be extended. Every component you see — the joint sliders,
the 3D view, the connection panel — was built using the same public APIs
available to you. This section shows you how.

### The Event System: How Everything Communicates

Hatch components never call each other directly. They publish events to
`StateChannel`, and other components subscribe to events they care about.

```
Slider moved → JOINT_COMMAND published → CommandHandler receives → Robot moves
                                                                    ↓
Robot state changes → ROBOT_STATE published → StateHandler updates model
                                             → JointControlPanel updates sliders
                                             → CartesianControlPanel updates display
                                             → KinematicDisplay updates 3D view
```

To add your own component, you subscribe to the events you need and publish
events when something changes.

### Example: A Simple Position Logger

Let's build a component that logs the robot's TCP position to a file whenever
it moves. Create `my_logger.py` anywhere on your Python path:

```python
"""A simple TCP position logger for Hatch."""

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
            f.write("timestamp,x,y,z,roll,pitch,yaw\n")

        # Subscribe to robot state updates
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        """Called every time the robot state changes."""
        # Get TCP pose in world frame
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return  # Frame not registered yet

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        timestamp = event.data.get('timestamp', 0)

        # Convert rotation matrix to Euler angles
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        rpy = R.from_matrix(T[:3, :3]).as_euler('xyz')

        # Append to log file
        with open(self._filepath, 'a') as f:
            f.write(f"{timestamp:.3f},{x:.4f},{y:.4f},{z:.4f},"
                    f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}\n")
```

**Wire it into Hatch** by adding a few lines to `MainWindow.__init__`:

```python
# After the robot is loaded, create the logger:
self.tcp_logger = TCPLogger(
    state_channel=self.state_channel,
    transform_registry=self.transform_registry,
    asset_id=asset_id
)
```

That's it. Your logger receives every robot state update and writes to a CSV
file. You can open it in any spreadsheet or analysis tool.

### Example: A Safety Zone Monitor

Let's build something more useful — a component that publishes a warning
if the TCP enters a defined zone:

```python
"""A safety zone monitor that warns when TCP enters restricted areas."""

import numpy as np
from core.world_state.event_types import EventType


class SafetyZoneMonitor:
    """
    Monitors TCP position and publishes warnings when it enters defined zones.

    Zones are axis-aligned bounding boxes defined in world coordinates.
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

        # Track which zones the TCP is currently inside
        self._active_zones = set()

        # Subscribe to robot state
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        """Check TCP position against all zones."""
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return

        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        for zone_name, (xmin, xmax, ymin, ymax, zmin, zmax) in self._zones.items():
            inside = (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax)

            if inside and zone_name not in self._active_zones:
                # TCP entered the zone
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
                # TCP left the zone
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

# Get the frame chain between two frames
chain = registry.get_chain("ur10_tcp", "ur10_base_link")

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
| `MODE_SWITCHED` | User switched between Simulate and Real |
| `CONNECTION_ESTABLISHED` | Connected to hardware |
| `CONNECTION_LOST` | Connection to hardware dropped |
| `ERROR_OCCURRED` | Something went wrong |

| Publish | When you want to tell the system |
|---------|----------------------------------|
| `ERROR_OCCURRED` | Your component detected a problem |
| `JOINT_COMMAND` | You want to move the robot (advanced) |
| `CARTESIAN_COMMAND` | You want to move the TCP (advanced) |

### Design Principles for Extensions

When building your own components, follow the same principles Hatch uses:

**Be an observer, not a controller.** Subscribe to events to know what's
happening. Don't modify the kinematic model or transform registry directly —
those are owned by `StateHandler`.

**Publish, don't call.** If your component detects something worth sharing
(a warning, a measurement, a state change), publish an event. Don't call
another component's methods directly.

**Clean up after yourself.** If your component subscribes to events, make
sure to unsubscribe when it's destroyed:

```python
def cleanup(self):
    self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
```

**Use rotation vectors, not quaternions.** Hatch's convention is rotation
vectors (`[rx, ry, rz]`) because they are human-readable. Use
`scipy.spatial.transform.Rotation` for conversions.

### When to Modify Hatch Itself

Extensions like the examples above work without changing Hatch's source code.
Consider modifying Hatch itself when:

- You need a new UI panel (dock widget in the main window)
- You're adding support for a new robot brand or protocol
- You've built something that every user would benefit from
- You've found a bug or performance issue

Before contributing, read the Architecture Document to understand the
principles that guide design decisions.

---

## Getting Help

### Where Things Live

| If you need to | Look here |
|---------------|-----------|
| Load a different robot | File → Load URDF |
| Connect to hardware | Motion Control → Robot Connection |
| Move individual joints | Motion Control → Joint Control |
| Move the TCP | Motion Control → Cartesian Control |
| Change the view | View menu or toolbar buttons |
| Adjust grid size/color | View → Grid Settings |

### Common Questions

**Q: Can I load two robots at once?**

Not in the current version. Hatch follows "one robot per session" to keep
the architecture simple. Restart Hatch to load a different robot.

**Q: Can I save the current robot state?**

Yes — build a component that subscribes to `ROBOT_STATE` and saves the
joint positions (see the TCP Logger example above).

**Q: Does Hatch support cameras?**

Camera support is under development. The architecture has a designed
extension point for sensors. See the Architecture Document for details.

**Q: How do I calibrate my sensor positions?**

Sensor calibration is planned but not yet available. Currently, sensor
positions come from the URDF file. For precise applications, pre-calibrate
your URDF transforms before loading.

---

*This guide covers Hatch v1.0. For more detail, see the Architecture Document.*
*Contributions and questions welcome.*
