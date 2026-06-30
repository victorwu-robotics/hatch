# Hatch User Guide

This guide covers everything you need to use Hatch daily — from creating your scene to controlling your robot and troubleshooting issues.

> **If you’re new to Hatch,** start with the [Quick Start Guide](quick_start.md) first.

---

## 1. Creating Your Scene

Hatch needs one thing to start: a URDF file that describes your robot and its environment. This file is the **single source of truth** for everything in the scene — robot arms, sensors, tools, tables, UGV bases, and their positions relative to each other.

### 1.1 What Hatch Supports

- **Serial robot arms** — a single chain of links connected by joints (UR, KUKA, Han's, Fanuc, etc.)
- **Fixed‑joint attachments** — tools, cameras, scanners (mount via fixed joints to the arm)
- **Static environment** — tables, fixtures, safety cages (mount via fixed joints from `world`)

**Not supported:** Parallel robots, branching chains, closed kinematic loops.

### 1.2 Organizing Your Files

Hatch follows the ROS package convention. Each component (robot, sensor, tool, UGV) lives in its own package directory:

```
~/hatch/assets/
├── scenes/ # Scene‑defining URDF files
│ └── my_scene/urdf/my_scene.urdf
├── robots/ # Robot URDFs and meshes
│ └── ur10/urdf/ur10.urdf
├── sensors/ # Sensor URDFs and meshes
├── ugv/ # Mobile base URDFs and meshes
└── tools/ # End‑effector URDFs and meshes
```

### 1.3 Writing Your Scene URDF

**Option 1: Plain URDF (simple scenes)**

```xml
<?xml version="1.0"?>
<robot name="my_scene">
  <link name="world"/>
  <link name="table"> ... </link>
  <joint name="world_to_table" type="fixed">
    <parent link="world"/> <child link="table"/>
  </joint>
  <include filename="package://ur10/urdf/ur10.urdf"/>
  <joint name="table_to_robot" type="fixed">
    <parent link="table"/> <child link="ur10_base_link"/>
  </joint>
</robot>
```

**Option 2: Xacro (complex scenes with many components)**

```xml
<?xml version="1.0"?>
<robot name="my_scene" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="package://ur10/urdf/ur10.urdf.xacro"/>
  <xacro:include filename="package://keyence/urdf/keyence.urdf.xacro"/>
  <link name="world"/>
  <xacro:ur10 prefix="" parent="world" xyz="0.5 0 0.75"/>
  <xacro:keyence prefix="scanner_" parent="ur10_tool0" xyz="0 0 0.05"/>
</robot>
```

**Supported xacro features:** ```<xacro:include>```, variables ```${}```, macros, macro calls.  

**Not supported:** Python expressions, conditional blocks. If you need these, pre‑process with xacro separately.

### 1.4 Mesh Files and package:// Paths

All mesh references must use ```package://``` URIs:

```xml
<mesh filename="package://ur10/meshes/base_link.stl"/>
```

Hatch searches for packages in this order:

1. The directory containing the URDF file

2. The parent and grandparent directories

3. ```~/hatch/assets/``` and its subdirectories

### 1.5 Loading Your Scene

1. Start Hatch: ```python -m ui.main_window```

2. Click File → Load URDF

3. Select your ```.urdf``` or ```.xacro``` file

Hatch processes ```.xacro``` files automatically.

> **The true kinematic root:** Hatch automatically detects the parent of the first moving joint. You don’t need to do anything special. If IK results look wrong, check the console for “True root” messages.

### 2. Controlling Your Robot
Once your scene is loaded, the **Motion Control** panel appears on the right.

### 2.1 Modes
Hatch has three operating modes. The mode dropdown is in the Robot Connection panel.

| Mode | IK Source | Robot Moves | Use Case |
|------|-----------|-------------|----------|
| **Simulate (Local IK)** | Hatch’s built‑in solver | Virtual only | Testing trajectories offline, learning the interface|
| **Simulate (Real IK)** | Real robot’s controller | Virtual only | Validating IK before moving hardware |
| **Real** | Real robot’s controller | Physical hardware | Production use |

**Switching between modes:**

- Connect to hardware to unlock Simulate (Real IK) and Real.

- When switching to Real, the virtual robot snaps to the real robot’s position.

- When switching back to Simulate, the virtual robot stays at the last known position.

### 2.2 Joint Control

The **Joint Control** tab shows a slider for each joint.

- **Drag a slider →** publishes a joint command → robot moves → 3D view updates.

- **Sliders are input devices, not state displays.** They show what you commanded, not where the robot actually is. The 3D view shows the robot’s actual state.

- **Home →** returns all joints to neutral (zero) position.

- **Zero All →** sets all joints to zero if within limits, or the closest valid angle.

- **Mouse wheel →** fine adjustment (hover over a slider and scroll).

### 2.3 Cartesian Control

The **Cartesian Control** tab lets you move the Tool Center Point (TCP).

- **Drag X, Y, Z, RX, RY, RZ sliders →** Hatch solves IK → robot moves to the target pose.

- **Current TCP display** shows where the TCP actually is (may differ from the target if IK failed or the robot hasn’t reached the target).

- **Step size** controls how much each notch moves: 1 mm, 1 cm, 1 degree.

- **Reset to Current** sets the target sliders to match the current TCP pose.

> **How the TCP is detected:** Hatch uses the last link in the kinematic chain as the TCP. If you want a different link as the TCP, name it ..._tcp or ..._tool0 in your URDF.

## 3. Understanding the 3D View

### 3.1 Viewing Camera Controls

- **Left‑click drag →** rotate

- **Middle‑click drag →** pan

- **Scroll →** zoom

- **Preset views →** Top, Front, Side, Isometric (View menu or toolbar)

- **Zoom to Fit (Ctrl+F) →** frames all objects

### 3.2 Grid Settings

The ground grid helps with spatial orientation.

- **Grid Size:** Adjust from 10 mm to 1.0 m

- **Grid Color:** Choose from presets or pick a custom color

- **Grid Controls panel:** View → Grid Settings → Show Grid Controls

### 3.3 What You See

- **Robot links →** rendered as 3D meshes from URDF files

- **Robot position →** updates in real time as joints move

- **TCP indicator →** auto‑detected from the URDF

- **Grid →** ground reference plane at z=0

- **Axes indicator →** Red=X, Green=Y, Blue=Z (bottom‑left corner)

## 4. Connecting to Hardware

Hatch supports Universal Robots via ur-rtde.

**Requirements:**

- Robot powered on, brake released

- e‑Series: Remote Control Mode enabled (PolyScope → Settings → System → Remote Control)

- Same network, ports 30001–30004 open

**Steps:**

1. Install ur-rtde: pip install ur-rtde

2. In Hatch: **Robots → Connect →** enter the robot’s IP

3. Wait for “Connected” status

4. Switch to **Real** mode to move hardware

> **If connection fails:** Press **Stop** on the teach pendant to clear any stuck script from a previous session. Verify network with ```ping ROBOT_IP```. Check firewall ports.

## 5. Using Sensors

Hatch supports RGB‑D cameras and laser scanners as point cloud sources.

### 5.1 Setting Up a Camera

Include the camera in your scene URDF and mount it to the robot:

```xml
<joint name="wrist_to_camera" type="fixed">
  <parent link="ur10_tool0"/>
  <child link="camera_depth_optical_frame"/>
  <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
</joint>
```
The camera’s depth optical frame is defined in the sensor’s own URDF file. 
This sensor URDF is included into the main scene URDF via `xacro:include`. 
Hatch reads the complete scene URDF and uses the optical frame as defined.

> **Example:** The Orbbec Gemini 335 URDF defines `camera_depth_optical_frame` as a link. 
> The user includes this URDF in their scene file, and Hatch automatically resolves all frames.

### 5.2 Camera Control Panel

- **Start/Stop →** begin or end camera streaming

- **Camera Type →** switch between available cameras

- **Resolution →** select from supported resolutions

- **ROI Settings →** clip point cloud to a 3D bounding box

- **Transform to World →** toggle world‑frame transformation

> For detailed integration instructions, see [Integrating Hardware](integrating_hardware.md).

## 6. Troubleshooting

## 6.1 Installation Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| **`ModuleNotFoundError: No module named 'vtk'`** | VTK not installed or version mismatch | Check `pip freeze | grep vtk` matches `requirements.txt`. Reinstall: `pip install --upgrade vtk` |
| **`qt.qpa.plugin: Could not load the Qt platform plugin "xcb"`** | Qt platform plugin missing or misconfigured | On Linux: `export QT_QPA_PLATFORM=xcb` before running Hatch. On headless servers, install `xvfb` and run: `xvfb-run python -m ui.main_window` |
| **`ImportError: libGL.so.1: cannot open shared object file`** | OpenGL library missing on headless Linux | Install: `sudo apt update && sudo apt install libgl1-mesa-glx` |
| **`ur_rtde` not found when connecting to UR robot** | `ur-rtde` not installed | Install: `pip install ur-rtde` |
| **Python version error (e.g., `SyntaxError`)** | Python version < 3.8 or > 3.11 | Hatch requires Python 3.8–3.11. Check with `python --version`. Use `pyenv` or a virtual environment. |
| **`pip install -r requirements.txt` fails** | Dependency conflict or outdated pip | Upgrade pip: `pip install --upgrade pip`. Then retry. If using a fresh virtual environment, ensure it is activated. |
---

## 6.2 Runtime Troubleshooting

| Problem | Likely Cause | Fix |
|---------|--------------|-----|
| **Robot appears as red cubes instead of meshes** | Mesh files not found | Check package:// paths. Place meshes in the correct package directory. |
| **IK fails or robot moves to wrong position** | Kinematic root detection issue | Check console for “True root” messages. Ensure your URDF’s first moving joint is correct. |
| **Connection to real robot fails** | Network or pendant state | Verify IP, check ports, press **Stop** on teach pendant. |
| **Connection drops during operation** | Network instability or robot protective stop | Use wired connection. Press **Stop** and reconnect. |
| **3D view is slow or stuttering** | Large mesh files or render load | Close other 3D applications. The render loop only updates when something changes. |
| **Sliders don’t match robot’s position** | By design — sliders show commands, not state | The 3D view shows the robot’s actual state. Sliders sync only on connection and mode switch. |

---
## 7. Further Reading

| If you want to… | Read this… |
|-----------------|------------|
| Understand the architecture and principles | [architecture.md](architecture.md) |
| Learn the story and philosophy behind Hatch | [philosophy.md](philosophy.md) |
| Add your own robot, camera, or sensor | [integrating_hardware.md](integrating_hardware.md) |
| Extend Hatch with new panels or displays | [developer_guide.md](developer_guide.md) |
| Deep‑dive into technical details | [technical_notes.md](technical_notes.md) |
| Learn 6‑DOF inverse kinematics | [inverse_kinematics.md](inverse_kinematics.md)|

---

Hatch (孵) 🐣 — built to understand, built to see.
