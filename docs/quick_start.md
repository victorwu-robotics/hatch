# Quick Start Guide

**Recommended OS:** Ubuntu 20.04. Windows and macOS have not been tested.

## Installation

### Prerequisites

- Python 3.10+
- pip
- Ubuntu 20.04 (recommended)
- A URDF file (any robot — try [Universal Robots description](https://github.com/ros-industrial/universal_robot) or your own)

### Step 1: Clone and install

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Step 2: Run Hatch

```bash
python -m ui.main_window
```

You should see a white 3D view with a grid and coordinate axes.

![Empty Hatch window with grid visible](images/startup_screen.png)

### Step 3: Load a robot

1. Click **File → Load URDF**
2. Navigate to a `.urdf` or `.xacro` file
3. The robot appears in the 3D view

![SCREENSHOT: Hatch with UR10 loaded, default pose](images/robot_loaded.png)

**Troubleshooting:** If meshes don't appear, check the console for path resolution errors. Hatch searches for `package://` URIs in the URDF's directory, its parent, and `~/hatch/assets/`.

### Step 4: Move the robot

The **Motion Control** panel appears automatically when a robot loads.

- **Joint Control** tab: Drag sliders to move individual joints
- **Cartesian Control** tab: Move the TCP in X, Y, Z, RX, RY, RZ

![SCREENSHOT: Motion Control panel with Joint Control tab active, sliders visible](images/main_window_ur10_loaded.png)

The 3D view updates in real time as you drag.

### Step 5: Connect to a real UR robot (optional)

1. Install the RTDE driver:
   ```bash
   pip install ur-rtde
   ```

2. In Hatch: **Robots → Connect** → enter the robot's IP address

3. Wait for "Connected" status

4. Switch mode to **Real** (the button is in the connection panel)

*[SCREENSHOT: Connection panel showing "Connected" status, mode dropdown]*

**Safety:** In `SIMULATE_LOCAL` and `SIMULATE_REAL_IK` modes, the robot does not move. Only `REAL` mode sends commands to hardware. The mode is always visible in the UI.

### Step 6: Try Cartesian control

1. Switch to the **Cartesian Control** tab
2. Drag the X, Y, Z sliders
3. Hatch solves IK and moves the robot (or virtual robot, depending on mode)

*[SCREENSHOT: Cartesian Control tab active, TCP position values visible]*

### Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| Black screen / no 3D view | VTK/Qt initialization failure | Check `vtk` and `PyQt5` versions match `requirements.txt`. On Linux, try `export QT_QPA_PLATFORM=xcb`. |
| Robot loads but no meshes | Mesh path not resolved | Use absolute paths in URDF, or place meshes in the same directory as the URDF. |
| "IK failed" in Cartesian mode | Target pose unreachable | Check joint limits. Try small movements first. |
| RTDE connection drops | Network/firewall | Verify robot IP. Check ports 30002 and 30004 are open. |
| High CPU when idle | Leaked subscriptions or polling loops | Check custom panels for `while` loops or uncleaned `StateChannel.subscribe()` calls. |

---

## Next Steps

- [Architecture](architecture.md) — Understand how it works
- [Developer Guide](developer_guide.md) — Add your own robot driver or display
- [Troubleshooting](troubleshooting.md) — Deeper problem solving
