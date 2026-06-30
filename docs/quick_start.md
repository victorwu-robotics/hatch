# Quick Start Guide

**Recommended OS:** Ubuntu 20.04. Windows and macOS are untested.

---

## 1. Install and Run

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window
```

You should see a white 3D view with a grid and coordinate axes.

![Startup Screen](images/startup_screen.png)

> **If the window appears but the 3D view is black:** On Linux, try setting the Qt platform plugin:  
`export QT_QPA_PLATFORM=xcb` before running Hatch.
If you are on a headless server, you may need to install `xvfb` and run with `xvfb-run python -m ui.main_window`.

## 2. Load a Robot

Click **File → Load URDF →** select a .urdf or .xacro file.

The robot appears in the 3D view.

![Robot Loaded](images/robot_loaded.png)

> **Where to find sample URDFs:** Sample robots (including the UR10 used in the screenshot) are available in the `assets/robots/` directory of the Hatch repository. You can also download the official Universal Robots description package from [ROS‑Industrial](https://github.com/ros-industrial/universal_robot) — just place the URDFs and meshes in your own `assets/` folder.

## 3. Move It
The **Motion Control** panel appears automatically.

- **Joint Control tab:** Drag sliders to move individual joints.

- **Cartesian Control tab:** Move the TCP in X, Y, Z, RX, RY, RZ.

The 3D view updates in real time.

![UR 10 loaded](images/main_window_ur10_loaded.png)

---

## 4. Connect to a Real UR Robot (Optional)

```bash
pip install ur-rtde
```
In Hatch: **Robots → Connect →** enter the robot’s IP → switch to **Real** mode.

> **Safety:** Only **Real** mode sends commands to hardware.  
> Simulate modes move only the virtual robot.

---

## Troubleshooting (Top 3 Issues)

| Problem | Fix |
|---------|-----|
| Black screen / no 3D view | Check `vtk` and `PyQt5` versions match `requirements.txt`. On Linux: `export QT_QPA_PLATFORM=xcb.` |
| Robot loads but no meshes | Use `package://` paths in your URDF. Place meshes in the same directory as the URDF. |
| RTDE connection fails | Verify IP, check ports 30002 and 30004, press **Stop** on the teach pendant. |

---
## Next Steps
- [User Guide](user_guide.md) — full reference for daily use

- [Documentation Table of Contents](../DOCUMENTATION.md) — find the right doc for your goal




---
Hatch (孵) 🐣 — Now that it moves, what will you build?.
