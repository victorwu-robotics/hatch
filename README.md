# Hatch (孵) 🐣

> An incubator for robotic ideas.  
> One robot. One URDF. No polling.

![Python](https://img.shields.io/badge/Python-3.11-blue?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

---

## What is Hatch?

Hatch is a single‑process, event‑driven platform for developing robot arm applications.  
Load any URDF, solve IK in real time, and control real hardware — all without polling loops, without ROS, and with direct VTK visualization.

**What makes it different?**
- **Everything is an event** — no polling, no busy‑waiting.
- **The URDF is the scene** — no separate world files, no launch files.
- **The architecture is derived** — every component exists because a need demanded it.

> *“A tool that makes decisions the user didn’t ask for is not intelligent — it is insubordinate.”*  
> — The Stubborn Student, [Philosophy](docs/philosophy.md)

---

## What It Looks Like

![Main window with a UR10 loaded](docs/images/main_window_ur10_loaded.png)

---

## Quick Start (2 minutes)

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window
```
**Load a robot:** File → Load URDF → select a .urdf or .xacro file.
**Move it:** Drag the joint sliders — the 3D view updates in real time.
**Connect to hardware (optional):** pip install ur-rtde, enter the robot’s IP, and switch to Real mode.

>**For a full walkthrough with screenshots**, see the [Quick Start Guide](docs/quick_start.md).

## Documentation

| If you are... | Start here... |
|---------------|---------------|
| New to Hatch and just want to see it work | quick_start.md (2 min) |
| An end user who will use Hatch daily | user_guide.md (30–60 min) |
| Interested in the story and philosophy | philosophy.md (10–30 min) |
| An advanced user who wants to understand how it works internally | architecture.md (45–60 min) |
| A developer debugging a technical issue | technical_notes.md (15–30 min per section) |
| A hardware integrator adding a new robot, camera, or sensor | integrating_hardware.md (30–60 min) |
| A contributor adding a new UI panel, display, or service | developer_guide.md (30–45 min) |
| Looking for method signatures and API details | api_reference.md (reference) |
| Learning or reviewing 6‑DOF inverse kinematics | inverse_kinematics.md (30–60 min) |

---

## License & Status
- License: MIT — free to use, modify, and distribute.

- Status: Active development. Version 1.0.0.

- Recommended OS: Ubuntu 20.04. Windows and macOS are untested.

---

Hatch (孵) 🐣 — built to understand, built to see.
