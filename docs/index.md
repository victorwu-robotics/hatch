---
layout: default
title: Hatch (孵) 🐣
---
# Hatch (孵) 🐣

> An incubator for robotic ideas.
> One robot. One URDF. No polling.

![Python](https://img.shields.io/badge/Python-3.11-blue?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

[Quick Start](#quick-start-2-minutes) · [Documentation](#documentation) · [Architecture](architecture.md) · [Philosophy](philosophy.md)

---

## What is Hatch?

Hatch is a single‑process, event‑driven platform for developing robot arm applications.  
Load any URDF, solve IK in real time, and control real hardware — all without polling loops, without ROS, and with direct VTK visualization.

**What makes it different?**
- **Everything is an event** — no polling, no busy‑waiting.
- **The URDF is the scene** — no separate world files, no launch files.
- **The architecture is derived** — every component exists because a need demanded it.

> *“A tool that makes decisions the user didn’t ask for is not intelligent — it is insubordinate.”*  
> — The Stubborn Student, [Philosophy](philosophy.md)

---
## What It Looks Like

![Main window with a UR10 loaded](images/main_window_ur10_loaded.png)

---

## Why Hatch?

Most robot platforms are built for **distributed systems** — warehouses full of robots, multiple computers, message serialization, and motion planners that make decisions the user didn't ask for.

Hatch is built for the **single engineer** working on **one robot, one session**. It doesn't hide complexity — it reveals it. It doesn't make decisions for you — it gives you the tools to understand and control every aspect of the robot's behavior.

> *“Understand them, or you will not fully utilise them. Understand your life, or you will not live fully on earth.”*  
> — The Stubborn Student, [Philosophy](philosophy.md)

---

## Quick Start (2 minutes)

```
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window
```

- **Load a robot:** File → Load URDF → select a `.urdf` or `.xacro` file.

- **Move it:** Drag joint sliders — the 3D view updates in real time.

- **Connect to hardware (optional):** ```pip install ur-rtde```, enter robot IP, and switch to **Real** mode.

> For a full walkthrough with screenshots, see the [Quick Start Guide](quick_start.md).

---

## Supported Hardware

| Hardware | Status | Notes |
|----------|--------|-------|
| Simulated Robot | ✅ Full | IK solving, state publishing, no hardware needed |
| Universal Robots (UR) | ✅ Working | RTDE interface via ur-rtde |
| Orbbec Camera | ✅ Working | Depth camera streaming |
| Keyence Laser Scanner | ✅ Working | Profile data streaming |
| RealSense Camera | 🔄 Planned | URDF‑mounted, no point cloud yet |

> For detailed integration instructions, see [Integrating Hardware](integrating_hardware.md).

---

## Documentation

| If you are... | Start here... |
|---------------|---------------|
| New to Hatch and just want to see it work | [quick_start.md](quick_start.md) (2 min) |
| An end user who will use Hatch daily | [user_guide.md](user_guide.md) (30–60 min) |
| Interested in the story and philosophy | [philosophy.md](philosophy.md) (10–30 min) |
| An advanced user who wants to understand how it works internally | [architecture.md](architecture.md) (45–60 min) |
| A developer debugging a technical issue | [technical_notes.md](technical_notes.md) (15–30 min per section) |
| A hardware integrator adding a new robot, camera, or sensor | [integrating_hardware.md](integrating_hardware.md) (30–60 min) |
| A contributor adding a new UI panel, display, or service | [developer_guide.md](developer_guide.md) (30–45 min) |
| Looking for method signatures and API details | [api_reference.md](api_reference.md) (reference) |
| Learning or reviewing 6‑DOF inverse kinematics | [inverse_kinematics.md](inverse_kinematics.md) (30–60 min) |

---

## License & Status

- License: MIT — free to use, modify, and distribute.

- Status: Active development. Version 1.0.0.

- Recommended OS: Ubuntu 20.04. Windows and macOS are untested.

---

*Hatch (孵) 🐣 — An incubator for robotic ideas. Version 1.0.0. MIT License.*
