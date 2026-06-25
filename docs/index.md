---
layout: default
title: Hatch (孵) 🐣
---

# Hatch (孵) 🐣

> A single-process, event-driven robotics application development platform.
> One robot. One URDF. No polling.

**Recommended OS:** Ubuntu 20.04. Windows and macOS have not been tested.

---

## What It Looks Like

*[SCREENSHOT: Main window with a UR10 loaded, joint sliders visible, 3D view showing the robot]*

Hatch loads any URDF, solves IK in real time, and controls real hardware — all in a single process, without polling loops, and with direct VTK visualization.

## Try Hatch in 5 Minutes

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python -m ui.main_window
```

**Load a robot:** File → Load URDF → select a `.urdf` or `.xacro` file.

**Move it:** Drag joint sliders — the 3D view updates in real time.

**Connect to hardware (optional):**
```bash
pip install ur-rtde  # for Universal Robots
# In Hatch: Robots → Connect → enter robot IP → switch to Real mode
```

*[SCREENSHOT: File → Load URDF dialog with a URDF selected]*

## Architecture at a Glance

Three core abstractions everything else builds on:

| Abstraction | Purpose | File |
|-------------|---------|------|
| **TransformRegistry** | All spatial relationships, lazy-evaluated | `core/world_state/transform_registry.py` |
| **StateChannel** | All events, publish/subscribe, no polling | `core/world_state/state_channel.py` |
| **KinematicModel** | Pure Python URDF parsing + FK/IK | `core/kinematics/kinematic_model.py` |

*[SCREENSHOT: Simplified data flow diagram — see [Architecture](architecture.md)]*

## Principles (Summary)

These principles were discovered through derivation — each one demanded by a need that arose during development.

| # | Principle | Discovered From |
|---|-----------|---------------|
| 0 | Individuals Before Groups | Need: one robot, one session |
| 1 | Single Process, Single Memory Space | Need: no serialization overhead |
| 2 | Event-Driven, No Polling | Need: decoupled communication |
| 3 | Visualizer as Mind-Prying Tool | Need: see the robot's true state |
| 4 | Everything in URDF | Need: describe the scene |
| 5 | Space = TransformRegistry | Need: know where everything is |
| 6 | Time = StateChannel | Need: components must communicate |
| 7 | Movements as Models | Need: commands as data |
| 8 | Pure Python | Need: rapid development |
| 9 | UI Separate from Services | Need: controls without coupling |
| 10 | One Robot Per Session | Need: clean boundaries |

See [Philosophy](philosophy.md) for the full derivation from first principles.

## Hardware Support

| Hardware | Status | Notes |
|----------|--------|-------|
| Simulated Robot | ✅ Full | IK solving, state publishing, no hardware needed |
| Universal Robots (UR) | ✅ Working | RTDE interface via `ur-rtde` |
| Orbbec Camera | ✅ Working | — |
| Keyence Laser Scanner | ✅ Working | — |
| RealSense Camera | 🔄 Planned | URDF-mounted, no point cloud yet |

## Documentation Paths

| I want to… | Start here |
|------------|-----------|
| Try Hatch now | [Getting Started](getting_started.md) (30 min) |
| Understand the philosophy | [Philosophy](philosophy.md) |
| Understand the architecture | [Architecture](architecture.md) |
| Read deep technical notes | [Technical Notes](technical_notes.md) |
| Extend Hatch | [Developer Guide](developer_guide.md) |
| Look up API details | [API Reference](api_reference.md) |
| Fix a problem | [Troubleshooting](troubleshooting.md) |

---

*Hatch (孵) — Built to understand. Version 1.0.0. MIT License.*
