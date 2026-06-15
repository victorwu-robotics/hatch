# 孵 (Hatch) Robotics Platform

> *"A platform is not defined by what it can do. It is defined by what it will not do — and why."*

Hatch is a lightweight, event-driven robotics platform for a single robot arm.
It gives you a control panel, a live 3D view, and a clean Python API — without
the complexity of ROS.

**Hatch is for those who cannot afford ROS.** Not just financially — the
cognitive complexity, the learning curve, the maintenance burden. If you have
one robot arm and a job to do, Hatch is built for you.

---

## Documentation

### Core

| Document | What It Covers |
|----------|---------------|
| [Architecture](architecture.md) | The derived architecture — every component, every principle, and why they exist |
| [Philosophy](philosophy.md) | Why Hatch exists — the Stubborn Student, the refusal to follow, the demand to understand |
| [User Guide](user_guide.md) | Getting started, controlling your robot, extending Hatch |

### Inverse Kinematics

| Document | What It Covers |
|----------|---------------|
| [Inverse Kinematics in Hatch](inverse_kinematics.md) | An intuitive guide with worked example and implementation reference |

### Hardware Integration

| Document | What It Covers |
|----------|---------------|
| [Camera Integration](camera_integration.md) | RGB-D camera pipeline and adding new cameras |
| [Event-Driven Drivers](event_driven_drivers.md) | Why Hatch never polls — the RTDE case study |
| [Non-Polling RTDE Driver](non-polling-rtde-driver.md) | Command-response pattern for UR robots |
| [Path Execution](path_execution.md) | Scoped streaming for trajectory following |
| [Keyence Scanner Lessons](keyence_scanner_lessons.md) | Reverse-engineering a proprietary sensor |

### Technical Notes

| Document | What It Covers |
|----------|---------------|
| [Kinematic Model vs Transform Registry](kinematic_model_vs_transform_registry.md) | The coordinate system distinction that defines Hatch |
| [Fixed Chain Tail](fixed_chain_tail.md) | How fixed joints before and after moving joints are handled |
| [URDF Processing](urdf_processing.md) | True kinematic root detection |
| [DAE Files & Scene Handling](dae_files_scene_handling.md) | Mesh loading pitfalls and fixes |

### Architecture History

| Version | Link |
|---------|------|
| V1.0 (original) | [architecture_v1.md](architecture_v1.md) |
| V2.0 (the fire) | [philosophy.md](philosophy.md) |
| V3.0 (current) | [architecture.md](architecture.md) |

---

## Quick Start

```bash
git clone https://github.com/victorwu-robotics/hatch.git
cd hatch
pip install -r requirements.txt
python -m ui.main_window
```
Then: File → Load URDF → select your robot's URDF or xacro file.

## Status
Active development. Core platform is functional and tested with real hardware
(UR10, Han's E15-PRO). Documentation is extensive and honest about limitations.

Next priorities: automated tests, configuration system, TCP switching UI,
Chinese documentation (母语版本).

孵 (Hatch) — to incubate. A platform that hatches your vision into reality.