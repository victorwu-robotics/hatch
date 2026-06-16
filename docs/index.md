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
| [哲学 (Chinese Philosophy)](philosophy_cn.md) | 孵的哲学文献 — 固执的学生，十大原则，论形式主义 |
| [User Guide](user_guide.md) | Getting started, controlling your robot, extending Hatch |

### Inverse Kinematics

| Document | What It Covers |
|----------|---------------|
| [Inverse Kinematics in Hatch](inverse_kinematics.md) | An intuitive guide with worked example and implementation reference |

### Hardware Integration

| Document | What It Covers |
|----------|---------------|
| [Integrating Hardware](integrating_hardware.md) | Robot drivers, cameras, sensors — the event-driven pattern, RTDE reference, path execution, camera pipeline, and the Keyence reverse-engineering case study |

### Technical Notes

| Document | What It Covers |
|----------|---------------|
| [Technical Notes](technical_notes.md) | Kinematic model vs. transform registry, fixed chain tail, URDF root detection, DAE mesh loading |

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
Then: **File → Load URDF →** select your robot's URDF or xacro file.

## Status
Active development. Core platform is functional and tested with real hardware
(UR10, Han's E15-PRO). Documentation is extensive and honest about limitations.

Next priorities: automated tests, configuration system, TCP switching UI,
additional Chinese documentation (母语版本).

孵 (Hatch) — to incubate. A platform that hatches your vision into reality.