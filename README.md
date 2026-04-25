

## `README.md` for Hatch

# Hatch (孵) Robotics Platform

> *"Understand them, or you will not fully utilise them. Understand your life, or you will not live fully on earth."*

Hatch is a principled, event-driven robotics application development platform. It is not a collection of tools — it is a **derived architecture** where every component exists because a principle demanded it.

The name **Hatch (孵)** represents the moment a new robot comes to life. The right side of the character (孚) signifies incubation and nurturing — bringing ideas into existence through careful development.

---

## Philosophy

Hatch is built on ten immutable principles:

| # | Principle |
|---|-----------|
| 0 | Individuals Before Groups |
| 1 | Single Process, Single Memory Space |
| 2 | Event-Driven, No Polling |
| 3 | Visualizer as Mind-Prying Tool |
| 4 | Everything in URDF |
| 5 | Space = TransformRegistry |
| 6 | Time = StateChannel |
| 7 | Movements as Models |
| 8 | Pure Python |
| 9 | UI Separate from Services |
| 10 | One Robot Per Session |

For detailed explanation, see the [Architecture Document](docs/architecture.md).

---

## Features

- **Pure event-driven architecture** — no polling, no busy-waiting
- **Single robot per session** — clean boundaries, no complex cleanup
- **URDF-native** — everything attached to the robot is described in URDF
- **TransformRegistry** — lazy evaluation, cache invalidation
- **StateChannel** — publish/subscribe event bus with history
- **VTK visualization** — direct rendering, no middleware
- **Joint and Cartesian control** — intuitive sliders
- **Simulate/Real mode switching** — test in simulation, run on hardware

---

## Quick Start

### Prerequisites

- Python 3.10 or higher
- pip

### Installation

```bash
# Clone the repository
git clone https://github.com/yourname/hatch.git
cd hatch

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run Hatch
python -m ui.main_window
```

### Loading a Robot

1. Click **File → Load URDF**
2. Select a URDF file
3. The robot appears in the 3D view

### Controlling the Robot

- **Joint Control**: Move individual joint sliders
- **Cartesian Control**: Move TCP in X, Y, Z, RX, RY, RZ
- **Mode Switching**: Switch between Simulate and Real modes

---

## Directory Structure

```
hatch/
├── core/                    # Core services
│   ├── mesh_loader.py       # Pure mesh loading
│   ├── robot_manager.py     # Robot lifecycle
│   ├── command_handler.py   # Command routing
│   ├── mode.py              # Mode enum
│   └── world_state/         # TransformRegistry, StateChannel
├── drivers/                 # Hardware drivers
│   ├── simulated_robot.py
│   ├── real_robot.py
│   └── ur_rtde_bridge.py
├── displays/                # Visualization
│   ├── kinematic_display.py
│   └── pointcloud_display.py
├── ui/                      # User interface
│   ├── main_window.py
│   ├── ui_builder.py
│   ├── menus/
│   └── panels/
└── assets/                  # Robot URDFs and meshes
    └── robots/
```

---

## Event Flow

```
User moves slider
    ↓
UI Panel publishes COMMAND event
    ↓
StateChannel distributes event
    ↓
CommandHandler receives, routes to active robot
    ↓
Robot executes command, updates kinematic model
    ↓
TransformRegistry updates transforms
    ↓
KinematicDisplay re-renders
    ↓
ROBOT_STATE event published
    ↓
UI panels update displays
```

---

## Hardware Support

| Hardware | Status | Notes |
|----------|--------|-------|
| Simulated Robot | ✅ Full | IK solving, state publishing |
| Universal Robots (UR) | ✅ Working | RTDE interface |
| RealSense Camera | 🔄 Planned | Will follow URDF principle |
| Orbbec Camera | 🔄 Planned | Will follow URDF principle |
| Laser Scanner | 🔄 Planned | Will follow URDF principle |

---

## License

MIT License

---

## Contributing

Hatch follows a **derivation-first** approach. Before adding a feature, ask:

1. *"What principle requires this?"*
2. *"Can this be derived from existing principles?"*
3. *"If not, what new principle is needed?"*

No feature is added without a principle justifying it.

---

## Documentation

- [Architecture Document](docs/architecture.md) — The immutable foundation
- API Reference — Coming soon
- Developer Guide — Coming soon

---

## The Name

**Hatch (孵)** — to incubate, to hatch. Where robot software is born.

🐣

---

*Version 1.0.0*

