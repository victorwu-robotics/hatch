# Developer Guide

**Recommended OS:** Ubuntu 20.04. Windows and macOS have not been tested.

---

## Adding a New Robot Driver

Hatch uses `RobotInterface` as the contract. Any robot that can move joints and report state can be integrated.

### 1. Implement the interface

Create `drivers/your_robot.py`:

```python
from drivers.robot_interface import RobotInterface
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
import numpy as np

class YourRobot(RobotInterface):
    def __init__(self, state_channel: StateChannel):
        self._channel = state_channel
        self._connected = False

    def move_joints(self, positions: list[float]) -> bool:
        # Send to your hardware
        return True

    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        # Your Cartesian command logic
        return True

    def get_state(self) -> dict:
        return {
            'joint_positions': [...],
            'tcp_pose': [...],
            'timestamp': time.time(),
            'source': 'your_robot'
        }

    def is_connected(self) -> bool:
        return self._connected

    def connect(self, ip: str, **kwargs) -> bool:
        # Your connection logic
        self._connected = True
        self._channel.publish(EventType.CONNECTION_ESTABLISHED,
                              data={'message': 'Connected'},
                              source='your_robot')
        return True

    def disconnect(self) -> None:
        self._connected = False

    def stop(self) -> None:
        # Emergency stop
        pass
```

### 2. Wire into MainWindow

In `ui/main_window.py`, `_setup_robot_components()`:

```python
from drivers.your_robot import YourRobot

# After creating SimulatedRobot and RealRobot:
self.your_robot = YourRobot(self.state_channel)
self.robot_manager.set_your_robot(self.your_robot)
```

### 3. Add mode support in CommandHandler

Extend `Mode` enum in `core/mode.py` if needed, or use existing modes. Update `CommandHandler._on_mode_switch_request()` to route commands to your robot.

---

## Adding a New Display Type

Displays are VTK-based visualizations that subscribe to `TransformRegistry` callbacks.

### 1. Create the display class

```python
# displays/my_display.py
import vtk
from core.world_state.transform_registry import TransformRegistry

class MyDisplay:
    def __init__(self, registry: TransformRegistry, asset_id: str):
        self.registry = registry
        self.asset_id = asset_id
        self.renderer = None
        self._needs_render = False
        registry.register_callback(self._on_transform_updated)

    def attach(self, renderer):
        self.renderer = renderer
        # Create VTK actors
        self._needs_render = True

    def detach(self):
        self.registry.remove_callback(self._on_transform_updated)
        # Remove actors

    def _on_transform_updated(self, frame_name: str, transform: np.ndarray):
        if not frame_name.startswith(f"{self.asset_id}_"):
            return
        # Update actors
        self._needs_render = True
```

### 2. Register in RobotManager

In `RobotManager.load_robot()`, after creating `KinematicDisplay`:

```python
my_display = MyDisplay(self.transform_registry, asset_id)
my_display.attach(self.engine.get_renderer())
self.engine.register_display(my_display)
```

---

## Adding a New UI Panel

UI panels must communicate via `StateChannel` events only.

### Pattern

```python
from PyQt5.QtWidgets import QWidget
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType

class MyPanel(QWidget):
    def __init__(self, state_channel: StateChannel):
        super().__init__()
        self._channel = state_channel
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_state)

    def _on_button_clicked(self):
        # Publish event — do NOT call core methods directly
        self._channel.publish(
            EventType.JOINT_COMMAND,
            data={'positions': [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]},
            source='my_panel'
        )

    def _on_state(self, event):
        positions = event.data.get('joint_positions')
        # Update UI
```

### Anti-patterns

❌ **Do NOT** import `RobotManager` into your panel and call `robot_manager.move_joints()`  
✅ **DO** publish `JOINT_COMMAND` and let `CommandHandler` route it

❌ **Do NOT** update `TransformRegistry` from a panel  
✅ **DO** subscribe to `ROBOT_STATE` and read from the event data

---

## The Event Contract

All components communicate via these events only:

| Event | Payload | Publisher | Typical Subscribers |
|-------|---------|-----------|---------------------|
| `ROBOT_LOAD_REQUEST` | `{'urdf_path': str, 'robot_id': str}` | FileMenu | RobotManager |
| `ROBOT_LOADED` | `{'asset_id': str, 'kinematic_model': KinematicModel}` | RobotManager | MainWindow, StateHandler, UI panels |
| `JOINT_COMMAND` | `{'positions': List[float], 'names': Optional[List[str]]}` | JointControlPanel | CommandHandler |
| `CARTESIAN_COMMAND` | `{'pose': np.ndarray, 'frame': str}` | CartesianControlPanel | CommandHandler |
| `MODE_SWITCH_REQUEST` | `{'mode': 'simulate' \| 'real'}` | RobotConnectionPanel | CommandHandler |
| `MODE_SWITCHED` | `{'mode': str}` | CommandHandler | UI panels |
| `ROBOT_STATE` | `{'joint_positions': List[float], 'tcp_pose': List[float], 'timestamp': float, 'source': str}` | SimulatedRobot, RealRobot | StateHandler, UI panels |
| `CONNECTION_ESTABLISHED` | `{'message': str}` | RealRobot | CommandHandler, UI panels |
| `CONNECTION_LOST` | `{'message': str}` | RealRobot | CommandHandler, UI panels |
| `ERROR_OCCURRED` | `{'error': str}` | Any component | MainWindow (dialog) |

**Rule:** If you need to communicate between components, use `StateChannel.publish()`. If you need data, subscribe to the appropriate event. Never import across layers.
