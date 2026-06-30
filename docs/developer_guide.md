# Developer Guide

This guide is for **contributors and integrators** who want to extend Hatch with new components — robot drivers, displays, UI panels, or custom services.

> **If you are an end user** looking for daily usage instructions, see the [User Guide](user_guide.md) instead.

---

## 1. Introduction

Hatch is built to be extended. Every component you see — the joint sliders, the 3D view, the connection panel — was built using the same public APIs available to you.

**The golden rule of extension:**

> **Observe, don’t control. Publish, don’t call. Clean up after yourself.**

---

## 2. The Extension Pattern

All extensions follow the same pattern:
Your Component
↓
Publishes events via StateChannel
↓
Hatch core components receive and act on them
↓
Your component subscribes to events it cares about

text

Your component **never**:
- Calls `RobotManager` or `KinematicModel` directly
- Modifies the `TransformRegistry` directly
- Imports UI components from `ui/` into core services
- Blocks the main thread with `time.sleep()` or `while` loops

---

## 3. Adding a New Robot Driver

Hatch uses `RobotInterface` as the contract for all robot arms. Any robot that can move joints and report state can be integrated.

### 3.1 Implement the Interface

Create a new file, e.g., `drivers/your_robot.py`:

```python
from drivers.robot_interface import RobotInterface
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
import numpy as np
import time

class YourRobot(RobotInterface):
    def __init__(self, state_channel: StateChannel):
        self._channel = state_channel
        self._connected = False
        self._joint_positions = [0.0] * 6

    def move_joints(self, positions: list[float]) -> bool:
        # Send joint command to your hardware
        # (Replace with actual hardware communication)
        self._joint_positions = positions
        self._publish_state()
        return True

    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        # Cartesian command (optional)
        # Hatch will handle IK if you implement this
        return True

    def get_state(self) -> dict:
        return {
            'joint_positions': self._joint_positions,
            'tcp_pose': self._compute_tcp(),  # Implement this
            'timestamp': time.time(),
            'source': 'your_robot'
        }

    def is_connected(self) -> bool:
        return self._connected

    def connect(self, ip: str, **kwargs) -> bool:
        # Your connection logic
        self._connected = True
        self._channel.publish(
            EventType.CONNECTION_ESTABLISHED,
            data={'message': f'Connected to {ip}'},
            source='your_robot'
        )
        return True

    def disconnect(self) -> None:
        self._connected = False
        self._channel.publish(
            EventType.CONNECTION_LOST,
            data={'message': 'Disconnected'},
            source='your_robot'
        )

    def stop(self) -> None:
        # Emergency stop
        self._connected = False
        self._joint_positions = [0.0] * 6

    def _compute_tcp(self) -> list:
        # Compute TCP pose from joint positions (FK)
        # Return as [x, y, z, rx, ry, rz]
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _publish_state(self) -> None:
        self._channel.publish(
            EventType.ROBOT_STATE,
            data=self.get_state(),
            source='your_robot'
        )
```

### 3.2 Wire into MainWindow
In `ui/main_window.py`, inside `_setup_robot_components()`:

```python
from drivers.your_robot import YourRobot

# After creating SimulatedRobot and RealRobot:
self.your_robot = YourRobot(self.state_channel)
self.robot_manager.set_robot(self.your_robot)  # or register as a custom driver
```

### 3.3 Add Mode Support in CommandHandler

Extend `Mode` enum in `core/mode.py` if needed, or use the existing modes (`SIMULATE_LOCAL`, `SIMULATE_REAL_IK`, `REAL`). Update `CommandHandler._on_mode_switch_request()` to route commands to your robot when the appropriate mode is active.

## 4. Adding a New Display Type
Displays are VTK‑based visualizations that subscribe to `TransformRegistry` callbacks.

### 4.1 Create the Display Class
Create a new file, e.g., `displays/my_display.py`:

```python
import vtk
import numpy as np
from core.world_state.transform_registry import TransformRegistry

class MyDisplay:
    def __init__(self, registry: TransformRegistry, asset_id: str, frame_filter: str = None):
        self.registry = registry
        self.asset_id = asset_id
        self.frame_filter = frame_filter or f"{asset_id}_"
        self.renderer = None
        self._needs_render = False
        self._actors = []
        registry.register_callback(self._on_transform_updated)

    def attach(self, renderer: vtk.vtkRenderer) -> None:
        self.renderer = renderer
        # Create your VTK actors here
        self._create_actors()
        self._needs_render = True

    def detach(self) -> None:
        self.registry.remove_callback(self._on_transform_updated)
        for actor in self._actors:
            self.renderer.RemoveActor(actor)
        self._actors.clear()

    def _create_actors(self) -> None:
        # Example: create a sphere at the TCP
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(0.02)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.0, 1.0, 0.0)  # Green
        self._actors.append(actor)
        if self.renderer:
            self.renderer.AddActor(actor)

    def _on_transform_updated(self, frame_name: str, transform: np.ndarray) -> None:
        if not frame_name.startswith(self.frame_filter):
            return
        # Update actor positions based on the new transform
        self._update_actors(frame_name, transform)
        self._needs_render = True

    def _update_actors(self, frame_name: str, transform: np.ndarray) -> None:
        # Example: move the sphere to the TCP frame
        if frame_name == f"{self.asset_id}_tcp":
            pos = transform[:3, 3]
            for actor in self._actors:
                actor.SetPosition(pos[0], pos[1], pos[2])

    def needs_render(self) -> bool:
        return self._needs_render

    def clear_render_flag(self) -> None:
        self._needs_render = False
```

### 4.2 Register the Display

In `RobotManager.load_robot()`, after creating the KinematicDisplay:

```python
my_display = MyDisplay(self.transform_registry, asset_id)
my_display.attach(self.engine.get_renderer())
self.engine.register_display(my_display)
```

## 5. Adding a New UI Panel

UI panels must communicate via StateChannel events only. They never call core services directly.

### 5.1 The Pattern

Create a new file, e.g., `ui/panels/my_panel.py`:

```python
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType

class MyPanel(QWidget):
    def __init__(self, state_channel: StateChannel):
        super().__init__()
        self._channel = state_channel
        self._setup_ui()
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_state)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout()
        self.button = QPushButton("Send Command")
        self.button.clicked.connect(self._on_button_clicked)
        layout.addWidget(self.button)
        self.setLayout(layout)

    def _on_button_clicked(self) -> None:
        # Publish an event — do NOT call core methods directly
        self._channel.publish(
            EventType.JOINT_COMMAND,
            data={'positions': [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]},
            source='my_panel'
        )

    def _on_state(self, event) -> None:
        positions = event.data.get('joint_positions')
        # Update UI based on robot state (e.g., display current joint angles)
        pass

    def cleanup(self) -> None:
        self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_state)
```

### 5.2 Wire into MainWindow

In `ui/main_window.py`, inside `_setup_ui()`:

```python
from ui.panels.my_panel import MyPanel

self.my_panel = MyPanel(self.state_channel)
self.ui_builder.add_dock_widget(self.my_panel, "My Panel")
```

### 5.3 Anti‑Patterns for UI Panels

| ❌ Do NOT | ✅ Do Instead |
|-----------|---------------|
| Import `RobotManager` and call `robot_manager.move_joints()` | Publish `JOINT_COMMAND` and let `CommandHandler` route it |
| Update `TransformRegistry` directly | Subscribe to `ROBOT_STATE` and read from the event data |
| Use `time.sleep()` in event handlers | Use `QTimer.singleShot()` for delayed actions |
| Forget to unsubscribe on panel close | Call `cleanup()` in the panel’s `closeEvent` |

## 6. The Event Contract

All components communicate via these events only. If you need to communicate between components, use `StateChannel.publish()`. If you need data, subscribe to the appropriate event.

| Event | Payload | Publisher | Typical Subscribers |
|-------|---------|-----------|---------------------|
| ROBOT_LOAD_REQUEST | `{'urdf_path': str, 'robot_id': str}` | FileMenu | RobotManager |
| ROBOT_LOADED | `{'asset_id': str, 'kinematic_model': KinematicModel}` | RobotManager | MainWindow, StateHandler, UI panels |
| ROBOT_UNLOAD_REQUEST | `{'robot_id': str}` | FileMenu | RobotManager |
| JOINT_COMMAND | `{'positions': List[float], 'names': Optional[List[str]]}` | JointControlPanel	CommandHandler |
| CARTESIAN_COMMAND | `{'pose': np.ndarray, 'frame': str}`| CartesianControlPanel | CommandHandler |
| MODE_SWITCH_REQUEST | `{'mode': 'simulate_local' | 'simulate_real_ik' | 'real'}`  RobotConnectionPanel | CommandHandler |
| MODE_SWITCHED | `{'mode': str}` | CommandHandler | UI panels |
| ROBOT_STATE | `{'joint_positions': List[float], 'tcp_pose': List[float], 'timestamp': float, 'source': str}` | SimulatedRobot, RealRobot | StateHandler, UI panels |
| CONNECTION_ESTABLISHED | `{'message': str}` | RealRobot | CommandHandler, UI panels |
| CONNECTION_LOST | `{'message': str}` | RealRobot | CommandHandler, UI panels |
| ERROR_OCCURRED | `{'error': str}` | Any component | MainWindow (dialog) |
---

## 7. Example: TCP Position Logger

This example demonstrates a pure observer component that logs the TCP position without modifying any state.

```python
from core.world_state.event_types import EventType
from scipy.spatial.transform import Rotation as R

class TCPLogger:
    """
    Logs TCP position to a file whenever the robot moves.
    Pure observer — does not modify the robot or the scene.
    """

    def __init__(self, state_channel, transform_registry, asset_id, filepath="tcp_log.csv"):
        self._channel = state_channel
        self._registry = transform_registry
        self._asset_id = asset_id
        self._filepath = filepath

        # Initialize the log file with a header
        with open(self._filepath, 'w') as f:
            f.write("timestamp,x,y,z,rx,ry,rz\n")

        # Subscribe to robot state updates
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return  # Frame not registered yet

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        timestamp = event.data.get('timestamp', 0)

        # Convert rotation matrix to rotation vector (human-readable)
        rotvec = R.from_matrix(T[:3, :3]).as_rotvec()

        # Append to log file
        with open(self._filepath, 'a') as f:
            f.write(f"{timestamp:.3f},{x:.4f},{y:.4f},{z:.4f},"
                    f"{rotvec[0]:.4f},{rotvec[1]:.4f},{rotvec[2]:.4f}\n")

    def cleanup(self):
        self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
```
Wire it into Hatch by adding a few lines to `MainWindow.__init__`:

```python
self.tcp_logger = TCPLogger(
    state_channel=self.state_channel,
    transform_registry=self.transform_registry,
    asset_id=asset_id
)
```

## 8. Example: Safety Zone Monitor
This example demonstrates a component that detects when the TCP enters a defined safety zone and publishes warnings.

```python
from core.world_state.event_types import EventType

class SafetyZoneMonitor:
    """
    Monitors TCP position and publishes warnings when it enters defined zones.
    """

    def __init__(self, state_channel, transform_registry, asset_id):
        self._channel = state_channel
        self._registry = transform_registry
        self._asset_id = asset_id

        # Define safety zones: {name: (x_min, x_max, y_min, y_max, z_min, z_max)}
        self._zones = {
            "table_surface": (0.5, 1.5, -0.5, 0.5, 0.6, 0.8),
            "camera_area": (-0.2, 0.2, 0.8, 1.2, 0.0, 0.5),
        }

        self._active_zones = set()
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

    def _on_robot_state(self, event):
        tcp_frame = f"{self._asset_id}_tcp"
        try:
            T = self._registry.get_transform(tcp_frame, "world")
        except ValueError:
            return

        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        for zone_name, (xmin, xmax, ymin, ymax, zmin, zmax) in self._zones.items():
            inside = (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax)

            if inside and zone_name not in self._active_zones:
                self._active_zones.add(zone_name)
                self._channel.publish(
                    EventType.ERROR_OCCURRED,
                    data={
                        'error': f"TCP entered zone: {zone_name}",
                        'severity': 'warning',
                        'tcp_position': [x, y, z]
                    },
                    source="safety_zone_monitor"
                )
            elif not inside and zone_name in self._active_zones:
                self._active_zones.discard(zone_name)

    def cleanup(self):
        self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
```

## 9. Cleanup & Best Practices

### 9.1 Always Unsubscribe

If your component subscribes to events, it must unsubscribe when destroyed:

```python
def cleanup(self):
    self._channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
```

### 9.2 Stop Background Threads

If your component spawns background threads, ensure they are stopped cleanly:

```python
def stop(self):
    self._running = False
    self._thread.join(timeout=1.0)
```

### 9.3 Close Sockets and Files

Always close network connections and file handles in your cleanup method.

### 9.4 Never Block the Main Thread

| ❌ Do NOT | ✅ Do Instead |
|-----------|---------------|
| `time.sleep(0.1)` in a UI callback | `QTimer.singleShot(100, callback)` |
| `while` loop waiting for data | Use `StateChannel.subscribe()` |
| Blocking I/O in the main thread | Move I/O to a background thread |

## 10. Summary: The Golden Rules

1. **Observe, don’t control** — subscribe to events, don’t call core methods directly.

2. **Publish, don’t call** — if your component detects something worth sharing, publish an event.

3. **Clean up after yourself** — unsubscribe, stop threads, close sockets.

4. **Use rotation vectors, not quaternions** — Hatch’s convention is human‑readable rotation vectors.

5. **Never block the main thread** — use QTimer and background threads for long operations.

---

## Directory Structure

hatch/  
├── core/ # Core services (TransformRegistry, StateChannel, KinematicModel)  
│ ├── kinematics/ # Forward/Inverse kinematics  
│ └── world_state/ # TransformRegistry, StateChannel, EventTypes  
├── drivers/ # Robot drivers (SimulatedRobot, RealRobot, UR bridge)  
├── displays/ # VTK visualizations (KinematicDisplay)  
├── viz/ # VisualizerEngine, VTK render window  
├── ui/ # MainWindow, panels, menus  
├── assets/ # URDF files, meshes, scenes  
└── tests/ # Unit tests (recommended)  

---

Hatch (孵) 🐣 —  built to extend, built to share.

