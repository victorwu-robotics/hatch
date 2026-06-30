# API Reference

This document provides the **method signatures** and **public interfaces** for Hatch’s core services.

> **For usage examples and extension patterns**, see the [Developer Guide](developer_guide.md).  
> **For end‑user documentation**, see the [User Guide](user_guide.md).

---

## 1. TransformRegistry

Located in `core/world_state/transform_registry.py`.

The `TransformRegistry` stores all spatial relationships in the scene. Transforms are lazy‑evaluated and cached. Callbacks notify interested parties when a transform changes.

### `FrameStatus`

```python
class FrameStatus(Enum):
    STATIC = "static"      # Fixed in URDF, never moves
    DYNAMIC = "dynamic"    # Changes during runtime (joints, detected objects)
```

### `register_frame(name, transform, parent, status, description)`

Register a new frame in the registry.

```python
def register_frame(
    self,
    name: str,
    transform: np.ndarray,      # 4x4 homogeneous transform
    parent: str,
    status: FrameStatus = FrameStatus.STATIC,
    description: str = ""
) -> None
```

**Raises:** `ValueError` if the parent frame does not exist.

### `update_frame(name, transform)`

Update an existing frame’s transform. Invalidates the cache for this frame and all descendants.

```python
def update_frame(self, name: str, transform: np.ndarray) -> None
```
**Raises:** `KeyError` if the frame does not exist.

### `get_transform(target, source)`

Get the transform from source frame to target frame. Lazy‑evaluated and cached.

```python
def get_transform(self, target: str, source: str) -> np.ndarray  # 4x4 matrix
```
**Example:**

`get_transform("ur10_tcp", "world")` → returns the TCP pose in world coordinates.

**Raises:** `ValueError` if either frame does not exist.

### `transform_point(point, from_frame, to_frame)`

Transform a 3D point from from_frame to to_frame.

```python
def transform_point(
    self,
    point: np.ndarray,   # shape (3,) or (N, 3)
    from_frame: str,
    to_frame: str
) -> np.ndarray
```

### `register_callback(callback)`

Register a function to be called whenever any transform changes.

```python
def register_callback(self, callback: Callable[[str, np.ndarray], None]) -> None
```

The callback receives:

- `frame_name` (str) — the name of the frame that changed

- `transform` (np.ndarray) — the new 4x4 transform

### `remove_callback(callback)`

Unregister a previously registered callback.

```python
def remove_callback(self, callback: Callable) -> None
```

### `verify_tree_integrity()`

Check for cycles, missing parents, or invalid transforms.

```python
def verify_tree_integrity(self) -> bool
```

## 2. StateChannel

Located in `core/world_state/state_channel.py`.

The `StateChannel` is the event bus. Components publish events to it, and other components subscribe to events they care about.

### `__init__(enable_history)`

```python
def __init__(self, enable_history: bool = False) -> None
```

If `enable_history` is `True`, the channel keeps a limited history of past events (default: 100 per event type).

### `subscribe(event_type, callback)`

Register a callback for a specific event type.

```python
def subscribe(
    self,
    event_type: EventType,
    callback: Callable[[Event], None]
) -> None
```

The callback receives an Event object with attributes:

- `event_type` (EventType)

- `data` (dict)

- `source` (str)

- `timestamp` (float)

- `description` (str, optional)

### `subscribe_all(callback)`

Register a callback for all event types.

```python
def subscribe_all(self, callback: Callable[[Event], None]) -> None
```

### `unsubscribe(event_type, callback)`

Unregister a previously subscribed callback.

```python
def unsubscribe(
    self,
    event_type: EventType,
    callback: Callable[[Event], None]
) -> None
```

### `publish(event_type, data, source, description)`

Publish an event to all subscribers.

```python
def publish(
    self,
    event_type: EventType,
    data: dict,
    source: str,
    description: str = ""
) -> None
```

### `get_history(event_type, limit)`

Retrieve recent events of a given type (if history is enabled).

```python
def get_history(
    self,
    event_type: EventType,
    limit: int = 100
) -> List[Event]
```

## 3. KinematicModel

Located in `core/kinematics/kinematic_model.py`.

The `KinematicModel` parses a URDF, builds the kinematic tree, and provides forward and inverse kinematics.

### `__init__(urdf_path, package_dirs, transform_registry, asset_id)`

```python
def __init__(
    self,
    urdf_path: str,
    package_dirs: List[str],
    transform_registry: TransformRegistry,
    asset_id: str
) -> None
```

### `load()`

Parse the URDF and build the kinematic tree. Must be called before any other methods.

```python
def load(self) -> None
```
**Raises:** `ValueError` if the URDF cannot be parsed or has no moving joints.

### `update_state(q)`

Set the joint positions and recompute forward kinematics.

```python
def update_state(self, q: List[float]) -> None
```

### `forward_kinematics(q)`

Compute the TCP pose for a given set of joint angles. **Non‑mutating** — does not change the internal state.

```python
def forward_kinematics(self, q: List[float]) -> np.ndarray  # 4x4 matrix
```

### `solve_ik_for_tcp(target_pose, q_guess)`

Solve inverse kinematics for the TCP pose. Returns a list of valid joint configurations.

```python
def solve_ik_for_tcp(
    self,
    target_pose: np.ndarray,  # 4x4 matrix
    q_guess: List[float]
) -> List[List[float]]
```

**Returns:** List of solutions, each a list of 6 joint angles. May be empty if no solution exists.

### `get_joint_info()`

Return information about the robot’s joints.

```python
def get_joint_info(self) -> dict
```

**Returns:** A dict with keys:

- `names` (List[str]) — joint names in order

- `limits` (List[Tuple[float, float]]) — (min, max) for each joint

- `current_positions` (List[float]) — current joint angles

### `get_arm_chain(base_link)`

Get the ordered list of joints from `base_link` to the tool mount.

```python
def get_arm_chain(self, base_link: str) -> List[str]
```

### `get_true_root()`

Return the detected kinematic root (the parent of the first moving joint).

```python
def get_true_root(self) -> str
```

### `set_ik_solver(solver)`

Attach an IK solver to the model.

```python
def set_ik_solver(self, solver: IKSolver) -> None
```

The solver must implement `solve(target_pose, q_guess, tool_length)` and return a list of solutions.

## 4. EventType Enum

Located in `core/world_state/event_types.py`.

```python
class EventType(Enum):
    # Robot lifecycle
    ROBOT_LOAD_REQUEST = "robot_load_request"
    ROBOT_LOADED = "robot_loaded"
    ROBOT_UNLOAD_REQUEST = "robot_unload_request"

    # Commands
    JOINT_COMMAND = "joint_command"
    CARTESIAN_COMMAND = "cartesian_command"

    # Modes
    MODE_SWITCH_REQUEST = "mode_switch_request"
    MODE_SWITCHED = "mode_switched"

    # Connection
    CONNECTION_REQUEST = "connection_request"
    CONNECTION_ESTABLISHED = "connection_established"
    CONNECTION_LOST = "connection_lost"
    DISCONNECTION_REQUEST = "disconnection_request"

    # State
    ROBOT_STATE = "robot_state"

    # Errors
    ERROR_OCCURRED = "error_occurred"
```

## 5. Mode Enum

Located in `core/mode.py`.

```python
class Mode(Enum):
    SIMULATE_LOCAL = "simulate_local"
    SIMULATE_REAL_IK = "simulate_real_ik"
    REAL = "real"
```

| Mode | IK Source | Robot Moves |
|------|-----------|-------------|
| SIMULATE_LOCAL | Hatch’s built‑in solver | Virtual only |
| SIMULATE_REAL_IK | Real robot’s controller | Virtual only |
| REAL | Real robot’s controller | Physical hardware |
---

## 6. IKSolver Interface (Abstract)

Located in `core/kinematics/ik_solver.py`.

If you implement a custom IK solver, it must inherit from IKSolver and implement the following method:

```python
class IKSolver(ABC):
    @abstractmethod
    def solve(
        self,
        target_pose: np.ndarray,  # 4x4 matrix
        q_guess: List[float],
        tool_length: float
    ) -> List[List[float]]:
        """
        Return a list of valid joint configurations.
        May return an empty list if no solution exists.
        """
        pass
```

## 7. RobotInterface (Abstract)

Located in `drivers/robot_interface.py`.

All robot drivers must implement this interface:

```python
class RobotInterface(ABC):
    @abstractmethod
    def move_joints(self, positions: List[float]) -> bool:
        pass

    @abstractmethod
    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        pass

    @abstractmethod
    def get_state(self) -> dict:
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def connect(self, ip: str, **kwargs) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass
```

---

Hatch (孵) 🐣 — Precision in code. Understanding in mind.

