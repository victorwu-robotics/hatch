# API Reference

## Core Services

### TransformRegistry

```python
from core.world_state.transform_registry import TransformRegistry, FrameStatus

registry = TransformRegistry()

# Register a frame
registry.register_frame(
    name="robot_tcp",
    transform=np.eye(4),
    parent="robot_base",
    status=FrameStatus.DYNAMIC,
    description="Tool Center Point"
)

# Query transform
T = registry.get_transform("robot_tcp", "world")  # 4x4 matrix

# Subscribe to changes
def on_update(frame_name, transform):
    print(f"{frame_name} moved")

registry.register_callback(on_update)
```

**Key methods:**
- `register_frame(name, transform, parent, status, description)` — Add or replace a frame
- `update_frame(name, transform)` — Update existing frame (invalidates cache)
- `get_transform(target, source)` — Lazy-evaluated transform query
- `transform_point(point, from_frame, to_frame)` — Transform 3D points
- `verify_tree_integrity()` — Check for cycles, missing parents, invalid transforms

---

### StateChannel

```python
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType

channel = StateChannel(enable_history=True)

def on_robot_state(event):
    print(f"Joints: {event.data['joint_positions']}")

channel.subscribe(EventType.ROBOT_STATE, on_robot_state)

channel.publish(
    EventType.JOINT_COMMAND,
    data={'positions': [0.0] * 6},
    source="my_component"
)
```

**Key methods:**
- `subscribe(event_type, callback)` — React to specific events
- `subscribe_all(callback)` — React to all events
- `publish(event_type, data, source, description)` — Emit event to all subscribers
- `get_history(event_type, limit)` — Recent events (if history enabled)

---

### KinematicModel

```python
from core.kinematics.kinematic_model import KinematicModel

model = KinematicModel(
    urdf_path="robot.urdf",
    package_dirs=["./assets"],
    transform_registry=registry,
    asset_id="ur10"
)
model.load()

# Get joint info
info = model.get_joint_info()  # names, limits, current positions

# Forward kinematics
tcp_pose = model.forward_kinematics([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])

# Inverse kinematics (if solver attached)
q_solution = model.solve_ik_for_tcp(target_pose, q_guess)
```

**Key methods:**
- `load()` — Parse URDF and build kinematic tree
- `update_state(q)` — Set joint positions and recompute FK
- `forward_kinematics(q)` — Compute TCP pose for given joints (non-mutating)
- `get_arm_chain(base_link)` — Get ordered list of joints from base to tool
- `get_true_root()` — Detected kinematic root (handles UR-style base_inertia)
