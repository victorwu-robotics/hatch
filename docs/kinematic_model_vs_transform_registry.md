# Hatch Technical Note: Kinematic Model vs. Transform Registry

## The Distinction That Defines Hatch's Architecture

One of the most misunderstood concepts in robotics software is the difference between **where the robot thinks it is** and **where the robot actually is in the world**. Hatch makes this distinction explicit.

---

## Two Coordinate Systems

| System | Purpose | Origin | Used By |
|--------|---------|--------|---------|
| **Robot Base Coordinates** | Kinematics, IK, joint angles | Robot's base (local) | Kinematic Model, UI Sliders, Command Generation |
| **World Coordinates** | Visualization, scene composition | Fixed world origin | Transform Registry, 3D Viewer, Camera Frames |

The robot does not know where it is in the world. It only knows where its joints are relative to its own base.

The world does not care about the robot's internal kinematics. It only knows where objects are in absolute space.

**Hatch maintains both and keeps them in sync.**

---

## The Two Core Components

### Kinematic Model

```python
# Lives in: core/kinematic_model.py
# Coordinates: Robot base (base_link is at origin)
# Purpose: Joint angles, forward kinematics, inverse kinematics
```

The kinematic model is **robot-specific**. It:
- Parses URDF and builds the kinematic tree
- Computes link transforms **relative to the robot base**
- Provides forward and inverse kinematics
- Does **not** know where the robot is in the world

When the kinematic model says "the TCP is at [0.5, 0.2, 0.3]", that is relative to the robot's base, not the world.

### Transform Registry

```python
# Lives in: core/world_state/transform_registry.py
# Coordinates: World (fixed origin)
# Purpose: Scene composition, visualization, multi-frame transforms
```

The transform registry is **world-aware**. It:
- Stores transforms for all frames (robot links, camera frames, tool frames)
- Applies the **world offset** to place the robot in the scene
- Lazy evaluation with cache invalidation
- Does **not** know about joints or kinematics

When the transform registry says "the TCP is at [0.5, 0.2, 0.3]", that is in world coordinates after applying the robot's mounting position.

---

## The Relationship

```
Kinematic Model (robot base coordinates)
    ↓
    compute_link_transforms()
    ↓
    all transforms relative to base_link
    ↓
    _update_registry()  ← applies world offset
    ↓
Transform Registry (world coordinates)
    ↓
    add world offset to base_link
    ↓
    compute all child transforms recursively
    ↓
Visualizer (renders in world coordinates)
```

The kinematic model **feeds into** the transform registry. The registry adds the world offset and makes the robot appear in the correct place.

---

## The Active Robot Concept

Hatch supports **one active robot at a time** (Principle #10). The active robot determines:

| Aspect | Simulate Mode | Real Mode |
|--------|---------------|-----------|
| Kinematic model source | Virtual model (URDF) | Real robot (via RTDE) |
| Joint angles | From user commands | From robot feedback |
| Transform registry updates | From kinematic model | From ROSBOT_STATE or kinematic model |

The transform registry **always reflects the active robot** (in world coordinates). It does not care whether the joint angles came from a slider or from the real robot.

---

## The UI Sliders: A Special Case

**UI sliders show user intent, not robot state** (by design, Principle #9).

| Scenario | Slider Behavior |
|----------|-----------------|
| Normal operation (Real mode) | Sliders show user commands, not robot feedback |
| After switching to Real mode | Sliders sync once to actual robot position |
| After connecting to real robot | Sliders sync once to actual robot position |
| Home/Zero buttons | Sliders update to target positions |

This prevents feedback loops while still allowing the user to see discrepancies between commanded and actual positions.

---

## Common Pitfall: Reading from the Wrong Source

| If You Need... | Read From... |
|----------------|--------------|
| Joint angles for kinematics or commands | **Kinematic Model** |
| Joint angles for visualization | **Transform Registry** (after world offset) |
| Joint angles for UI slider sync | **Kinematic Model** (after updating it from robot state) |

Reading from the wrong source causes the robot to appear in the wrong place or sliders to show incorrect values.

---

## The Sync Flow for Real Mode

```
Real robot moves (via command or external)
    ↓
RTDE publishes ROBOT_STATE (joint angles in robot base coordinates)
    ↓
StateHandler receives ROBOT_STATE
    ↓
┌─────────────────────────────────────────────┐
│ Two updates must happen:                    │
│ 1. Kinematic Model: update_state(positions) │
│ 2. Transform Registry: receives from        │
│    kinematic_model._update_registry()       │
└─────────────────────────────────────────────┘
    ↓
Visualizer updates (world coordinates)
    ↓
(Optional) UI sliders sync (on mode switch only)
```

If step 1 is missing, the kinematic model becomes stale. UI sliders will show incorrect values even though the visualizer looks correct.

---

## The World Offset

The transform registry does not assume the robot base is at the world origin. Instead:

```python
# In TransformRegistry
self.set("robot_base_link", world_offset_transform, parent="world")
```

All other robot links are then computed relative to `robot_base_link`. This allows:

| Capability | How |
|------------|-----|
| Robot mounted anywhere in the scene | Set `world → base_link` transform |
| Multiple robots (future) | Each has its own `base_link` under world |
| Mobile robot moving | Update `base_link` transform over time |

---

## Ownership Checklist

You own this architecture when you can answer:

| Question | Answer |
|----------|--------|
| What coordinate system does the kinematic model use? | Robot base coordinates (base_link at origin) |
| What coordinate system does the transform registry use? | World coordinates (fixed origin) |
| Who applies the world offset? | Transform registry, when registering `robot_base_link` |
| Why do UI sliders sometimes show different values than the visualizer? | Sliders show user intent; visualizer shows reality |
| When switching to Real mode, what must be updated? | Kinematic model (from robot state), then UI sliders (from kinematic model) |
| What happens if `ROBOT_STATE` updates the registry but not the kinematic model? | Visualizer correct, UI sliders stale |

---

## Conclusion

The kinematic model and transform registry are **not redundant**. They serve different purposes at different coordinate systems. Understanding this distinction is essential for:

- Correct robot visualization
- Accurate UI feedback
- Safe mode switching
- Future multi-robot and mobile robot support

This document captures Hatch's design decision. It is not a bug. It is a **feature** — one that most platforms blur or ignore.

Hatch makes the distinction explicit. That is why you own it.

---

*Document version 1.0*
*For Hatch (孵) Robotics Platform*
 🐣