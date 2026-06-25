# Technical Notes

This document collects technical deep-dives on specific aspects of Hatch's
architecture. Each section stands alone — read what you need.

---

## 1. Kinematic Model vs. Transform Registry

One of the most misunderstood concepts in robotics software is the difference
between **where the robot thinks it is** and **where the robot actually is in
the world**. Hatch makes this distinction explicit.

### 1.1 Two Coordinate Systems

| System | Purpose | Origin | Used By |
|--------|---------|--------|---------|
| **Robot Base Coordinates** | Kinematics, IK, joint angles | Robot's base (local) | Kinematic Model, UI Sliders, Command Generation |
| **World Coordinates** | Visualization, scene composition | Fixed world origin | Transform Registry, 3D Viewer, Camera Frames |

The robot does not know where it is in the world. It only knows where its
joints are relative to its own base.

The world does not care about the robot's internal kinematics. It only knows
where objects are in absolute space.

**Hatch maintains both and keeps them in sync.**

### 1.2 The Two Core Components

**Kinematic Model** (`core/kinematic_model.py`)

Robot-specific. Parses URDF, builds the kinematic tree, computes link transforms
**relative to the robot base**. Provides forward and inverse kinematics. Does
**not** know where the robot is in the world.

When the kinematic model says "the TCP is at [0.5, 0.2, 0.3]", that is relative
to the robot's base, not the world.

**Transform Registry** (`core/world_state/transform_registry.py`)

World-aware. Stores transforms for all frames (robot links, camera frames, tool
frames). Applies the **world offset** to place the robot in the scene. Lazy
evaluation with cache invalidation. Does **not** know about joints or kinematics.

When the transform registry says "the TCP is at [0.5, 0.2, 0.3]", that is in
world coordinates after applying the robot's mounting position.

### 1.3 The Relationship

```
Kinematic Model (robot base coordinates)
 ↓
 compute_link_transforms()
 ↓
 all transforms relative to base_link
 ↓
 _update_registry() ← applies world offset
 ↓
Transform Registry (world coordinates)
 ↓
 add world offset to base_link
 ↓
 compute all child transforms recursively
 ↓
Visualizer (renders in world coordinates)
```

The kinematic model **feeds into** the transform registry. The registry adds
the world offset and makes the robot appear in the correct place.

### 1.4 The World Offset

The transform registry does not assume the robot base is at the world origin:

```python
# In TransformRegistry
self.set("robot_base_link", world_offset_transform, parent="world")
```

All other robot links are computed relative to `robot_base_link`. This allows:

| Capability | How |
|------------|-----|
| Robot mounted anywhere in the scene | Set `world → base_link` transform |
| Multiple robots (future) | Each has its own `base_link` under world |
| Mobile robot moving | Update `base_link` transform over time |

### 1.5 The UI Sliders: A Special Case

**UI sliders show user intent, not robot state** (by design).

| Scenario | Slider Behavior | Rationale |
|----------|-----------------|-----------|
| Normal operation (Real mode) | Sliders show user commands, not robot feedback | Sliders are user intent. Robot state is shown in the 3D view. |
| After connecting to hardware | Sliders sync ONCE to actual robot position | The user needs to know where the robot is before they start commanding it. |
| After switching to Real mode | Sliders sync ONCE to actual robot position | The user may have moved sliders in Simulate mode. Real mode must reflect reality. |
| During active slider drag | Sliders ignore incoming robot state | Prevents feedback loop: user drags → robot moves → state comes back → would overwrite slider |
| Home/Zero buttons | Sliders update to target positions | User explicitly requested a known position. |

**Why sliders do not track robot state continuously:**

A natural expectation is that sliders should always show where the robot
actually is. But in Hatch, the slider is an **input device**, not a **display
device**. The 3D view shows robot state. The sliders show user intent.

This distinction matters because of the feedback loop it prevents:

1. User drags slider → command sent to robot
2. Robot moves → state published back
3. If slider updated from state → slider moves → new command sent → loop

By syncing sliders only on connection and mode switch — and never during
normal operation — Hatch eliminates this class of bug entirely. The slider
shows what the user *wants*. The 3D view shows what the robot *is doing*.

This reflects the principle of UI Separate from Services. The slider panel
is a command interface, not a monitoring dashboard.

### 1.6 Common Pitfalls

| If You Need... | Read From... |
|----------------|--------------|
| Joint angles for kinematics or commands | **Kinematic Model** |
| Joint angles for visualization | **Transform Registry** (after world offset) |
| Joint angles for UI slider sync | **Kinematic Model** (after updating from robot state) |

Reading from the wrong source causes the robot to appear in the wrong place
or sliders to show incorrect values.

### 1.7 The Sync Flow for Real Mode

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
│    kinematic model via StateHandler         │
└─────────────────────────────────────────────┘
 ↓
Visualizer updates (world coordinates)
 ↓
(Optional) UI sliders sync (on mode switch only)
```

If step 1 is missing, the kinematic model becomes stale. UI sliders will show
incorrect values even though the visualizer looks correct.

---

## 2. The Fixed Chain Tail

The robot only moves between the first and last moving joints. The kinematic
chain is bounded by two points.

### 2.1 Before the First Moving Joint

Fixed joints connect the world to the true kinematic base. These links —
pedestals, mounting plates, base frames — never move. `KinematicModel._find_true_root()`
walks backward through fixed joints to find the true base.

### 2.2 After the Last Moving Joint

Fixed joints connect the last moving link to sensors, tools, flanges, and
end-effectors. These links move as a rigid assembly with the last moving link's
child. `StateHandler._build_arm_chain_links()` includes all fixed children of
arm chain links, so sensors and tools are updated when the arm moves.

The `TransformRegistry` tracks each frame individually. A future optimization
would combine all post-wrist links into a single compound VTK actor, since
they move as one rigid body.

### 2.3 TCP Switching (Planned)

A robot wrist often carries multiple attachments: a welding torch, a camera,
a laser scanner. Each has a working point that can serve as the TCP. Hatch
will support switching between them.

**Data model:** Every fixed link after the last moving joint is a potential
TCP. `KinematicModel` already walks the fixed chain to set `tool_mount_link`.
This will be extended to expose the full list of available endpoints.

**UI:** A dropdown in the Cartesian Control panel lists all available TCPs
with their offset distance from the wrist center. Selecting a different TCP
updates `tool_mount_link`, and the IK solver automatically targets the new link.

**Display:** The active TCP is highlighted in green in the Joint Frame panel.
Other available endpoints are listed but not highlighted.

**Implementation path:**

1. `KinematicModel` builds a list of all fixed endpoints during URDF parsing
2. `KinematicModel.set_active_tcp(link_name)` changes `tool_mount_link`
3. IK solver, displays, and panels reference `tool_mount_link` — they follow automatically
4. `CartesianControlPanel` adds a TCP selector dropdown populated from the endpoint list

This feature is planned but not yet implemented. Currently, Hatch uses the
naked tool mount point (last fixed link after the last moving joint) as the TCP.

---

## 3. URDF Kinematic Root Detection

### 3.1 The Problem

The URDF standard implicitly assumes that `base_link` is the kinematic root
of the robot. Many real robots violate this assumption.

| Robot | Issue |
|-------|-------|
| Universal Robots UR10 | `base_inertia` (fixed joint with 180° rotation) between `base_link` and first moving joint |
| Custom robots | Multiple fixed joints before first moving joint |
| Mobile manipulators | Robot base is not at world origin |

Using `base_link` as the kinematic root for inverse kinematics produces
**wrong results**.

### 3.2 The UR10 Case Study

The Universal Robots UR10 URDF has this structure:

```
world
 └── base_link (first link, kinematic root? NO)
     └── base_inertia (fixed joint, 180° rotation)
         └── shoulder_pan_joint (FIRST MOVING JOINT)
             └── shoulder_link
                 └── ...
```

| Frame | Type | Purpose |
|-------|------|---------|
| `base_link` | Link | Mounting point to world |
| `base_inertia` | Link | 180° rotation hack (fixed joint) |
| `shoulder_pan_joint` | Joint | First moving joint |

The **true kinematic root** for IK is the **parent of the first moving joint** —
which is `base_inertia`, not `base_link`.

Because `base_inertia` rotates 180°, the entire kinematic chain is flipped
relative to `base_link`. Using `base_link` as the kinematic root gives
systematically wrong IK solutions.

### 3.3 Implementation

The correct solution has four steps:

**Step 1:** Traverse the joint tree. Ignore fixed joints. Find the first joint
with type `revolute`, `continuous`, or `prismatic`. For UR10: `shoulder_pan_joint`.

**Step 2:** The **parent** of the first moving joint is the true kinematic root.
For UR10: `base_inertia`.

**Step 3:** If the URDF has a link called `base_link`, that is the **mounting
point** to the world — not the kinematic root.

**Step 4:** The transform chain:
```
world → base_link (mounting position, user-defined)
      → base_inertia (fixed offset, may include 180°)
      → shoulder_pan_joint (moving)
```

For IK: use `base_inertia` as the root frame. For visualization: apply all
transforms from `world`.

```python
def find_true_root(urdf_tree):
    # Find the true kinematic root: parent of first non-fixed joint
    for joint in urdf_tree.findall('joint'):
        joint_type = joint.get('type')
        if joint_type in ['revolute', 'continuous', 'prismatic']:
            parent_link = joint.find('parent').get('link')
            return parent_link

    # No moving joints — use first link as fallback
    first_link = urdf_tree.find('link')
    return first_link.get('name') if first_link is not None else "base_link"
```

### 3.4 Comparison with Other Platforms

| Platform | Assumes base_link = kinematic root | Handles UR10 correctly |
|----------|------------------------------------|------------------------|
| ROS urdfdom | Yes | No (requires workaround) |
| ROS-Industrial | Partial | Adds extra base frame |
| URDF specification | Yes (implicitly) | No |
| Hatch | No — detects true root | Yes |

### 3.5 User Guidance

When processing any URDF:

1. Do not assume `base_link` is the kinematic root
2. Find the first moving joint
3. Use its parent as the kinematic root for IK
4. Preserve all fixed joints in the transform chain
5. Allow the robot to be positioned anywhere in the world via `world → base_link`

If your robot appears rotated 180° or IK gives unexpected results, check for
a fixed joint with a rotation before the first moving joint.

---

## 4. DAE Mesh Loading

### 4.1 The Problem: 1000x Scale

When loading a Keyence LJ-V7200 laser scanner from a `.dae` (Collada) file,
the mesh appeared "1000 times too big and miles away" in Hatch, despite the
same file working correctly in the older `robo_platform` and in ROS.

### 4.2 Root Cause and Fix

The `MeshLoader._load_dae` method used `scene.geometry.values()` to extract
meshes from the Collada file. This returns raw vertex data **without applying
the scene graph transforms**. The scene graph contains scale and translation
nodes that are essential for correct rendering.

The Keyence DAE file stores its geometry in **millimeters** (mesh ~230mm wide).
The scene graph contains transforms that scale and position the mesh correctly.
`scene.geometry.values()` ignores these transforms, producing a 230-meter-wide
mesh at the wrong location. `scene.dump()` applies the transforms, producing
a 0.23-meter-wide mesh at the correct position.

**The fix:**

```python
# Before (wrong):
for geometry in scene.geometry.values():
    if isinstance(geometry, trimesh.Trimesh):
        meshes.append(geometry)

# After (correct):
meshes = scene.dump()
```

`scene.dump()` returns mesh copies with all scene node transforms applied —
including the scale that converts millimeters to meters.

### 4.3 How the Bug Was Introduced

The original mesh loading code used `scene.dump()` and worked correctly. During
a refactoring that extracted `MeshLoader` as a standalone service, the DAE
loading path was rewritten. The `scene.dump()` call was inadvertently replaced
with `scene.geometry.values()` — a function that returns raw vertex data
without applying scene graph transforms. The abstraction was correct; the data
extraction was not.

This is a known risk when refactoring working code: the new structure can be
right while a single data flow detail is wrong. The lesson is not to avoid
refactoring, but to verify the output of the new path against the old path
before declaring the refactoring complete.

### 4.4 Why ROS Works

ROS (via `rviz` and the ROS URDF loader) handles DAE files through
`collada_parser` or `assimp`, which automatically apply scene transforms.
The ROS toolchain has always processed Collada scene graphs correctly.

### 4.5 Additional Fixes Applied During This Session

**Fixed Chain Tail in StateHandler** (`core/state_handler.py`): Extended
`_build_arm_chain_links` to include **all fixed children** of arm chain links
(sensors, tools, flanges). Previously only the tool mount link was included.
Now any link connected via fixed joints to a moving link is tracked and
updated when the arm moves.

**Empty Frame Registration in RobotManager** (`core/robot_manager.py`):
`_register_initial_transforms` now handles intermediate frames with no visual
geometry (optical frames, depth frames) that serve as coordinate references
for sensors. These frames are registered with proper parent-before-child
ordering using a two-pass approach.

**JointFrameDisplay Guards** (`displays/joint_frame_display.py`): Added guards
for static models with no moving joints, missing tool mount link, and graceful
fallback to displaying all links as frames when no arm chain exists.

**Preprocessor Macro Handling** (`core/urdf_preprocessor.py`): The preprocessor
requires both a macro definition and a macro call in the file. For standalone
sensor loading, a macro call must be present after the macro definition.

### 4.6 Lessons

1. **Collada files contain scene graphs with transforms.** Raw geometry
   extraction (`geometry.values()`) ignores these transforms. Always use
   `scene.dump()` when you need the mesh as it appears in the scene.

2. **When refactoring working code, preserve the data flow.** The old loader
   worked. The new loader abstracted the file handling but changed the data
   extraction method. The abstraction was correct; the data extraction was not.

3. **Debug mesh issues by checking bounds.** The polydata bounds showed 230
   meters — a clear sign that millimeter data was being treated as meters.

---

*This document combines the Kinematic Model vs Transform Registry guide, Fixed
Chain Tail specification, URDF Processing note, and DAE Mesh Loading post-mortem
into a single technical reference.*
