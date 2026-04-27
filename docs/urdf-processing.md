# URDF Kinematic Root Detection: Clarifying a Long-Standing Ambiguity

*Why your UR10 IK fails and how to fix it — a lesson from Hatch*

---

## The Problem

The URDF standard implicitly assumes that `base_link` is the kinematic root of the robot. Many real robots violate this assumption.

| Robot | Issue |
|-------|-------|
| Universal Robots UR10 | `base_inertia` (fixed joint with 180° rotation) between `base_link` and first moving joint |
| Custom robots | Multiple fixed joints before first moving joint |
| Mobile manipulators | Robot base is not at world origin |

Using `base_link` as the kinematic root for inverse kinematics produces **wrong results**.

---

## The UR10 Case Study

### Universal Robots UR10 URDF has this structure:
```text
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

The **true kinematic root** for IK is the **parent of the first moving joint** — which is `base_inertia`, not `base_link`.

---

## Why the 180° Rotation Confusion

| Frame | Rotation from previous |
|-------|------------------------|
| `base_link` | Identity (world→base_link) |
| `base_inertia` | 180° about Z (fixed joint) |
| `shoulder_pan_joint` | 0° (revolute) |

Because `base_inertia` rotates 180°, the entire kinematic chain is flipped relative to `base_link`. If you use `base_link` as the kinematic root, IK gives wrong results.

---

## The Correct Solution

### Step 1: Identify the First Moving Joint

Traverse the joint tree. Ignore fixed joints. Find the first joint with:
- Type `revolute`, `continuous`, or `prismatic`

For UR10: `shoulder_pan_joint`

### Step 2: Identify the Kinematic Root

The **parent** of the first moving joint is the true kinematic root for IK.

For UR10: `base_inertia`

### Step 3: Identify the True Base Link

If the URDF has a link called `base_link`, that is the **mounting point** to the world — not the kinematic root.

### Step 4: The Transform Chain
```text
world → base_link (mounting position, user-defined)
   → base_inertia (fixed offset, may include 180°)
       → shoulder_pan_joint (moving)

```text

- For IK: use `base_inertia` as the root frame
- For visualization: apply all transforms from `world`
```
---

## Implementation (Python)

```python
def find_true_root(urdf_tree):
    """
    Find the true kinematic root of the robot.
    
    The true root is the parent of the first non-fixed joint.
    This handles URDFs where base_link is not the kinematic root.
    """
    # Find first moving joint (ignore fixed)
    first_moving_joint = None
    
    for joint in urdf_tree.findall('joint'):
        joint_type = joint.get('type')
        if joint_type in ['revolute', 'continuous', 'prismatic']:
            first_moving_joint = joint
            break
    
    if first_moving_joint is not None:
        parent_link = first_moving_joint.find('parent').get('link')
        return parent_link
    
    # No moving joints — use first link as fallback
    first_link = urdf_tree.find('link')
    return first_link.get('name') if first_link is not None else "base_link"
```
## Comparison with Other Platforms
| Platform | Assumes base_link = kinematic root	| Handles UR10 correctly |
|----------|------------------------------------|------------------------|
| ROS urdfdom |	✅ Yes |	❌ No (requires workaround) |
| ROS-Industrial |	⚠️ Partial |	⚠️ Adds extra base frame |
| URDF specification | ✅ Yes (implicitly) |	❌ No |
| Hatch | ❌ No — detects true root |	✅ Yes |

## User Guidance
When processing any URDF:

1. Do not assume base_link is the kinematic root

2. Find the first moving joint

3. Use its parent as the kinematic root for IK

4. Preserve all fixed joints in the transform chain

5. Allow the robot to be positioned anywhere in the world (world → base_link transform should be user-configurable)

## Testing Your URDF
If your robot appears rotated 180° or IK gives unexpected results:

1. Check if there is a fixed joint with a rotation before the first moving joint

2. Verify that base_link is not the true kinematic root

The solution is to detect the true root as described above.

---
## Contribution to the Community
This clarification is a genuine contribution to robotics software. Most URDF parsers blindly follow the base_link convention. By documenting and implementing this correctly, we help the community:

- Understand why URDFs with fixed joints before the first moving joint exist

- Process any URDF correctly, regardless of "conventions"

- Eliminate the need for ROS-Industrial workarounds

- Build more robust inverse kinematics

---

## References
URDF Specification: http://wiki.ros.org/urdf/XML

Universal Robots URDF: https://github.com/ros-industrial/universal_robot

Hatch Robotics Platform: https://github.com/victorwu-robotics/hatch

This document is part of Hatch (孵), a principled, event-driven robotics platform.

"Understand them, or you will not fully utilise them."