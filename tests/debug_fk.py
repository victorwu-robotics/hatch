#!/usr/bin/env python3
"""Debug: compare model FK with the numerical solver's FK at zero config."""

import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path.home() / "hatch"))

from core.kinematics.kinematic_model import KinematicModel

# Load model
urdf_path = Path.home() / "hatch/assets/robots/E15_Pro/urdf/E15_Pro.urdf"
package_dirs = [
    str(urdf_path.parent),
    str(urdf_path.parent.parent),
    str(Path.home() / "hatch" / "assets"),
    str(Path.home() / "hatch" / "assets" / "robots"),
]

model = KinematicModel(
    urdf_path=str(urdf_path),
    package_dirs=package_dirs,
    asset_id="E15_Pro"
)
model.load()

# Get the arm chain
true_root = model.get_true_root()
arm_chain = model.get_arm_chain(true_root)
print(f"Arm chain: {arm_chain}")
print(f"Joint names: {model.get_joint_info()['names']}")

# Test at zero configuration
q = np.zeros(6)
print(f"\n=== FK at q = {q} ===")

# Method 1: model.forward_kinematics
T_fk = model.forward_kinematics(q)
print(f"\nmodel.forward_kinematics(q):")
print(f"  position: ({T_fk[0,3]:.4f}, {T_fk[1,3]:.4f}, {T_fk[2,3]:.4f})")
print(f"  rotation:\n{T_fk[:3, :3]}")

# Method 2: model.get_tcp_pose
T_tcp = model.get_tcp_pose()
print(f"\nmodel.get_tcp_pose():")
print(f"  position: ({T_tcp[0,3]:.4f}, {T_tcp[1,3]:.4f}, {T_tcp[2,3]:.4f})")
print(f"  rotation:\n{T_tcp[:3, :3]}")

# Method 3: model.link_transforms at the tool mount link
if hasattr(model, 'tool_mount_link') and model.tool_mount_link:
    T_link = model.link_transforms.get(model.tool_mount_link)
    if T_link is not None:
        print(f"\nmodel.link_transforms['{model.tool_mount_link}']:")
        print(f"  position: ({T_link[0,3]:.4f}, {T_link[1,3]:.4f}, {T_link[2,3]:.4f})")
        print(f"  rotation:\n{T_link[:3, :3]}")

# Print all link transforms
print(f"\n=== All link transforms at q=0 ===")
for link_name, T in model.link_transforms.items():
    pos = T[:3, 3]
    print(f"  {link_name} position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"  {link_name} rotation:\n{T[:3, :3]}")

# Joint-to-joint relative transforms
print(f"\n=== Joint-to-joint relative transforms ===")
current_link = true_root
for joint_name in arm_chain:
    joint = model.joints[joint_name]
    child_link = joint['child']
    T_parent = model.link_transforms[current_link]
    T_child = model.link_transforms[child_link]
    T_rel = np.linalg.inv(T_parent) @ T_child
    print(f"  {joint_name}: {current_link} → {child_link}")
    print(f"    pos: ({T_rel[0,3]:.4f}, {T_rel[1,3]:.4f}, {T_rel[2,3]:.4f})")
    print(f"    rot:\n{T_rel[:3, :3]}")
    current_link = child_link