#!/usr/bin/env python3
"""
Extract complete D-H parameters from the working KinematicModel.

For each joint, computes the relative transform at zero configuration
and decomposes it into standard D-H parameters including theta offset.
"""

import numpy as np
from math import pi, atan2, sin, cos, sqrt
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

model = KinematicModel(urdf_path=str(urdf_path), package_dirs=package_dirs)
model.load()

true_root = model.get_true_root()
arm_chain = model.get_arm_chain(true_root)

print("Complete Standard D-H Parameters (including theta offsets):\n")

dh_params = []
current_link = true_root

for joint_name in arm_chain:
    joint = model.joints[joint_name]
    child_link = joint['child']

    # Get world transforms at zero configuration
    T_parent = model.link_transforms[current_link]
    T_child = model.link_transforms[child_link]

    # Relative transform from parent to child at θ=0
    T_rel = np.linalg.inv(T_parent) @ T_child

    # Extract the joint axis direction in the parent frame
    # The joint rotates about Z in its own frame, so we need Z_child expressed in parent frame
    Z_child_in_parent = T_rel[:3, 2]  # Third column of relative rotation

    print(f"{joint_name} ({current_link} → {child_link}):")
    print(f"  Joint axis in parent frame: ({Z_child_in_parent[0]:.4f}, "
          f"{Z_child_in_parent[1]:.4f}, {Z_child_in_parent[2]:.4f})")

    # The D-H convention assumes the joint rotates about Z of its own frame.
    # At θ=0, the child frame's Z is aligned with the joint axis.
    # The transform is: RotZ(θ)·TransZ(d)·TransX(a)·RotX(α)
    #
    # At θ=0: T_rel = TransZ(d)·TransX(a)·RotX(α)
    #
    # The third column of T_rel is the Z-axis of the child frame:
    #   Z_child = [0, -sin(α), cos(α)] after RotX(α)
    # The fourth column is the position: [a, -d·sin(α), d·cos(α)]

    z_child = T_rel[:3, 2]
    pos = T_rel[:3, 3]

    # Extract α from Z_child: Z_child = [0, -sin(α), cos(α)]
    alpha = atan2(-z_child[1], z_child[2])

    # Verify that Z_child[0] ≈ 0 (X component of Z should be zero in standard D-H)
    if abs(z_child[0]) > 0.01:
        print(f"  ⚠ WARNING: Z_child[0] = {z_child[0]:.4f} — not pure RotX(α)")
        print(f"    This joint cannot be represented in standard D-H!")

    # Extract d and a from position: pos = [a, -d·sin(α), d·cos(α)]
    a = pos[0]

    if abs(sin(alpha)) > 1e-10:
        d = -pos[1] / sin(alpha)
    elif abs(cos(alpha)) > 1e-10:
        d = pos[2] / cos(alpha)
    else:
        d = 0.0

    # For the θ offset: we need to find θ such that
    # RotZ(θ)·TransZ(d)·TransX(a)·RotX(α) = T_rel
    # Since we already matched d, a, α, the remaining rotation about Z is θ.
    #
    # After removing d, a, α from T_rel, the residual rotation should be RotZ(θ).
    # At θ=0: the X-axis of the child is [cos(α)? no]
    # Actually: the first column of RotX(α) is [1, 0, 0]
    # So the first column of T_rel should be [cos(θ), sin(θ), 0]
    # → θ = atan2(T_rel[1,0], T_rel[0,0])

    theta_offset = atan2(T_rel[1, 0], T_rel[0, 0])

    print(f"  α = {alpha:.4f} ({alpha/pi:.4f}π)")
    print(f"  a = {a:.4f} m")
    print(f"  d = {d:.4f} m")
    print(f"  θ_offset = {theta_offset:.4f} ({theta_offset/pi:.4f}π)")
    print()

    dh_params.append({
        'a': a, 'd': d, 'alpha': alpha, 'theta_offset': theta_offset
    })

    current_link = child_link

# Print the complete D-H table
print("\n=== Complete D-H Table ===")
print(f"{'Joint':<12} {'a (m)':<10} {'d (m)':<10} {'α (rad)':<10} {'α (π)':<10} {'θ_offset':<10}")
print("-" * 62)
for i, p in enumerate(dh_params):
    print(f"{arm_chain[i]:<12} {p['a']:<10.4f} {p['d']:<10.4f} "
          f"{p['alpha']:<10.4f} {p['alpha']/pi:<10.4f} {p['theta_offset']:<10.4f}")

# Now verify with standard D-H FK
def dh_transform_std(a, d, alpha, theta):
    """Standard D-H: RotZ(θ)·TransZ(d)·TransX(a)·RotX(α)."""
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])

print("\n=== Verify D-H FK at zero configuration ===")
T = np.eye(4)
for i, p in enumerate(dh_params):
    T = T @ dh_transform_std(p['a'], p['d'], p['alpha'], p['theta_offset'])

print(f"TCP position: ({T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f})")
print(f"Expected:     (0.0000, 0.0000, 1.5620)")