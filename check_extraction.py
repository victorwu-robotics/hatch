from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry
import numpy as np

for name, path in [
    ("UR10",   "assets/robots/ur_description/urdf/ur10_nominal.urdf"),
    ("FR5",    "assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf"),
    ("Elfin",  "assets/robots/E15_Pro/urdf/E15_Pro.urdf"),
]:
    model = KinematicModel(path)
    geom = extract_arm_geometry(model)
    print(f"\n=== {name} ===")
    print(geom.describe())

# Exercise the A-target invariants on a non-zero pose.
#
# UR10 §9.2 example: P_flange = (0.5, 0.3, 0.8), identity orientation.
#   ẑ_flange = (0, 0, 1);  P5 = (0.5, 0.3, 0.7078);  θ₁_right = 0.2552;
#   lat = −0.5·sin(0.2552) + 0.3·cos(0.2552) = 0.16402;
#   cos θ₅ = (0.16402 − 0.163941)/0.0922 ≈ 0.00086  ⇒  θ₅ ≈ π/2.
#
# The flip sign is NOT yet pinned. This block prints BOTH branches.
# Run compute_fk at the solver's eventual solution and see which branch
# reproduces T_flange; record the result in the commit message and in
# docs/unified_IK.md Part 6.10. Until then, do not treat either as the
# convention.

def _sample(name, path, p5, z_flange, theta1, theta5):
    model = KinematicModel(path)
    geom = extract_arm_geometry(model)
    n = geom.arm_plane_normal(theta1)
    print(f"\n=== {name}: A-target invariants ===")
    print(f"  wrist_normal_offset = {geom.wrist_normal_offset:.6f}")
    print(f"  wrist_radial_offset = {geom.wrist_radial_offset:.6f}")
    print(f"  cos θ₅              = {np.cos(theta5):.6f}")
    for flip in (False, True):
        label = "flip=True " if flip else "flip=False"
        z_j5 = geom.j5_axis(z_flange, theta1, theta5, flip)
        B = p5 - geom.wrist_radial_offset * z_j5
        A = geom.wrist_root_target(p5, z_j5, theta1)
        on_plane = abs(float(np.dot(A - geom.axes[1].point, n)))
        print(f"  --- {label} ---")
        print(f"    z_J5            = {z_j5.round(6)}")
        print(f"    |z_J5|          = {np.linalg.norm(z_j5):.12f}  (must be 1)")
        print(f"    z_J5 · n        = {float(np.dot(z_j5, n)):.3e}  (must be ~0)")
        print(f"    z_J5 · z_flange = {float(np.dot(z_j5, z_flange/np.linalg.norm(z_flange))):.6f}"
              f"  (must be cos θ₅ = {np.cos(theta5):.6f})")
        print(f"    |B − A|         = {np.linalg.norm(B - A):.6f}"
              f"  (must be wrist_normal_offset)")
        print(f"    |A − J2_origin| in-plane residual = {on_plane:.3e}  (~0)")
        print(f"    elbow_to_wrist(target A) = {geom.elbow_to_wrist(A):.6f}")

_sample("UR10", "assets/robots/ur_description/urdf/ur10_nominal.urdf",
        np.array([0.5, 0.3, 0.7078]), np.array([0.0, 0.0, 1.0]),
        0.2552, np.pi/2)

_sample("Elfin", "assets/robots/E15_Pro/urdf/E15_Pro.urdf",
        np.array([0.5, 0.3, 0.7078]), np.array([0.0, 0.0, 1.0]),
        float(np.arctan2(0.3, 0.5)), np.pi/2)