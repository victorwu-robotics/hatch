"""
FK vs PoE test.

For each arm (UR10, FR5, Elfin), one random q within limits:

  A. Full FK with origin rotations:
        R_fk = R_origin_1 · R(a1_local, q1) · R_origin_2 · R(a2_local, q2)
               · ... · R_origin_6 · R(a6_local, q6)
     Compare to R_flange_in_base (from model.get_flange_pose, brought to base).

  B. Simplified PoE with base-frame axes:
        R_poe = R(a1_base, q1) · R(a2_base, q2) · ... · R(a6_base, q6)
     Compare to R_flange_in_base.

  C. Wrist-only comparison:
        R03_fk = R_origin_1·R(a1_local,q1)·R_origin_2·R(a2_local,q2)·R_origin_3·R(a3_local,q3)
        R_wrist_target = R03_fk^T · R_flange_in_base
        R_wrist_fk = R_origin_4·R(a4_local,q4)·R_origin_5·R(a5_local,q5)·R_origin_6·R(a6_local,q6)
        Compare R_wrist_fk to R_wrist_target.

Prints residuals for each. Whichever form gives ~1e-15 is the correct form
to use in the solver.
"""

import numpy as np
from pathlib import Path

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    return np.array([
        [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)],
    ])


def joint_origin_rotation(model, joint):
    """Rotation part of the joint's origin (xyz, rpy) transform."""
    T = model._compose_transform(joint['origin_xyz'], joint['origin_rpy'])
    return T[:3, :3]


def joint_axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return a / n


def test_arm(name, urdf_path, seed=0):
    print(f"\n{'='*70}\n  {name}  (seed={seed})\n{'='*70}")
    rng = np.random.default_rng(seed)

    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)
    base_link = model.get_true_root()

    arm_chain = model.get_arm_chain(base_link_name=base_link)
    limits = geom.joint_limits

    q = np.array([rng.uniform(lo, hi) for (lo, hi) in limits])
    if abs(q[4]) < 0.05:
        q[4] = 0.05

    print(f"  q = {np.array2string(q, precision=4)}")

    # --- Get R_flange in base frame ---
    T_flange_world = model.get_flange_pose(q)
    transforms = model.compute_fk(q)
    T_base_world = transforms[base_link]
    T_base_world_inv = np.linalg.inv(T_base_world)
    T_flange_in_base = T_base_world_inv @ T_flange_world
    R_flange_in_base = T_flange_in_base[:3, :3]

    # --- A: Full FK with origin rotations ---
    R_fk = np.eye(3)
    for i, jname in enumerate(arm_chain):
        joint = model.joints[jname]
        R_origin = joint_origin_rotation(model, joint)
        axis_local = joint_axis_local(joint)
        R_joint = rodrigues(axis_local, q[i])
        R_fk = R_fk @ R_origin @ R_joint
    res_fk = float(np.linalg.norm(R_fk - R_flange_in_base))

    # --- B: Simplified PoE with base-frame axes ---
    a_base = [ax.direction for ax in geom.axes]
    R_poe = np.eye(3)
    for i in range(6):
        R_poe = R_poe @ rodrigues(a_base[i], q[i])
    res_poe = float(np.linalg.norm(R_poe - R_flange_in_base))

    # --- C: Wrist-only, both forms ---
    # R03 with origins (correct FK form)
    R03_fk = np.eye(3)
    for i in range(3):
        joint = model.joints[arm_chain[i]]
        R_origin = joint_origin_rotation(model, joint)
        axis_local = joint_axis_local(joint)
        R_joint = rodrigues(axis_local, q[i])
        R03_fk = R03_fk @ R_origin @ R_joint

    R_wrist_target_fk = R03_fk.T @ R_flange_in_base

    # R_wrist from origins-included FK on the last three joints
    R_wrist_fk = np.eye(3)
    for i in range(3, 6):
        joint = model.joints[arm_chain[i]]
        R_origin = joint_origin_rotation(model, joint)
        axis_local = joint_axis_local(joint)
        R_joint = rodrigues(axis_local, q[i])
        R_wrist_fk = R_wrist_fk @ R_origin @ R_joint
    res_wrist_fk = float(np.linalg.norm(R_wrist_fk - R_wrist_target_fk))

    # R03 with simplified PoE (base-frame axes)
    R03_poe = np.eye(3)
    for i in range(3):
        R03_poe = R03_poe @ rodrigues(a_base[i], q[i])
    R_wrist_target_poe = R03_poe.T @ R_flange_in_base

    # R_wrist from simplified PoE
    R_wrist_poe = np.eye(3)
    for i in range(3, 6):
        R_wrist_poe = R_wrist_poe @ rodrigues(a_base[i], q[i])
    res_wrist_poe = float(np.linalg.norm(R_wrist_poe - R_wrist_target_poe))

    # --- Report ---
    print(f"\n  Full-chain reconstruction (base-frame R_flange):")
    print(f"    A. FK with origins           : {res_fk:.3e}")
    print(f"    B. Simplified PoE            : {res_poe:.3e}")

    print(f"\n  Wrist subchain reconstruction:")
    print(f"    C1. FK with origins          : {res_wrist_fk:.3e}")
    print(f"    C2. Simplified PoE           : {res_wrist_poe:.3e}")

    # --- Per-joint origin rotations (to see if they are non-trivial) ---
    print(f"\n  Joint origin rotations (angle of R_origin about its axis):")
    for i, jname in enumerate(arm_chain):
        joint = model.joints[jname]
        R_origin = joint_origin_rotation(model, joint)
        # Angle of a rotation matrix
        angle = float(np.arccos(np.clip((np.trace(R_origin) - 1.0) / 2.0,
                                        -1.0, 1.0)))
        # Rotation axis
        if abs(angle) < 1e-9:
            axis_str = "(identity)"
        else:
            s_vec = np.array([
                R_origin[2,1] - R_origin[1,2],
                R_origin[0,2] - R_origin[2,0],
                R_origin[1,0] - R_origin[0,1],
            ]) / (2 * np.sin(angle))
            axis_str = f"axis={np.array2string(s_vec, precision=3)}"
        print(f"    J{i+1} ({jname:24s}): angle={angle:+.4f} rad  {axis_str}")


def main():
    base = Path.home() / "hatch" / "assets" / "robots"
    arms = [
        ("UR10",  base / "ur_description" / "urdf" / "ur10_nominal.urdf"),
        ("FR5",   base / "Farino" / "frcobot_description" / "urdf" / "fr5v6.urdf"),
        ("Elfin", base / "E15_Pro" / "urdf" / "E15_Pro.urdf"),
    ]
    for name, path in arms:
        if not path.exists():
            print(f"SKIP {name}: {path} not found")
            continue
        try:
            test_arm(name, path, seed=0)
        except Exception as e:
            print(f"ERROR in {name}: {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()