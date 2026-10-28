"""
Frame confirmation for UR10/FR5/Elfin.

For each arm, one q, confirm:
  1. compute_fk[base_link] is identity (FR5, Elfin) or 180°-about-Z
     (UR10) — the frame structure we suspect.
  2. get_flange_pose(q) is in the true-root frame: it should equal
     compute_fk[true_root]^-1 @ (flange transform in world). We test
     this by checking whether the flange's third column matches a6(q)
     in the true-root frame (the earlier identity check, at machine
     precision).
  3. The FK-with-origins reconstruction in the TRUE-ROOT frame
     (R_origin_1·R(a1_local,q1)·...·R_origin_6·R(a6_local,q6))
     equals R_flange_true_root. If this is ~1e-15 for all three
     arms, the corrected form is confirmed:
        R_flange = R_origin_1·R(a1_local,θ1)·...·R_origin_6·R(a6_local,θ6)
     in the true-root frame, with a_i_local the URDF axis and
     R_origin_i the joint's origin rotation.
  4. The wrist subchain, same form, isolates correctly.

No solver. No new hypotheses. Just confirmation of the frame story.
"""

import numpy as np
from pathlib import Path
from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12: return np.eye(3)
    axis = axis / n
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    return np.array([
        [c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
        [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)],
    ])


def origin_rot(model, joint):
    return model._compose_transform(joint['origin_xyz'], joint['origin_rpy'])[:3, :3]


def axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    return a / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])


def test_arm(name, urdf_path, seed=0):
    print(f"\n{'='*70}\n  {name}\n{'='*70}")
    rng = np.random.default_rng(seed)
    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)
    base_link = model.get_true_root()
    arm_chain = model.get_arm_chain(base_link_name=base_link)
    limits = geom.joint_limits

    q = np.array([rng.uniform(lo, hi) for (lo, hi) in limits])
    if abs(q[4]) < 0.05: q[4] = 0.05

    print(f"  true_root = {base_link}")
    print(f"  arm_chain = {arm_chain}")

    # (1) compute_fk[base_link] vs identity
    transforms = model.compute_fk(q)
    T_base = transforms[base_link]
    res_base_identity = float(np.linalg.norm(T_base - np.eye(4)))
    print(f"\n  (1) ||compute_fk[base_link] - I||_F = {res_base_identity:.3e}")
    if res_base_identity > 1e-6:
        # print the rotation angle of T_base
        R_base = T_base[:3, :3]
        ang = float(np.arccos(np.clip((np.trace(R_base) - 1.0) / 2.0, -1, 1)))
        print(f"      base_link rotation angle = {ang:.6f} rad "
              f"({ang/np.pi:.4f}π)")

    # (2) get_flange_pose is in true-root frame:
    #     R_flange[:,2] == a6(q) in true-root frame.
    R_flange = model.get_flange_pose(q)[:3, :3]

    # Compute a6(q) in true-root frame directly via FK
    joint6 = model.joints[arm_chain[5]]
    parent6 = joint6['parent']
    T_parent_world = transforms[parent6]
    T_joint_world = T_parent_world @ model._compose_transform(
        joint6['origin_xyz'], joint6['origin_rpy'])
    # bring to true-root frame
    T_true_root_world = transforms[base_link]
    T_joint_in_root = np.linalg.inv(T_true_root_world) @ T_joint_world
    a6_at_q_root = T_joint_in_root[:3, :3] @ axis_local(joint6)
    a6_at_q_root /= np.linalg.norm(a6_at_q_root)
    res_a6 = float(np.linalg.norm(R_flange[:,2] - a6_at_q_root))
    print(f"\n  (2) ||R_flange[:,2] - a6(q)_in_true_root|| = {res_a6:.3e}")
    print(f"      (small -> get_flange_pose is in true-root frame)")

    # (3) FK-with-origins in TRUE-ROOT frame
    R_fk = np.eye(3)
    for i, jname in enumerate(arm_chain):
        joint = model.joints[jname]
        R_fk = R_fk @ origin_rot(model, joint) @ rodrigues(axis_local(joint), q[i])
    res_fk = float(np.linalg.norm(R_fk - R_flange))
    print(f"\n  (3) FK-with-origins vs R_flange (true-root frame):")
    print(f"      ||R_fk - R_flange||_F = {res_fk:.3e}")
    print(f"      (small -> FK-with-origins in true-root frame is the "
          f"correct form)")

    # (4) Wrist subchain in the same frame
    R03_fk = np.eye(3)
    for i in range(3):
        joint = model.joints[arm_chain[i]]
        R03_fk = R03_fk @ origin_rot(model, joint) @ rodrigues(axis_local(joint), q[i])
    R_wrist_target = R03_fk.T @ R_flange

    R_wrist_fk = np.eye(3)
    for i in range(3, 6):
        joint = model.joints[arm_chain[i]]
        R_wrist_fk = R_wrist_fk @ origin_rot(model, joint) @ rodrigues(axis_local(joint), q[i])
    res_wrist = float(np.linalg.norm(R_wrist_fk - R_wrist_target))
    print(f"\n  (4) Wrist subchain, FK-with-origins:")
    print(f"      ||R_wrist_fk - R03_fk^T·R_flange|| = {res_wrist:.3e}")
    print(f"      (small -> wrist inversion must include origin rotations)")


def main():
    base = Path.home() / "hatch" / "assets" / "robots"
    for name, p in [
        ("UR10",  base / "ur_description" / "urdf" / "ur10_nominal.urdf"),
        ("FR5",   base / "Farino" / "frcobot_description" / "urdf" / "fr5v6.urdf"),
        ("Elfin", base / "E15_Pro" / "urdf" / "E15_Pro.urdf"),
    ]:
        if not p.exists():
            print(f"SKIP {name}: {p}")
            continue
        try:
            test_arm(name, p)
        except Exception as e:
            print(f"ERROR in {name}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()