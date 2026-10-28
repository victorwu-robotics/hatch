"""
Diagnose phi_0.

For each arm, print axis directions at zero pose, zero-pose joint positions,
and three candidate phi_0 values. Then, for one random q_true, print phi
and compare to q_true[0] under each phi_0 candidate.
"""

import numpy as np
from pathlib import Path

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry
from core.kinematics.solver_v2 import rodrigues, wrap


def _origin_rot(model, joint):
    return model._compose_transform(joint['origin_xyz'],
                                    joint['origin_rpy'])[:3, :3]


def _axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    return a / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])


def _joint_frame_0(model, arm_chain, idx):
    transforms = model.compute_fk(np.zeros(6))
    base = model.get_true_root()
    T_base_inv = np.linalg.inv(transforms[base])
    jname = arm_chain[idx]
    j = model.joints[jname]
    T_p = transforms[j['parent']]
    T_o = model._compose_transform(j['origin_xyz'], j['origin_rpy'])
    return T_base_inv @ T_p @ T_o


def diagnose(name, urdf_path, seed=0):
    print(f"\n{'='*70}\n  {name}\n{'='*70}")
    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)
    arm_chain = model.get_arm_chain(base_link_name=model.get_true_root())
    limits = geom.joint_limits

    a1 = geom.axes[0].direction / np.linalg.norm(geom.axes[0].direction)
    a2 = geom.axes[1].direction / np.linalg.norm(geom.axes[1].direction)
    a4 = geom.axes[3].direction / np.linalg.norm(geom.axes[3].direction)

    print(f"  a1_base = {a1.round(4)}")
    print(f"  a2_base = {a2.round(4)}")
    print(f"  a4_base = {a4.round(4)}")

    T1 = _joint_frame_0(model, arm_chain, 0)
    T2 = _joint_frame_0(model, arm_chain, 1)
    T3 = _joint_frame_0(model, arm_chain, 2)
    T4 = _joint_frame_0(model, arm_chain, 3)
    T5 = _joint_frame_0(model, arm_chain, 4)

    J1_origin = T1[:3, 3]
    J2_origin = T2[:3, 3]
    J3_origin = T3[:3, 3]
    A_0 = T4[:3, 3]
    print(f"  J1_origin = {J1_origin.round(4)}")
    print(f"  J2_origin = {J2_origin.round(4)}")
    print(f"  J3_origin = {J3_origin.round(4)}")
    print(f"  A_0 (J4)  = {A_0.round(4)}")

    # Basis perpendicular to a1
    e_x = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(e_x, a1)) > 0.9:
        e_x = np.array([0.0, 1.0, 0.0])
    e_x = e_x - np.dot(e_x, a1) * a1
    e_x = e_x / np.linalg.norm(e_x)
    e_y = np.cross(a1, e_x)
    e_y = e_y / np.linalg.norm(e_y)
    print(f"  e_x = {e_x.round(4)},  e_y = {e_y.round(4)}")

    # Candidate phi_0 values
    a2_perp = a2 - np.dot(a2, a1) * a1
    a2_perp = a2_perp / np.linalg.norm(a2_perp)
    phi_0_from_a2 = float(np.arctan2(np.dot(a2_perp, e_y), np.dot(a2_perp, e_x)))

    n_perp = np.cross(a1, a2)
    n_perp = n_perp / np.linalg.norm(n_perp)
    phi_0_from_cross = float(np.arctan2(np.dot(n_perp, e_y), np.dot(n_perp, e_x)))

    v = J3_origin - J2_origin
    v_perp = v - np.dot(v, a1) * a1
    if np.linalg.norm(v_perp) < 1e-6:
        phi_0_from_pos = None
    else:
        phi_0_from_pos = float(np.arctan2(np.dot(v_perp, e_y), np.dot(v_perp, e_x)))

    v2 = A_0 - J2_origin
    v2_perp = v2 - np.dot(v2, a1) * a1
    if np.linalg.norm(v2_perp) < 1e-6:
        phi_0_from_A = None
    else:
        phi_0_from_A = float(np.arctan2(np.dot(v2_perp, e_y), np.dot(v2_perp, e_x)))

    print(f"\n  phi_0 candidates:")
    print(f"    from a2_base     : {phi_0_from_a2:+.4f} rad")
    print(f"    from cross(a1,a2): {phi_0_from_cross:+.4f} rad")
    print(f"    from J3-J2 pos   : "
          f"{f'{phi_0_from_pos:+.4f} rad' if phi_0_from_pos is not None else 'undefined'}")
    print(f"    from A-J2 pos    : "
          f"{f'{phi_0_from_A:+.4f} rad' if phi_0_from_A is not None else 'undefined'}")

    # One random q_true, check phi
    rng = np.random.default_rng(seed)
    q_true = np.array([rng.uniform(lo, hi) for (lo, hi) in limits])
    T_flange = model.get_flange_pose(q_true)
    P5 = T_flange[:3, 3] - geom.flange_offset * T_flange[:3, 2]
    P5_rel = P5 - J1_origin
    P5_perp = P5_rel - np.dot(P5_rel, a1) * a1
    phi = float(np.arctan2(np.dot(P5_perp, e_y), np.dot(P5_perp, e_x)))

    print(f"\n  One random q_true (seed={seed}):")
    print(f"    q_true = {q_true.round(4)}")
    print(f"    q_true[0] = {q_true[0]:+.4f}")
    print(f"    phi = {phi:+.4f}")
    print(f"    phi - q_true[0] = {wrap(phi - q_true[0]):+.4f}")
    print(f"\n    phi_target for each phi_0:")
    for label, p0 in [("a2_base", phi_0_from_a2),
                      ("cross", phi_0_from_cross),
                      ("pos", phi_0_from_pos),
                      ("A", phi_0_from_A)]:
        if p0 is None:
            print(f"      {label:10s}: undefined")
        else:
            pt = wrap(phi - p0)
            pt_neg = wrap(-(phi - p0))
            print(f"      {label:10s}: phi - p0 = {pt:+.4f}  "
                  f"| -(phi - p0) = {pt_neg:+.4f}  "
                  f"| q_true[0] = {q_true[0]:+.4f}")


def main():
    base = Path.home() / "hatch" / "assets" / "robots"
    arms = [
        ("UR10",  base / "ur_description" / "urdf" / "ur10_nominal.urdf"),
        ("FR5",   base / "Farino" / "frcobot_description" / "urdf" / "fr5v6.urdf"),
        ("Elfin", base / "E15_Pro" / "urdf" / "E15_Pro.urdf"),
    ]
    for name, path in arms:
        if not path.exists():
            print(f"SKIP {name}")
            continue
        try:
            diagnose(name, path)
        except Exception as e:
            print(f"ERROR in {name}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()