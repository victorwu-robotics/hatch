"""
Diagnostic test for solver_v2 (corrected).

Checks Claims 1-8 for each arm over N=50 trials.
"""

import numpy as np
from pathlib import Path

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry
from core.kinematics.solver_v2 import UnifiedIKSolverV2, rodrigues, wrap


def _origin_rot(model, joint):
    return model._compose_transform(joint['origin_xyz'],
                                    joint['origin_rpy'])[:3, :3]


def _axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    return a / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])


def _p5_from_fk(model, arm_chain, q):
    transforms = model.compute_fk(q)
    base = model.get_true_root()
    T_base_inv = np.linalg.inv(transforms[base])

    def joint_origin_axis(idx):
        jname = arm_chain[idx]
        j = model.joints[jname]
        T_p = transforms[j['parent']]
        T_o = model._compose_transform(j['origin_xyz'], j['origin_rpy'])
        T_joint = T_base_inv @ T_p @ T_o
        return T_joint[:3, 3], T_joint[:3, :3] @ _axis_local(j)

    p5_pt, p5_ax = joint_origin_axis(4)
    p6_pt, p6_ax = joint_origin_axis(5)
    u, v = p5_ax, p6_ax
    w = p6_pt - p5_pt
    uv = np.dot(u, v)
    denom = 1 - uv*uv
    if abs(denom) < 1e-9:
        return p5_pt
    s = (np.dot(w, u) - np.dot(w, v) * uv) / denom
    return p5_pt + s * u


def _A_from_fk(model, arm_chain, q):
    """FK-derived A (J4 joint origin in true-root frame)."""
    transforms = model.compute_fk(q)
    base = model.get_true_root()
    T_base_inv = np.linalg.inv(transforms[base])
    j4 = model.joints[arm_chain[3]]
    T_p = transforms[j4['parent']]
    T_o = model._compose_transform(j4['origin_xyz'], j4['origin_rpy'])
    T_j4 = T_base_inv @ T_p @ T_o
    return T_j4[:3, 3], T_j4[:3, :3] @ _axis_local(j4)


def _R03_from_fk(model, arm_chain, q):
    R = np.eye(3)
    for i in range(3):
        j = model.joints[arm_chain[i]]
        R = R @ _origin_rot(model, j) @ rodrigues(_axis_local(j), q[i])
    return R


def _R_wrist_true(model, arm_chain, q):
    R = np.eye(3)
    for i in range(3, 6):
        j = model.joints[arm_chain[i]]
        R = R @ _origin_rot(model, j) @ rodrigues(_axis_local(j), q[i])
    return R


def _diff(a, b):
    return float(np.max(np.abs(wrap(np.asarray(a) - np.asarray(b)))))


def _match_angles(a, b, tol=1e-4):
    d_direct = _diff(a, b)
    b_flip = np.array([wrap(b[0] + np.pi), wrap(-b[1]), wrap(b[2] + np.pi)])
    d_flip = _diff(a, b_flip)
    return (d_direct < tol, d_flip < tol)


def test_arm(name, urdf_path, n_trials=50, seed=0):
    print(f"\n{'='*70}\n  {name}\n{'='*70}")
    rng = np.random.default_rng(seed)
    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)
    arm_chain = model.get_arm_chain(base_link_name=model.get_true_root())
    solver = UnifiedIKSolverV2(model, geom)

    claim_pass = {f"C{i}": 0 for i in range(1, 9)}
    fail_details = []

    for trial in range(n_trials):
        q_true = np.array([rng.uniform(lo, hi) for (lo, hi) in geom.joint_limits])
        if abs(q_true[4]) < 0.1:
            q_true[4] = 0.1

        T_flange = model.get_flange_pose(q_true)
        diag = solver.solve_with_diagnostics(T_flange)
        trial_ok = {f"C{i}": False for i in range(1, 9)}

        # C1: P5
        P5_fk = _p5_from_fk(model, arm_chain, q_true)
        if np.linalg.norm(diag['P5'] - P5_fk) < 1e-6:
            trial_ok['C1'] = True

        # C2: q_true[0] in theta1 candidates
        for theta1, sigma, tag1 in diag['theta1_candidates']:
            if abs(wrap(theta1 - q_true[0])) < 1e-4:
                trial_ok['C2'] = True
                break

        # Find matching entry
        matching_entry = None
        for entry in diag['per_theta1']:
            if abs(wrap(entry['theta1'] - q_true[0])) < 1e-4:
                matching_entry = entry
                break

        # C3: P_J4 matches FK-derived A
        if matching_entry is not None:
            A_fk, _ = _A_from_fk(model, arm_chain, q_true)
            if np.linalg.norm(matching_entry['P_J4'] - A_fk) < 1e-4:
                trial_ok['C3'] = True

        # C4: q_true[1:3] in arm_candidates
        if matching_entry is not None:
            for theta2, theta3, tag in matching_entry['arm_candidates']:
                if (abs(wrap(theta2 - q_true[1])) < 1e-4
                        and abs(wrap(theta3 - q_true[2])) < 1e-4):
                    trial_ok['C4'] = True
                    break

        # C5, C6, C7: match the arm branch to q_true[1:3]
        if matching_entry is not None:
            for arm_entry in matching_entry['arm_details']:
                if (abs(wrap(arm_entry['theta2'] - q_true[1])) < 1e-4
                        and abs(wrap(arm_entry['theta3'] - q_true[2])) < 1e-4):
                    R03_true = _R03_from_fk(model, arm_chain, q_true)
                    if np.linalg.norm(arm_entry['R03_fk'] - R03_true) < 1e-9:
                        trial_ok['C5'] = True

                    R_wrist_true = _R_wrist_true(model, arm_chain, q_true)
                    if np.linalg.norm(arm_entry['R_wrist_target']
                                      - R_wrist_true) < 1e-9:
                        trial_ok['C6'] = True

                    for (t4, t5, t6, res) in arm_entry['wrist_candidates']:
                        d_dir, d_flip = _match_angles(
                            q_true[3:6], np.array([t4, t5, t6]))
                        if d_dir or d_flip:
                            trial_ok['C7'] = True
                            break
                    break

        # C8: some solution reproduces T_flange
        for sol in diag['solutions']:
            T_sol = model.get_flange_pose(sol.q)
            if np.linalg.norm(T_sol - T_flange) < 1e-4:
                trial_ok['C8'] = True
                break

        for k in trial_ok:
            if trial_ok[k]:
                claim_pass[k] += 1
        if not all(trial_ok.values()):
            fail_details.append((trial, q_true.copy(), trial_ok.copy(),
                                 diag, matching_entry))

    print(f"\n  Claim pass rates (N={n_trials}):")
    for i in range(1, 9):
        k = f"C{i}"
        print(f"    {k}: {claim_pass[k]}/{n_trials}")

    if fail_details:
        print(f"\n  {len(fail_details)} trials had failures. First 3:")
        for (trial, q_true, ok, diag, entry) in fail_details[:3]:
            print(f"\n    trial {trial}  q_true = {q_true.round(4)}")
            failed = [k for k, v in ok.items() if not v]
            print(f"      failed: {failed}")
            print(f"      theta1_candidates = "
                  f"{[(round(t[0],4), t[1], t[2]) for t in diag['theta1_candidates']]}")
            if entry is not None:
                A_fk, _ = _A_from_fk(model, arm_chain, q_true)
                print(f"      P_J4 = {entry['P_J4'].round(4)}  A_fk = {A_fk.round(4)}")
                print(f"      |P_J4 - A_fk| = {np.linalg.norm(entry['P_J4'] - A_fk):.2e}")
                print(f"      arm_candidates = "
                      f"{[(round(a[0],4), round(a[1],4), a[2]) for a in entry['arm_candidates']]}")


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
            test_arm(name, path, n_trials=50, seed=0)
        except Exception as e:
            print(f"ERROR in {name}: {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()