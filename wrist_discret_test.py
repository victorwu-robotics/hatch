"""
Wrist discreteness test — v2, origin-included form.

Same structure as wrist_discret_test.py, but:
  - R03 includes origin rotations: R03_fk.
  - R_wrist_target = R03_fk^T · R_flange.
  - Product inversion solves
        R_wrist_target = R_origin_4·R(a4_local,t4)
                       · R_origin_5·R(a5_local,t5)
                       · R_origin_6·R(a6_local,t6)
    for (t4, t5, t6).
  - ZYZ comparison kept for reference (ZYZ uses R03_fk^T · R_flange too,
    so the comparison is fair).
"""

import numpy as np
from pathlib import Path

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

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


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def angle_diff(a, b):
    return wrap(np.asarray(a) - np.asarray(b))


def angles_match_report(a, b, tol=1e-4):
    direct = np.max(np.abs(angle_diff(a, b)))
    flipped_b = np.array([wrap(b[0] + np.pi), wrap(-b[1]), wrap(b[2] + np.pi)])
    flipped = np.max(np.abs(angle_diff(a, flipped_b)))
    which = "direct" if direct < tol else ("flip" if flipped < tol else "FAIL")
    return which, direct, flipped


def origin_rot(model, joint):
    return model._compose_transform(joint['origin_xyz'], joint['origin_rpy'])[:3, :3]


def axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    return a / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])


# ----------------------------------------------------------------------
# ZYZ on R_wrist_target (kept for reference)
# ----------------------------------------------------------------------

def extract_zyz(R_wrist):
    r13, r23, r33 = R_wrist[0,2], R_wrist[1,2], R_wrist[2,2]
    r31, r32 = R_wrist[2,0], R_wrist[2,1]
    sin5_sq = 1.0 - r33**2
    if sin5_sq < 1e-12:
        if r33 > 0:
            return [(0.0, 0.0, np.arctan2(R_wrist[1,0], R_wrist[0,0]))]
        else:
            return [(0.0, np.pi, np.arctan2(-R_wrist[1,0], -R_wrist[0,0]))]
    sin5 = np.sqrt(sin5_sq)
    t5a = np.arctan2(sin5, r33)
    t4a = np.arctan2(r23 / sin5, r13 / sin5)
    t6a = np.arctan2(r32 / sin5, -r31 / sin5)
    t5b = np.arctan2(-sin5, r33)
    t4b = np.arctan2(-r23 / sin5, -r13 / sin5)
    t6b = np.arctan2(-r32 / sin5, r31 / sin5)
    return [(t4a, t5a, t6a), (t4b, t5b, t6b)]


# ----------------------------------------------------------------------
# Origin-included product inversion
# ----------------------------------------------------------------------

def _R_wrist_pred(t4, t5, t6, R_o4, a4, R_o5, a5, R_o6, a6):
    return (R_o4 @ rodrigues(a4, t4)
            @ R_o5 @ rodrigues(a5, t5)
            @ R_o6 @ rodrigues(a6, t6))


def _t6_residual(R_prefix_inv, R_target, R_o6, a6):
    """Given R_prefix_inv = (R_o4·R(a4,t4)·R_o5·R(a5,t5))^T and target,
    solve for best t6 such that R_o6·R(a6,t6) = R_prefix_inv · R_target."""
    R6_want = R_prefix_inv @ R_target  # should equal R_o6·R(a6,t6)
    # Solve R(a6,t6) = R_o6^T · R6_want
    R6_local = R_o6.T @ R6_want
    c = (np.trace(R6_local) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    s_vec = np.array([
        R6_local[2,1] - R6_local[1,2],
        R6_local[0,2] - R6_local[2,0],
        R6_local[1,0] - R6_local[0,1],
    ]) / 2.0
    s = float(np.dot(s_vec, a6))
    t6 = float(np.arctan2(s, c))
    # residual = how far R6_want is from a rotation about a6 (through R_o6)
    R6_pred = R_o6 @ rodrigues(a6, t6)
    return t6, float(np.linalg.norm(R6_pred - R6_want))


def _refine_origin(R_target, R_o4, a4, R_o5, a5, R_o6, a6,
                   t4, t5, t6, iters=30):
    params = np.array([t4, t5, t6], dtype=float)
    for _ in range(iters):
        R_pred = _R_wrist_pred(params[0], params[1], params[2],
                               R_o4, a4, R_o5, a5, R_o6, a6)
        res_vec = (R_pred - R_target).flatten()
        J = np.zeros((9, 3))
        eps = 1e-6
        for k in range(3):
            p2 = params.copy(); p2[k] += eps
            R2 = _R_wrist_pred(p2[0], p2[1], p2[2],
                               R_o4, a4, R_o5, a5, R_o6, a6)
            J[:, k] = (R2 - R_pred).flatten() / eps
        try:
            step, *_ = np.linalg.lstsq(J, -res_vec, rcond=None)
        except np.linalg.LinAlgError:
            break
        params = params + step
        if np.linalg.norm(step) < 1e-10:
            break
    R_pred = _R_wrist_pred(params[0], params[1], params[2],
                           R_o4, a4, R_o5, a5, R_o6, a6)
    return float(params[0]), float(params[1]), float(params[2]), \
           float(np.linalg.norm(R_pred - R_target))


def invert_wrist_product_origin(R_target,
                                R_o4, a4, R_o5, a5, R_o6, a6,
                                tol=1e-6, grid=180):
    """Invert R_target = R_o4·R(a4,t4)·R_o5·R(a5,t5)·R_o6·R(a6,t6)."""
    t4g = np.linspace(-np.pi, np.pi, grid, endpoint=False)
    t5g = np.linspace(0, np.pi, grid, endpoint=False)

    best = []
    for t5 in t5g:
        R45 = R_o5 @ rodrigues(a5, t5)
        for t4 in t4g:
            R44 = R_o4 @ rodrigues(a4, t4)
            R_prefix_inv = (R44 @ R45).T
            t6, _ = _t6_residual(R_prefix_inv, R_target, R_o6, a6)
            R_pred = _R_wrist_pred(t4, t5, t6, R_o4, a4, R_o5, a5, R_o6, a6)
            res = np.linalg.norm(R_pred - R_target)
            if res < 0.15:
                best.append((t4, t5, t6))

    refined = []
    for (t4, t5, t6) in best:
        t4r, t5r, t6r, res = _refine_origin(
            R_target, R_o4, a4, R_o5, a5, R_o6, a6, t4, t5, t6)
        if res < tol:
            refined.append((t4r, t5r, t6r, res))

    out = []
    for (t4, t5, t6, res) in refined:
        dup = False
        for (u4, u5, u6, _) in out:
            if (abs(wrap(t4 - u4)) < 1e-3 and abs(wrap(t5 - u5)) < 1e-3
                    and abs(wrap(t6 - u6)) < 1e-3):
                dup = True; break
            if (abs(wrap(t4 + np.pi - u4)) < 1e-3
                    and abs(wrap(-t5 - u5)) < 1e-3
                    and abs(wrap(t6 + np.pi - u6)) < 1e-3):
                dup = True; break
        if not dup:
            out.append((t4, t5, t6, res))
    return out


# ----------------------------------------------------------------------
# Main test
# ----------------------------------------------------------------------

def test_arm(name, urdf_path, n_trials=20, seed=0, grid=180):
    print(f"\n{'='*70}\n  {name}\n{'='*70}")
    rng = np.random.default_rng(seed)
    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)
    base_link = model.get_true_root()
    arm_chain = model.get_arm_chain(base_link_name=base_link)
    limits = geom.joint_limits

    # Origin rotations and local axes for the last 3 joints
    j4 = model.joints[arm_chain[3]]
    j5 = model.joints[arm_chain[4]]
    j6 = model.joints[arm_chain[5]]
    R_o4 = origin_rot(model, j4)
    R_o5 = origin_rot(model, j5)
    R_o6 = origin_rot(model, j6)
    a4 = axis_local(j4)
    a5 = axis_local(j5)
    a6 = axis_local(j6)

    print(f"  wrist_offset  = {geom.wrist_offset:.6f}")
    print(f"  flange_offset = {geom.flange_offset:.6f}")

    zyz_direct = zyz_flip = zyz_fail = 0
    prod_direct = prod_flip = prod_fail = 0
    solution_counts = []

    for trial in range(n_trials):
        q = np.array([rng.uniform(lo, hi) for (lo, hi) in limits])
        if abs(q[4]) < 0.05:
            q[4] = 0.05

        T_flange = model.get_flange_pose(q)
        R_flange = T_flange[:3, :3]

        # R03 with origins
        R03_fk = np.eye(3)
        for i in range(3):
            j = model.joints[arm_chain[i]]
            R03_fk = R03_fk @ origin_rot(model, j) @ rodrigues(axis_local(j), q[i])

        R_wrist_target = R03_fk.T @ R_flange

        # --- ZYZ (reference) ---
        matched = False
        for (t4, t5, t6) in extract_zyz(R_wrist_target):
            which, _, _ = angles_match_report(q[3:6], np.array([t4, t5, t6]))
            if which == "direct": zyz_direct += 1; matched = True; break
            if which == "flip":   zyz_flip   += 1; matched = True; break
        if not matched: zyz_fail += 1

        # --- Origin-included product inversion ---
        prod_cands = invert_wrist_product_origin(
            R_wrist_target, R_o4, a4, R_o5, a5, R_o6, a6, grid=grid)
        solution_counts.append(len(prod_cands))
        matched = False
        for (t4, t5, t6, res) in prod_cands:
            which, _, _ = angles_match_report(q[3:6], np.array([t4, t5, t6]))
            if which == "direct": prod_direct += 1; matched = True; break
            if which == "flip":   prod_flip   += 1; matched = True; break
        if not matched: prod_fail += 1

    print(f"\n  ZYZ on R_wrist_target (reference):")
    print(f"    direct={zyz_direct}/{n_trials}  flip={zyz_flip}/{n_trials}  "
          f"fail={zyz_fail}/{n_trials}")
    print(f"\n  Origin-included product inversion (grid={grid}):")
    print(f"    direct={prod_direct}/{n_trials}  flip={prod_flip}/{n_trials}  "
          f"fail={prod_fail}/{n_trials}")
    print(f"    solutions per q: min={min(solution_counts)} "
          f"max={max(solution_counts)} mean={np.mean(solution_counts):.2f}")


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
            test_arm(name, path, n_trials=20, seed=0, grid=180)
        except Exception as e:
            print(f"ERROR in {name}: {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()