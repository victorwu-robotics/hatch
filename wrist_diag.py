"""
Standalone wrist diagnostic.

For each arm (UR10, FR5, Elfin), one random q within limits:
  1. Compute T_flange, R_wrist = R03^T · R_flange.
  2. Print ||R(a4,q3)·R(a5,q4)·R(a6,q5) − R_wrist||
     (should be ~1e-15 if the composition is correct).
  3. Print ||flip(q3,q4,q5) − R_wrist||
     (for Elfin should be ~1e-15; for UR/FR if NOT ~1e-15, flip
      is not a symmetry and the dedup in invert_wrist_product is
      collapsing the true solution with a spurious one).
  4. Run the product inversion. Print each found (t4,t5,t6) and its
     diff to true q[3:6] under direct and flip relations.
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
    axis = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    return np.array([
        [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)],
    ])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _t6_and_residual(R6, a6):
    c = (np.trace(R6) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    s_vec = np.array([R6[2,1] - R6[1,2], R6[0,2] - R6[2,0],
                      R6[1,0] - R6[0,1]]) / 2.0
    s = float(np.dot(s_vec, a6))
    t6 = float(np.arctan2(s, c))
    R6_pred = rodrigues(a6, t6)
    return t6, float(np.linalg.norm(R6_pred - R6))


def _refine(R_target, a4, a5, a6, t4, t5, t6, iters=25):
    params = np.array([t4, t5, t6], dtype=float)
    for _ in range(iters):
        R_pred = (rodrigues(a4, params[0]) @ rodrigues(a5, params[1])
                  @ rodrigues(a6, params[2]))
        res_vec = (R_pred - R_target).flatten()
        J = np.zeros((9, 3))
        eps = 1e-6
        for k in range(3):
            p2 = params.copy(); p2[k] += eps
            R2 = (rodrigues(a4, p2[0]) @ rodrigues(a5, p2[1])
                  @ rodrigues(a6, p2[2]))
            J[:, k] = (R2 - R_pred).flatten() / eps
        try:
            step, *_ = np.linalg.lstsq(J, -res_vec, rcond=None)
        except np.linalg.LinAlgError:
            break
        params = params + step
        if np.linalg.norm(step) < 1e-10:
            break
    R_pred = (rodrigues(a4, params[0]) @ rodrigues(a5, params[1])
              @ rodrigues(a6, params[2]))
    return float(params[0]), float(params[1]), float(params[2]), \
           float(np.linalg.norm(R_pred - R_target))


def invert_wrist_product_no_flip_dedup(R_target, a4, a5, a6,
                                       tol=1e-6, grid=180):
    """Same as invert_wrist_product, but dedup ONLY direct duplicates.

    The flip-dedup is intentionally removed so we can see whether the
    flip branch is a true symmetry or not.
    """
    t4g = np.linspace(-np.pi, np.pi, grid, endpoint=False)
    t5g = np.linspace(0, np.pi, grid, endpoint=False)

    best = []
    for t5 in t5g:
        R5 = rodrigues(a5, t5)
        for t4 in t4g:
            R4 = rodrigues(a4, t4)
            R6_approx = (R4 @ R5).T @ R_target
            t6, _ = _t6_and_residual(R6_approx, a6)
            R_pred = R4 @ R5 @ rodrigues(a6, t6)
            res = np.linalg.norm(R_pred - R_target)
            if res < 0.15:
                best.append((t4, t5, t6))

    refined = []
    for (t4, t5, t6) in best:
        t4r, t5r, t6r, res = _refine(R_target, a4, a5, a6, t4, t5, t6)
        if res < tol:
            refined.append((t4r, t5r, t6r, res))

    out = []
    for (t4, t5, t6, res) in refined:
        dup = False
        for (u4, u5, u6, _) in out:
            if (abs(wrap(t4 - u4)) < 1e-3 and abs(wrap(t5 - u5)) < 1e-3
                    and abs(wrap(t6 - u6)) < 1e-3):
                dup = True; break
        if not dup:
            out.append((t4, t5, t6, res))
    return out


# ----------------------------------------------------------------------
# One-trial diagnostic
# ----------------------------------------------------------------------

def diagnostic_one_trial(name, urdf_path, seed=0):
    print(f"\n{'='*70}\n  {name}  (seed={seed})\n{'='*70}")
    rng = np.random.default_rng(seed)
    model = KinematicModel(urdf_path)
    geom = extract_arm_geometry(model)

    a1, a2, a3, a4, a5, a6 = [ax.direction for ax in geom.axes]
    limits = geom.joint_limits

    q = np.array([rng.uniform(lo, hi) for (lo, hi) in limits])
    if abs(q[4]) < 0.05:
        q[4] = 0.05

    T_flange = model.get_flange_pose(q)
    R_flange = T_flange[:3, :3]
    R03 = (rodrigues(a1, q[0]) @ rodrigues(a2, q[1])
           @ rodrigues(a3, q[2]))
    R_wrist = R03.T @ R_flange

    print(f"  true q       = {np.array2string(q, precision=4)}")
    print(f"  true q[3:6]  = {np.array2string(q[3:6], precision=4)}")

    # --- Composition check (direct) ---
    R_true_direct = (rodrigues(a4, q[3]) @ rodrigues(a5, q[4])
                     @ rodrigues(a6, q[5]))
    err_direct = float(np.linalg.norm(R_true_direct - R_wrist))
    print(f"\n  ||R(a4,q3)·R(a5,q4)·R(a6,q5) − R_wrist|| = {err_direct:.3e}")

    # --- Composition check (flip) ---
    R_true_flip = (rodrigues(a4, q[3] + np.pi)
                   @ rodrigues(a5, -q[4])
                   @ rodrigues(a6, q[5] + np.pi))
    err_flip = float(np.linalg.norm(R_true_flip - R_wrist))
    print(f"  ||R(a4,q3+π)·R(a5,−q4)·R(a6,q5+π) − R_wrist|| = {err_flip:.3e}")

    # --- Check a4, a5, a6 orthogonality / parallelism ---
    print(f"\n  a4·a5 = {np.dot(a4, a5):+.4f}   (expect 0)")
    print(f"  a5·a6 = {np.dot(a5, a6):+.4f}   (expect 0)")
    print(f"  a4·a6 = {np.dot(a4, a6):+.4f}   "
          f"(expect ±1 for spherical, else intermediate)")

    # --- Inversion without flip-dedup ---
    print(f"\n  Inversion (NO flip-dedup):")
    sols = invert_wrist_product_no_flip_dedup(R_wrist, a4, a5, a6, grid=180)
    print(f"  found {len(sols)} solution(s):")
    for (t4, t5, t6, res) in sols:
        print(f"    ({t4:+.6f}, {t5:+.6f}, {t6:+.6f})  res={res:.2e}")
        diff_direct = np.array([wrap(t4 - q[3]),
                                wrap(t5 - q[4]),
                                wrap(t6 - q[5])])
        diff_flip = np.array([wrap(t4 - (q[3] + np.pi)),
                              wrap(t5 - (-q[4])),
                              wrap(t6 - (q[5] + np.pi))])
        print(f"      diff to true (direct) = {np.array2string(diff_direct, precision=4)}")
        print(f"      diff to true (flip)   = {np.array2string(diff_flip, precision=4)}")


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
            diagnostic_one_trial(name, path, seed=0)
        except Exception as e:
            print(f"ERROR in {name}: {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()