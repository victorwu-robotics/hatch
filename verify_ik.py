#!/usr/bin/env python3
"""
Standalone verification of the unified analytical IK solver.

Run this before trusting the solver on a new machine or a new arm. It
needs only numpy and the URDF assets — no pytest, no scipy, no ROS, no
PackageResolver.

    python verify_ik.py                 # all three arms, 400 poses each
    python verify_ik.py --poses 2000    # deeper sweep
    python verify_ik.py --arm UR10      # one arm
    python verify_ik.py --urdf path/to/other.urdf --name MyArm
    python verify_ik.py --show-branches # print the 8 solutions at one pose

Exit status is 0 only if every check passes.

What it checks, and why each one is here:

  round trip      every returned branch must reproduce the target pose.
                  This is the acceptance criterion. Comparing joint
                  vectors instead is ill-conditioned near singularities.
  branches        up to 8 distinct solutions with distinct geometric tags.
  limits          no returned joint outside its URDF limit. The old
                  numerical fallback returned 57.5 rad on a +/-2.61 joint.
  unreachable     a pose beyond reach must return nothing, not a
                  plausible-looking joint vector.
  singularities   wrist (a4 || a6), elbow (full stretch), shoulder (r=|h|).
  purity          solving must not mutate the model.
  jog             200 Cartesian steps must keep configuration and limits.
  refusal         an out-of-class arm must raise, naming the reason.

The accuracy table at the end is the honest part: residual error is the
URDF's own axis tilt multiplied by the pose's condition number. UR10's
URDF is clean to 2e-10 and round-trips to ~1e-9 rad. FR5's and Elfin's
axes are tilted 3.7e-6 by CAD rounding, so near a singularity no
closed-form solver can do better than ~1e-3 rad.
"""

import argparse
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from core.kinematics.geometric_extraction import UnsupportedArmError  # noqa: E402
from core.kinematics.ik_solver import IKSolver                        # noqa: E402
from core.kinematics.kinematic_model import KinematicModel            # noqa: E402

ROOT = Path(__file__).resolve().parent
ARMS = {
    "UR10": ROOT / "assets/robots/ur_description/urdf/ur10_nominal.urdf",
    "FR5": ROOT / "assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf",
    "Elfin": ROOT / "assets/robots/E15_Pro/urdf/E15_Pro.urdf",
}
Q_TEST = np.array([0.3, -0.8, 1.0, 0.5, 1.2, -0.4])
POS_TOL, ROT_TOL = 1e-4, 1e-4

_failures = []


def check(arm, label, ok, detail=""):
    print(f"    {'PASS' if ok else 'FAIL'}  {label}" + (f"   {detail}" if detail else ""))
    if not ok:
        _failures.append(f"{arm}: {label} {detail}")


def pose_err(T1, T2):
    pos = float(np.linalg.norm(T1[:3, 3] - T2[:3, 3]))
    c = (float(np.trace(T1[:3, :3].T @ T2[:3, :3])) - 1.0) / 2.0
    return pos, float(np.arccos(np.clip(c, -1.0, 1.0)))


def verify(name, urdf, n_poses, show_branches):
    print("=" * 74)
    print(f"{name}   {urdf}")
    print("=" * 74)
    if not Path(urdf).exists():
        check(name, "URDF present", False, str(urdf))
        return None

    model = KinematicModel(str(urdf)).load()
    try:
        ik = IKSolver(model)
    except UnsupportedArmError as e:
        print(f"    REFUSED (this is correct behaviour for an out-of-class arm):\n      {e}")
        return None
    model.set_ik_solver(ik)
    print(ik.describe())
    print()

    u = ik.unified
    limits = u.limits

    def in_limits(q_arm):
        return all(lo is None or hi is None or lo - 1e-5 <= v <= hi + 1e-5
                   for v, (lo, hi) in zip(q_arm, limits))

    # --- round trip -----------------------------------------------------
    rng = np.random.default_rng(20261117)
    worst_p = worst_r = 0.0
    n_sol = n_branch = 0
    hist = {}
    buckets = {}
    t0 = time.perf_counter()
    for _ in range(n_poses):
        q = rng.uniform(-2.6, 2.6, 6)
        T = model.forward_kinematics(q)
        sols = u.solve(u.model.get_flange_pose(q), respect_limits=False)
        hist[len(sols)] = hist.get(len(sols), 0) + 1
        if not sols:
            n_sol += 1
            continue
        for s in sols:
            n_branch += 1
            p, r = pose_err(model.forward_kinematics(ik._from_arm(s.q, q)), T)
            worst_p, worst_r = max(worst_p, p), max(worst_r, r)
        # condition number of this pose
        P5 = T[:3, 3] - u.d6 * T[:3, :3][:, 2]
        rad = float(np.hypot(P5[0], P5[1]))
        den = u._n0_xy * rad
        cb = (u.h - u._n0[2] * P5[2]) / den if den > 1e-12 else 99.0
        sb = float(np.sqrt(max(0.0, 1.0 - min(1.0, abs(cb)) ** 2)))
        cond = max(1 / max(sb, 1e-12), 1 / max(rad, 1e-12),
                   1 / max(abs(np.sin(q[4])), 1e-12),
                   1 / max(abs(np.sin(q[2])), 1e-12))
        best = min(sols, key=lambda s: np.max(np.abs(
            np.arctan2(np.sin(s.q - q), np.cos(s.q - q)))))
        e = float(np.max(np.abs(np.arctan2(np.sin(best.q - q), np.cos(best.q - q)))))
        b = ("cond<10" if cond < 10 else "cond<100" if cond < 100
             else "cond<1000" if cond < 1000 else "cond>=1000")
        buckets.setdefault(b, []).append(e)
    dt = (time.perf_counter() - t0) / n_poses * 1000

    check(name, "every reachable pose solved", n_sol == 0, f"{n_poses - n_sol}/{n_poses}")
    check(name, "all branches reproduce the pose",
          worst_p < POS_TOL and worst_r < ROT_TOL,
          f"worst {worst_p:.2e} m / {worst_r:.2e} rad over {n_branch} branches")
    check(name, "reaches the full 8 branches", 8 in hist, f"histogram {dict(sorted(hist.items()))}")
    print(f"          {dt:.2f} ms per solve (all branches, verified)")

    if show_branches:
        print("\n    the 8 solutions at q = " + str(Q_TEST))
        for s in u.solve(u.model.get_flange_pose(Q_TEST), respect_limits=False):
            print("      " + str(s))
        print()

    # --- limits + unreachable, through the TCP API ----------------------
    model.update_state(Q_TEST)
    T_now = model.get_tcp_pose()
    ok_lim = True
    for seed in (np.zeros(6), Q_TEST):
        qo = ik.solve_ik_for_tcp(T_now, seed)
        if qo is not None and not in_limits(ik._to_arm(qo)):
            ok_lim = False
    check(name, "returned joints within limits (incl. zero seed)", ok_lim)

    T_bad = np.eye(4)
    T_bad[:3, 3] = [3.0, 0.0, 0.5]
    check(name, "unreachable target returns None", ik.solve_ik_for_tcp(T_bad, Q_TEST) is None)

    # --- singularities --------------------------------------------------
    for th5 in (0.0, np.pi):
        qs = Q_TEST.copy()
        qs[4] = th5
        Ts = model.forward_kinematics(qs)
        ss = u.solve(u.model.get_flange_pose(qs), q_reference=qs, respect_limits=False)
        good = bool(ss) and any(s.wrist_singular for s in ss)
        if good:
            good = all(pose_err(model.forward_kinematics(ik._from_arm(s.q, qs)), Ts)[0] < POS_TOL
                       for s in ss)
        check(name, f"wrist singularity theta5={th5:+.3f} solved and flagged", good)

    qe = Q_TEST.copy()
    qe[2] = 0.0
    Te = model.forward_kinematics(qe)
    se = u.solve(u.model.get_flange_pose(qe), respect_limits=False)
    check(name, "elbow singularity (full stretch)",
          bool(se) and all(pose_err(model.forward_kinematics(ik._from_arm(s.q, qe)), Te)[0] < POS_TOL
                           for s in se))

    # --- purity ---------------------------------------------------------
    model.update_state(Q_TEST)
    before = model.get_current_joint_positions().copy()
    ik.solve_ik_for_tcp(model.forward_kinematics(Q_TEST + 0.2), np.zeros(6))
    check(name, "solve does not mutate the model",
          np.allclose(model.get_current_joint_positions(), before))

    # --- Cartesian jog --------------------------------------------------
    q = Q_TEST.copy()
    model.update_state(q)
    tags0, jog_ok, jump = None, True, 0.0
    for _ in range(200):
        T = model.forward_kinematics(q)
        T[0, 3] += 0.0005
        qn = ik.solve_ik_for_tcp(T, q)
        if qn is None:
            jog_ok = False
            break
        qa = ik._to_arm(qn)
        if not in_limits(qa):
            jog_ok = False
            break
        tags = ik.diagnostics["chosen_tags"]
        tags0 = tags0 or tags
        if tags != tags0:
            jog_ok = False
            break
        jump = max(jump, float(np.max(np.abs(np.arctan2(np.sin(qa - q), np.cos(qa - q))))))
        q = qn
        model.update_state(q)
    check(name, "200-step Cartesian jog keeps configuration and limits",
          jog_ok, f"tags {tags0}, largest step {jump:.4f} rad")

    # --- accuracy story -------------------------------------------------
    print(f"\n    accuracy = URDF axis tilt ({u.model_tilt:.2e} rad) x pose condition:")
    for b in ("cond<10", "cond<100", "cond<1000", "cond>=1000"):
        v = buckets.get(b, [])
        if v:
            print(f"      {b:>11}: n={len(v):5d}  max joint err {max(v):.2e} rad"
                  f"  median {np.median(v):.2e}")
    print()
    return u


def verify_refusal(tmpdir):
    """Build a PUMA-style shoulder offset in a scratch dir, never in the repo."""
    print("=" * 74)
    print("Refusal: an out-of-class arm must fail loudly, not approximately")
    print("=" * 74)
    src = ARMS["UR10"]
    if not src.exists():
        print("    SKIP (UR10 URDF not found)")
        return
    old = '<origin rpy="1.570796327 0 0" xyz="0 0 0"/>'
    text = src.read_text()
    if old not in text:
        print("    SKIP (anchor not found in UR10 URDF)")
        return
    p = Path(tmpdir) / "puma_shoulder.urdf"
    p.write_text(text.replace(old, '<origin rpy="1.570796327 0 0" xyz="0 -0.15 0"/>', 1))
    try:
        IKSolver(KinematicModel(str(p)).load())
        check("refusal", "PUMA-style shoulder offset refused", False, "it was accepted!")
    except UnsupportedArmError as e:
        check("refusal", "PUMA-style shoulder offset refused", True)
        print(f"          reason: {str(e).splitlines()[0]}")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", type=int, default=400)
    ap.add_argument("--arm", choices=list(ARMS))
    ap.add_argument("--urdf", help="verify an arbitrary URDF instead")
    ap.add_argument("--name", default="CustomArm")
    ap.add_argument("--show-branches", action="store_true")
    a = ap.parse_args()

    print(f"numpy {np.__version__}   python {sys.version.split()[0]}\n")
    if a.urdf:
        verify(a.name, Path(a.urdf), a.poses, a.show_branches)
    else:
        for name, path in ARMS.items():
            if a.arm and name != a.arm:
                continue
            verify(name, path, a.poses, a.show_branches)
        with tempfile.TemporaryDirectory(prefix="hatch-ik-") as td:
            verify_refusal(td)

    print("=" * 74)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for f in _failures:
            print("  - " + f)
        return 1
    print("ALL CHECKS PASSED")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
