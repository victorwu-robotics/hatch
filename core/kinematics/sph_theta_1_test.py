#!/usr/bin/env python3
"""
Diagnostic: check the spherical-wrist theta-1 branch on the Elfin.

The derivation says there are two shoulder solutions for a spherical wrist:
    theta_1 = phi          (pointing toward the target)
    theta_1 = phi + pi     (pointing away)

The Elfin solver works without adding the second one. This script checks
whether the second solution is:
    (A) always outside J1 limits (excluded by the arm), or
    (B) sometimes inside J1 limits and ignored by the solver (a latent bug), or
    (C) sometimes inside J1 limits and selected by proximity (a heuristic).

Not a permanent test. Run once, read the output, then delete.
"""

import numpy as np
from math import atan2, pi

import logging
logging.basicConfig(level=logging.WARNING)

from pathlib import Path
import sys
sys.path.insert(0, str(Path.home() / "hatch"))

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry
from core.kinematics.analytical_ik_solver import AnalyticalIKSolver


# ----------------------------------------------------------------------
# 1. Load the Elfin.
# ----------------------------------------------------------------------
URDF = Path.home() / "hatch/assets/robots/E15_Pro/urdf/E15_Pro.urdf"
PACKAGE_DIRS = [
    str(URDF.parent),
    str(URDF.parent.parent),
    str(Path.home() / "hatch" / "assets"),
    str(Path.home() / "hatch" / "assets" / "robots"),
]

model = KinematicModel(urdf_path=str(URDF), package_dirs=PACKAGE_DIRS)
model.load()

# Zero the tool so the model returns the flange, not the TCP.
model.set_tool_transform(np.eye(4))

# Extract geometry in the true root frame.
geom = extract_arm_geometry(model)
print("Extraction:")
print(geom.describe())
print()

# J1 limits.
j1_lower, j1_upper = geom.joint_limits[0]
print(f"J1 limits: [{j1_lower:.4f}, {j1_upper:.4f}] rad "
      f"({j1_lower/pi:.3f}pi, {j1_upper/pi:.3f}pi)")
print()

# ----------------------------------------------------------------------
# 2. Read the solver parameters from the geometry.
# ----------------------------------------------------------------------
# d1  = shoulder height: base (J1 point) to J2 along the vertical.
# a2  = upper arm length: J2 to J3 (these axes are parallel, so the
#       perpendicular distance between them IS the link length).
# a3  = forearm length: J3 to the WRIST CENTER (NOT axis_gap(2), because
#       J3 and J4 may intersect, making axis_gap(2) = 0).
# d6  = tool offset from wrist center to TCP.
#
# In the true root frame at zero configuration:
#   J1 point is at z = 0.262 (the shoulder height reference).
#   J2 point is at z = 0.262.
#   J3 point is at z = 0.992.
#   wrist center is at z = 1.562.
#
# d1 is the distance from the base plane to J2. For this robot, J1's
# point is the base plane, and J2's point is the shoulder, so:
#   d1 = J2.point_z - J1.point_z  = 0.262 - 0.262 = 0.0
#
# Wait — that gives 0. Let us reconsider. The solver expects d1 as the
# "shoulder height offset (base to J2 along Z, at zero)". Looking at
# the solver's math, it uses z_rel = z - d1 to bring the target into a
# frame where J2 is at the origin. So d1 should be J2's z-coordinate in
# the true root frame, which is 0.262.
#
# d1 = J2's z-coordinate.

d1 = float(geom.axes[1].point[2])                         # 0.262
a2 = float(geom.axis_gap(1))                              # 0.730
a3 = float(np.linalg.norm(geom.wrist_center
                          - geom.axes[2].point))          # 0.570
d6 = float(geom.tool_offset)                              # 0.200

print(f"Solver params: d1={d1:.6f} a2={a2:.6f} a3={a3:.6f} d6={d6:.6f}")
print(f"  (a3 was axis_gap(2)={geom.axis_gap(2):.6f}; corrected to "
      f"distance J3->wrist_center)")
print()

solver = AnalyticalIKSolver(
    kinematic_model=model,
    d1=d1,
    a2=a2,
    a3=a3,
    d6=d6,
)


# ----------------------------------------------------------------------
# 3. Pick test configurations, run FK, and compare theta_1 candidates.
# ----------------------------------------------------------------------
test_configs = [
    np.zeros(6),
    np.array([0.0, -0.5, 1.0, 0.0, 0.8, 0.0]),
    np.array([0.5, -0.3, 0.7, 0.2, 0.9, -0.4]),
    np.array([-0.7, -0.6, 1.2, 0.3, 1.1, 0.5]),
    np.array([1.2, -0.4, 0.6, -0.3, 0.7, 0.9]),
    np.array([-1.5, -0.2, 0.5, 0.4, 0.6, -0.8]),
]


def wrap(a):
    return (a + pi) % (2 * pi) - pi


def in_limits(a, lo, hi):
    """Check whether angle a (mod 2pi) lies within [lo, hi]."""
    for k in (-1, 0, 1):
        cand = a + k * 2 * pi
        if lo <= cand <= hi:
            return True, cand
    return False, None


# Precompute P5 in the flange frame, so we can locate the wrist center
# for any pose.
T_flange_zero = model.forward_kinematics(np.zeros(6))
P5_in_flange = np.linalg.inv(T_flange_zero) @ np.append(geom.wrist_center, 1.0)
P5_in_flange = P5_in_flange[:3]

print("=" * 78)
print("Per-pose theta-1 analysis")
print("=" * 78)

for i, q_true in enumerate(test_configs):
    T_flange = model.forward_kinematics(q_true)
    P5 = (T_flange @ np.append(P5_in_flange, 1.0))[:3]

    phi = atan2(P5[1], P5[0])
    cand_a = wrap(phi)
    cand_b = wrap(phi + pi)

    ok_a, _ = in_limits(cand_a, j1_lower, j1_upper)
    ok_b, _ = in_limits(cand_b, j1_lower, j1_upper)

    solver_solutions = solver.solve_ik(T_flange)

    print(f"\nPose {i}:  q_true = {np.round(q_true, 4).tolist()}")
    print(f"  P5 (true root)  = {P5.round(6).tolist()}")
    print(f"  phi             = {phi:+.6f}  ({phi/pi:+.4f}pi)")
    print(f"  candidate A     = {cand_a:+.6f}  ({cand_a/pi:+.4f}pi)  "
          f"{'in limits' if ok_a else 'OUT OF LIMITS'}")
    print(f"  candidate B     = {cand_b:+.6f}  ({cand_b/pi:+.4f}pi)  "
          f"{'in limits' if ok_b else 'OUT OF LIMITS'}")
    print(f"  solver returned {len(solver_solutions)} solution(s)")

    for j, q in enumerate(solver_solutions):
        q1 = wrap(q[0])
        d_a = abs(wrap(q1 - cand_a))
        d_b = abs(wrap(q1 - cand_b))
        which = "A" if d_a < d_b else "B"
        print(f"    sol {j}: q1 = {q1:+.6f} ({q1/pi:+.4f}pi)  "
              f"closest to candidate {which}  "
              f"(d_a={d_a:.4f}, d_b={d_b:.4f})")

print()
print("=" * 78)
print("Reading:")
print("  - If candidate B is always OUT OF LIMITS, outcome (A).")
print("  - If candidate B is sometimes in limits but the solver always")
print("    returns candidate A, outcome (B): latent gap.")
print("  - If candidate B is sometimes in limits and the solver sometimes")
print("    returns B, outcome (C): proximity-based selection.")
print("=" * 78)