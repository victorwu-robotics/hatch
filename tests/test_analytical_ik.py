#!/usr/bin/env python3
"""
Test script for the Analytical IK Solver.

Loads the E15-Pro URDF, creates the solver, and tests:
1. FK at zero configuration
2. IK for the zero pose (should return something close to [0,0,0,0,0,0])
3. IK for a displaced pose (move TCP in Z, keep orientation)
4. IK for an orientation change (keep position, change orientation)
5. Verify that orientation change doesn't move the TCP position

Run from the hatch directory:
    python tests/test_analytical_ik.py
"""

import numpy as np
from math import pi
from pathlib import Path
import sys

sys.path.insert(0, str(Path.home() / "hatch"))

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.analytical_ik_solver import AnalyticalIKSolver


def print_pose(label, T):
    """Print a 4x4 pose in readable form."""
    pos = T[:3, 3]
    print(f"  {label}: position=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")


def main():
    # Load the model
    urdf_path = Path.home() / "hatch/assets/robots/E15_Pro/urdf/E15_Pro.urdf"
    package_dirs = [
        str(urdf_path.parent),
        str(urdf_path.parent.parent),
        str(Path.home() / "hatch" / "assets"),
        str(Path.home() / "hatch" / "assets" / "robots"),
    ]

    print("Loading URDF...")
    model = KinematicModel(
        urdf_path=str(urdf_path),
        package_dirs=package_dirs,
        asset_id="E15_Pro"
    )
    model.load()
    print(f"  Tool mount link: {model.tool_mount_link}")
    print(f"  True root: {model.get_true_root()}")

    # Create solver
    print("\nCreating analytical IK solver...")
    solver = AnalyticalIKSolver(model)

    # Start from a non-singular pose (avoid all-zeros)
    np.random.seed(42)
    q_start = np.array([0.3, -0.5, 0.8, 0.2, 0.5, -0.3])
    model.update_state(q_start)
    T_start = model.get_tcp_pose()
    print(f"\nStarting from non-singular pose:")
    print(f"  Joint angles: {np.array2string(q_start, precision=4)}")
    print_pose("  TCP pose", T_start)

    # Test 1: FK at start pose
    print("\n=== Test 1: FK at start pose ===")
    T_fk = solver.forward_kinematics(q_start)
    pos_err = np.linalg.norm(T_fk[:3, 3] - T_start[:3, 3])
    print(f"  Position error vs model: {pos_err:.6f}m")
    if pos_err < 1e-3:
        print("  ✅ PASS")
    else:
        print("  ❌ FAIL")

    # Test 2: IK returns to start pose
    print("\n=== Test 2: IK for current pose ===")
    q_solution = solver.solve_ik_for_tcp(T_start, q_guess=q_start)
    if q_solution is not None:
        T_check = solver.forward_kinematics(q_solution)
        pos_err = np.linalg.norm(T_check[:3, 3] - T_start[:3, 3])
        print(f"  Position error: {pos_err:.6f}m")
        if pos_err < 1e-3:
            print("  ✅ PASS")
        else:
            print("  ❌ FAIL")
    else:
        print("  ❌ FAIL - no solution found")

    # Test 3: Move TCP in Z only
    print("\n=== Test 3: Move TCP down by 0.1m, keep orientation ===")
    T_target = T_start.copy()
    T_target[2, 3] -= 0.1
    q_solution = solver.solve_ik_for_tcp(T_target, q_guess=q_start)
    if q_solution is not None:
        T_check = solver.forward_kinematics(q_solution)
        pos_err = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
        print(f"  Position error: {pos_err:.6f}m")
        if pos_err < 1e-3:
            print("  ✅ PASS")
        else:
            print("  ❌ FAIL")
    else:
        print("  ❌ FAIL - no solution found")

    # Test 4: Change orientation, keep position
    print("\n=== Test 4: Rotate 30° about Z, keep position ===")
    from scipy.spatial.transform import Rotation as R
    T_rotated = T_start.copy()
    R_z30 = R.from_rotvec([0, 0, pi/6]).as_matrix()
    T_rotated[:3, :3] = R_z30 @ T_rotated[:3, :3]
    print_pose("  Target", T_rotated)

    q_solution = solver.solve_ik_for_tcp(T_rotated, q_guess=q_start)
    if q_solution is not None:
        T_check = solver.forward_kinematics(q_solution)
        pos_err = np.linalg.norm(T_check[:3, 3] - T_rotated[:3, 3])
        print(f"  Position error: {pos_err:.6f}m")
        print(f"  TCP position: ({T_check[0,3]:.4f}, {T_check[1,3]:.4f}, {T_check[2,3]:.4f})")
        print(f"  Original:     ({T_start[0,3]:.4f}, {T_start[1,3]:.4f}, {T_start[2,3]:.4f})")
        if pos_err < 1e-3:
            print("  ✅ PASS")
        else:
            print("  ❌ FAIL - position drifted")
    else:
        print("  ❌ FAIL - no solution found")

    # Test 5: Multiple random poses
    print("\n=== Test 5: Random poses ===")
    passed = 0
    for i in range(20):
        T_rand = T_start.copy()
        T_rand[:3, 3] += np.random.uniform(-0.15, 0.15, 3)
        rand_rot = R.from_rotvec(np.random.uniform(-1.0, 1.0, 3)).as_matrix()
        T_rand[:3, :3] = rand_rot @ T_rand[:3, :3]

        q_solution = solver.solve_ik_for_tcp(T_rand, q_guess=q_start)
        if q_solution is not None:
            T_check = solver.forward_kinematics(q_solution)
            pos_err = np.linalg.norm(T_check[:3, 3] - T_rand[:3, 3])
            if pos_err < 1e-3:
                passed += 1
            else:
                print(f"  Pose {i+1}: position error {pos_err:.4f}m")
        else:
            print(f"  Pose {i+1}: no solution")
        q_start = q_solution if q_solution is not None else q_start

    print(f"  Passed {passed}/20 random poses")
    if passed == 20:
        print("  ✅ PASS")
    else:
        print(f"  ❌ FAIL - {20-passed} poses failed")

if __name__ == "__main__":
    main()