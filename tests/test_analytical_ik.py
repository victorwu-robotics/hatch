#!/usr/bin/env python3
"""
Test script for the Analytical IK Solver with official D-H parameters.

Tests:
1. FK at zero configuration matches model FK
2. IK for the current pose returns a valid solution
3. IK for a displaced pose (position only)
4. IK for an orientation change (position stays fixed)
5. Random poses within workspace
"""

import numpy as np
from math import pi
from pathlib import Path
import sys

sys.path.insert(0, str(Path.home() / "hatch"))

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.analytical_ik_solver import AnalyticalIKSolver


def print_pose(label, T):
    pos = T[:3, 3]
    print(f"  {label}: position=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")


def main():
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

    print("\nCreating analytical IK solver...")
    solver = AnalyticalIKSolver(model)

    solver.test_wrist_correct()
    solver.test_wrist_center_from_known_pose()

    # Test 1: FK at zero

    print("\n=== Test 1: FK at zero configuration ===")
    q_zero = np.zeros(6)
    T_fk = solver.forward_kinematics(q_zero)
    model.update_state(q_zero)
    T_model = model.get_tcp_pose()
    pos_err = np.linalg.norm(T_fk[:3, 3] - T_model[:3, 3])
    print(f"  FK pos: ({T_fk[0,3]:.4f}, {T_fk[1,3]:.4f}, {T_fk[2,3]:.4f})")
    print(f"  Model:  ({T_model[0,3]:.4f}, {T_model[1,3]:.4f}, {T_model[2,3]:.4f})")
    print(f"  Error: {pos_err:.6f}m")
    assert pos_err < 1e-3, "FK at zero doesn't match model!"
    print("  ✅ PASS")

    # Test 2: Start from a non-singular pose
    print("\n=== Test 2: IK from a bent pose ===")
    np.random.seed(42)
    q_start = np.array([0.3, -0.5, 0.8, 0.2, 0.5, -0.3])
    model.update_state(q_start)
    T_start = model.get_tcp_pose()
    print(f"  Start pose: ({T_start[0,3]:.4f}, {T_start[1,3]:.4f}, {T_start[2,3]:.4f})")

    q_sol = solver.solve_ik_for_tcp(T_start, q_guess=q_start)
    assert q_sol is not None, "No IK solution for current pose!"
    model.update_state(q_sol)
    T_check = model.get_tcp_pose()
    pos_err = np.linalg.norm(T_check[:3, 3] - T_start[:3, 3])
    print(f"  Solution found, position error: {pos_err:.6f}m")
    assert pos_err < 1e-3, "IK solution doesn't match target position!"
    print("  ✅ PASS")


    # Test 3: Position change only
    print("\n=== Test 3: Move TCP by (0.1, 0.1, -0.1) ===")
    T_target = T_start.copy()
    T_target[:3, 3] += [0.1, 0.1, -0.1]
    q_sol = solver.solve_ik_for_tcp(T_target, q_guess=q_start)
    assert q_sol is not None, "No IK solution for displaced pose!"
    model.update_state(q_sol)
    T_check = model.get_tcp_pose()
    pos_err = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
    print(f"  Position error: {pos_err:.6f}m")
    assert pos_err < 1e-3, "Position error too large!"
    print("  ✅ PASS")

    # Test 4: Orientation change only (position must stay fixed)
    print("\n=== Test 4: Rotate 30° about Z, keep position ===")
    from scipy.spatial.transform import Rotation as R
    T_rotated = T_start.copy()
    R_z30 = R.from_rotvec([0, 0, pi/6]).as_matrix()
    T_rotated[:3, :3] = R_z30 @ T_rotated[:3, :3]
    q_sol = solver.solve_ik_for_tcp(T_rotated, q_guess=q_start)
    assert q_sol is not None, "No IK solution for rotated pose!"
    model.update_state(q_sol)
    T_check = model.get_tcp_pose()
    pos_err = np.linalg.norm(T_check[:3, 3] - T_rotated[:3, 3])
    print(f"  Position error: {pos_err:.6f}m")
    print(f"  TCP position: ({T_check[0,3]:.4f}, {T_check[1,3]:.4f}, {T_check[2,3]:.4f})")
    print(f"  Target:       ({T_rotated[0,3]:.4f}, {T_rotated[1,3]:.4f}, {T_rotated[2,3]:.4f})")
    assert pos_err < 1e-3, "Position drifted during rotation!"
    print("  ✅ PASS")

    # Test 5: Random poses
    print("\n=== Test 5: 20 random poses ===")
    passed = 0
    current_q = q_start.copy()
    for i in range(20):
        T_rand = T_start.copy()
        T_rand[:3, 3] += np.random.uniform(-0.15, 0.15, 3)
        rand_rot = R.from_rotvec(np.random.uniform(-1.0, 1.0, 3)).as_matrix()
        T_rand[:3, :3] = rand_rot @ T_rand[:3, :3]

        q_sol = solver.solve_ik_for_tcp(T_rand, q_guess=current_q)
        if q_sol is not None:
            model.update_state(q_sol)
            T_check = model.get_tcp_pose()
            pos_err = np.linalg.norm(T_check[:3, 3] - T_rand[:3, 3])
            if pos_err < 1e-3:
                passed += 1
                current_q = q_sol
            else:
                print(f"  Pose {i+1}: position error {pos_err:.4f}m")
        else:
            print(f"  Pose {i+1}: no solution")

    print(f"  Passed {passed}/20")
    assert passed == 20, f"Only {passed}/20 random poses passed!"
    print("  ✅ PASS")

    print("\n🎉 All tests passed!")


if __name__ == "__main__":
    main()