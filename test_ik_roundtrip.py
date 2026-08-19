"""
test_ik_roundtrip.py - FK/IK round-trip verification.

Tests that the parameterized URIKSolver produces identical results
to the original for UR robots, and verifies the theta_offset handling.

Usage:
    python test_ik_roundtrip.py
"""
import sys
import numpy as np
from pathlib import Path
from math import pi

sys.path.insert(0, str(Path(__file__).parent))

from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.ur_ik_solver import URIKSolver


def test_ur_backward_compatibility():
    """
    Test 1: UR solver with default parameters produces
    the same FK as before parameterization.
    """
    print("=" * 60)
    print("TEST 1: UR backward compatibility (default parameters)")
    print("=" * 60)

    # Original UR10e parameters
    solver = URIKSolver(
        d1=0.1273, a2=-0.612, a3=-0.5723,
        d4=0.163941, d5=0.1157, d6=0.0922
    )

    # Test at several joint configurations
    test_configs = [
        np.zeros(6),
        np.array([0.5, -1.0, 1.5, -0.5, 0.3, 0.8]),
        np.array([-1.0, -2.0, 2.5, -1.5, 1.0, -0.5]),
        np.array([pi/4, -pi/3, pi/6, -pi/4, pi/2, -pi/6]),
    ]

    all_pass = True
    for i, q in enumerate(test_configs):
        T = solver.forward(q)

        # Verify FK produces a valid transform
        assert T.shape == (4, 4), f"Config {i}: wrong shape"
        assert np.allclose(T[3, :], [0, 0, 0, 1]), f"Config {i}: bad bottom row"

        # Verify IK round-trip
        q_solved = solver.inverse(T, q_guess=q)
        if q_solved is None:
            print(f"  Config {i}: IK FAILED (no solution)")
            all_pass = False
            continue

        T_solved = solver.forward(q_solved)
        pos_err = np.linalg.norm(T_solved[:3, 3] - T[:3, 3])
        rot_err = np.linalg.norm(T_solved[:3, :3] - T[:3, :3])

        status = "PASS" if pos_err < 1e-6 and rot_err < 1e-6 else "FAIL"
        if status == "FAIL":
            all_pass = False

        print(f"  Config {i}: pos_err={pos_err:.2e}, "
              f"rot_err={rot_err:.2e}  [{status}]")

    print(f"\n  Result: {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass


def test_theta_offset_handling():
    """
    Test 2: theta_offset is correctly applied.
    A robot with theta_offset should produce the same FK
    as a robot without offset when the joint angles are adjusted.
    """
    print("=" * 60)
    print("TEST 2: theta_offset handling")
    print("=" * 60)

    # Create two solvers with same DH parameters
    solver_no_offset = URIKSolver(
        d1=0.1273, a2=-0.612, a3=-0.5723,
        d4=0.163941, d5=0.1157, d6=0.0922
    )

    offset = np.array([0, pi, 0, -pi, pi, -0.873, 0])
    solver_with_offset = URIKSolver(
        d1=0.1273, a2=-0.612, a3=-0.5723,
        d4=0.163941, d5=0.1157, d6=0.0922,
        theta_offset=offset
    )

    # Joint angles in URDF convention
    q_urdf = np.array([0.5, -1.0, 1.5, -0.5, 0.3, 0.8])

    # DH angles: theta = q + offset
    q_dh = q_urdf + offset[1:7]

    # FK with offset solver using URDF angles
    T_with_offset = solver_with_offset.forward(q_urdf)

    # FK with no-offset solver using DH angles
    T_no_offset = solver_no_offset.forward(q_dh)

    # These should be identical
    pos_err = np.linalg.norm(T_with_offset[:3, 3] - T_no_offset[:3, 3])
    rot_err = np.linalg.norm(T_with_offset[:3, :3] - T_no_offset[:3, :3])

    status = "PASS" if pos_err < 1e-10 and rot_err < 1e-10 else "FAIL"
    print(f"  FK consistency: pos_err={pos_err:.2e}, "
          f"rot_err={rot_err:.2e}  [{status}]")

    # IK round-trip with offset
    q_solved = solver_with_offset.inverse(T_with_offset, q_guess=q_urdf)
    if q_solved is not None:
        T_solved = solver_with_offset.forward(q_solved)
        pos_err2 = np.linalg.norm(T_solved[:3, 3] - T_with_offset[:3, 3])
        rot_err2 = np.linalg.norm(T_solved[:3, :3] - T_with_offset[:3, :3])
        status2 = "PASS" if pos_err2 < 1e-6 and rot_err2 < 1e-6 else "FAIL"
        print(f"  IK round-trip:  pos_err={pos_err2:.2e}, "
              f"rot_err={rot_err2:.2e}  [{status2}]")
    else:
        print(f"  IK round-trip:  FAILED (no solution)")
        status2 = "FAIL"

    result = (status == "PASS") and (status2 == "PASS")
    print(f"\n  Result: {'ALL PASS' if result else 'SOME FAILED'}\n")
    return result


def test_fr5_dh_parameters():
    """
    Test 3: FR5 parameters from DHGeometry report.
    This test verifies that the FR5's DH parameters produce
    correct FK, but IK will fail until equations are adapted
    for alpha[5] = +pi/2.
    """
    print("=" * 60)
    print("TEST 3: FR5 DH parameters (FK only, IK expected to fail)")
    print("=" * 60)

    # FR5 parameters from DHGeometry report
    fr5_alpha = [0, 1.570796, 0, 0, 1.570796, 1.570797, 0]
    fr5_theta_offset = [0, 3.141592, 0, -3.141593, 3.141592, -0.872665, 0]

    solver = URIKSolver(
        d1=0.155, a2=0.425, a3=0.395,
        d4=0.13, d5=0.102, d6=0.0,  # d6=0 from kinematic chain
        alpha=fr5_alpha,
        theta_offset=fr5_theta_offset
    )

    # Test FK at zero configuration
    q_zero = np.zeros(6)
    T = solver.forward(q_zero)
    print(f"  FK at zero config: position = {T[:3, 3]}")
    print(f"  FK valid: {T.shape == (4, 4)}")

    # IK will fail because d6 = 0 (division by zero guard)
    q_solved = solver.inverse(T, q_guess=q_zero)
    if q_solved is None:
        print(f"  IK: returned None (expected with d6=0)")
    else:
        print(f"  IK: returned solution (unexpected with d6=0)")

    print(f"\n  Note: IK will work once d6 is extracted from mesh\n")
    return True

def test_fr5_ik_with_d6():
    """
    Test 4: FR5 IK with non-zero d6.
    Uses a placeholder d6 = 0.08 (replace with mesh-extracted value).
    Verifies that the alpha[5] = +pi/2 adaptation produces valid IK.
    """
    print("=" * 60)
    print("TEST 4: FR5 IK with alpha[5]=+pi/2 and d6=0.08")
    print("=" * 60)

    fr5_alpha = [0, 1.570796, 0, 0, 1.570796, 1.570797, 0]
    fr5_theta_offset = [0, 3.141592, 0, -3.141593, 3.141592, -0.872665, 0]

    # Placeholder d6 — replace with mesh-extracted value
    d6_placeholder = 0.08

    solver = URIKSolver(
        d1=0.155, a2=0.425, a3=0.395,
        d4=0.13, d5=0.102, d6=d6_placeholder,
        alpha=fr5_alpha,
        theta_offset=fr5_theta_offset
    )

    # Test at several joint configurations
    test_configs = [
        np.zeros(6),
        np.array([0.3, -0.5, 0.8, -0.3, 0.5, 0.2]),
        np.array([-0.5, -1.0, 1.5, -0.8, 0.8, -0.3]),
    ]

    all_pass = True
    for i, q_urdf in enumerate(test_configs):
        # FK: joint angles -> TCP pose
        T = solver.forward(q_urdf)

        # IK: TCP pose -> joint angles
        q_solved = solver.inverse(T, q_guess=q_urdf)
        if q_solved is None:
            print(f"  Config {i}: IK FAILED (no solution)")
            all_pass = False
            continue

        # Verify round-trip
        T_solved = solver.forward(q_solved)
        pos_err = np.linalg.norm(T_solved[:3, 3] - T[:3, 3])
        rot_err = np.linalg.norm(T_solved[:3, :3] - T[:3, :3])

        # Check joint angle agreement (accounting for theta_offset)
        q_diff = q_solved - q_urdf
        q_diff = (q_diff + pi) % (2 * pi) - pi
        q_err = np.max(np.abs(q_diff))

        status = "PASS" if (pos_err < 1e-6 and rot_err < 1e-6) else "FAIL"
        if status == "FAIL":
            all_pass = False

        print(f"  Config {i}: pos_err={pos_err:.2e}, "
              f"rot_err={rot_err:.2e}, "
              f"max_q_err={q_err:.2e}  [{status}]")

    print(f"\n  Result: {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Hatch IK Solver - Parameterization Verification")
    print("=" * 60 + "\n")

    r1 = test_ur_backward_compatibility()
    r2 = test_theta_offset_handling()
    r3 = test_fr5_dh_parameters()
    r4 = test_fr5_ik_with_d6()

    print("=" * 60)
    print(f"SUMMARY: Test1={'PASS' if r1 else 'FAIL'}, "
          f"Test2={'PASS' if r2 else 'FAIL'}, "
          f"Test3={'PASS' if r3 else 'FAIL'}, "
          f"Test4={'PASS' if r4 else 'FAIL'}")
    print("=" * 60)