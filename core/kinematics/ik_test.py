import numpy as np
import math
import random
from scipy.spatial.transform import Rotation as R
from analytical_ik_solver import AnalyticalIKSolver
from kinematic_model import KinematicModel
from pathlib import Path
import sys
sys.path.insert(0, str(Path.home() / "hatch"))

def test_ik_validation(solver, num_tests=100):
    """
    Test that inverse kinematics correctly reproduces forward kinematics.
    This is the gold-standard test for IK solvers.
    """
    print("=" * 60)
    print("TEST 1: Forward/Inverse Kinematics Validation")
    print("=" * 60)
    
    success_count = 0
    position_errors = []
    orientation_errors = []
    
    for i in range(num_tests):
        # Generate random valid joint angles
        random_joints = np.array([
            random.uniform(-2.5, 2.5),  # J1
            random.uniform(-2.0, 2.0),  # J2
            random.uniform(-2.0, 2.0),  # J3
            random.uniform(-2.5, 2.5),  # J4
            random.uniform(-2.0, 2.0),  # J5
            random.uniform(-2.5, 2.5)   # J6
        ])
        
        # Forward kinematics: joints -> pose
        original_pose = solver.forward_kinematics(random_joints)
        
        # Inverse kinematics: pose -> joints
        all_solutions = solver.solve_ik(original_pose)
        
        if all_solutions:
            # Get closest solution to original
            solved_joints = solver._select_closest(all_solutions, random_joints)
            
            # Compute pose from solved joints
            solved_pose = solver.forward_kinematics(solved_joints)
            
            # Calculate errors
            pos_error = np.linalg.norm(solved_pose[:3, 3] - original_pose[:3, 3])
            position_errors.append(pos_error)
            
            # Orientation error (Frobenius norm of rotation difference)
            rot_error = np.linalg.norm(solved_pose[:3, :3] - original_pose[:3, :3])
            orientation_errors.append(rot_error)
            
            if pos_error < 0.001:  # 1mm tolerance
                success_count += 1
                
    # Report results
    print(f"Tests run: {num_tests}")
    print(f"Successful IK solves (within 1mm): {success_count}/{num_tests} ({100*success_count/num_tests:.1f}%)")
    
    if position_errors:
        print(f"\nPosition Error Statistics:")
        print(f"  Mean: {np.mean(position_errors)*1000:.3f} mm")
        print(f"  Std Dev: {np.std(position_errors)*1000:.3f} mm")
        print(f"  Max: {np.max(position_errors)*1000:.3f} mm")
        print(f"  Min: {np.min(position_errors)*1000:.3f} mm")
        
        print(f"\nOrientation Error Statistics:")
        print(f"  Mean: {np.mean(orientation_errors):.6f} rad")
        print(f"  Max: {np.max(orientation_errors):.6f} rad")
    
    return success_count == num_tests

# In ik_test.py, add this before or after your existing tests

def test_wrist_axes(solver):
    """Debug: see which axis each wrist joint rotates about at zero."""
    print("\n" + "="*60)
    print("WRIST AXIS TEST")
    print("="*60)
    
    q = np.zeros(6)
   
    # All zeros
    T0 = solver.forward_kinematics(q)
    print(f"\nAll joints zero:")
    print(f"  Position: {T0[:3, 3]}")
    print(f"  Rotation:\n{T0[:3, :3]}")
    
    # θ4 = 0.1 rad
    q4 = q.copy()
    q4[3] = 0.1
    T4 = solver.forward_kinematics(q4)
    print(f"\nθ4 = 0.1 rad (joint 4):")
    print(f"  Position change: {T4[:3, 3] - T0[:3, 3]}")
    print(f"  Rotation axis (from rotvec): {R.from_matrix(T0[:3,:3].T @ T4[:3,:3]).as_rotvec()}")
    
    # θ5 = 0.1 rad
    q5 = q.copy()
    q5[4] = 0.1
    T5 = solver.forward_kinematics(q5)
    print(f"\nθ5 = 0.1 rad (joint 5):")
    print(f"  Position change: {T5[:3, 3] - T0[:3, 3]}")
    print(f"  Rotation axis (from rotvec): {R.from_matrix(T0[:3,:3].T @ T5[:3,:3]).as_rotvec()}")
    
    # θ6 = 0.1 rad
    q6 = q.copy()
    q6[5] = 0.1
    T6 = solver.forward_kinematics(q6)
    print(f"\nθ6 = 0.1 rad (joint 6):")
    print(f"  Position change: {T6[:3, 3] - T0[:3, 3]}")
    print(f"  Rotation axis (from rotvec): {R.from_matrix(T0[:3,:3].T @ T6[:3,:3]).as_rotvec()}")

def test_orientation_roundtrip(solver):
    """Test that a known orientation produces correct wrist angles."""
    print("\n" + "="*60)
    print("ORIENTATION ROUNDTRIP TEST")
    print("="*60)
    
    # Arm at zero
    θ1, θ2, θ3 = 0.0, 0.0, 0.0
    
    # Test pure Z rotation (RZ slider)
    R_target = R.from_rotvec([0, 0, 0.3]).as_matrix()  # 0.3 rad about Z
    
    # What does _solve_wrist produce?
    R03 = solver._compute_R03(θ1, θ2, θ3)
    print(f"R03:\n{R03}")
    
    R_wrist_target = R03.T @ R_target
    print(f"R_wrist_target:\n{R_wrist_target}")
    
    # Extract ZYZ
    solutions = solver._extract_zyz_euler(R_wrist_target)
    print(f"Extracted wrist angles: {solutions}")
    
    # Expected: rotation about Z should come from θ4 and/or θ6
    # Not from θ5 (which is about Y)

def test_inverted_orientation(solver):
    """Test IK with TCP pointing straight down."""
    print("\n" + "="*60)
    print("INVERTED ORIENTATION TEST")
    print("="*60)
    
    # TCP at some reachable position, pointing straight down
    pos = np.array([0.3, 0.2, 0.3])
    
    # Pointing down: Z is -Z_world
    # Need X and Y to complete the rotation matrix
    # Tool pointing down: approach vector = [0, 0, -1]
    # Let's say tool X = world X, tool Y = world -Y (to keep right-handed)
    R_down = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])
    
    target_pose = np.eye(4)
    target_pose[:3, :3] = R_down
    target_pose[:3, 3] = pos
    
    # Solve IK
    solutions = solver.solve_ik(target_pose)
    
    for i, q in enumerate(solutions[:2]):  # Just show first 2
        # Forward kinematics to verify
        T = solver.forward_kinematics(q)
        pos_err = np.linalg.norm(T[:3, 3] - pos)
        rot_err = np.linalg.norm(T[:3, :3] - R_down)
        print(f"\nSolution {i+1}: {np.degrees(q)}")
        print(f"  Position error: {pos_err:.6f}")
        print(f"  Rotation error: {rot_err:.6f}")
    
    # Now test: what if we apply a small RZ rotation?
    R_rz = R.from_rotvec([0, 0, 0.2]).as_matrix()
    R_test = R_down @ R_rz  # Rotate about world Z after pointing down
    # Or: R_test = R_rz @ R_down  # Rotate about world Z before?
    # Which one does your slider do?
    
    print("\nSmall RZ rotation applied to inverted pose:")
    print(f"Original R:\n{R_down}")
    print(f"After RZ(0.2) post-multiplied:\n{R_test}")
    
    target_pose[:3, :3] = R_test
    solutions = solver.solve_ik(target_pose)
    for i, q in enumerate(solutions[:1]):
        T = solver.forward_kinematics(q)
        print(f"Result pose:\n{T[:3, :3]}")

def run_complete_test_suite():
    """
    Run all tests to validate the IK solver.
    """
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


    # Initialize your solver
    solver = AnalyticalIKSolver(model, d1=0.262, a2=0.73, a3=0.57, d6=0.20)
    
    print("\n" + "█" * 60)
    print("  HAN'S E15-PRO IK SOLVER TEST SUITE")
    print("█" * 60)
    
    # Run tests
    
    test_wrist_axes(solver)
    test_orientation_roundtrip(solver)
    test_inverted_orientation(solver)
    
    test1 = test_ik_validation(solver, num_tests=100)
    # test2 = test_workspace_coverage(solver)
    # test3 = test_path_tracking(solver)
    # test4 = test_joint_limits(solver, num_tests=50)
    # test5 = test_against_trusted_library(solver)
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"✓ Forward/Inverse Validation: {'PASS' if test1 else 'FAIL'}")
    # print(f"✓ Workspace Coverage: {'PASS' if test2 else 'FAIL'}")
    # print(f"✓ Path Tracking: {'COMPLETE'}")
    # print(f"✓ Joint Limits: {'PASS' if test4 else 'FAIL'}")
    
    # if test1 and test2 and test4:
    #     print("\n🎉 ALL TESTS PASSED! IK solver is ready for use.")
    # else:
    #     print("\n⚠️ Some tests failed. Review output for details.")

if __name__ == "__main__":
    run_complete_test_suite()