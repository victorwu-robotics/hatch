import numpy as np
import math
import random
from elf_ik import GeometricIKSolver
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
        ik_solutions = solver.inverse_kinematics(original_pose, random_joints)
        
        if ik_solutions:
            # Get closest solution to original
            solved_joints = ik_solutions[0]
            
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
    solver = GeometricIKSolver(model)
    
    print("\n" + "█" * 60)
    print("  HAN'S E15-PRO IK SOLVER TEST SUITE")
    print("█" * 60)
    
    # Run tests
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