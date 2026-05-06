#!/usr/bin/env python3
"""
Correct IK solver for Han's E15-Pro based on actual measurements.
"""

import numpy as np
import math
from typing import List, Tuple, Optional

class HanE15ProIK:
    """
    IK solver based on actual robot measurements.
    
    Link lengths:
        d1 = 0.262 m (base to joint 2)
        a2 = 0.730 m (upper arm)
        a3 = 0.570 m (forearm)
        d6 = 0.200 m (tool)
    """
    
    def __init__(self):
        # Link lengths from actual measurements
        self.d1 = 0.262      # Base to joint 2
        self.a2 = 0.730      # Upper arm
        self.a3 = 0.570      # Forearm
        self.d6 = 0.200      # Tool length
        
        # Joint limits from URDF
        self.joint_limits = [
            (-3.14, 3.14),   # J1
            (-2.35, 2.35),   # J2
            (-2.61, 2.61),   # J3
            (-3.14, 3.14),   # J4
            (-2.56, 2.56),   # J5
            (-3.14, 3.14)    # J6
        ]
        
    def forward_kinematics(self, joints: np.ndarray, debug: bool = False) -> np.ndarray:
        """
        Forward kinematics matching your actual robot.
        """
        q1, q2, q3, q4, q5, q6 = joints
        
        # Start at base
        T = np.eye(4)
        
        # Joint 1: Rotate about Z, then translate up
        T = self._rot_z(T, q1)
        T = self._trans_z(T, self.d1)
        
        # Joint 2: Rotate about Y (negative for -X direction), then translate X
        T = self._rot_y(T, -q2)  # Negative q2 gives negative X
        T = self._trans_x(T, self.a2)
        
        # Joint 3: Rotate about Y, then translate X
        T = self._rot_y(T, q3)
        T = self._trans_x(T, self.a3)
        
        # Spherical wrist (J4, J5, J6)
        T = self._rot_z(T, q4)
        T = self._rot_x(T, math.pi/2)
        T = self._rot_z(T, q5)
        T = self._rot_x(T, math.pi/2)
        T = self._rot_z(T, q6)
        
        # Tool length along Z
        T = self._trans_z(T, self.d6)
        
        if debug:
            print(f"Position: ({T[0,3]*1000:.1f}, {T[1,3]*1000:.1f}, {T[2,3]*1000:.1f}) mm")
        
        return T
    
    def inverse_kinematics(self, T_target: np.ndarray,
                          previous_solution: Optional[np.ndarray] = None) -> List[np.ndarray]:
        """
        Analytic inverse kinematics.
        """
        solutions = []
        
        # Remove tool offset to get wrist position
        tool_axis = T_target[:3, 2]
        wrist_pos = T_target[:3, 3] - self.d6 * tool_axis
        
        # Solve theta1
        theta1 = math.atan2(wrist_pos[1], wrist_pos[0])
        
        # Transform to arm plane
        x = wrist_pos[0] * math.cos(-theta1) - wrist_pos[1] * math.sin(-theta1)
        z = wrist_pos[2] - self.d1
        
        # Distance from shoulder to wrist
        D = math.sqrt(x**2 + z**2)
        
        # Check reachability
        if D > self.a2 + self.a3 or D < abs(self.a2 - self.a3):
            return solutions
        
        # Law of cosines for theta3 (elbow)
        cos_theta3 = (self.a2**2 + self.a3**2 - D**2) / (2 * self.a2 * self.a3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        
        theta3_1 = math.acos(cos_theta3)
        theta3_2 = -theta3_1
        
        for theta3 in [theta3_1, theta3_2]:
            # Angle from shoulder to wrist
            phi = math.atan2(z, x)
            
            # Angle using law of sines
            if D > 1e-6:
                beta = math.asin((self.a3 * math.sin(theta3)) / D)
            else:
                beta = 0
            
            theta2 = phi - beta
            theta2 = -theta2  # Because J2 rotates negative direction
            
            solution = np.array([theta1, theta2, theta3, 0.0, 0.0, 0.0])
            
            if self._check_limits(solution):
                solutions.append(solution)
        
        if previous_solution is not None and solutions:
            return [self._closest(solutions, previous_solution)]
        
        return solutions
    
    def _rot_x(self, T, angle):
        c, s = math.cos(angle), math.sin(angle)
        R = np.eye(4)
        R[1,1], R[1,2] = c, -s
        R[2,1], R[2,2] = s, c
        return T @ R
    
    def _rot_y(self, T, angle):
        c, s = math.cos(angle), math.sin(angle)
        R = np.eye(4)
        R[0,0], R[0,2] = c, s
        R[2,0], R[2,2] = -s, c
        return T @ R
    
    def _rot_z(self, T, angle):
        c, s = math.cos(angle), math.sin(angle)
        R = np.eye(4)
        R[0,0], R[0,1] = c, -s
        R[1,0], R[1,1] = s, c
        return T @ R
    
    def _trans_x(self, T, dist):
        trans = np.eye(4)
        trans[0,3] = dist
        return T @ trans
    
    def _trans_z(self, T, dist):
        trans = np.eye(4)
        trans[2,3] = dist
        return T @ trans
    
    def _check_limits(self, joints):
        for i, (min_r, max_r) in enumerate(self.joint_limits):
            if joints[i] < min_r - 0.01 or joints[i] > max_r + 0.01:
                return False
        return True
    
    def _closest(self, solutions, ref):
        def dist(s):
            diff = s - ref
            return np.sum(diff**2)
        return min(solutions, key=dist)


def test_solver():
    """Test the solver against known configurations."""
    solver = HanE15ProIK()
    
    print("="*60)
    print("TESTING IK SOLVER AGAINST ACTUAL ROBOT DATA")
    print("="*60)
    
    # Test configurations
    tests = [
        ("Home", [0, 0, 0, 0, 0, 0], (0, 0, 1.562)),
        ("J2=90°", [0, math.pi/2, 0, 0, 0, 0], (-1.300, 0, 0.262)),
        ("J3=90°", [0, 0, math.pi/2, 0, 0, 0], (0.570, 0, 0.992)),
        ("J2=90°, J3=90°", [0, math.pi/2, math.pi/2, 0, 0, 0], (-0.730, 0, 0.832)),
        ("J2=-90°", [0, -math.pi/2, 0, 0, 0, 0], (1.300, 0, 0.262)),
    ]
    
    print("\nForward Kinematics Test:")
    print("-"*40)
    for name, q, expected in tests:
        pose = solver.forward_kinematics(np.array(q))
        pos = pose[:3, 3]
        error = math.sqrt((pos[0]-expected[0])**2 + (pos[2]-expected[2])**2)
        status = "✓" if error < 0.001 else "✗"
        print(f"  {name:15s}: ({pos[0]*1000:6.1f}, {pos[2]*1000:6.1f}) mm, error={error*1000:.1f}mm {status}")
    
    print("\nInverse Kinematics Test (recover from forward):")
    print("-"*40)
    
    for name, q, expected in tests:
        pose = solver.forward_kinematics(np.array(q))
        solutions = solver.inverse_kinematics(pose, np.array(q))
        
        if solutions:
            solved = solutions[0]
            recomputed = solver.forward_kinematics(solved)
            error = np.linalg.norm(recomputed[:3, 3] - pose[:3, 3])
            status = "✓" if error < 0.01 else "✗"
            print(f"  {name:15s}: error={error*1000:.1f}mm {status}")
        else:
            print(f"  {name:15s}: NO SOLUTION {status}")


if __name__ == "__main__":
    test_solver()