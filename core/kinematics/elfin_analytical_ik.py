import numpy as np

class ElfinIK:
    """
    Analytical Inverse Kinematics for 6-DOF robot with DH parameters:
    Joint  θ (DH)     d       a       α
    1      θ₁        0.262   0       90°
    2      θ₂+90°    0       0.73    180°
    3      θ₃+90°    0       0       90°
    4      θ₄        0.57    0       -90°
    5      θ₅        0       0       90°
    6      θ₆        0.200   0       0°
    """
    
    def __init__(self):
        # DH parameters
        self.d1 = 0.262  # Joint 1 offset
        self.a2 = 0.73   # Link 2 length
        self.d4 = 0.57   # Joint 4 offset (wrist center distance from frame 3)
        self.d6 = 0.200  # End effector offset
        
        # Joint limits (example values, adjust to your robot)
        self.joint_limits = [
            (-np.pi, np.pi),      # θ1: ±180°
            (-np.pi/2, np.pi/2),  # θ2: ±90° (typical for this design)
            (-np.pi, np.pi),      # θ3: ±180°
            (-np.pi, np.pi),      # θ4: ±180°
            (-np.pi, np.pi),      # θ5: ±180°
            (-np.pi, np.pi),      # θ6: ±180°
        ]
    
    def solve_ik(self, T_desired, check_limits=True):
        """
        Complete inverse kinematics solver.
        
        Parameters:
        T_desired: 4x4 homogeneous transformation matrix of end effector
        check_limits: If True, filter solutions by joint limits
        
        Returns:
        List of valid joint angle solutions [(θ1, θ2, θ3, θ4, θ5, θ6), ...]
        """
        P_desired = T_desired[:3, 3]
        R_desired = T_desired[:3, :3]
        
        # Step 1: Calculate wrist center position
        # The wrist center is at the origin of frame 4 (intersection of J4,J5,J6 axes)
        # In end effector frame, it's located d6 back along Z-axis
        wrist_center = P_desired - self.d6 * R_desired[:, 2]
        
        # Step 2: Check reachability
        r = np.linalg.norm(wrist_center - np.array([0, 0, self.d1]))
        if r > self.a2 + self.d4 + 0.001:  # Small tolerance
            return []  # Unreachable
        
        # Step 3: Solve arm (first 3 joints)
        arm_solutions = self._solve_arm(wrist_center)
        
        if not arm_solutions:
            return []
        
        # Step 4: Solve wrist (last 3 joints) for each arm solution
        all_solutions = []
        
        for θ1, θ2, θ3 in arm_solutions:
            # Get orientation of frame 3
            T_03 = self._fk_arm(θ1, θ2, θ3)
            R_03 = T_03[:3, :3]
            
            # Desired wrist orientation matrix
            R_36 = R_03.T @ R_desired
            
            # Solve wrist
            wrist_solutions = self._solve_wrist(R_36)
            
            for θ4, θ5, θ6 in wrist_solutions:
                # Verify complete solution
                T_06 = self._fk_full(θ1, θ2, θ3, θ4, θ5, θ6)
                error_pos = np.linalg.norm(T_06[:3, 3] - P_desired)
                error_orient = np.linalg.norm(T_06[:3, :3] - R_desired)
                
                if error_pos < 1e-4 and error_orient < 1e-4:
                    solution = (θ1, θ2, θ3, θ4, θ5, θ6)
                    
                    # Check joint limits if requested
                    if check_limits:
                        if self._check_limits(solution):
                            all_solutions.append(solution)
                    else:
                        all_solutions.append(solution)
        
        return all_solutions
    
    def _solve_arm(self, P_wrist):
        """
        Analytical solution for first 3 joints.
        
        From the forward kinematics:
        x = -a2*sin(θ2)*cos(θ1) + d4*(sin(θ1)*sin(θ3) - sin(θ2)*cos(θ1)*cos(θ3))
        y = -a2*sin(θ1)*sin(θ2) - d4*sin(θ1)*sin(θ2)*cos(θ3) - d4*sin(θ3)*cos(θ1)
        z = a2*cos(θ2) + d1 + d4*cos(θ2)*cos(θ3)
        """
        x, y, z = P_wrist
        
        # Step 1: Solve for θ3 using distance equation
        # x² + y² + (z-d1)² = a2² + d4² + 2*a2*d4*cos(θ3)
        r_sq = x**2 + y**2 + (z - self.d1)**2
        cos_θ3 = (r_sq - self.a2**2 - self.d4**2) / (2 * self.a2 * self.d4)
        
        # Check if point is reachable
        if abs(cos_θ3) > 1.0001:
            return []
        
        cos_θ3 = np.clip(cos_θ3, -1.0, 1.0)
        
        solutions = []
        
        # Two solutions for θ3 (elbow up/down)
        for θ3 in [np.arccos(cos_θ3), -np.arccos(cos_θ3)]:
            # Compute helper variables
            K = self.a2 + self.d4 * np.cos(θ3)
            B = self.d4 * np.sin(θ3)
            
            if abs(K) < 1e-9:
                continue  # Singularity, skip
            
            # Step 2: Compute θ2 from z-equation
            cos_θ2 = (z - self.d1) / K
            
            if abs(cos_θ2) > 1.0001:
                continue
            
            cos_θ2 = np.clip(cos_θ2, -1.0, 1.0)
            
            # Step 3: Solve for θ1 using the x-y equations
            # From: B = x*sin(θ1) - y*cos(θ1)
            # This can be written as: B = sqrt(x²+y²) * sin(θ1 - atan2(y,x))
            
            r_xy = np.sqrt(x**2 + y**2)
            
            if r_xy < 1e-9:
                # On Z-axis, θ1 is arbitrary (choose 0 or π)
                θ1_options = [0.0, np.pi]
            else:
                sin_diff = B / r_xy
                
                if abs(sin_diff) > 1.0001:
                    continue
                
                sin_diff = np.clip(sin_diff, -1.0, 1.0)
                
                φ = np.arctan2(y, x)  # atan2(y,x)
                γ = np.arcsin(sin_diff)
                
                # Two solutions: θ1 - φ = γ or θ1 - φ = π - γ
                θ1_options = [φ + γ, φ + np.pi - γ]
            
            # Verify each θ1 candidate
            for θ1 in θ1_options:
                # Normalize θ1 to [-π, π]
                θ1 = np.arctan2(np.sin(θ1), np.cos(θ1))
                
                # Compute sin(θ2) from A equation
                A = x * np.cos(θ1) + y * np.sin(θ1)
                sin_θ2 = -A / K
                sin_θ2 = np.clip(sin_θ2, -1.0, 1.0)
                
                θ2 = np.arctan2(sin_θ2, cos_θ2)
                
                # Verify using forward kinematics
                T_03 = self._fk_arm(θ1, θ2, θ3)
                wrist_test = T_03 @ np.array([0, 0, self.d4, 1])
                error = np.linalg.norm(wrist_test[:3] - P_wrist)
                
                if error < 1e-6:
                    solutions.append((θ1, θ2, θ3))
        
        return solutions
    
    def _solve_wrist(self, R_36):
        """
        Analytical solution for wrist joints (θ4, θ5, θ6).
        
        Given R_36, the rotation matrix from frame 3 to frame 6.
        From DH parameters with α4=-90°, α5=90°, α6=0°:
        
        R_36 = [c4c5c6-s4s6, -c4c5s6-s4c6, c4s5]
               [s4c5c6+c4s6, -s4c5s6+c4c6, s4s5]
               [-s5c6,       s5s6,        c5  ]
        """
        r13 = R_36[0, 2]
        r23 = R_36[1, 2]
        r33 = R_36[2, 2]
        r31 = R_36[2, 0]
        r32 = R_36[2, 1]
        
        solutions = []
        
        # Check for wrist singularity (θ5 ≈ 0 or π)
        if abs(r33) > 0.9999:
            # Singularity: axes 4 and 6 are aligned
            # Only sum/difference of θ4 and θ6 is determined
            if r33 > 0:
                # θ5 = 0
                θ5 = 0.0
                θ4 = 0.0  # Arbitrary choice
                θ6 = np.arctan2(R_36[1, 0], R_36[0, 0])
            else:
                # θ5 = π
                θ5 = np.pi
                θ4 = 0.0  # Arbitrary choice
                θ6 = np.arctan2(-R_36[1, 0], -R_36[0, 0])
            
            solutions.append((θ4, θ5, θ6))
        else:
            # Non-singular case: two solutions
            θ5_options = [np.arccos(r33), -np.arccos(r33)]
            
            for θ5 in θ5_options:
                sin_θ5 = np.sin(θ5)
                
                # θ4 = atan2(r23/sin(θ5), r13/sin(θ5))
                θ4 = np.arctan2(r23 / sin_θ5, r13 / sin_θ5)
                
                # θ6 = atan2(r32/sin(θ5), -r31/sin(θ5))
                θ6 = np.arctan2(r32 / sin_θ5, -r31 / sin_θ5)
                
                solutions.append((θ4, θ5, θ6))
        
        return solutions
    
    def _fk_arm(self, θ1, θ2, θ3):
        """Forward kinematics for first 3 joints (returns T_03)"""
        θ2_DH = θ2 + np.pi/2  # DH offset
        θ3_DH = θ3 + np.pi/2  # DH offset
        
        # T_01
        T_01 = np.array([
            [np.cos(θ1), 0, np.sin(θ1), 0],
            [np.sin(θ1), 0, -np.cos(θ1), 0],
            [0, 1, 0, self.d1],
            [0, 0, 0, 1]
        ])
        
        # T_12
        T_12 = np.array([
            [np.cos(θ2_DH), 0, np.sin(θ2_DH), self.a2 * np.cos(θ2_DH)],
            [np.sin(θ2_DH), 0, -np.cos(θ2_DH), self.a2 * np.sin(θ2_DH)],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        
        # T_23
        T_23 = np.array([
            [np.cos(θ3_DH), 0, np.sin(θ3_DH), 0],
            [np.sin(θ3_DH), 0, -np.cos(θ3_DH), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        
        return T_01 @ T_12 @ T_23
    
    def _fk_full(self, θ1, θ2, θ3, θ4, θ5, θ6):
        """Complete forward kinematics (returns T_06)"""
        T_03 = self._fk_arm(θ1, θ2, θ3)
        
        # T_34 (α=-90°)
        T_34 = np.array([
            [np.cos(θ4), 0, -np.sin(θ4), 0],
            [np.sin(θ4), 0, np.cos(θ4), 0],
            [0, -1, 0, self.d4],
            [0, 0, 0, 1]
        ])
        
        # T_45 (α=90°)
        T_45 = np.array([
            [np.cos(θ5), 0, np.sin(θ5), 0],
            [np.sin(θ5), 0, -np.cos(θ5), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        
        # T_56 (α=0°)
        T_56 = np.array([
            [np.cos(θ6), -np.sin(θ6), 0, 0],
            [np.sin(θ6), np.cos(θ6), 0, 0],
            [0, 0, 1, self.d6],
            [0, 0, 0, 1]
        ])
        
        return T_03 @ T_34 @ T_45 @ T_56
    
    def _check_limits(self, solution):
        """Check if solution respects joint limits"""
        for i, (angle, (lower, upper)) in enumerate(zip(solution, self.joint_limits)):
            if angle < lower or angle > upper:
                return False
        return True
    
    def fk(self, joint_angles):
        """Public forward kinematics method"""
        return self._fk_full(*joint_angles)


# Test and demonstrate
if __name__ == "__main__":
    ik = ElfinIK()
    
    print("="*60)
    print("ELFIN ROBOT INVERSE KINEMATICS SOLVER")
    print("="*60)
    
    # Test 1: Known configuration
    print("\nTest 1: Recover known configuration")
    print("-"*40)
    target_angles_deg = [30, 45, 60, 10, 20, 30]
    target_angles = np.radians(target_angles_deg)
    
    # Forward kinematics
    T = ik.fk(target_angles)
    
    # Inverse kinematics
    solutions = ik.solve_ik(T)
    
    print(f"Target angles: {target_angles_deg}")
    print(f"Solutions found: {len(solutions)}")
    
    # Find matching solution
    for i, sol in enumerate(solutions):
        angles_deg = np.degrees(sol)
        if np.allclose(angles_deg, target_angles_deg, atol=0.01):
            print(f"✓ Solution {i+1} matches target exactly!")
    
    # Test 2: Random pose
    print("\nTest 2: Random configuration")
    print("-"*40)
    np.random.seed(123)
    random_angles = np.random.uniform(-np.pi/2, np.pi/2, 6)
    random_angles[1] = np.random.uniform(-np.pi/4, np.pi/4)  # θ2 limited
    random_angles[4] = np.random.uniform(0.1, np.pi-0.1)  # Avoid wrist singularity
    
    T_random = ik.fk(random_angles)
    solutions_random = ik.solve_ik(T_random)
    
    print(f"Random angles: {np.degrees(random_angles).round(1)}")
    print(f"Solutions found: {len(solutions_random)}")
    
    # Verify all solutions
    print("\nVerifying all solutions...")
    all_valid = True
    for i, sol in enumerate(solutions_random):
        T_check = ik.fk(sol)
        error = np.linalg.norm(T_check - T_random)
        if error > 1e-3:
            print(f"✗ Solution {i+1} error: {error:.2e}")
            all_valid = False
    
    if all_valid:
        print("✓ All solutions are valid!")
    
    # Test 3: Unreachable pose
    print("\nTest 3: Unreachable pose")
    print("-"*40)
    T_far = np.eye(4)
    T_far[:3, 3] = [10, 0, 0]  # Way outside workspace
    T_far[:3, :3] = np.eye(3)
    
    solutions_far = ik.solve_ik(T_far)
    print(f"Solutions for far point: {len(solutions_far)} (should be 0)")
    
    print("\n" + "="*60)
    print("All tests completed successfully!")