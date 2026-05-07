"""
Analytic IK Solver for Elfin15.
Zero config axes: J1=+Z, J2=-Y, J3=+Y, J4=+Z, J5=+Y, J6=+Z
Arm: moves in XZ plane after J1 rotation
Wrist: ZYZ Euler wrist in forearm frame
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt
from typing import List, Tuple
import logging

logger = logging.getLogger(__name__)


class GeometricIKSolver:
    def __init__(self, kinematic_model):
        self.model = kinematic_model
        
        # Fixed kinematic parameters (from URDF)
        self.shoulder_height = 0.262   # J1 to J2 along Z
        self.l_upper = 0.730           # J2 to J3 along Y
        self.l_forearm = 0.570         # J3 to wrist centre (bent link4 along Z in link3 frame)
        self.d_tool = np.array([0.0, 0.0, 0.200])  # Wrist centre to TCP in TCP frame
        
        # Identify key links
        arm_chain = self.model.get_arm_chain(self.model.get_true_root())
        joint4 = self.model.joints[arm_chain[3]]
        
        # Link3 = forearm, the frame before wrist joints start
        self.wrist_parent_link = joint4['parent']  # elfin_link3
        
        # The TCP link (end effector)
        self.tcp_link = self.model.tool_mount_link
        
        logger.info(f"Elfin15 IK Solver: shoulder={self.shoulder_height}, upper={self.l_upper}, "
                   f"forearm={self.l_forearm}, tool={self.d_tool[2]}")
        logger.info(f"Wrist: ZYZ Euler (J4=Z, J5=Y, J6=Z in forearm frame)")

    def solve_ik_for_tcp(self, target_pose, q_guess=None):
        """Solve IK for target TCP pose (4x4 homogeneous matrix)."""
        p_tcp = target_pose[:3, 3]
        R_tcp = target_pose[:3, :3]

        # DIAGNOSTIC - check what tool_mount_link actually is
        print(f"\n=== WRIST CENTER DEBUG ===")
        print(f"tool_mount_link = {self.tcp_link}")
        
        # Check all link positions at zero
        q_zero = np.zeros(6)
        self.model.update_state(q_zero)
        for link_name in ['elfin_link3', 'elfin_link4', 'elfin_link5', 'elfin_link6', 'elfin_end_link']:
            if link_name in self.model.link_transforms:
                T = self.model.link_transforms[link_name]
                print(f"{link_name} pos: {T[:3, 3]}")
        
        # What is our d_tool?
        print(f"self.d_tool = {self.d_tool}")
        
        # Compute wrist center with current d_tool
        p_wrist = p_tcp - R_tcp @ self.d_tool
        print(f"Computed wrist center: {p_wrist}")
        
        # Now check: if we set J1-J3 to some solution and zero J4-J6, where is the TCP?
        # Get an arm solution
        arm_sols = self._solve_arm(p_wrist)
        if arm_sols:
            q_test = np.array([arm_sols[0][0], arm_sols[0][1], arm_sols[0][2], 0, 0, 0])
            self.model.update_state(q_test)
            T_tcp_zero_wrist = self.model.get_tcp_pose()
            print(f"Arm only (J4-J6=0) TCP pos: {T_tcp_zero_wrist[:3, 3]}")
            print(f"Target TCP pos: {p_tcp}")
            print(f"Position error: {np.linalg.norm(T_tcp_zero_wrist[:3, 3] - p_tcp):.4f}")
        
        print(f"=========================\n")

        # Step 1: Compute wrist centre
        p_wrist = p_tcp - R_tcp @ self.d_tool
        logger.debug(f"Wrist centre: {p_wrist}")
        
        # Step 2: Solve arm (J1-J3)
        arm_solutions = self._solve_arm(p_wrist)
        logger.info(f"Found {len(arm_solutions)} arm solutions")
        
        if not arm_solutions:
            logger.warning("No arm solutions found")
            return None
        
        # Step 3: Solve wrist (J4-J6) for each arm solution
        for q1, q2, q3 in arm_solutions:
            wrist_solutions = self._solve_wrist(target_pose, np.array([q1, q2, q3]))
            logger.debug(f"Arm ({q1:.4f}, {q2:.4f}, {q3:.4f}): {len(wrist_solutions)} wrist solutions")
            
            for q4, q5, q6 in wrist_solutions:
                q = np.array([q1, q2, q3, q4, q5, q6])
                q = self._wrap_angles(q)
                
                # Verify
                self.model.update_state(q)
                T_check = self.model.get_tcp_pose()
                pos_err = np.linalg.norm(T_check[:3, 3] - p_tcp)
                rot_err = np.linalg.norm(T_check[:3, :3] - R_tcp)
                
                if pos_err < 1e-3 and rot_err < 1e-3:
                    logger.info(f"VALID: q={q}, pos_err={pos_err:.6f}, rot_err={rot_err:.6f}")
                    return q
                else:
                    logger.debug(f"REJECTED: q={q}, pos_err={pos_err:.4f}, rot_err={rot_err:.4f}")
        
        logger.warning("No valid IK solution found")
        return None

    def _solve_arm(self, wrist_center):
        x, y, z = wrist_center
        r_xy = sqrt(x**2 + y**2)
        
        if r_xy < 1e-6:
            q1_options = [0.0]
        else:
            q1_base = atan2(y, x)
            q1_options = [q1_base, q1_base + pi]
        
        dz = z - self.shoulder_height
        d = sqrt(r_xy**2 + dz**2)
        
        max_reach = self.l_upper + self.l_forearm
        min_reach = abs(self.l_upper - self.l_forearm)
        
        if d > max_reach + 1e-6 or d < min_reach - 1e-6:
            return []
        
        solutions = []
        for q1 in q1_options:
            beta = atan2(dz, r_xy)
            cos_q3 = (self.l_upper**2 + self.l_forearm**2 - d**2) / (2 * self.l_upper * self.l_forearm)
            cos_q3 = np.clip(cos_q3, -1.0, 1.0)
            
            for q3 in [acos(cos_q3), -acos(cos_q3)]:
                alpha = atan2(self.l_forearm * sin(q3), self.l_upper + self.l_forearm * cos(q3))
                # Upper arm at J2=0 is VERTICAL (π/2 from horizontal)
                q2 = pi/2 - (beta - alpha)
                solutions.append((q1, q2, q3))
        
        return solutions

    def _solve_wrist(self, T_target, q123):
        from scipy.spatial.transform import Rotation as R
        
        R_tcp_target = T_target[:3, :3]
        
        q_full = np.concatenate([q123, np.zeros(3)])
        self.model.update_state(q_full)
        T_forearm = self.model.link_transforms[self.wrist_parent_link]
        R_forearm = T_forearm[:3, :3]

        # R_tcp = R_forearm @ R_wrist @ R_tool_const
        # R_wrist_needed = R_forearm.T @ R_tcp @ R_tool_const.T
        R_tool_const = np.array([[1, 0, 0],
                                [0, 0, -1],
                                [0, 1, 0]])  # Rx(π/2) - adjust if needed

        R_wrist_needed = R_forearm.T @ R_tcp_target @ R_tool_const.T
        r = R.from_matrix(R_wrist_needed)
        
        # Try ZYZ
        q4, q5, q6 = r.as_euler('ZYZ', degrees=False)
        
        solutions = [
            (q4, q5, q6),
            (q4 + pi, -q5, q6 + pi),
            # Key fix: the orientation is flipped, so negate q5
            (q4, -q5, q6),
            (q4 + pi, q5, q6 + pi),
            # Maybe offset by pi/2 on q4/q6
            (q4 + pi/2, q5, q6 - pi/2),
            (q4 - pi/2, q5, q6 + pi/2),
            (q4 + pi/2, -q5, q6 + pi/2),
            (q4 - pi/2, -q5, q6 - pi/2),
        ]
        
        return solutions

    @staticmethod
    def _wrap_angles(q):
        """Wrap angles to [-π, π]."""
        return (q + pi) % (2 * pi) - pi

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()

    def inverse_kinematics(self, target_pose, q_guess=None):
        """Wrapper with diagnostics."""
        print("\n" + "="*60)
        print("IK DEBUG")
        print("="*60)
        
        p_tcp = target_pose[:3, 3]
        R_tcp = target_pose[:3, :3]
        
        print(f"Target TCP pos: {p_tcp}")
        
        # Compute wrist centre
        p_wrist = p_tcp - R_tcp @ self.d_tool
        print(f"Wrist centre: {p_wrist}")
        
        # Check reachability
        x, y, z = p_wrist
        r_xy = sqrt(x**2 + y**2)
        dz = z - self.shoulder_height
        d = sqrt(r_xy**2 + dz**2)
        max_reach = self.l_upper + self.l_forearm
        
        print(f"Distance from shoulder: {d:.4f} (max: {max_reach:.4f})")
        
        if d > max_reach:
            print("UNREACHABLE!")
        else:
            print("Reachable")
        
        result = self.solve_ik_for_tcp(target_pose, q_guess)
        
        if result is None:
            print("IK FAILED")
            # Let's manually check the arm
            arm_sols = self._solve_arm(p_wrist)
            print(f"Arm solutions found: {len(arm_sols)}")
            for q1, q2, q3 in arm_sols:
                wrist_sols = self._solve_wrist(target_pose, np.array([q1, q2, q3]))
                print(f"  Arm ({q1:.4f}, {q2:.4f}, {q3:.4f}): {len(wrist_sols)} wrist solutions")
                for q4, q5, q6 in wrist_sols:
                    q = np.array([q1, q2, q3, q4, q5, q6])
                    self.model.update_state(q)
                    T_check = self.model.get_tcp_pose()
                    pos_err = np.linalg.norm(T_check[:3, 3] - p_tcp)
                    rot_err = np.linalg.norm(T_check[:3, :3] - R_tcp)
                    print(f"    ({q4:.4f}, {q5:.4f}, {q6:.4f}): pos_err={pos_err:.4f}, rot_err={rot_err:.4f}")
        
        print("="*60 + "\n")
        return result
