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


class HanE15ProIK_Simple:
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

    def _solve_arm(self, p_wrist):
        """
        Solve J1, J2, J3 geometrically.
        J1 rotates arm to point toward wrist centre (about world Z).
        J2 and J3 move arm in the XZ plane (after J1 rotation).
        Zero config: J2=-Y, J3=+Y.
        """
        x, y, z = p_wrist
        
        # J1: Base rotation about world Z
        r_xy = sqrt(x**2 + y**2)
        if r_xy < 1e-6:
            q1_options = [0.0]
        else:
            q1_base = atan2(y, x)
            q1_options = [q1_base, q1_base + pi]
        
        # Distance from shoulder (J2) to wrist centre
        # J2 is at [0, 0, shoulder_height] in world frame
        dz = z - self.shoulder_height
        d = sqrt(r_xy**2 + dz**2)
        
        max_reach = self.l_upper + self.l_forearm  # 0.730 + 0.570 = 1.300
        min_reach = abs(self.l_upper - self.l_forearm)  # |0.730 - 0.570| = 0.160
        
        logger.debug(f"Arm: r_xy={r_xy:.4f}, dz={dz:.4f}, d={d:.4f}")
        logger.debug(f"Reach: [{min_reach:.4f}, {max_reach:.4f}]")
        
        if d > max_reach + 1e-6 or d < min_reach - 1e-6:
            logger.debug(f"Arm unreachable: d={d:.4f}")
            return []
        
        solutions = []
        for q1 in q1_options:
            # In the arm plane (XZ after J1 rotation):
            # horizontal distance = r_xy, vertical = dz
            beta = atan2(dz, r_xy)  # Angle from horizontal to wrist line
            
            # Law of cosines for elbow angle (J3)
            cos_q3 = (self.l_upper**2 + self.l_forearm**2 - d**2) / \
                     (2 * self.l_upper * self.l_forearm)
            cos_q3 = np.clip(cos_q3, -1.0, 1.0)
            
            for q3 in [acos(cos_q3), -acos(cos_q3)]:
                # Angle from upper arm to wrist line
                alpha = atan2(self.l_forearm * sin(q3),
                            self.l_upper + self.l_forearm * cos(q3))
                q2 = beta - alpha
                solutions.append((q1, q2, q3))
                logger.debug(f"Arm solution: q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}")
        
        return solutions

    def _solve_wrist(self, T_target, q123):
        """
        Analytic wrist solution using ZYZ Euler decomposition.
        Zero config: J4=+Z, J5=+Y, J6=+Z in forearm frame.
        R_wrist(q4,q5,q6) = Rz(q4) @ Ry(q5) @ Rz(q6)
        """
        from scipy.spatial.transform import Rotation as R
        
        R_tcp_target = T_target[:3, :3]
        
        # Get forearm (link3) orientation after setting J1-J3
        q_full = np.concatenate([q123, np.zeros(3)])
        self.model.update_state(q_full)
        T_forearm = self.model.link_transforms[self.wrist_parent_link]
        R_forearm = T_forearm[:3, :3]
        
        # Rotation needed from forearm to TCP:
        # R_tcp_target = R_forearm @ R_wrist @ I  (R_tool_const = I for end link)
        # R_wrist = R_forearm^T @ R_tcp_target
        R_wrist_needed = R_forearm.T @ R_tcp_target
        
        # Decompose as ZYZ Euler angles (J4=Z, J5=Y, J6=Z)
        r = R.from_matrix(R_wrist_needed)
        
        try:
            q4, q5, q6 = r.as_euler('ZYZ', degrees=False)
        except Exception:
            # Fallback to ZXZ if ZYZ fails
            logger.debug("ZYZ failed, trying ZXZ")
            q4, q5, q6 = r.as_euler('ZXZ', degrees=False)
        
        logger.debug(f"Wrist ZYZ: q4={q4:.4f}, q5={q5:.4f}, q6={q6:.4f}")
        
        # Two solutions (wrist flip):
        # Solution 1: (q4, q5, q6)
        # Solution 2: (q4+π, -q5, q6+π)
        return [
            (q4, -q5, q6),
            (q4 + pi, q5, q6 + pi)
        ]

    @staticmethod
    def _wrap_angles(q):
        """Wrap angles to [-π, π]."""
        return (q + pi) % (2 * pi) - pi

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()
