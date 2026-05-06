"""
Geometric IK Solver for 6-DOF serial robots with spherical wrists.

Solves joints 1-3 geometrically to position the wrist center (joint 5).
Solves joints 4-6 as a spherical wrist for orientation.
Uses link lengths verified from the kinematic model's actual positions.

Principle #7: Movements as Models.
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt
from typing import Optional, List, Tuple
import logging

logger = logging.getLogger(__name__)


class GeometricIKSolver:
    """
    Geometric IK solver using verified link lengths.

    Link lengths (verified from joint frame positions at zero config):
    - Base to shoulder (J1/J2): 0.262m
    - Upper arm (J2 to J3/J4): 0.730m
    - Forearm (J3/J4 to wrist center J5/J6): 0.570m
    - Tool offset (wrist to TCP): 0.200m

    Wrist center is at J5 (intersection of joints 4,5,6 axes).
    """

    def __init__(self, kinematic_model):
        self.model = kinematic_model

        # Verified link lengths
        self.shoulder_height = 0.262
        self.l_upper = 0.730
        self.l_forearm = 0.570
        self.tool_length = 0.200

        # Key frames
        arm_chain = self.model.get_arm_chain(self.model.get_true_root())
        self.tcp_link = self.model.tool_mount_link
        
        # Joint 4's child link = wrist center frame
        joint4 = self.model.joints[arm_chain[3]]
        self.wrist_center_link = joint4['child']  # elfin_link5

        logger.info(f"Geometric IK: upper arm={self.l_upper:.3f}m, "
                   f"forearm={self.l_forearm:.3f}m, "
                   f"shoulder={self.shoulder_height:.3f}m, "
                   f"tool={self.tool_length:.3f}m")

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """
        Solve IK for target TCP pose.

        Args:
            target_pose: 4x4 desired TCP pose in base frame.
            q_guess: Initial joint guess for solution selection.

        Returns:
            6 joint angles, or None if no solution.
        """
        # Step 1: Compute wrist center (J5) from TCP
        p_tcp = target_pose[:3, 3]
        R_tcp = target_pose[:3, :3]
        tool_dir = R_tcp[:, 2]  # Z-axis of TCP = tool pointing direction
        p_wrist = p_tcp - tool_dir * self.tool_length

        # Step 2: Solve joints 1-3 geometrically
        q123_solutions = self._solve_arm(p_wrist)
        if not q123_solutions:
            return None

        # Step 3: Solve wrist orientation for each arm solution
        best_solution = None
        best_error = float('inf')

        for q1, q2, q3 in q123_solutions:
            q_wrist = self._solve_wrist(target_pose, np.array([q1, q2, q3]))
            if q_wrist is not None:
                q_full = np.array([q1, q2, q3, q_wrist[0], q_wrist[1], q_wrist[2]])
                q_full = self._wrap_angles(q_full)

                self.model.update_state(q_full)
                T_check = self.model.get_tcp_pose()
                pos_err = np.linalg.norm(T_check[:3, 3] - target_pose[:3, 3])

                if pos_err < 1e-3:
                    if q_guess is not None:
                        diff = q_full - q_guess
                        diff = (diff + pi) % (2*pi) - pi
                        dist = np.sum(np.abs(diff))
                        if dist < best_error:
                            best_error = dist
                            best_solution = q_full
                    else:
                        return q_full

        # At the end, before returning:
        result = best_solution  # or the first solution if q_guess is None
        print(f"[GeometricIK] Returning: {result}")
        return result

        # return best_solution

    def _solve_arm(self, wrist_center: np.ndarray) -> List[Tuple[float, float, float]]:
        """
        Solve joints 1-3 to position the wrist center.

        Joint 1: rotate the arm plane around base Z.
        Joints 2-3: law of cosines in the arm plane.
        """
        logger.info("[Solve Arm] just entered.")
        print(f"[Solve Arm] wrist_center=({wrist_center[0]:.4f}, {wrist_center[1]:.4f}, {wrist_center[2]:.4f})")

        x, y, z = wrist_center

        # ---- Joint 1: base rotation ----
        r_xy = sqrt(x**2 + y**2)
        q1 = atan2(y, x) if r_xy > 1e-6 else 0.0
        q1_candidates = [q1, q1 + pi] if r_xy > 1e-6 else [0.0]

        solutions = []

        for q1_val in q1_candidates:
            # Distance from shoulder to wrist center
            dz = z - self.shoulder_height
            d = sqrt(r_xy**2 + dz**2)

            # Law of cosines: triangle formed by upper arm, forearm, shoulder-to-wrist
            if d > self.l_upper + self.l_forearm or d < abs(self.l_upper - self.l_forearm):
                continue

            cos_q3 = (self.l_upper**2 + self.l_forearm**2 - d**2) / (2 * self.l_upper * self.l_forearm)
            cos_q3 = np.clip(cos_q3, -1.0, 1.0)

            for q3 in [acos(cos_q3), -acos(cos_q3)]:
                beta = atan2(r_xy, dz)
                gamma = atan2(self.l_forearm * sin(q3),
                            self.l_upper + self.l_forearm * cos(q3))
                q2 = beta - gamma

                solutions.append((q1_val, q2, q3))

        print(f"[Solve Arm] Found {len(solutions)} solutions")
        return solutions

    def _solve_wrist(self, T_target, q123):
        """Solve wrist joints 4-6 numerically."""
        from scipy.spatial.transform import Rotation as R

        # Start from current wrist state, not zeros
        current_joints = self.model.get_current_joint_positions()
        if len(current_joints) >= 6:
            q_wrist = np.array(current_joints[3:6], dtype=float)
        else:
            q_wrist = np.zeros(3)

        for iteration in range(30):
            q_full = np.concatenate([q123, q_wrist])
            self.model.update_state(q_full)
            T_current = self.model.get_tcp_pose()
            
            pos_err = T_target[:3, 3] - T_current[:3, 3]
            R_err_mat = T_target[:3, :3] @ T_current[:3, :3].T
            ori_err = R.from_matrix(R_err_mat).as_rotvec()
            error = np.concatenate([pos_err, ori_err])
            error_norm = np.linalg.norm(error)
            
            if iteration == 0:
                print(f"[Wrist] iter 0: pos_err={np.linalg.norm(pos_err):.4f}, ori_err={np.linalg.norm(ori_err):.4f}")
                print(f"[Wrist] q_wrist start: {q_wrist}")

            if error_norm < 1e-6:
                break

            J = np.zeros((6, 3))
            delta = 1e-4
            p_current = T_current[:3, 3]
            R_current = T_current[:3, :3]

            for i in range(3):
                q_pert = q_wrist.copy()
                q_pert[i] += delta
                q_full_pert = np.concatenate([q123, q_pert])
                self.model.update_state(q_full_pert)
                T_pert = self.model.get_tcp_pose()
                J[:3, i] = (T_pert[:3, 3] - p_current) / delta
                R_diff = T_pert[:3, :3] @ R_current.T
                J[3:, i] = R.from_matrix(R_diff).as_rotvec() / delta

            if iteration == 0:
                print(f"[Wrist] Jacobian:\n{J}")

            lam = 0.1
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), error)
                if iteration == 0:
                    print(f"[Wrist] dq: {dq}")
                    print(f"[Wrist] dq norm: {np.linalg.norm(dq):.6f}")
            except np.linalg.LinAlgError:
                print(f"[Wrist] Singular Jacobian at iter {iteration}")
                break
            
            q_wrist = q_wrist + dq[:3]
            if iteration == 0:
                print(f"[Wrist] q_wrist after update: {q_wrist}")

        print(f"[Wrist] Final q_wrist: {q_wrist}")
        print(f"[Wrist] Final error: {error_norm:.6f}")

        q_full = np.concatenate([q123, q_wrist])
        self.model.update_state(q_full)
        T_check = self.model.get_tcp_pose()
        if np.linalg.norm(T_check[:3, 3] - T_target[:3, 3]) < 1e-3:
            return q_wrist
        return None

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        self.model.update_state(q)
        return self.model.get_tcp_pose()

    @staticmethod
    def _wrap_angles(q: np.ndarray) -> np.ndarray:
        return (q + pi) % (2*pi) - pi