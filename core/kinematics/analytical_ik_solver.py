"""
Analytical IK Solver for spherical wrist robots.

Uses Pieper decoupling: position the wrist center with joints 1-3,
then orient the tool with joints 4-6.

All geometric parameters measured from the model's FK at init.
No D-H parameters. No numerical iteration for the arm.

Principle #7: Movements as Models.
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt
from typing import Optional, List, Tuple
import logging

logger = logging.getLogger(__name__)


class AnalyticalIKSolver:
    """
    Analytical IK for spherical wrist robots.

    Measures link lengths and tool offset from the model at startup.
    Solves joints 1-3 geometrically (law of cosines).
    Solves joints 4-6 by decomposing the remaining rotation.
    """

    def __init__(self, kinematic_model):
        self.model = kinematic_model

        # Measure all geometry from the model at zero configuration
        self._measure_geometry()

        logger.info(f"Analytical IK: upper arm={self.l_upper:.4f}m, "
                   f"forearm={self.l_forearm:.4f}m, "
                   f"shoulder={self.shoulder_height:.4f}m, "
                   f"tool_offset={self.tool_offset:.4f}m")

    def _measure_geometry(self):
        """Measure all geometric parameters from the model."""
        q_zero = np.zeros(6)
        self.model.update_state(q_zero)

        # Identify key frames from the arm chain
        arm_chain = self.model.get_arm_chain(self.model.get_true_root())

        # Start from the true base (before any moving joints)
        true_root = self.model.get_true_root()
        T_base = self.model.link_transforms[true_root]
        base_pos = T_base[:3, 3]

        # Joint positions at zero configuration, starting from the true base
        positions = [base_pos]  # Position 0 = true base
        
        current_link = true_root
        for joint_name in arm_chain:
            joint = self.model.joints[joint_name]
            child_link = joint['child']
            T = self.model.link_transforms[child_link]
            positions.append(T[:3, 3])
            current_link = child_link

        # Now positions[0] = base, positions[1] = J1 child, positions[2] = J2 child, ...
        # Joint i's position is positions[i+1] (since positions[0] is base)
        
        self.shoulder_height = np.linalg.norm(positions[2] - positions[0])  # J2 - base
        self.l_upper = np.linalg.norm(positions[3] - positions[2])          # J3 - J2
        self.l_forearm = np.linalg.norm(positions[6] - positions[3])        # J6 - J3

        # Tool offset: TCP to wrist center distance
        T_tcp = self.model.get_tcp_pose()
        T_wrist = self.model.link_transforms[self.model.joints[arm_chain[5]]['child']]
        offset_world = T_tcp[:3, 3] - T_wrist[:3, 3]
        self.tool_offset = np.linalg.norm(offset_world)

        # Wrist center link
        self.wrist_center_link = self.model.joints[arm_chain[5]]['child']

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
        # Step 1: Compute wrist center from target TCP
        p_tcp = target_pose[:3, 3]
        R_tcp = target_pose[:3, :3]
        tool_dir = R_tcp[:, 2]  # Z-axis = tool direction
        p_wrist = p_tcp - tool_dir * self.tool_offset

        print(f"[Analytical] TCP=({p_tcp[0]:.4f},{p_tcp[1]:.4f},{p_tcp[2]:.4f})")
        print(f"[Analytical] tool_dir=({tool_dir[0]:.4f},{tool_dir[1]:.4f},{tool_dir[2]:.4f})")
        print(f"[Analytical] tool_offset={self.tool_offset:.4f}")
        print(f"[Analytical] wrist=({p_wrist[0]:.4f},{p_wrist[1]:.4f},{p_wrist[2]:.4f})")
        print(f"[Analytical] shoulder={self.shoulder_height:.4f}, upper={self.l_upper:.4f}, forearm={self.l_forearm:.4f}")

        # Step 2: Solve joints 1-3 geometrically
        q123_solutions = self._solve_arm(p_wrist)

        if arm_sols:
            q1, q2, q3 = arm_sols[0]
            q_test = np.array([q1, q2, q3, 0, 0, 0])
            self.model.update_state(q_test)
            
            # Get actual link5 position from FK
            T_link5 = self.model.link_transforms['elfin_link5']
            p_wrist_achieved = T_link5[:3, 3]
            
            print(f"\n=== ARM SOLUTION VERIFICATION ===")
            print(f"Arm solution: q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}")
            print(f"Target wrist centre: {p_wrist}")
            print(f"Achieved link5 position: {p_wrist_achieved}")
            print(f"Wrist position error: {np.linalg.norm(p_wrist_achieved - p_wrist):.6f}")
            
            # Also check link3 (forearm) to understand the arm plane
            T_link3 = self.model.link_transforms['elfin_link3']
            print(f"Link3 (forearm) position: {T_link3[:3, 3]}")
            print(f"Link3 Z-axis: {T_link3[:3, 2]}")
            
            # And TCP
            T_tcp = self.model.get_tcp_pose()
            print(f"TCP position (J4-J6=0): {T_tcp[:3, 3]}")
            print(f"TCP Z-axis (J4-J6=0): {T_tcp[:3, 2]}")
            print(f"===================================\n")

        print(f"[Analytical] Arm solutions: {len(q123_solutions)}")
        if not q123_solutions:
            return None

        # Step 3: Solve joints 4-6 for each arm solution
        best_solution = None
        best_dist = float('inf')

        for q1, q2, q3 in q123_solutions:
            q_wrist_solutions = self._solve_wrist(
                target_pose, np.array([q1, q2, q3])
            )
            for q4, q5, q6 in q_wrist_solutions:
                q = np.array([q1, q2, q3, q4, q5, q6])
                q = self._wrap_angles(q)

                # Verify against model FK
                self.model.update_state(q)
                T_check = self.model.get_tcp_pose()
                pos_err = np.linalg.norm(T_check[:3, 3] - target_pose[:3, 3])
                print(f"[Verify] q=({q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f},{q[4]:.4f},{q[5]:.4f}) pos_err={pos_err:.6f}")

                if pos_err < 1e-3:
                    if q_guess is not None:
                        diff = q - q_guess
                        diff = (diff + pi) % (2*pi) - pi
                        dist = np.sum(np.abs(diff))
                        if dist < best_dist:
                            best_dist = dist
                            best_solution = q.copy()
                    else:
                        return q

        return best_solution

    def _solve_arm(self,
                   wrist_center: np.ndarray) -> List[Tuple[float, float, float]]:
        """
        Solve joints 1-3 to position the wrist center.

        Joint 1: rotate the arm plane.
        Joints 2-3: law of cosines in the arm plane.
        """
        x, y, z = wrist_center

        # Joint 1: base rotation
        r_xy = sqrt(x**2 + y**2)
        q1 = -atan2(y, x) if r_xy > 1e-6 else 0.0
        q1_candidates = [q1, q1 + pi] if r_xy > 1e-6 else [0.0]

        solutions = []

        for q1_val in q1_candidates:
            dz = z - self.shoulder_height
            d = sqrt(r_xy**2 + dz**2)

            if d > self.l_upper + self.l_forearm or d < abs(self.l_upper - self.l_forearm):
                continue

            cos_q3 = (self.l_upper**2 + self.l_forearm**2 - d**2) / (
                2 * self.l_upper * self.l_forearm
            )
            cos_q3 = np.clip(cos_q3, -1.0, 1.0)

            for q3 in [acos(cos_q3), -acos(cos_q3)]:
                beta = atan2(r_xy, dz)
                gamma = atan2(
                    self.l_forearm * sin(q3),
                    self.l_upper + self.l_forearm * cos(q3)
                )
                q2 = beta - gamma

                solutions.append((q1_val, q2, q3))

        return solutions

    def numeric_solve_wrist(self, T_target, q123):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation as R

        pos_target = T_target[:3, 3]
        rot_target = R.from_matrix(T_target[:3, :3])

        def wrist_error(q_wrist):
            q_full = np.concatenate([q123, q_wrist])
            self.model.update_state(q_full)
            T = self.model.get_tcp_pose()
            pos_err = pos_target - T[:3, 3]
            rot_err = (rot_target * R.from_matrix(T[:3, :3]).inv()).as_rotvec()
            return np.concatenate([pos_err, rot_err])

        q_wrist_guess = np.zeros(3)
        
        res = least_squares(
            wrist_error,
            q_wrist_guess,
            method='lm',
            max_nfev=50,
            xtol=1e-10,
            ftol=1e-10,
        )

        print(f"[Wrist] success={res.success}, nfev={res.nfev}, "
            f"cost={res.cost:.6f}, optimality={res.optimality:.6f}")
        print(f"[Wrist] q_wrist={res.x}")

        # Verify
        q_full = np.concatenate([q123, res.x])
        self.model.update_state(q_full)
        T_check = self.model.get_tcp_pose()
        pos_err = np.linalg.norm(T_check[:3, 3] - pos_target)
        print(f"[Wrist] Final pos_err={pos_err:.6f}")

        q4, q5, q6 = res.x
        return [(q4, q5, q6), (q4 + np.pi, -q5, q6 + np.pi)]

    def _solve_wrist(self,
                     T_target: np.ndarray,
                     q123: np.ndarray) -> List[Tuple[float, float, float]]:
        """
        Solve wrist joints 4-6 for orientation.

        With the arm fixed, the remaining rotation from wrist frame
        to target TCP is decomposed into the three wrist joint angles.
        Uses the model's FK to get the wrist frame orientation.
        """
        from scipy.spatial.transform import Rotation as R

        # Set arm joints, zero wrist
        q_full = np.concatenate([q123, np.zeros(3)])
        self.model.update_state(q_full)

        # Get wrist frame orientation (at J6)
        T_wrist = self.model.link_transforms[self.wrist_center_link]
        R_wrist = T_wrist[:3, :3]
        R_target = T_target[:3, :3]

        # Remaining rotation: R_wrist @ R_wrist_joints = R_target
        R_needed = R_wrist.T @ R_target

        # Decompose into ZYZ Euler angles
        r = R.from_matrix(R_needed)
        euler = r.as_euler('ZYZ', degrees=False)
        # euler = r.as_euler('ZXZ', degrees=False)

        q4, q5, q6 = euler[0], -euler[1], euler[2]

        # Two solutions: elbow equivalent for the wrist
        solutions = [
            (q4, q5, q6),
            (q4 + pi, -q5, q6 + pi)
        ]

        for label, (q4_val, q5_val, q6_val) in [
            ("sol1", solutions[0]), ("sol2", solutions[1])
        ]:
            q_full = np.concatenate([q123, [q4_val, q5_val, q6_val]])
            self.model.update_state(q_full)
            T_check = self.model.get_tcp_pose()
            pos_err = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
            print(f"[Wrist {label}] q=({q4_val:.4f},{q5_val:.4f},{q6_val:.4f}) pos_err={pos_err:.4f}")

        return solutions

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()

    @staticmethod
    def _wrap_angles(q: np.ndarray) -> np.ndarray:
        """Wrap angles to [-π, π]."""
        return (q + pi) % (2*pi) - pi