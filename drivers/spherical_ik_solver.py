"""
Spherical Wrist IK Solver - Analytical solution for robots where
the last three joint axes intersect at a single point.

Solves the position problem (joints 1-3) and orientation problem
(joints 4-6) separately.
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, asin, sqrt
from typing import Optional, List


class SphericalWristIKSolver:
    """
    Analytical IK solver for 6-DOF robots with spherical wrists.

    The wrist center (intersection of last three joint axes) is
    positioned by joints 1-3. The TCP orientation is achieved
    by joints 4-6.

    This solver requires D-H parameters specific to the robot.
    """

    def __init__(self, dh_params: List[dict]):
        """
        Initialize with D-H parameters.

        Args:
            dh_params: List of 6 dicts, each with:
                'a': link length (meters)
                'd': link offset (meters)
                'alpha': link twist (radians)
        """
        self.dh = dh_params

    def forward(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics: joint angles → TCP pose."""
        T = np.eye(4)
        for i, (dh, angle) in enumerate(zip(self.dh, q)):
            a, d, alpha = dh['a'], dh['d'], dh['alpha']
            T_i = np.array([
                [cos(angle), -sin(angle)*cos(alpha),  sin(angle)*sin(alpha), a*cos(angle)],
                [sin(angle),  cos(angle)*cos(alpha), -cos(angle)*sin(alpha), a*sin(angle)],
                [0,           sin(alpha),             cos(alpha),             d],
                [0,           0,                      0,                      1]
            ])
            T = T @ T_i
        return T

    def inverse(self, T_target: np.ndarray, q_guess: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Solve IK for target TCP pose.

        Args:
            T_target: 4x4 desired TCP pose in base frame.
            q_guess: Optional initial guess (for solution selection).

        Returns:
            6 joint angles, or None if unreachable.
        """
        solutions = self._solve_all(T_target)

        if not solutions:
            return None

        if q_guess is not None:
            # Return solution closest to q_guess
            best = None
            best_dist = float('inf')
            for q in solutions:
                diff = q - q_guess
                diff = (diff + pi) % (2*pi) - pi
                dist = np.sum(np.abs(diff))
                if dist < best_dist:
                    best_dist = dist
                    best = q
            return best

        return solutions[0]

    def _solve_all(self, T_06: np.ndarray) -> List[np.ndarray]:
        """
        Compute all IK solutions.

        Strategy:
        1. Find wrist center position from TCP pose
        2. Solve joints 1-3 to position the wrist center
        3. Solve joints 4-6 for the remaining orientation
        """
        # Wrist center: move back along TCP z-axis by d6
        d6 = self.dh[5]['d']
        wrist_center = T_06 @ np.array([0, 0, -d6, 1])

        solutions = []

        # ===== Solve Joint 1 =====
        x_wc, y_wc = wrist_center[0], wrist_center[1]

        for q1 in self._solve_q1(x_wc, y_wc):
            # ===== Solve Joints 2 and 3 =====
            # Transform wrist center to frame 1
            T_01 = self._dh_transform(0, q1)
            wrist_in_1 = np.linalg.inv(T_01) @ wrist_center
            x1, y1, z1 = wrist_in_1[0], wrist_in_1[1], wrist_in_1[2]

            for q2, q3 in self._solve_q2_q3(x1, y1, z1):
                # ===== Solve Joints 4, 5, 6 =====
                # Compute orientation from frame 3 to TCP
                T_03 = self._forward_to_joint([q1, q2, q3], 3)
                T_36 = np.linalg.inv(T_03) @ T_06

                for q4, q5, q6 in self._solve_wrist(T_36):
                    q = np.array([q1, q2, q3, q4, q5, q6])
                    q = self._wrap_angles(q)

                    # Verify
                    T_check = self.forward(q)
                    if np.allclose(T_check[:3, 3], T_06[:3, 3], atol=1e-3):
                        solutions.append(q)

        return solutions

    def _solve_q1(self, x: float, y: float) -> List[float]:
        """Solve for the base rotation that positions the wrist in XY plane."""
        r = sqrt(x*x + y*y)
        if r < 1e-6:
            return [0.0]  # Wrist center on Z axis, any q1 works

        d1 = self.dh[0]['d']
        if r < abs(d1) - 1e-6:
            return []  # Unreachable

        phi = atan2(y, x)
        # Two solutions: left/right
        offset = acos(d1 / r) if r >= abs(d1) else 0
        return [phi + offset - pi/2, phi - offset - pi/2]

    def _solve_q2_q3(self, x: float, y: float, z: float) -> List[tuple]:
        """Solve for elbow position (joints 2 and 3)."""
        a2 = self.dh[1]['a']
        a3 = self.dh[2]['a']

        # Distance from shoulder to wrist center
        r = sqrt(x*x + y*y + z*z)

        if r > a2 + a3 or r < abs(a2 - a3):
            return []  # Unreachable

        # Law of cosines
        cos_q3 = (r*r - a2*a2 - a3*a3) / (2*a2*a3)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)

        solutions = []
        for q3 in [acos(cos_q3), -acos(cos_q3)]:
            # Solve q2
            beta = atan2(z, sqrt(x*x + y*y))
            gamma = atan2(a3*sin(q3), a2 + a3*cos(q3))
            q2 = beta - gamma
            solutions.append((q2, q3))

        return solutions

    def _solve_wrist(self, T_36: np.ndarray) -> List[tuple]:
        """Solve wrist joints 4, 5, 6 from orientation matrix."""
        # Extract rotation matrix
        R = T_36[:3, :3]

        solutions = []

        # q5 from the (2,2) element
        cos_q5 = R[1, 2] if abs(R[1, 2]) <= 1.0 else np.sign(R[1, 2])

        for q5 in [acos(cos_q5), -acos(cos_q5)]:
            sin_q5 = sin(q5)
            if abs(sin_q5) < 1e-6:
                # Singularity: q4 and q6 are coupled
                q4 = 0.0
                q6 = atan2(R[2, 0], R[0, 0])
                solutions.append((q4, q5, q6))
            else:
                q4 = atan2(R[2, 2]/sin_q5, -R[0, 2]/sin_q5)
                q6 = atan2(-R[1, 0]/sin_q5, R[1, 1]/sin_q5)
                solutions.append((q4, q5, q6))

        return solutions

    def _forward_to_joint(self, q: List[float], n: int) -> np.ndarray:
        """Forward kinematics up to joint n (0-indexed)."""
        T = np.eye(4)
        for i in range(min(n, len(q))):
            T = T @ self._dh_transform(i, q[i])
        return T

    def _dh_transform(self, i: int, theta: float) -> np.ndarray:
        """D-H transformation matrix for joint i."""
        a = self.dh[i]['a']
        d = self.dh[i]['d']
        alpha = self.dh[i]['alpha']
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),             d],
            [0,           0,                      0,                      1]
        ])

    @staticmethod
    def _wrap_angles(q: np.ndarray) -> np.ndarray:
        """Wrap angles to [-π, π]."""
        return (q + pi) % (2*pi) - pi