"""
Spherical Wrist IK Solver – Standard D-H Convention (Craig's original)

Solves IK analytically for 6‑DOF robots with a spherical wrist, where the
last three joint axes intersect at a single point. Uses the standard
Denavit‑Hartenberg convention: RotZ(θ) · TransZ(d) · TransX(a) · RotX(α).

Parameters are extracted from the URDF's zero‑configuration transforms,
so theta offsets are all zero.

Principle #7: Movements as Models. IK is a pure function of geometry.
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt
from typing import Optional, List


class SphericalWristIKSolver:
    """Analytical IK solver for spherical wrist robots using standard D‑H."""

    def __init__(self, dh_params: List[dict]):
        if len(dh_params) != 6:
            raise ValueError(f"Expected 6 D‑H parameter sets, got {len(dh_params)}")
        self.dh = dh_params

    def forward(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using standard D‑H convention."""
        T = np.eye(4)
        for i, theta in enumerate(q):
            T = T @ self._dh_transform(i, theta)
        return T

    def _dh_transform(self, i: int, theta: float) -> np.ndarray:
        """Standard D‑H transformation: RotZ(θ)·TransZ(d)·TransX(a)·RotX(α)."""
        a = self.dh[i]['a']
        d = self.dh[i]['d']
        alpha = self.dh[i]['alpha']
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])

    def inverse(self,
                T_target: np.ndarray,
                q_guess: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Solve IK. Returns closest solution to q_guess if provided."""
        solutions = self._solve_all(T_target)
        if not solutions:
            return None

        if q_guess is not None:
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
        """Compute all IK solutions (up to 8)."""
        d1 = self.dh[0]['d']
        a2 = self.dh[1]['a']
        d4 = self.dh[3]['d']
        d6 = self.dh[5]['d']

        # Wrist centre: step back along TCP Z-axis by d6
        wrist_centre = T_06 @ np.array([0, 0, -d6, 1])
        x_wc, y_wc, z_wc = wrist_centre[0], wrist_centre[1], wrist_centre[2]

        solutions = []

        # ---- Solve theta1 ----
        r_xy = sqrt(x_wc**2 + y_wc**2)
        if r_xy < 1e-4:
            theta1_candidates = [0.0]
        elif r_xy < abs(d1) - 1e-6:
            return []
        else:
            phi = atan2(y_wc, x_wc)
            offset = acos(np.clip(d1 / r_xy, -1.0, 1.0))
            theta1_candidates = [
                phi + offset - pi/2,
                phi - offset - pi/2
            ]

        for t1 in theta1_candidates:
            T_01 = self._dh_transform(0, t1)
            wc_in_1 = np.linalg.inv(T_01) @ wrist_centre
            x1, y1, z1 = wc_in_1[0], wc_in_1[1], wc_in_1[2]

            # Full 3D distance from shoulder to wrist centre
            r = sqrt(x1**2 + y1**2 + z1**2)

            if r > abs(a2) + d4 or r < abs(abs(a2) - d4):
                continue

            # ---- Solve theta3 (elbow up/down) ----
            cos_t3 = (r**2 - a2**2 - d4**2) / (2 * a2 * d4)
            cos_t3 = np.clip(cos_t3, -1.0, 1.0)

            for t3 in [acos(cos_t3), -acos(cos_t3)]:
                # ---- Solve theta2 ----
                beta = atan2(z1, x1)
                gamma = atan2(d4 * sin(t3), abs(a2) + d4 * cos(t3))
                t2 = beta - gamma

                # Compute transform up to joint 3
                q123 = [t1, t2, t3]
                T_03 = self._forward_to_joint(q123, 3)
                T_36 = np.linalg.inv(T_03) @ T_06

                # ---- Solve wrist (theta4, theta5, theta6) ----
                for t4, t5, t6 in self._solve_wrist(T_36):
                    q = np.array([t1, t2, t3, t4, t5, t6])
                    q = self._wrap_angles(q)

                    T_check = self.forward(q)
                    if np.allclose(T_check[:3, 3], T_06[:3, 3], atol=1e-3):
                        solutions.append(q)

        return solutions

    def _solve_wrist(self, T_36: np.ndarray) -> List[tuple]:
        """Solve wrist joints from orientation matrix."""
        R = T_36[:3, :3]
        solutions = []

        cos_t5 = np.clip(R[1, 2], -1.0, 1.0)
        for t5 in [acos(cos_t5), -acos(cos_t5)]:
            sin_t5 = sin(t5)
            if abs(sin_t5) < 1e-6:
                t4 = 0.0
                t6 = atan2(R[2, 0], R[0, 0])
                solutions.append((t4, t5, t6))
            else:
                t4 = atan2(R[2, 2]/sin_t5, -R[0, 2]/sin_t5)
                t6 = atan2(-R[1, 0]/sin_t5, R[1, 1]/sin_t5)
                solutions.append((t4, t5, t6))

        return solutions

    def _forward_to_joint(self, q: List[float], n: int) -> np.ndarray:
        """Forward kinematics up to joint n (0-indexed)."""
        T = np.eye(4)
        for i in range(min(n, len(q))):
            T = T @ self._dh_transform(i, q[i])
        return T

    @staticmethod
    def _wrap_angles(q: np.ndarray) -> np.ndarray:
        """Wrap angles to [-π, π]."""
        return (q + pi) % (2*pi) - pi