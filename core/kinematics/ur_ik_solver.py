"""
ur_ik_solver.py - Parameterized analytical IK for UR-style robots.

CHANGED: Constructor now accepts alpha and theta_offset as optional
         parameters. Defaults reproduce the original UR behaviour
         exactly, so existing UR robots are unaffected.

NEW:     theta_offset handling in forward() and inverse().
         The relationship between DH angle θ and joint angle q is:
             θ_i = q_i + theta_offset_i
"""
import numpy as np
from numpy import linalg
from math import cos, sin, atan2, acos, asin, sqrt, pi
from typing import Optional, List


class URIKSolver:
    """UR-style IK solver with parameterized DH parameters."""

    def __init__(self,
                 d1: float = 0.1273,
                 a2: float = -0.612,
                 a3: float = -0.5723,
                 d4: float = 0.163941,
                 d5: float = 0.1157,
                 d6: float = 0.0922,
                 # NEW: optional parameters with UR defaults
                 alpha: Optional[List[float]] = None,
                 theta_offset: Optional[List[float]] = None):
        """
        Initialize IK solver with DH parameters.

        Args:
            d1, a2, a3, d4, d5, d6: Standard UR DH lengths.
            alpha: Twist angles. Default: UR convention.
                   [0, π/2, 0, 0, π/2, -π/2, 0]
            theta_offset: Joint zero offsets (DH angle minus URDF angle).
                   Default: all zeros (UR convention).
        """
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6

        self.d = np.array([0, d1, 0, 0, d4, d5, d6])
        self.a = np.array([0, 0, a2, a3, 0, 0, 0])

        # CHANGED: alpha is now a parameter with UR default
        if alpha is None:
            self.alpha = np.array([0, pi/2, 0, 0, pi/2, -pi/2, 0])
        else:
            self.alpha = np.array(alpha)

        # NEW: theta_offset handling
        if theta_offset is None:
            self.theta_offset = np.zeros(7)
        else:
            self.theta_offset = np.array(theta_offset)

    # =================================================================
    # Forward Kinematics
    # =================================================================

    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        Forward kinematics from joint angles q.
        Applies theta_offset: θ = q + offset.
        """
        # NEW: apply theta_offset
        theta = np.array(q, dtype=float) + self.theta_offset[1:7]
        return self._forward_theta(theta)

    def _forward_theta(self, theta: np.ndarray) -> np.ndarray:
        """
        Forward kinematics from DH angles theta.
        This is the raw FK without offset application.
        Used internally by _all_solutions for verification.
        """
        T_01 = self._T(1, theta)
        T_12 = self._T(2, theta)
        T_23 = self._T(3, theta)
        T_34 = self._T(4, theta)
        T_45 = self._T(5, theta)
        T_56 = self._T(6, theta)
        return T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56

    def _T(self, n: int, th: np.ndarray) -> np.ndarray:
        """
        Modified DH transformation matrix from frame n-1 to n.
        UNCHANGED from original.
        """
        return np.array([
            [cos(th[n-1]), -sin(th[n-1]), 0, self.a[n-1]],
            [sin(th[n-1]) * cos(self.alpha[n-1]),
             cos(th[n-1]) * cos(self.alpha[n-1]),
             -sin(self.alpha[n-1]),
             -sin(self.alpha[n-1]) * self.d[n]],
            [sin(th[n-1]) * sin(self.alpha[n-1]),
             cos(th[n-1]) * sin(self.alpha[n-1]),
             cos(self.alpha[n-1]),
             cos(self.alpha[n-1]) * self.d[n]],
            [0, 0, 0, 1]
        ])

    # =================================================================
    # Inverse Kinematics
    # =================================================================

    def inverse(self, T_target: np.ndarray,
                q_guess: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Inverse kinematics.
        Returns joint angles q (URDF convention).
        """
        # _all_solutions returns theta values (DH convention)
        solutions_theta = self._all_solutions(T_target)

        if not solutions_theta:
            return None

        # NEW: convert theta to q by subtracting theta_offset
        solutions_q = [
            theta - self.theta_offset[1:7]
            for theta in solutions_theta
        ]

        if q_guess is not None:
            best = None
            best_dist = float('inf')
            for q in solutions_q:
                diff = q - q_guess
                diff = (diff + pi) % (2 * pi) - pi
                dist = np.sum(np.abs(diff))
                if dist < best_dist:
                    best_dist = dist
                    best = q
            return best
        else:
            return solutions_q[0]

    def _all_solutions(self, T_06: np.ndarray) -> List[np.ndarray]:
        """
        Compute all 8 IK solutions in DH angle space (theta).
        CHANGED: alpha5_sign adjustment for theta 5 and theta 6.
        CHANGED: orientation verification added.
        """
        solutions = []

        # CHANGED: compute sign adjustment for alpha[5]
        # For UR (alpha[5] < 0): alpha5_sign = 1 (no change)
        # For FR5 (alpha[5] > 0): alpha5_sign = -1 (flip sign)
        alpha5_sign = -np.sign(self.alpha[5]) if abs(self.alpha[5]) > 1e-6 else 1.0

        P_05 = T_06 @ np.array([0, 0, -self.d6, 1])

        # ===== Theta 1 =====
        P_05y = P_05[1]
        P_05x = P_05[0]
        r = linalg.norm(P_05[0:2])
        if r < 1e-6:
            return solutions
        if r < self.d4 - 1e-6:
            return solutions

        phi_1 = atan2(P_05y, P_05x)
        if r < self.d4:
            return solutions
        phi_2 = acos(self.d4 / r)
        q1_left = phi_1 + phi_2 + pi / 2
        q1_right = phi_1 - phi_2 + pi / 2

        for q1_idx, q1 in enumerate([q1_left, q1_right]):
            # ===== Theta 5 =====
            P_06x = T_06[0, 3]
            P_06y = T_06[1, 3]
            P_16y = P_06x * sin(q1) - P_06y * cos(q1)

            numerator = P_16y - self.d4
            denominator = self.d6

            if abs(denominator) < 1e-10:
                continue

            # CHANGED: apply alpha5_sign ONLY to theta5
            cos_q5 = alpha5_sign * numerator / denominator

            if abs(cos_q5) > 1.0 + 1e-6:
                continue

            cos_q5 = np.clip(cos_q5, -1.0, 1.0)
            q5_1 = acos(cos_q5)
            q5_2 = -q5_1

            for q5 in [q5_1, q5_2]:
                # ===== Theta 6 =====
                # CHANGED: NO alpha5_sign here. Original equation unchanged.
                T_60 = linalg.inv(T_06)
                X_60x = T_60[0, 0]
                X_60y = T_60[1, 0]
                Y_60x = T_60[0, 1]
                Y_60y = T_60[1, 1]

                sin_q5 = sin(q5)
                if abs(sin_q5) < 1e-6:
                    continue

                term_1 = (Y_60y * cos(q1) - X_60y * sin(q1)) / sin_q5
                term_2 = (X_60x * sin(q1) - Y_60x * cos(q1)) / sin_q5
                q6 = atan2(term_1, term_2)

                # ===== Theta 3 =====
                q_vec = np.zeros(6)
                q_vec[0] = q1
                q_vec[4] = q5
                q_vec[5] = q6

                T_10 = linalg.inv(self._T(1, q_vec))
                T_65 = linalg.inv(self._T(6, q_vec))
                T_54 = linalg.inv(self._T(5, q_vec))
                T_14 = T_10 @ T_06 @ T_65 @ T_54

                P_14xz_sq = T_14[0, 3]**2 + T_14[2, 3]**2
                numerator = P_14xz_sq - self.a[2]**2 - self.a[3]**2
                denominator = 2 * self.a[2] * self.a[3]

                if abs(denominator) < 1e-6:
                    continue

                cos_q3 = np.clip(numerator / denominator, -1.0, 1.0)
                q3_1 = acos(cos_q3)
                q3_2 = -q3_1

                for q3 in [q3_1, q3_2]:
                    # ===== Theta 2 =====
                    P_14xz = sqrt(T_14[0, 3]**2 + T_14[2, 3]**2)
                    if P_14xz < 1e-6:
                        continue

                    operand = np.clip((-self.a[3] * sin(q3)) / P_14xz,
                                    -1.0, 1.0)
                    q2 = (atan2(-T_14[2, 3], -T_14[0, 3])
                        - asin(operand))

                    # ===== Theta 4 =====
                    q_vec[1] = q2
                    q_vec[2] = q3
                    T_32 = linalg.inv(self._T(3, q_vec))
                    T_21 = linalg.inv(self._T(2, q_vec))
                    T_34 = T_32 @ T_21 @ T_14
                    q4 = atan2(T_34[1, 0], T_34[0, 0])

                    q = np.array([q1, q2, q3, q4, q5, q6])
                    q = self._wrap_angles(q)

                    # CHANGED: verify both position AND orientation
                    T_verify = self._forward_theta(q)
                    pos_error = np.linalg.norm(
                        T_verify[:3, 3] - T_06[:3, 3]
                    )
                    rot_error = np.linalg.norm(
                        T_verify[:3, :3] - T_06[:3, :3]
                    )

                    if pos_error < 0.001 and rot_error < 0.01:
                        is_new = True
                        for existing in solutions:
                            if np.all(np.abs(q - existing) < 0.01):
                                is_new = False
                                break
                        if is_new:
                            solutions.append(q)

        return solutions

    def _wrap_angles(self, q: np.ndarray) -> np.ndarray:
        """Wrap angles to [-π, π]. UNCHANGED."""
        return (q + pi) % (2 * pi) - pi