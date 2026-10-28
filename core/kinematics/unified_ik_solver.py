"""
Unified inverse kinematics solver for 6R arms.
Derived from pure geometry: the tangent circle construction.

The rigid triangle A-B-C is fixed by the URDF.
The arm plane is perpendicular to the J2 axis (rotated by theta1).
The projection of C (P5) onto the arm plane is C_proj.
The circle is centered at C_proj with radius AC_length.
The tangent lines from O to this circle give the Wrist Root A.

No DH parameters. No arbitrary constants. One unified truth.

Boundary: true root to flange only. See docs/CONSTITUTION.md Part 3.5.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum
import logging

logger = logging.getLogger(__name__)

_ZERO_TOL = 1e-6
_PRECONDITION_TOL = 1e-4
_REACH_TOL = 1e-5
_SINGULARITY_TOL = 1e-6


class BranchTag(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"
    FLIP = "flip"
    NO_FLIP = "no-flip"


@dataclass
class IKSolution:
    q: np.ndarray
    tags: dict

    def __repr__(self) -> str:
        tags_str = ", ".join(f"{k}={v.value}" for k, v in self.tags.items())
        q_str = np.array2string(self.q, precision=4)
        return f"IKSolution(q={q_str}, tags=({tags_str}))"


class UnifiedIKSolver:
    def __init__(self, geometry):
        self.geom = geometry
        self._precheck()
        self._compute_parameters()

    def _precheck(self) -> None:
        if len(self.geom.axes) != 6:
            raise RuntimeError(f"UnifiedIKSolver requires exactly 6 joints, got {len(self.geom.axes)}.")

        twist_12 = self.geom.axis_twist(1)
        if not (abs(twist_12) < _PRECONDITION_TOL or abs(abs(twist_12) - np.pi) < _PRECONDITION_TOL):
            raise RuntimeError(
                f"Unsupported arm geometry: J2 and J3 are not parallel "
                f"(axis_twist(1) = {twist_12:.6f} rad)."
            )

        j5 = self.geom.axes[4]
        j6 = self.geom.axes[5]
        closest_gap = self._closest_approach(j5, j6)
        if closest_gap > _PRECONDITION_TOL:
            raise RuntimeError(
                f"Unsupported arm geometry: J5 and J6 do not intersect "
                f"(closest approach = {closest_gap:.6f} m)."
            )

    @staticmethod
    def _closest_approach(a, b) -> float:
        u = a.direction
        v = b.direction
        w = b.point - a.point
        n = np.cross(u, v)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9:
            return float(np.linalg.norm(w - np.dot(w, u) * u))
        return float(abs(np.dot(w, n / n_norm)))

    def _compute_parameters(self) -> None:
        self.base_height = float(self.geom.axes[1].point[2])
        self.upper_arm = float(self.geom.axis_gap(1))
        self.forearm_to_root = float(self.geom.axis_gap(2))
        
        # The rigid triangle A-B-C
        self.BC_length = float(self.geom.wrist_offset)
        self.AB_length = abs(float(self.geom.axes[3].point[1]))
        self.AC_length = np.sqrt(self.AB_length**2 + self.BC_length**2)
        
        self.flange_offset = float(self.geom.flange_offset)

        self.axis_dirs = [ax.direction.copy() for ax in self.geom.axes]

        if self.forearm_to_root < _ZERO_TOL:
            self.forearm_to_root = self.geom.forearm_length()

        logger.info(
            f"UnifiedIKSolver parameters: base_height={self.base_height:.4f}, "
            f"upper_arm={self.upper_arm:.4f}, forearm_to_root={self.forearm_to_root:.4f}, "
            f"AB_length={self.AB_length:.4f}, BC_length={self.BC_length:.4f}, "
            f"AC_length={self.AC_length:.4f}, flange_offset={self.flange_offset:.4f}"
        )

    def solve(self, T_flange: np.ndarray, q_current: Optional[np.ndarray] = None) -> List[IKSolution]:
        T_flange = np.asarray(T_flange, dtype=float)
        if T_flange.shape != (4, 4):
            raise ValueError(f"Expected 4x4 transform, got shape {T_flange.shape}")

        P5 = self._compute_p5(T_flange)
        solutions = self._build_solutions(T_flange, P5)

        if q_current is not None and solutions:
            solutions = self._sort_by_continuity(solutions, q_current)

        return solutions

    def select_best(self, T_flange: np.ndarray, q_current: np.ndarray) -> Optional[IKSolution]:
        solutions = self.solve(T_flange, q_current=q_current)
        if not solutions:
            return None
        return solutions[0]

    def _compute_p5(self, T_flange: np.ndarray) -> np.ndarray:
        P_flange = T_flange[:3, 3]
        a_flange = T_flange[:3, 2]
        return P_flange - self.flange_offset * a_flange

    def _build_solutions(self, T_flange: np.ndarray, P5: np.ndarray) -> List[IKSolution]:
        P_flange = T_flange[:3, 3]
        a_flange = T_flange[:3, 2]
        R_flange = T_flange[:3, :3]

        solutions = []
        O = np.array([0.0, 0.0, self.base_height])
        C = P5

        # The shoulder tangency: from O, draw tangents to the circle
        # of radius AC_length centered at P5.
        x5, y5 = P5[0], P5[1]
        r = float(np.sqrt(x5 * x5 + y5 * y5))
        phi = float(np.arctan2(y5, x5))

        if r < self.AC_length - _REACH_TOL:
            return []

        ratio = self.AC_length / r
        if ratio > 1.0 + _REACH_TOL:
            return []
        alpha = float(np.arcsin(max(-1.0, min(1.0, ratio))))

        shoulder_candidates = [
            (self._wrap(phi - alpha), BranchTag.RIGHT),
            (self._wrap(phi + alpha), BranchTag.LEFT),
        ]

        for theta1, shoulder_tag in shoulder_candidates:
            R_z = self._rodrigues(np.array([0.0, 0.0, 1.0]), theta1)
            n_hat = R_z @ self.axis_dirs[1]
            n_hat = n_hat / np.linalg.norm(n_hat)

            C_proj = C - np.dot(C - O, n_hat) * n_hat

            OC_proj = C_proj - O
            d = np.linalg.norm(OC_proj)

            if d < self.AC_length - _REACH_TOL:
                continue

            OT_length = np.sqrt(max(0.0, d**2 - self.AC_length**2))
            alpha = np.arcsin(min(1.0, self.AC_length / d)) if d > _ZERO_TOL else 0.0

            OC_hat = OC_proj / d if d > _ZERO_TOL else np.array([1.0, 0.0, 0.0])

            perp = np.cross(n_hat, OC_hat)
            perp_norm = np.linalg.norm(perp)
            if perp_norm < _ZERO_TOL:
                continue
            perp = perp / perp_norm

            for sign in [+1, -1]:
                T = O + OT_length * (np.cos(alpha) * OC_hat + sign * np.sin(alpha) * perp)
                A = T

                OA = A - O
                D_arm = np.linalg.norm(OA)

                if D_arm < _ZERO_TOL:
                    continue

                j3_origin = self.geom.axes[2].point
                ref_vec = j3_origin - O
                ref_vec = ref_vec - np.dot(ref_vec, n_hat) * n_hat
                ref_norm = np.linalg.norm(ref_vec)
                if ref_norm < _ZERO_TOL:
                    continue
                ref_hat = ref_vec / ref_norm

                psi = np.arctan2(np.dot(OA, np.cross(n_hat, ref_hat)), np.dot(OA, ref_hat))

                L1 = self.upper_arm
                L2 = self.forearm_to_root

                c3 = (D_arm**2 - L1**2 - L2**2) / (2 * L1 * L2)
                if abs(c3) > 1.0 + _REACH_TOL:
                    continue
                c3 = max(-1.0, min(1.0, c3))

                theta3_pos = np.arccos(c3)
                theta3_neg = -theta3_pos

                for theta3, elbow_tag in [(theta3_pos, BranchTag.UP), (theta3_neg, BranchTag.DOWN)]:
                    s3 = np.sin(theta3)
                    K1 = L1 + L2 * c3
                    K2 = L2 * s3

                    theta2 = psi - np.arctan2(K2, K1)
                    theta2 = self._wrap(theta2)
                    theta3 = self._wrap(theta3)

                    R03 = (
                        self._rodrigues(self.axis_dirs[0], theta1) @
                        self._rodrigues(self.axis_dirs[1], theta2) @
                        self._rodrigues(self.axis_dirs[2], theta3)
                    )
                    R_wrist = R03.T @ R_flange

                    r13 = R_wrist[0, 2]
                    r23 = R_wrist[1, 2]
                    r33 = R_wrist[2, 2]
                    r31 = R_wrist[2, 0]
                    r32 = R_wrist[2, 1]

                    sin5_sq = 1.0 - r33**2
                    if sin5_sq < 1e-12:
                        theta5 = 0.0 if r33 > 0 else np.pi
                        theta4 = 0.0
                        theta6 = np.arctan2(R_wrist[1, 0], R_wrist[0, 0]) if r33 > 0 else np.arctan2(-R_wrist[1, 0], R_wrist[0, 0])
                        wrist_solutions = [(theta4, theta5, theta6)]
                    else:
                        sin5 = np.sqrt(sin5_sq)
                        theta5 = np.arctan2(sin5, r33)
                        theta4 = np.arctan2(r23 / sin5, r13 / sin5)
                        theta6 = np.arctan2(r32 / sin5, -r31 / sin5)
                        wrist_solutions = [
                            (theta4, theta5, theta6),
                            (self._wrap(theta4 + np.pi), self._wrap(-theta5), self._wrap(theta6 + np.pi))
                        ]

                    for theta4, theta5, theta6 in wrist_solutions:
                        q = np.array([theta1, theta2, theta3, theta4, theta5, theta6], dtype=float)
                        tags = {
                            'shoulder': shoulder_tag,
                            'elbow': elbow_tag,
                            'wrist': BranchTag.NO_FLIP if theta5 >= 0 else BranchTag.FLIP
                        }

                        if not self._within_limits(q):
                            continue

                        solutions.append(IKSolution(q=q, tags=tags))

        return solutions

    def _within_limits(self, q: np.ndarray) -> bool:
        for i, (lower, upper) in enumerate(self.geom.joint_limits):
            theta = float(q[i])
            if lower <= theta <= upper:
                continue
            wrapped_plus = theta + 2 * np.pi
            wrapped_minus = theta - 2 * np.pi
            if lower <= wrapped_plus <= upper:
                q[i] = wrapped_plus
                continue
            if lower <= wrapped_minus <= upper:
                q[i] = wrapped_minus
                continue
            return False
        return True

    def _sort_by_continuity(self, solutions: List[IKSolution], q_current: np.ndarray) -> List[IKSolution]:
        q_current = np.asarray(q_current, dtype=float)
        current_tags = self._infer_tags(q_current)

        def rank(sol: IKSolution) -> Tuple[int, float]:
            matches = sum(1 for k in ('shoulder', 'elbow', 'wrist') if sol.tags.get(k) == current_tags.get(k))
            dist = float(np.sum(self._angle_diff(sol.q, q_current) ** 2))
            return (-matches, dist)

        return sorted(solutions, key=rank)

    def _infer_tags(self, q: np.ndarray) -> dict:
        tags = {}
        tags['wrist'] = BranchTag.NO_FLIP if q[4] >= 0 else BranchTag.FLIP
        tags['elbow'] = BranchTag.UP if q[2] >= 0 else BranchTag.DOWN
        tags['shoulder'] = BranchTag.RIGHT if abs(q[0]) < np.pi / 2 else BranchTag.LEFT
        return tags

    @staticmethod
    def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        c = np.cos(angle)
        s = np.sin(angle)
        x, y, z = axis

        return np.array([
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ])

    @staticmethod
    def _wrap(angle: float) -> float:
        return float((angle + np.pi) % (2 * np.pi) - np.pi)

    def _angle_diff(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        diff = a - b
        return (diff + np.pi) % (2 * np.pi) - np.pi