"""
Unified inverse kinematics solver for arm-plane 6R arms.

Solves for the joint angles that place the flange at a given pose, using
the geometric derivation in docs/unified_IK.md. Handles both spherical
wrist arms (wrist_offset = 0) and offset wrist arms (UR, FR).

Input:  flange pose in the true-root frame (4x4 homogeneous transform).
Output: list of tagged joint solutions, up to 8 per pose.

The caller selects a solution by continuity of branch (shoulder, elbow,
wrist tags matching the current configuration). Proximity-based selection
is a fallback, not the primary criterion.

This solver does NOT handle:
- Arms where J2 and J3 are not parallel (no arm plane).
- Arms where J5 and J6 do not intersect (no wrist center).
- Arms with more or fewer than 6 revolute joints.

For robots outside this class, the solver raises RuntimeError at
construction time. See docs/CONSTITUTION.md Appendix B.

Principles: #4 (Everything in URDF), #7 (Movements as Models).
Boundary: true root to flange only. See docs/CONSTITUTION.md Part 3.5.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum

import logging
logger = logging.getLogger(__name__)


# Tolerance for treating a quantity as exactly zero.
# Below this, the spherical / degenerate formula applies; above, the
# general (offset) formula. See unified_IK.md Part IV §8.
_ZERO_TOL = 1e-6

# Tolerance for the precondition checks (J2 || J3, J5 intersects J6).
_PRECONDITION_TOL = 1e-4

# Tolerance for reachability checks (|cos| <= 1).
_REACH_TOL = 1e-5

# Tolerance for the wrist singularity (sin θ5 ≈ 0).
_SINGULARITY_TOL = 1e-6


# ----------------------------------------------------------------------
# Data types
# ----------------------------------------------------------------------

class BranchTag(str, Enum):
    """Branch tags carried by each solution."""
    # Shoulder
    LEFT = "left"
    RIGHT = "right"
    # Elbow
    UP = "up"
    DOWN = "down"
    # Wrist
    FLIP = "flip"
    NO_FLIP = "no-flip"


@dataclass
class IKSolution:
    """
    One tagged joint solution.

    q:      the 6-vector of joint angles, in arm chain order.
    tags:   dict of branch tags, keys are 'shoulder', 'elbow', 'wrist'.
            Values are BranchTag members.

    Two solutions may have the same joint vector but different tags in
    edge cases (e.g., when a joint angle coincides at a singular pose).
    Both are preserved so that branch continuity is well-defined even at
    singularities.
    """
    q: np.ndarray
    tags: dict

    def __repr__(self) -> str:
        tags_str = ", ".join(f"{k}={v.value}" for k, v in self.tags.items())
        q_str = np.array2string(self.q, precision=4)
        return f"IKSolution(q={q_str}, tags=({tags_str}))"


# ----------------------------------------------------------------------
# Solver
# ----------------------------------------------------------------------

class UnifiedIKSolver:
    """
    Analytical IK solver for arm-plane 6R arms.

    Constructs from an ArmGeometry (from geometric_extraction.py) and
    precomputes the solver parameters from the geometry: d1, a2, a3, d6,
    wrist_offset, and the base-frame axis directions.

    Raises RuntimeError if the arm is not in the supported class.
    """

    def __init__(self, geometry):
        """
        Args:
            geometry: ArmGeometry instance from extract_arm_geometry().

        Raises:
            RuntimeError: if the arm is not in the supported class
                (J2/J3 not parallel, J5/J6 do not intersect, wrong
                number of joints).
        """
        self.geom = geometry
        self._precheck()
        self._compute_parameters()

    # ------------------------------------------------------------------
    # Precondition checks (Step 0)
    # ------------------------------------------------------------------

    def _precheck(self) -> None:
        """Verify the arm is in the supported class. Refuse loudly."""
        if len(self.geom.axes) != 6:
            raise RuntimeError(
                f"UnifiedIKSolver requires exactly 6 joints, got "
                f"{len(self.geom.axes)}."
            )

        # J2 and J3 must be parallel (twist ≈ 0 or ≈ π).
        twist_12 = self.geom.axis_twist(1)
        if not (abs(twist_12) < _PRECONDITION_TOL or
                abs(twist_12 - np.pi) < _PRECONDITION_TOL):
            raise RuntimeError(
                f"Unsupported arm geometry: J2 and J3 are not parallel "
                f"(axis_twist(1) = {twist_12:.6f} rad, expected 0 or pi). "
                f"The analytical solver requires an arm-plane 6R arm."
            )

        # J5 and J6 must intersect.
        # The correct measure is the perpendicular distance between the two
        # axes (lines), not between their origins. Use the common-perpendicular
        # midpoint from the extraction layer.
        j5 = self.geom.axes[4]
        j6 = self.geom.axes[5]
        closest_gap = self._closest_approach(j5, j6)
        if closest_gap > _PRECONDITION_TOL:
            raise RuntimeError(
                f"Unsupported arm geometry: J5 and J6 do not intersect "
                f"(closest approach = {closest_gap:.6f} m). "
                f"The analytical solver requires intersecting wrist axes."
            )

    @staticmethod
    def _closest_approach(a, b) -> float:
        """
        Perpendicular distance between two lines (axes).

        For intersecting lines, this is 0.
        For parallel lines, this is the distance between the parallel lines.
        For skew lines, this is the length of the common perpendicular.
        """
        u = a.direction
        v = b.direction
        w = b.point - a.point
        n = np.cross(u, v)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9:
            # Parallel: perpendicular component of w.
            return float(np.linalg.norm(w - np.dot(w, u) * u))
        return float(abs(np.dot(w, n / n_norm)))

    # ------------------------------------------------------------------
    # Parameter extraction
    # ------------------------------------------------------------------

    def _compute_parameters(self) -> None:
        """
        Extract the scalar parameters used by the derivation from the
        ArmGeometry. These are cached once per solver instance.

        See unified_IK.md Parts IV-VI for the meaning of each parameter.
        """
        # d1: shoulder height. The Z coordinate of J2's axis origin,
        # in the true-root frame. The true root is usually at the base,
        # so this is the base-to-shoulder distance along the base axis.
        self.d1 = float(self.geom.axes[1].point[2])

        # a2: upper arm length. axis_gap(1) is the perpendicular distance
        # between J2 and J3 axes. For a parallel-axis pair, this is the
        # link length.
        self.a2 = float(self.geom.axis_gap(1))

        # a3: forearm length. distance from J3's axis to P5.
        # NOT axis_gap(2). See unified_IK.md Part VI §3.
        self.a3 = float(self.geom.forearm_length())

        # d6: flange offset. distance from P5 to the flange.
        self.d6 = float(self.geom.flange_offset)

        # wrist_offset: perpendicular distance from J4's axis to P5.
        # Zero for spherical wrists, nonzero for offset wrists.
        self.wrist_offset = float(self.geom.wrist_offset)

        # Base-frame axis directions. Used to compute R03 (Step 5).
        self.axis_dirs = [ax.direction.copy() for ax in self.geom.axes]

        # --- Calibrate theta2 reference ---
        # Position of J3 at zero config, relative to shoulder.
        u0 = self.geom.axes[2].point[0]
        w0 = self.geom.axes[2].point[2] - self.d1
        
        self.ref_angle = float(np.arctan2(w0, u0))

        # Determine sign: does positive q2 increase or decrease psi?
        # Velocity of J3 when q2 increases is cross(a2_dir, vec_shoulder_to_J3).
        vec3 = np.array([u0, 0.0, w0])
        v = np.cross(self.axis_dirs[1], vec3)
        
        eps = 1e-4
        psi0 = np.arctan2(w0, u0)
        psi_pert = np.arctan2(w0 + v[2] * eps, u0 + v[0] * eps)
        
        diff = (psi_pert - psi0 + np.pi) % (2 * np.pi) - np.pi
        self.theta2_sign = 1.0 if diff > 0 else -1.0

        logger.info(
            f"UnifiedIKSolver parameters: d1={self.d1:.4f}, a2={self.a2:.4f}, "
            f"a3={self.a3:.4f}, d6={self.d6:.4f}, "
            f"wrist_offset={self.wrist_offset:.4f}, "
            f"ref_angle={self.ref_angle:.4f}, "
            f"theta2_sign={self.theta2_sign:+.0f}"
        )

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def solve(
        self,
        T_flange: np.ndarray,
        q_current: Optional[np.ndarray] = None,
    ) -> List[IKSolution]:
        """
        Solve IK for the given flange pose.

        Args:
            T_flange: 4x4 homogeneous transform, flange pose in the
                      true-root frame.
            q_current: optional current joint configuration. If provided,
                       the returned list is sorted with the solution whose
                       branch matches q_current's branch first. The list
                       still contains all solutions; selection is the
                       caller's final call.

        Returns:
            List of IKSolution, up to 8. May be empty if the pose is
            unreachable.
        """
        T_flange = np.asarray(T_flange, dtype=float)
        if T_flange.shape != (4, 4):
            raise ValueError(f"Expected 4x4 transform, got shape {T_flange.shape}")

        # Step 1: compute P5.
        P5 = self._compute_p5(T_flange)

        # Steps 2-7: build the full solution set.
        solutions = self._build_solutions(T_flange, P5)

        # Sort by branch continuity if q_current is provided.
        if q_current is not None and solutions:
            solutions = self._sort_by_continuity(solutions, q_current)

        return solutions

    def select_best(
        self,
        T_flange: np.ndarray,
        q_current: np.ndarray,
    ) -> Optional[IKSolution]:
        """
        Solve IK and return the single best solution by branch continuity.

        Prefers solutions whose tags match q_current's branch. Falls back
        to the closest solution in joint space if no branch match exists.

        Returns None if no solution exists.
        """
        solutions = self.solve(T_flange, q_current=q_current)
        if not solutions:
            return None
        return solutions[0]

    # ------------------------------------------------------------------
    # Step 1: P5
    # ------------------------------------------------------------------

    def _compute_p5(self, T_flange: np.ndarray) -> np.ndarray:
        """P5 = P_flange - d6 * a_flange."""
        P_flange = T_flange[:3, 3]
        a_flange = T_flange[:3, 2]
        return P_flange - self.d6 * a_flange

    # ------------------------------------------------------------------
    # Steps 2-7: full solution construction
    # ------------------------------------------------------------------

    def _build_solutions(
        self,
        T_flange: np.ndarray,
        P5: np.ndarray,
    ) -> List[IKSolution]:
        """
        Build the full tagged solution set for the given flange pose.

        This is the orchestration of Steps 2-7. Each step is delegated
        to a helper so the derivation is readable against the document.
        """
        P_flange = T_flange[:3, 3]
        a_flange = T_flange[:3, 2]
        R_flange = T_flange[:3, :3]

        # Step 2: shoulder (theta1 candidates with sigma).
        shoulder_candidates = self._step2_shoulder(P5)
        if not shoulder_candidates:
            return []

        solutions = []
        for theta1, sigma, shoulder_tag in shoulder_candidates:
            # Step 3: wrist angle (theta5 candidates).
            theta5_candidates = self._step3_wrist(
                theta1, sigma, P_flange, a_flange
            )
            if not theta5_candidates:
                continue

            for theta5, wrist_tag in theta5_candidates:
                # Step 4: arm chain (theta2, theta3).
                arm_candidates = self._step4_arm_chain(theta1, theta5, P5)
                if not arm_candidates:
                    continue

                for theta2, theta3, elbow_tag in arm_candidates:
                    # Step 5: wrist roll (theta4, theta6).
                    wrist_result = self._step5_wrist_roll(
                        theta1, theta2, theta3, theta5, R_flange
                    )
                    if wrist_result is None:
                        continue
                    theta4, theta6 = wrist_result

                    # Step 6: assemble.
                    q = np.array(
                        [theta1, theta2, theta3, theta4, theta5, theta6],
                        dtype=float,
                    )

                    tags = {
                        'shoulder': shoulder_tag,
                        'elbow': elbow_tag,
                        'wrist': wrist_tag,
                    }

                    # Step 7: filter by joint limits.
                    if not self._within_limits(q):
                        continue

                    solutions.append(IKSolution(q=q, tags=tags))

        return solutions

    # ------------------------------------------------------------------
    # Step 2: shoulder (theta1)
    # ------------------------------------------------------------------

    def _step2_shoulder(
        self, P5: np.ndarray
    ) -> List[Tuple[float, float, BranchTag]]:
        """
        Return list of (theta1, sigma, tag) tuples.

        For offset wrists: two solutions theta1 = phi +/- alpha.
        For spherical wrists: two solutions {phi, phi + pi}.

        sigma is +1 for the right-shoulder branch, -1 for the left.
        """
        x5, y5 = P5[0], P5[1]
        r = float(np.sqrt(x5 * x5 + y5 * y5))
        phi = float(np.arctan2(y5, x5))

        if self.wrist_offset < _ZERO_TOL:
            # Spherical case.
            # Two solutions: phi (right, toward P5) and phi + pi (left, away).
            return [
                (phi, +1.0, BranchTag.RIGHT),
                (self._wrap(phi + np.pi), -1.0, BranchTag.LEFT),
            ]

        # Offset case.
        if r < self.wrist_offset - _REACH_TOL:
            # P5 is inside the circle; no tangent line exists.
            return []

        ratio = self.wrist_offset / r
        if ratio > 1.0 + _REACH_TOL:
            return []
        # Clamp to [-1, 1] to absorb floating point at the boundary.
        ratio = max(-1.0, min(1.0, ratio))
        alpha = float(np.arcsin(ratio))

        return [
            (self._wrap(phi - alpha), +1.0, BranchTag.RIGHT),
            (self._wrap(phi + alpha), -1.0, BranchTag.LEFT),
        ]

    # ------------------------------------------------------------------
    # Step 3: wrist angle (theta5)
    # ------------------------------------------------------------------

    def _step3_wrist(
        self,
        theta1: float,
        sigma: float,
        P_flange: np.ndarray,
        a_flange: np.ndarray,
    ) -> List[Tuple[float, BranchTag]]:
        """
        Return list of (theta5, tag) tuples.

        Two branches:
        - If d6 > tol: solve from flange position (o_flange).
        - If d6 <= tol: solve from flange orientation (a_flange · n_hat).
        """
        n_hat = np.array([-np.sin(theta1), np.cos(theta1), 0.0])

        if self.d6 > _ZERO_TOL:
            # Position-determined branch.
            o_flange = float(np.dot(P_flange, n_hat))
            c = (o_flange - sigma * self.wrist_offset) / self.d6
        else:
            # Orientation-determined branch.
            c = float(np.dot(a_flange, n_hat))

        if abs(c) > 1.0 + _REACH_TOL:
            return []
        c = max(-1.0, min(1.0, c))

        theta5_pos = float(np.arccos(c))       # in [0, pi]
        theta5_neg = -theta5_pos

        # Convention: positive theta5 is no-flip, negative is flip.
        return [
            (theta5_pos, BranchTag.NO_FLIP),
            (theta5_neg, BranchTag.FLIP),
        ]

    # ------------------------------------------------------------------
    # Step 4: arm chain (theta2, theta3)
    # ------------------------------------------------------------------

    def _step4_arm_chain(
        self,
        theta1: float,
        theta5: float,
        P5: np.ndarray,
    ) -> List[Tuple[float, float, BranchTag]]:
        """
        Return list of (theta2, theta3, tag) tuples.

        Project P5 into the arm plane, then solve the two-link chain
        (J2, J3) with the law of cosines and law of sines.

        Note: this is the position problem for J2 and J3 only. J4 does
        not affect P5's position, so it is not part of this step.
        """
        x5, y5, z5 = P5[0], P5[1], P5[2]

        # Project into arm plane.
        u = x5 * np.cos(theta1) + y5 * np.sin(theta1)
        w = z5

        delta_u = u
        delta_w = w - self.d1
        D2 = delta_u * delta_u + delta_w * delta_w
        D = float(np.sqrt(D2))

        if D < _ZERO_TOL:
            # P5 is at the shoulder. Degenerate; refuse.
            return []

        # Law of cosines for theta3.
        c3 = (self.a2 * self.a2 + self.a3 * self.a3 - D2) / (
            2.0 * self.a2 * self.a3
        )
        if abs(c3) > 1.0 + _REACH_TOL:
            return []
        c3 = max(-1.0, min(1.0, c3))

        theta3_pos = float(np.arccos(c3))    # in [0, pi]
        theta3_neg = -theta3_pos

        # Law of sines for theta2.
        # sin(beta) = a3 * sin(theta3) / D
        # theta2 = psi - beta * sign(theta3) * theta2_sign
        psi = float(np.arctan2(delta_w, delta_u))
        psi_rel = self._wrap(psi - self.ref_angle)

        results = []
        for theta3, tag in [(theta3_pos, BranchTag.UP), (theta3_neg, BranchTag.DOWN)]:
            s3 = np.sin(theta3)
            ratio = self.a3 * s3 / D
            if abs(ratio) > 1.0 + _REACH_TOL:
                continue
            ratio = max(-1.0, min(1.0, ratio))
            beta = float(np.arcsin(ratio))

            # theta2 relative to the zero-configuration angle.
            theta2 = self.theta2_sign * psi_rel - beta * np.sign(theta3) if theta3 != 0 else self.theta2_sign * psi_rel

            results.append((self._wrap(theta2), self._wrap(theta3), tag))

        return results

    # ------------------------------------------------------------------
    # Step 5: wrist roll (theta4, theta6)
    # ------------------------------------------------------------------

    def _step5_wrist_roll(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
        theta5: float,
        R_flange: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        """
        Return (theta4, theta6) or None if the wrist decomposition fails.

        Uses R03 computed via Rodrigues rotation about the actual URDF
        axis directions. This is the convention-independent form: it
        works for any arm whose J4-J5-J6 axes satisfy the Z-Y-Z structure
        in frame 3 (which the supported class does, by construction).
        """
        # Compute R03 = R(a1, theta1) . R(a2, theta2) . R(a3, theta3).
        R03 = (
            self._rodrigues(self.axis_dirs[0], theta1) @
            self._rodrigues(self.axis_dirs[1], theta2) @
            self._rodrigues(self.axis_dirs[2], theta3)
        )

        # R_wrist = R03^T . R_flange.
        R_wrist = R03.T @ R_flange

        sin5 = np.sin(theta5)
        if abs(sin5) < _SINGULARITY_TOL:
            # Wrist singularity: theta4 and theta6 are coupled.
            # Convention: set theta4 = 0, solve theta6 from remaining entries.
            return self._step5_singular(R_wrist, theta5)

        # Normal case.
        theta4 = float(np.arctan2(
            R_wrist[1, 2] / sin5,
            R_wrist[0, 2] / sin5,
        ))
        theta6 = float(np.arctan2(
            R_wrist[2, 1] / sin5,
            -R_wrist[2, 0] / sin5,
        ))

        return self._wrap(theta4), self._wrap(theta6)

    def _step5_singular(
        self,
        R_wrist: np.ndarray,
        theta5: float,
    ) -> Tuple[float, float]:
        """
        Handle the wrist singularity (theta5 = 0 or pi).

        At singularity, theta4 and theta6 are not independent. Set
        theta4 = 0 and solve theta6 from the combined rotation.

        For theta5 = 0: R_wrist = R_z(theta4 + theta6).
        For theta5 = pi: R_wrist = R_z(theta4 - theta6) . R_y(pi).
        """
        theta4 = 0.0

        if np.cos(theta5) > 0:
            # theta5 = 0: R_z(theta4 + theta6) = R_wrist.
            # theta6 = atan2(R_wrist[1,0], R_wrist[0,0]) - theta4.
            theta6 = float(np.arctan2(R_wrist[1, 0], R_wrist[0, 0])) - theta4
        else:
            # theta5 = pi: R_z(theta4 - theta6) = R_wrist . R_y(-pi).
            # After the R_y(pi) removal, theta6 = -(atan2(R[1,0], R[0,0]) - theta4).
            theta6 = -float(np.arctan2(R_wrist[1, 0], R_wrist[0, 0])) + theta4

        return self._wrap(theta4), self._wrap(theta6)

    # ------------------------------------------------------------------
    # Step 7: joint limit filtering
    # ------------------------------------------------------------------

    def _within_limits(self, q: np.ndarray) -> bool:
        """
        Return True if all joint angles are within their limits,
        allowing wrap by ±2*pi.

        Joint limits are checked modulo 2π. The two shoulder solutions
        phi and phi + pi differ by pi and both may be valid if the limits
        allow. See unified_IK.md Part VII Step 7.
        """
        for i, (lower, upper) in enumerate(self.geom.joint_limits):
            theta = float(q[i])
            if lower <= theta <= upper:
                continue
            # Try wrapping by +2pi or -2pi.
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

    # ------------------------------------------------------------------
    # Selection helpers
    # ------------------------------------------------------------------

    def _sort_by_continuity(
        self,
        solutions: List[IKSolution],
        q_current: np.ndarray,
    ) -> List[IKSolution]:
        """
        Sort solutions so that the one whose branch best matches
        q_current is first.

        Matching is by branch tags. If no exact tag match exists, falls
        back to joint-space distance.
        """
        q_current = np.asarray(q_current, dtype=float)

        # Determine current branch from the joint angles.
        current_tags = self._infer_tags(q_current)

        def rank(sol: IKSolution) -> Tuple[int, float]:
            # Primary: how many tags match.
            matches = sum(
                1 for k in ('shoulder', 'elbow', 'wrist')
                if sol.tags.get(k) == current_tags.get(k)
            )
            # Secondary: joint-space distance (smaller is better).
            dist = float(np.sum(
                self._angle_diff(sol.q, q_current) ** 2
            ))
            # Sort key: more matches first (so negative), then smaller distance.
            return (-matches, dist)

        return sorted(solutions, key=rank)

    def _infer_tags(self, q: np.ndarray) -> dict:
        """
        Infer the branch tags of a given joint configuration.

        This is a heuristic based on the joint angles. It is used only
        for sorting, not for solving. If the heuristic is wrong, the
        sorting is wrong, but the solution set is still correct.
        """
        # The most reliable tag is shoulder: theta1 relative to phi.
        # Elbow: sign of theta3.
        # Wrist: sign of theta5.
        tags = {}

        # Wrist: sign of theta5.
        if q[4] >= 0:
            tags['wrist'] = BranchTag.NO_FLIP
        else:
            tags['wrist'] = BranchTag.FLIP

        # Elbow: sign of theta3.
        if q[2] >= 0:
            tags['elbow'] = BranchTag.UP
        else:
            tags['elbow'] = BranchTag.DOWN

        # Shoulder: harder to infer from theta1 alone without the target.
        # If theta1 is within +/- pi/2 of the target direction, it is
        # likely right-shoulder; else left-shoulder. But we do not have
        # the target here. Use a neutral default; the rank function will
        # prefer solutions that match the other two tags.
        # For a first guess, use the joint-space distance, not tags.
        # The sort function handles the shoulder-match case correctly
        # even if the current_tags shoulder entry is a guess.
        if abs(q[0]) < np.pi / 2:
            tags['shoulder'] = BranchTag.RIGHT
        else:
            tags['shoulder'] = BranchTag.LEFT

        return tags

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
        """Rotation matrix for rotation about unit axis by angle."""
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
        """Wrap angle to [-pi, pi]."""
        return float((angle + np.pi) % (2 * np.pi) - np.pi)

    def _angle_diff(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Element-wise angular difference, wrapped to [-pi, pi]."""
        diff = a - b
        return (diff + np.pi) % (2 * np.pi) - np.pi