"""
Geometric extraction from a KinematicModel.

Reads the URDF-derived kinematic model and produces a pure geometric
description of the arm: joint axes as lines in space, the wrist center,
the flange offset, and the five geometric quantities the IK derivation
uses.

No DH parameters. No frame assignment beyond the true-root frame that the
model already provides.

All quantities are in the true-root frame (the parent of the first
movable revolute joint). The outer boundary is the flange — the tool
mount link, the end of the fixed-joint chain that follows the last
movable revolute joint. Everything before the true root and everything
after the flange is the controller's responsibility — see
docs/CONSTITUTION.md Part 3.5.

The output is the input to the unified IK solver.

The five geometric quantities
-----------------------------
For each consecutive pair of joint axes, indexed 1-based so the index
matches the joint number:

    axis_gap(i)    — perpendicular distance between axes i and i+1
                     (i = 1..5; axis_gap(2) is J2–J3, the upper arm)
    axis_twist(i)  — angle between axes i and i+1 (i = 1..5)
    axis_shift(i)  — signed distance along axis i between the feet of
                     the common perpendiculars with axes i-1 and i+1
                     (i = 2..5; origin-dependent)

Plus the fifth quantity:

    wrist_normal_offset = |AB| — perpendicular distance from the arm
                          plane (containing J2's and J3's origins, normal
                          to J2's axis) to J5's axis. The offset between
                          the arm plane and the J5-axis plane. Used for
                          theta_1 (tangency) and theta_5 (lateral offset).

The right triangle at B
-----------------------
The three wrist quantities are the sides of the right triangle A–B–C,
right-angled at B:

    wrist_normal_offset = |AB| — normal to the arm plane (the fifth
                          quantity; the solver's scalar for theta_1 and
                          theta_5).

    wrist_radial_offset = |BC| — parallel to the arm plane, in the plane
                          through B at distance |AB| from the arm plane.
                          Radial from J4's axis. (Derived; for the wrist
                          inversion's geometry.)

    wrist_offset        = |AC| — the hypotenuse, sqrt(|AB|^2 + |BC|^2).
                          Distance from the wrist root A to the wrist
                          center C. Zero exactly when the wrist is
                          spherical. (Derived; for the arm-chain target
                          distance elbow_to_wrist.)

All three are zero for a spherical wrist. The solver takes only |AB| as
a scalar; |BC| and |AC| are consequences of |AB| and the position of C.

Derived lengths the solver uses
-------------------------------
    elbow_to_wrist()   — distance from J3's axis to A (the wrist root,
                         where J4's axis meets the arm plane). This is
                         the forearm length in the law of cosines. NOT
                         axis_gap(3), and NOT the physical forearm link.

    flange_offset      — distance from the wrist center to the flange
                         (d6 in conventional notation).

Origin-independence
-------------------
All wrist quantities are computed from the axes as lines, not from the
joint origins. The distinction matters: joint origins can be shifted
along their axes without changing the geometry, but axis intersections
cannot. For the joint-4 origin in particular, UR and FR place it at
opposite ends of the A–B segment, so any origin-derived computation of
wrist_normal_offset would disagree between the two arms. The line-based
computation does not.

Note on axis_shift vs wrist_normal_offset
-----------------------------------------
axis_shift(3) measures from the J4 origin to the foot of the common
perpendicular with J5, along J4's axis. It is origin-dependent, and it
does NOT equal wrist_normal_offset in general. For UR10, axis_shift(3)
is 0.0 while wrist_normal_offset is 0.163941 — the two are not the same
quantity. The IK solver must use wrist_normal_offset, not axis_shift(3).

Class condition for theta_1 (the projection)
--------------------------------------------
The tangency derivation in theta_1 projects onto the X-Y plane, which
preserves distances only when the perpendicular direction between the
arm plane and the J5-plane is horizontal (i.e., J4's axis is
horizontal). When wrist_normal_offset > 0, J4's axis must be
horizontal; when it is zero, P5 is on the arm plane and no condition
on J4 is needed. This is checked in check_supported_class().

Tolerance and URDF rounding
---------------------------
URDFs produced by CAD exporters (e.g., SolidWorks) round their rpy
values to a few decimal places. The FR5's URDF has rpy="1.5708" where
the exact value is pi/2 = 1.5707963..., which introduces a perpendicular
error of about 4 microradians. The extraction checks the supported-class
conditions against a tolerance of 1e-4 rad (about 5.7 millidegrees),
which is large enough to accept CAD rounding and small enough to reject
a genuine geometric feature (which would be degrees, not microradians).
The tolerance is stated here, not hidden: an arm that exceeds it is
refused loudly, not approximated.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import logging
logger = logging.getLogger(__name__)


# Angle tolerance, in radians. Large enough to accept CAD rounding
# (SolidWorks rounds rpy to 5 decimal places, giving errors of order
# 1e-5 rad), small enough to reject real geometric features (which
# would be degrees, i.e. 1e-2 rad or larger). See the module docstring.
_ANGLE_TOL = 1e-4         # radians
_INTERSECT_TOL = 1e-4     # metres


class UnsupportedArmError(RuntimeError):
    """
    Raised when an arm's geometry is outside the supported class.

    The analytical IK solver requires an arm-plane 6R arm with
    perpendicular, intersecting wrist axes. If the arm violates the
    class, the solver refuses — it does not approximate. See
    docs/unified_IK.md, Appendix B.

    CAD-exported URDFs round their angles to a few decimal places; the
    class check tolerates this (see _ANGLE_TOL). An arm that exceeds the
    tolerance is refused, not approximated.
    """
    pass


@dataclass
class Axis:
    """A joint axis as a line in the true-root frame."""
    name: str
    point: np.ndarray      # (3,) a point on the axis, in true-root frame
    direction: np.ndarray  # (3,) unit vector along the axis, in true-root frame

    def project(self, p: np.ndarray) -> np.ndarray:
        """Project a point onto this line."""
        v = p - self.point
        return self.point + np.dot(v, self.direction) * self.direction

    def distance_to(self, p: np.ndarray) -> float:
        """Perpendicular distance from a point to this line."""
        v = p - self.point
        perp = v - np.dot(v, self.direction) * self.direction
        return float(np.linalg.norm(perp))

    def intersect_plane(self,
                        plane_point: np.ndarray,
                        plane_normal: np.ndarray) -> np.ndarray:
        """
        Intersect this line with a plane.

        Args:
            plane_point: a point on the plane.
            plane_normal: a normal to the plane (need not be unit length).

        Returns:
            The intersection point.

        Raises:
            ValueError: if the line is parallel to the plane (no unique
                        intersection). Used in diagnostic paths only; the
                        solver's wrist root A is computed by projection
                        (see ArmGeometry.wrist_root()), not by this method.
        """
        n = plane_normal / np.linalg.norm(plane_normal)
        denom = np.dot(self.direction, n)
        if abs(denom) < _ANGLE_TOL:
            raise ValueError(
                f"Axis {self.name} is parallel to the plane "
                f"(dot(direction, normal) = {denom:.3e}); "
                f"no unique intersection."
            )
        t = np.dot(plane_point - self.point, n) / denom
        return self.point + t * self.direction


@dataclass
class ArmGeometry:
    """
    Pure geometric description of a 6R arm.

    All quantities are in the true-root frame (the frame of the parent
    of the first movable revolute joint, as returned by
    KinematicModel.get_true_root()).

    The arm proper is bounded by the true root at one end and the flange
    at the other. The flange is the tool mount link — the end of the
    fixed-joint chain that follows the last movable revolute joint. It
    is a property of the arm's mechanical description, not of any
    specific tool. Everything past the flange is the controller's
    responsibility. See docs/CONSTITUTION.md Part 3.5.

    No DH parameters. The solver computes whatever distances and angles
    it needs from these lines directly.
    """
    # The six joint axes, in kinematic order J1..J6.
    axes: List[Axis]

    # The wrist center: the point where J5 and J6 axes intersect (spherical
    # wrist) or come closest (offset wrist). In the true-root frame.
    # This is C = P5 in the derivation's notation.
    wrist_center: np.ndarray

    # Distance from the wrist center to the flange (tool mount link),
    # along the flange's Z axis. This is d6 in the conventional notation.
    # A property of the arm alone; not the same as a distance to any
    # attached tool or TCP.
    flange_offset: float

    # Joint limits, in kinematic order. (lower, upper) per joint.
    joint_limits: List[Tuple[float, float]]

    # Names of the joints, in kinematic order.
    joint_names: List[str]

    # ------------------------------------------------------------------
    # Wrist quantities. Computed in __post_init__ from the axes as lines.
    # ------------------------------------------------------------------

    # wrist_normal_offset = |AB|.
    # The fifth quantity. Perpendicular distance from the arm plane to
    # J5's axis. Used for theta_1 (tangency) and theta_5 (lateral offset).
    # This is the ONLY wrist scalar the solver takes as input.
    wrist_normal_offset: float = field(init=False)

    # wrist_radial_offset = |BC|.
    # Perpendicular distance from the wrist center C to J4's axis. BC
    # lies in the plane through B at distance |AB| from the arm plane.
    # Derived; used by the wrist inversion's geometry and by describe().
    wrist_radial_offset: float = field(init=False)

    # wrist_offset = |AC|.
    # The hypotenuse: sqrt(|AB|^2 + |BC|^2). Distance from the wrist
    # root A to the wrist center C. Zero exactly when the wrist is
    # spherical. Derived; used by describe() and by the tests.
    wrist_offset: float = field(init=False)

    def __post_init__(self):
        self.wrist_normal_offset = self._compute_wrist_normal_offset()
        self.wrist_radial_offset = float(
            self.axes[3].distance_to(self.wrist_center)
        )
        self.wrist_offset = float(np.sqrt(
            self.wrist_normal_offset ** 2 + self.wrist_radial_offset ** 2
        ))

    # ------------------------------------------------------------------
    # The wrist triangle and wrist root
    # ------------------------------------------------------------------

    def _compute_wrist_normal_offset(self) -> float:
        """
        Compute |AB|: perpendicular distance from the arm plane to J5's
        axis.

        B is J4's axis ∩ J5's axis. The arm plane contains J2's origin
        and is normal to J2's axis. |AB| = |dot(B − J2's origin, n̂)|,
        where n̂ is J2's direction.

        This is origin-independent: B is an axis intersection (invariant)
        and J2's origin lies on the arm plane (its perpendicular distance
        from the plane is zero). The projection of B onto the plane is
        the wrist root A; |AB| is the distance from A to B along J4's
        axis.
        """
        j2 = self.axes[1]
        j4 = self.axes[3]
        j5 = self.axes[4]

        n = j2.direction / np.linalg.norm(j2.direction)
        B = _closest_point_between_axes(j4, j5)
        signed = float(np.dot(B - j2.point, n))
        return abs(signed)

    def wrist_root(self) -> np.ndarray:
        """
        A = the wrist root: the projection of B onto the arm plane,
        along the plane's normal.

        This is well-defined for every arm in the class, whether J4's
        axis pierces the arm plane (offset wrist) or lies in it
        (spherical wrist). For the spherical wrist, B is on the plane,
        and A = B = C.
        """
        j2 = self.axes[1]
        n = j2.direction / np.linalg.norm(j2.direction)
        B = _closest_point_between_axes(self.axes[3], self.axes[4])
        signed = float(np.dot(B - j2.point, n))
        return B - signed * n

    # ------------------------------------------------------------------
    # Derived lengths the arm chain uses
    # ------------------------------------------------------------------

    def elbow_to_wrist(self) -> float:
        """
        Distance from J3's axis to A (the wrist root).

        This is the forearm length in the law of cosines (Part 7). It is
        NOT axis_gap(3): the latter is the perpendicular distance between
        J3 and J4 axes, which coincides with elbow_to_wrist only for a
        spherical wrist. For an offset wrist, they differ, and the law
        of cosines uses this distance.
        """
        return float(np.linalg.norm(self.wrist_root() - self.axes[2].point))

    # ------------------------------------------------------------------
    # Axis quantities, 1-based indexing.
    # axis_gap(i) is between axes[i-1] and axes[i]. So axis_gap(2) is
    # J2–J3, matching the joint numbering.
    # ------------------------------------------------------------------

    def axis_twist(self, i: int) -> float:
        """
        Angle between axes i and i+1, in [0, π]. i = 1..5.
        axis_twist(2) is the J2–J3 twist (0 for UR/FR, π for Elfin).
        """
        u = self.axes[i - 1].direction
        v = self.axes[i].direction
        return float(np.arctan2(np.linalg.norm(np.cross(u, v)),
                                np.dot(u, v)))

    def axes_parallel(self, i: int) -> bool:
        """True if axes i and i+1 are parallel (within _ANGLE_TOL). i = 1..5."""
        u = self.axes[i - 1].direction
        v = self.axes[i].direction
        return float(np.linalg.norm(np.cross(u, v))) < _ANGLE_TOL

    def axes_perpendicular(self, i: int) -> bool:
        """True if axes i and i+1 are perpendicular (within _ANGLE_TOL). i = 1..5."""
        u = self.axes[i - 1].direction
        v = self.axes[i].direction
        return abs(float(np.dot(u, v))) < _ANGLE_TOL

    def axes_intersect(self, i: int) -> bool:
        """True if axes i and i+1 intersect (within _INTERSECT_TOL). i = 1..5."""
        a = self.axes[i - 1]
        b = self.axes[i]
        closest = _closest_point_between_axes(a, b)
        return max(a.distance_to(closest), b.distance_to(closest)) < _INTERSECT_TOL

    def axis_gap(self, i: int) -> float:
        """
        Perpendicular distance between axes i and i+1. i = 1..5.

        axis_gap(2) is the gap between J2 and J3 (the upper arm).
        axis_gap(3) is the gap between J3 and J4.

        This is NOT elbow_to_wrist(). See elbow_to_wrist() for the
        forearm length the law of cosines uses.
        """
        u = self.axes[i - 1].direction
        v = self.axes[i].direction
        w = self.axes[i].point - self.axes[i - 1].point
        n = np.cross(u, v)
        n_norm = np.linalg.norm(n)
        if n_norm < _ANGLE_TOL:
            return float(np.linalg.norm(w - np.dot(w, u) * u))
        return float(abs(np.dot(w, n / n_norm)))

    def common_perpendicular_foot(self, i: int) -> np.ndarray:
        """
        The point on axes[i-1] that is closest to axes[i].
        i = 1..5.
        """
        u = self.axes[i - 1].direction
        v = self.axes[i].direction
        p = self.axes[i - 1].point
        q = self.axes[i].point
        w = q - p
        n = np.cross(u, v)
        if np.linalg.norm(n) < _ANGLE_TOL:
            return p + np.dot(q - p, u) * u
        s = (np.dot(w, u) - np.dot(w, v) * np.dot(u, v)) / (1 - np.dot(u, v) ** 2)
        return p + s * u

    def axis_shift(self, i: int) -> float:
        """
        The signed distance along axis i between the foot of the common
        perpendicular with axis i-1 and the foot of the common
        perpendicular with axis i+1. i = 2..5 (needs both neighbours).

        Origin-dependent. NOT wrist_normal_offset — see the module
        docstring. For UR10, axis_shift(3) = 0.0 while
        wrist_normal_offset = 0.163941. The solver uses
        wrist_normal_offset, never axis_shift(i).
        """
        if i < 2 or i > 5:
            raise IndexError(
                f"axis_shift undefined for joint {i}; needs both "
                f"neighbours (2 <= i <= 5)"
            )
        foot_prev = self.common_perpendicular_foot(i - 1)
        foot_next = self.common_perpendicular_foot(i)
        u = self.axes[i - 1].direction
        return float(np.dot(foot_next - foot_prev, u))

    # ------------------------------------------------------------------
    # Class check
    # ------------------------------------------------------------------

    def check_supported_class(self):
        """
        Verify the arm is in the class the analytical solver supports.

        Conditions (all within _ANGLE_TOL / _INTERSECT_TOL):
          1. J2 and J3 are parallel.
          2. J3 and J4 are parallel — ONLY when wrist_normal_offset > 0.
             When the offset is nonzero, the theta_1 projection and the
             theta_2/theta_3 law of cosines require J4 normal to the arm
             plane, which requires J4 parallel to J2/J3. When the offset
             is zero, P5 is on the arm plane, J4 need not be parallel to
             J2/J3, and no condition is needed. Elfin's J4 is vertical
             (not parallel to J2/J3) with offset zero, and passes.
          3. J4 and J5 are perpendicular.
          4. J5 and J6 are perpendicular.
          5. J4 and J6 are parallel.
          6. J4 and J5 intersect.
          7. J5 and J6 intersect.
          8. If wrist_normal_offset > 0, J4's axis is horizontal.
             (The projection condition for theta_1. If the offset is
             zero, P5 is on the arm plane and no condition is needed —
             which is why Elfin, with vertical J4 and zero offset,
             passes.)

        Raises:
            UnsupportedArmError: if any condition fails, with a message
                naming the violation and the measured value.
        """
        # 1. J2 || J3
        if not self.axes_parallel(2):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J2 and J3 are not parallel "
                f"(axis_twist(2) = {self.axis_twist(2):.6f} rad, "
                f"expected 0 or π within {_ANGLE_TOL:.1e})."
            )

        # 2. J3 || J4 — only when the offset is nonzero. The theta_1
        # projection and the theta_2/theta_3 law of cosines require J4
        # normal to the arm plane (J4 || J2/J3) when the wrist is offset.
        # When the offset is zero, P5 is on the arm plane and J4 may be
        # any direction (Elfin's J4 is vertical, with offset zero).
        if self.wrist_normal_offset > _ANGLE_TOL:
            if not self.axes_parallel(3):
                raise UnsupportedArmError(
                    f"Unsupported arm geometry: wrist_normal_offset = "
                    f"{self.wrist_normal_offset:.6f} m > 0, but J3 and J4 "
                    f"are not parallel (axis_twist(3) = "
                    f"{self.axis_twist(3):.6f} rad, expected 0 or π within "
                    f"{_ANGLE_TOL:.1e}). The theta_1 projection and the "
                    f"law of cosines require J4 parallel to J2/J3 when the "
                    f"offset is nonzero."
                )

        # 3. J4 ⟂ J5
        if not self.axes_perpendicular(4):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J5 are not perpendicular "
                f"(axis_twist(4) = {self.axis_twist(4):.6f} rad, "
                f"expected π/2 within {_ANGLE_TOL:.1e})."
            )

        # 4. J5 ⟂ J6
        if not self.axes_perpendicular(5):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J5 and J6 are not perpendicular "
                f"(axis_twist(5) = {self.axis_twist(5):.6f} rad, "
                f"expected π/2 within {_ANGLE_TOL:.1e})."
            )

        # 5. J4 || J6
        u4 = self.axes[3].direction
        u6 = self.axes[5].direction
        if float(np.linalg.norm(np.cross(u4, u6))) > _ANGLE_TOL:
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J6 are not parallel "
                f"(angle between = "
                f"{float(np.arctan2(np.linalg.norm(np.cross(u4, u6)), np.dot(u4, u6))):.6f} rad, "
                f"expected 0 or π within {_ANGLE_TOL:.1e})."
            )

        # 6. J4 and J5 intersect
        if not self.axes_intersect(4):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J5 axes do not intersect "
                f"(closest distance > {_INTERSECT_TOL:.1e} m)."
            )

        # 7. J5 and J6 intersect
        if not self.axes_intersect(5):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J5 and J6 axes do not intersect "
                f"(closest distance > {_INTERSECT_TOL:.1e} m)."
            )

        # 8. If wrist_normal_offset > 0, J4's axis must be horizontal.
        # The theta_1 derivation projects onto the X-Y plane; the
        # perpendicular direction between the arm plane and the J5-plane
        # must be horizontal for the projection to preserve |AB|.
        # When the offset is zero, P5 is on the arm plane and no
        # condition is needed.
        if self.wrist_normal_offset > _ANGLE_TOL:
            u4 = self.axes[3].direction
            # Horizontal means perpendicular to the true-root Z axis.
            z_axis = np.array([0.0, 0.0, 1.0])
            if abs(float(np.dot(u4, z_axis))) > _ANGLE_TOL:
                raise UnsupportedArmError(
                    f"Unsupported arm geometry: wrist_normal_offset = "
                    f"{self.wrist_normal_offset:.6f} m > 0, but J4's axis "
                    f"is not horizontal (|dot(u4, ẑ)| = "
                    f"{abs(float(np.dot(u4, z_axis))):.6f}). The theta_1 "
                    f"projection requires J4 horizontal when the offset "
                    f"is nonzero."
                )

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def describe(self) -> str:
        """Human-readable summary. Useful in tests and logs."""
        lines = ["ArmGeometry:"]
        for i, ax in enumerate(self.axes):
            lines.append(f"  J{i+1} {ax.name}: point={ax.point.round(6)}, "
                         f"dir={ax.direction.round(6)}")
        lines.append(f"  wrist_center: {self.wrist_center.round(6)}")
        lines.append("")
        lines.append("  Wrist triangle at B (right angle at B):")
        lines.append(f"    wrist_normal_offset (|AB|): "
                     f"{self.wrist_normal_offset:.6f}  "
                     f"[plane-to-J5-axis; solver scalar]")
        lines.append(f"    wrist_radial_offset (|BC|): "
                     f"{self.wrist_radial_offset:.6f}  "
                     f"[C to J4 axis, in parallel plane]")
        lines.append(f"    wrist_offset        (|AC|): "
                     f"{self.wrist_offset:.6f}  "
                     f"[hypotenuse; A to C]")
        lines.append("")
        lines.append(f"  wrist_root A: {self.wrist_root().round(6)}")
        lines.append(f"  elbow_to_wrist (J3->A): {self.elbow_to_wrist():.6f}")
        lines.append(f"  flange_offset: {self.flange_offset:.6f}")
        lines.append(f"  axis_gap(2) (J2->J3, upper arm): {self.axis_gap(2):.6f}")
        lines.append(f"  axis_gap(3) (J3->J4): {self.axis_gap(3):.6f}")
        for i in range(1, 6):
            lines.append(
                f"  axis_twist({i}) = {self.axis_twist(i):.6f} rad "
                f"({self.axis_twist(i)/np.pi:.4f}π);  "
                f"axis_gap({i}) = {self.axis_gap(i):.6f}"
            )
        lines.append(f"  axis_shift(3) = {self.axis_shift(3):.6f} "
                    f"(along J3; origin-dependent)")
        lines.append(f"  axis_shift(4) = {self.axis_shift(4):.6f} "
                    f"(along J4; origin-dependent)")
        return "\n".join(lines)


def extract_arm_geometry(model, base_link: Optional[str] = None) -> ArmGeometry:
    """
    Extract ArmGeometry from a KinematicModel.

    Args:
        model: a loaded KinematicModel.
        base_link: the link to express geometry in. Defaults to the model's
                   true root, which is the parent of the first moving joint.

    Returns:
        ArmGeometry with all quantities in the base_link frame.

    Raises:
        UnsupportedArmError: if the arm is not in the supported class
            (Appendix B of docs/unified_IK.md). The tolerance is 1e-4
            rad / 1e-4 m, which accepts CAD rounding but rejects genuine
            geometric features.
    """
    if base_link is None:
        base_link = model.get_true_root()

    arm_chain = model.get_arm_chain(base_link_name=base_link)
    if len(arm_chain) != 6:
        raise UnsupportedArmError(
            f"Expected exactly 6 revolute joints in arm chain, "
            f"got {len(arm_chain)}: {arm_chain}. "
            f"The analytical solver requires a 6R arm."
        )

    q_zero = np.zeros(6)
    transforms = model.compute_fk(q_zero)

    T_base = transforms[base_link]
    T_base_inv = np.linalg.inv(T_base)

    axes = []
    for joint_name in arm_chain:
        joint = model.joints[joint_name]
        parent_link = joint['parent']
        T_parent_world = transforms[parent_link]

        origin_in_joint_frame = model._compose_transform(
            joint['origin_xyz'], joint['origin_rpy']
        )
        T_joint_world = T_parent_world @ origin_in_joint_frame
        T_joint_in_base = T_base_inv @ T_joint_world

        point = T_joint_in_base[:3, 3]

        axis_local = np.array(joint['axis'], dtype=float)
        axis_local = axis_local / np.linalg.norm(axis_local)
        direction = T_joint_in_base[:3, :3] @ axis_local
        direction = direction / np.linalg.norm(direction)

        axes.append(Axis(name=joint_name, point=point, direction=direction))

    # axes[4] is J5, axes[5] is J6 (0-based list access; the *indices*
    # for axis_gap etc. are 1-based and handled by the methods).
    wrist_center = _closest_point_between_axes(axes[4], axes[5])

    T_flange = model.get_flange_pose(q_zero)
    flange_pos = T_flange[:3, 3]
    flange_offset = float(np.linalg.norm(flange_pos - wrist_center))

    joint_limits = []
    for joint_name in arm_chain:
        lim = model.joints[joint_name]['limit']
        joint_limits.append((lim['lower'], lim['upper']))

    geometry = ArmGeometry(
        axes=axes,
        wrist_center=wrist_center,
        flange_offset=flange_offset,
        joint_limits=joint_limits,
        joint_names=list(arm_chain),
    )

    geometry.check_supported_class()
    return geometry


def _closest_point_between_axes(a: Axis, b: Axis) -> np.ndarray:
    """
    The midpoint of the shortest segment between two lines.

    For intersecting lines, this is the intersection point.
    For parallel lines, this is an arbitrary point on the midline.
    For skew lines, this is the midpoint of the common perpendicular.

    Returns a point in the true-root frame.
    """
    u = a.direction
    v = b.direction
    p = a.point
    q = b.point
    w = q - p

    uv = np.dot(u, v)
    denom = 1.0 - uv * uv

    if denom < _ANGLE_TOL:
        return a.project(q)

    s = (np.dot(w, u) - np.dot(w, v) * uv) / denom
    t = (np.dot(w, u) * uv - np.dot(w, v)) / denom
    point_a = p + s * u
    point_b = q + t * v
    return 0.5 * (point_a + point_b)
