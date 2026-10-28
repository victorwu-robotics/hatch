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
For each consecutive pair of joint axes (i, i+1):

    axis_gap(i)    — perpendicular distance between the two axes
    axis_twist(i)  — angle between the two axis directions
    axis_shift(i)  — signed distance along axis i between the feet of
                     the common perpendiculars with axes i-1 and i+1
                     (origin-dependent; defined relative to joint origins)

Plus the two wrist quantities:

    wrist_offset        = |BC| — perpendicular distance from the J4 axis
                          to the wrist center C (= P5). The radius of the
                          circle traced by the wrist-1 link. The quantity
                          that decides spherical (zero) vs offset (nonzero)
                          wrist. Used for theta_5.

    wrist_normal_offset = |AB| — distance along the J4 axis from the arm
                          plane (containing J2's and J3's origins, normal
                          to J2's axis) to the intersection of J4's axis
                          with J5's axis. The offset between the arm plane
                          and the J5-axis plane. Used for theta_1.

Both wrist quantities are computed from the axes as lines, not from the
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
_INTERSECT_TOL = 1e-4     # metres (same reasoning: CAD rounding moves
                          # origins by micrometres, not millimetres)


class UnsupportedArmError(RuntimeError):
    """
    Raised when an arm's geometry is outside the supported class.

    The analytical IK solver requires an arm-plane 6R arm with
    perpendicular, intersecting wrist axes. If the arm violates the
    class, the solver refuses — it does not approximate. See
    docs/unified_IK.md, "What Hatch Refuses to Do".

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
                        intersection). The supported-class check guarantees
                        the J4 axis is normal to the arm plane, so this
                        cannot happen for a valid arm.
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
    wrist_center: np.ndarray

    # Distance from the wrist center to the flange (tool mount link),
    # along the flange's Z axis. This is the "d6" of the IK derivation
    # (docs/unified_IK.md, Part 6 §3). It is a property of the arm alone
    # and is not the same as a distance to any attached tool or TCP.
    flange_offset: float

    # Joint limits, in kinematic order. (lower, upper) per joint.
    joint_limits: List[Tuple[float, float]]

    # Names of the joints, in kinematic order.
    joint_names: List[str]

    # The perpendicular distance from the J4 axis to the wrist center.
    # = |BC| in the A/B/C notation. The radius of the circle the J4 end
    # traces around P5, and the quantity that decides whether the wrist
    # is spherical (zero) or offset (nonzero). Used for theta_5.
    #
    # Stored as a convenience; computed from axes[3] and wrist_center.
    wrist_offset: float = field(init=False)

    # The distance along the J4 axis from the arm plane (containing J2's
    # and J3's origins, normal to J2's axis) to the intersection of J4's
    # axis with J5's axis. = |AB| in the A/B/C notation. The offset
    # between the arm plane and the J5-axis plane. Used for theta_1.
    #
    # This is NOT axis_shift(3): for UR10, axis_shift(3) is 0.0 while
    # wrist_normal_offset is 0.163941. See the module docstring.
    wrist_normal_offset: float = field(init=False)

    def __post_init__(self):
        self.wrist_offset = self.axes[3].distance_to(self.wrist_center)
        self.wrist_normal_offset = self._compute_wrist_normal_offset()

    def _compute_wrist_normal_offset(self) -> float:
        """
        Compute |AB|: the distance along the J4 axis from the arm plane
        to the intersection of J4's axis with J5's axis.

        A = J4's axis ∩ arm plane. The arm plane contains J2's origin
            (and J3's origin) and is normal to J2's axis direction.
        B = J4's axis ∩ J5's axis.

        Both A and B are computed from the axes as lines, not from the
        joint origins. This is what makes the result origin-independent
        and identical in form for UR, FR, and Elfin arms.
        """
        j2 = self.axes[1]
        j4 = self.axes[3]
        j5 = self.axes[4]

        # The arm plane: through J2's origin, normal to J2's axis direction.
        arm_plane_point = j2.point
        arm_plane_normal = j2.direction

        # A = J4's axis ∩ arm plane.
        A = j4.intersect_plane(arm_plane_point, arm_plane_normal)

        # B = J4's axis ∩ J5's axis.
        B = _closest_point_between_axes(j4, j5)

        return float(np.linalg.norm(A - B))

    # ------------------------------------------------------------------
    # Derived quantities the solver will ask for.
    # These are computed on demand, not stored as a parameter table.
    # ------------------------------------------------------------------

    def forearm_length(self) -> float:
        """
        Distance from J3's axis to the wrist center P5.

        This is the corrected a3 of the IK derivation (docs/unified_IK.md,
        Part 7 §3). It is NOT axis_gap(2): the latter is the perpendicular
        distance between J3 and J4 axes, which coincides with the forearm
        length only for a spherical wrist. For an offset wrist (UR, FR),
        they differ, and the law of cosines in Part 7 §5 uses this
        distance, not axis_gap(2).
        """
        return float(np.linalg.norm(self.wrist_center - self.axes[2].point))

    def axis_twist(self, i: int) -> float:
        """
        Angle between axis i and axis i+1, in [0, π].
        This is the renamed 'alpha' of the derivation.
        """
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        return float(np.arctan2(np.linalg.norm(np.cross(u, v)),
                                np.dot(u, v)))

    def axes_parallel(self, i: int) -> bool:
        """True if axis i and axis i+1 are parallel (within _ANGLE_TOL)."""
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        return float(np.linalg.norm(np.cross(u, v))) < _ANGLE_TOL

    def axes_perpendicular(self, i: int) -> bool:
        """True if axis i and axis i+1 are perpendicular (within _ANGLE_TOL)."""
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        return abs(float(np.dot(u, v))) < _ANGLE_TOL

    def axes_intersect(self, i: int) -> bool:
        """True if axis i and axis i+1 intersect (within _INTERSECT_TOL)."""
        a = self.axes[i]
        b = self.axes[i + 1]
        closest = _closest_point_between_axes(a, b)
        dist_a = a.distance_to(closest)
        dist_b = b.distance_to(closest)
        return max(dist_a, dist_b) < _INTERSECT_TOL

    def axis_gap(self, i: int) -> float:
        """
        Perpendicular distance between axis i and axis i+1.
        This is the renamed 'a' of the derivation.

        Note: for i = 2, this is NOT the forearm length. See
        forearm_length() for the corrected a3.
        """
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        w = self.axes[i + 1].point - self.axes[i].point
        n = np.cross(u, v)
        n_norm = np.linalg.norm(n)
        if n_norm < _ANGLE_TOL:
            # Parallel axes: perpendicular component of w.
            return float(np.linalg.norm(w - np.dot(w, u) * u))
        return float(abs(np.dot(w, n / n_norm)))

    def common_perpendicular_foot(self, i: int) -> np.ndarray:
        """
        The point on axis i that is closest to axis i+1.
        This is the foot of the common perpendicular, on axis i.
        """
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        p = self.axes[i].point
        q = self.axes[i + 1].point
        w = q - p
        n = np.cross(u, v)
        n_norm = np.linalg.norm(n)
        if n_norm < _ANGLE_TOL:
            # Parallel axes: project q onto axis i.
            return p + np.dot(q - p, u) * u
        # Standard skew-line closest-point formula.
        s = (np.dot(w, u) - np.dot(w, v) * np.dot(u, v)) / (1 - np.dot(u, v) ** 2)
        return p + s * u

    def axis_shift(self, i: int) -> float:
        """
        The signed distance along axis i between the foot of the common
        perpendicular with axis i-1 and the foot of the common
        perpendicular with axis i+1.

        Origin-dependent: measured from joint i's origin implicitly (the
        axes[i].point). This is why axis_shift(3) is NOT
        wrist_normal_offset — see the module docstring. For UR10,
        axis_shift(3) = 0.0 while wrist_normal_offset = 0.163941.

        Only defined for 1 <= i <= 4 (needs both neighbours).
        For i = 0 or i = 5, raises IndexError.
        """
        if i < 1 or i > 4:
            raise IndexError(f"axis_shift undefined for joint {i}; needs neighbours")
        foot_prev = self.common_perpendicular_foot(i - 1)
        foot_next = self.common_perpendicular_foot(i)
        u = self.axes[i].direction
        return float(np.dot(foot_next - foot_prev, u))

    def check_supported_class(self):
        """
        Verify the arm is in the class the analytical solver supports.

        The supported class (see docs/unified_IK.md, "What Hatch Refuses
        to Do"):

          1. J2 and J3 are parallel (within _ANGLE_TOL).
          2. J3 and J4 are parallel (within _ANGLE_TOL).
          3. J4 and J5 are perpendicular (within _ANGLE_TOL).
          4. J5 and J6 are perpendicular (within _ANGLE_TOL).
          5. J4 and J6 are parallel (within _ANGLE_TOL).
          6. J4 and J5 intersect (within _INTERSECT_TOL).
          7. J5 and J6 intersect (within _INTERSECT_TOL).

        The tolerance is 1e-4 rad (~5.7 millidegrees) — large enough to
        accept CAD rounding, small enough to reject real geometric
        features. An arm that exceeds the tolerance is refused loudly;
        it is not approximated. See the module docstring.

        Raises:
            UnsupportedArmError: if any condition fails, with a message
                naming the violation and the measured value.
        """
        # 1. J2 || J3
        if not self.axes_parallel(1):
            twist = self.axis_twist(1)
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J2 and J3 are not parallel "
                f"(axis_twist(1) = {twist:.6f} rad, expected 0 or π "
                f"within {_ANGLE_TOL:.1e}). "
                f"The analytical solver requires an arm-plane 6R arm."
            )

        # 2. J3 || J4
        if not self.axes_parallel(2):
            twist = self.axis_twist(2)
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J3 and J4 are not parallel "
                f"(axis_twist(2) = {twist:.6f} rad, expected 0 or π "
                f"within {_ANGLE_TOL:.1e}). "
                f"The analytical solver requires an arm-plane 6R arm."
            )

        # 3. J4 ⟂ J5
        if not self.axes_perpendicular(3):
            twist = self.axis_twist(3)
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J5 are not perpendicular "
                f"(axis_twist(3) = {twist:.6f} rad, expected π/2 "
                f"within {_ANGLE_TOL:.1e}). "
                f"The analytical solver requires perpendicular wrist axes."
            )

        # 4. J5 ⟂ J6
        if not self.axes_perpendicular(4):
            twist = self.axis_twist(4)
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J5 and J6 are not perpendicular "
                f"(axis_twist(4) = {twist:.6f} rad, expected π/2 "
                f"within {_ANGLE_TOL:.1e}). "
                f"The analytical solver requires perpendicular wrist axes."
            )

        # 5. J4 || J6
        u4 = self.axes[3].direction
        u6 = self.axes[5].direction
        if float(np.linalg.norm(np.cross(u4, u6))) > _ANGLE_TOL:
            twist = float(np.arctan2(
                np.linalg.norm(np.cross(u4, u6)), np.dot(u4, u6)))
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J6 are not parallel "
                f"(angle between = {twist:.6f} rad, expected 0 or π "
                f"within {_ANGLE_TOL:.1e}). "
                f"The analytical solver requires the wrist-1 and flange "
                f"axes to be parallel."
            )

        # 6. J4 and J5 intersect
        if not self.axes_intersect(3):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J4 and J5 axes do not intersect "
                f"(closest distance > {_INTERSECT_TOL:.1e} m). "
                f"The analytical solver requires intersecting wrist axes."
            )

        # 7. J5 and J6 intersect
        if not self.axes_intersect(4):
            raise UnsupportedArmError(
                f"Unsupported arm geometry: J5 and J6 axes do not intersect "
                f"(closest distance > {_INTERSECT_TOL:.1e} m). "
                f"The analytical solver requires intersecting wrist axes."
            )

    def describe(self) -> str:
        """Human-readable summary. Useful in tests and logs."""
        lines = ["ArmGeometry:"]
        for i, ax in enumerate(self.axes):
            lines.append(f"  J{i+1} {ax.name}: point={ax.point.round(6)}, "
                         f"dir={ax.direction.round(6)}")
        lines.append(f"  wrist_center:        {self.wrist_center.round(6)}")
        lines.append(f"  wrist_offset (|BC|): {self.wrist_offset:.6f}")
        lines.append(f"  wrist_normal_offset (|AB|): {self.wrist_normal_offset:.6f}")
        lines.append(f"  flange_offset:       {self.flange_offset:.6f}")
        lines.append(f"  forearm_length (J3->P5): {self.forearm_length():.6f}")
        lines.append(f"  axis_gap(2) (J3->J4):    {self.axis_gap(2):.6f}")
        for i in range(5):
            lines.append(f"  axis_twist[{i}] = {self.axis_twist(i):.6f} rad "
                         f"({self.axis_twist(i)/np.pi:.4f}π)")
            lines.append(f"  axis_gap[{i}]   = {self.axis_gap(i):.6f}")
        lines.append(f"  axis_shift[3]   = {self.axis_shift(3):.6f} "
                     f"(origin-dependent; compare wrist_normal_offset)")
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
            (J2 ∥ J3 ∥ J4, J4 ⟂ J5 ⟂ J6, J4 ∥ J6, wrist axes intersect).
            The tolerance is 1e-4 rad / 1e-4 m, which accepts CAD rounding
            but rejects genuine geometric features.
    """
    if base_link is None:
        base_link = model.get_true_root()

    # Get the arm chain: the six revolute joints, in order.
    arm_chain = model.get_arm_chain(base_link_name=base_link)
    if len(arm_chain) != 6:
        raise UnsupportedArmError(
            f"Expected exactly 6 revolute joints in arm chain, "
            f"got {len(arm_chain)}: {arm_chain}. "
            f"The analytical solver requires a 6R arm."
        )

    # Zero configuration for the six arm joints only. Building q_zero from
    # all non-fixed joints would include any joint outside the 6R chain
    # (e.g., a gripper finger), and get_flange_pose would receive an array
    # of the wrong length.
    q_zero = np.zeros(6)
    transforms = model.compute_fk(q_zero)

    T_base = transforms[base_link]
    T_base_inv = np.linalg.inv(T_base)

    # Build the six axes as lines in the base frame.
    axes = []
    for joint_name in arm_chain:
        joint = model.joints[joint_name]
        parent_link = joint['parent']

        # Transform the joint frame into the base frame.
        T_parent_world = transforms[parent_link]

        # The joint origin, in base frame.
        origin_in_joint_frame = model._compose_transform(
            joint['origin_xyz'], joint['origin_rpy']
        )
        T_joint_world = T_parent_world @ origin_in_joint_frame
        T_joint_in_base = T_base_inv @ T_joint_world

        point = T_joint_in_base[:3, 3]

        # The joint axis direction, in base frame. The axis is specified in
        # the joint frame; the joint frame's orientation relative to base is
        # the rotation part of T_joint_in_base.
        axis_local = np.array(joint['axis'], dtype=float)
        axis_local = axis_local / np.linalg.norm(axis_local)
        direction = T_joint_in_base[:3, :3] @ axis_local
        direction = direction / np.linalg.norm(direction)

        axes.append(Axis(name=joint_name, point=point, direction=direction))

    # Wrist center: the point where the J5 and J6 axes intersect or come
    # closest. Uses axes[4] (J5) and axes[5] (J6).
    wrist_center = _closest_point_between_axes(axes[4], axes[5])

    # Flange offset: distance from the wrist center to the flange (tool
    # mount link), in the true-root frame. The flange is the end of the
    # fixed-joint chain following the last revolute joint — the outer
    # kinematic boundary. get_flange_pose returns it directly in the
    # true-root frame; the tool and any fixture beyond the flange are
    # not consulted. See docs/CONSTITUTION.md Part 3.5.
    T_flange = model.get_flange_pose(q_zero)
    flange_pos = T_flange[:3, 3]
    flange_offset = float(np.linalg.norm(flange_pos - wrist_center))

    # Joint limits, in arm chain order.
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

    # Verify the arm is in the supported class. This refuses loudly if it
    # is not — the caller knows immediately that the analytical solver
    # cannot handle this arm.
    geometry.check_supported_class()

    return geometry


def _closest_point_between_axes(a: Axis, b: Axis) -> np.ndarray:
    """
    The midpoint of the shortest segment between two lines.

    For intersecting lines, this is the intersection point.
    For parallel lines, this is an arbitrary point on the midline
    (the choice doesn't affect the geometry — any point equidistant works).
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
        # Parallel axes: pick the point on a closest to q, then the
        # corresponding point on b. The midpoint is on the midline.
        return a.project(q)

    s = (np.dot(w, u) - np.dot(w, v) * uv) / denom
    t = (np.dot(w, u) * uv - np.dot(w, v)) / denom
    # s is parameter along a, t along b. Negate t because we defined w = q - p.
    point_a = p + s * u
    point_b = q + t * v
    return 0.5 * (point_a + point_b)
