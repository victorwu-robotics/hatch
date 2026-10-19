"""
Geometric extraction from a KinematicModel.

Reads the URDF-derived kinematic model and produces a pure geometric
description of the arm: joint axes as lines in space, the wrist center,
and the flange offset. No DH parameters. No frame assignment beyond the
true-root frame that the model already provides.

All quantities are in the true-root frame (the parent of the first
movable revolute joint). The outer boundary is the flange — the tool
mount link, the end of the fixed-joint chain that follows the last
movable revolute joint. Everything before the true root and everything
after the flange is the controller's responsibility — see
docs/CONSTITUTION.md Part 3.5.

The output is the input to the unified IK solver.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import logging
logger = logging.getLogger(__name__)


# Tolerance for treating two axes as parallel or intersecting.
# Below this, we treat the geometry as exact; above, as a real feature.
_PARALLEL_TOL = 1e-6      # sin of angle between axes
_INTERSECT_TOL = 1e-6     # distance between skew lines, in metres


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
    # (docs/unified_IK.md, Part IV §2). It is a property of the arm alone
    # and is not the same as a distance to any attached tool or TCP.
    flange_offset: float

    # Joint limits, in kinematic order. (lower, upper) per joint.
    joint_limits: List[Tuple[float, float]]

    # Names of the joints, in kinematic order.
    joint_names: List[str]

    # The perpendicular distance from the J4 axis to the wrist center.
    # This is the radius of the circle the J4 end traces around P5, and
    # the quantity that decides whether the wrist is spherical (zero) or
    # offset (nonzero). Called "J4 wrist offset" in the derivation. The
    # J5 offset is zero by construction for the supported class.
    #
    # Stored as a convenience; the solver could recompute it from axes[3]
    # and wrist_center, but it is the single most important number in the
    # derivation, so it is worth naming.
    wrist_offset: float = field(init=False)

    def __post_init__(self):
        self.wrist_offset = self.axes[3].distance_to(self.wrist_center)

    # ------------------------------------------------------------------
    # Derived quantities the solver will ask for.
    # These are computed on demand, not stored as a parameter table.
    # ------------------------------------------------------------------

    def forearm_length(self) -> float:
        """
        Distance from J3's axis to the wrist center P5.

        This is the corrected a3 of the IK derivation (docs/unified_IK.md,
        Part VI §3). It is NOT axis_gap(2): the latter is the perpendicular
        distance between J3 and J4 axes, which coincides with the forearm
        length only for a spherical wrist. For an offset wrist (UR, FR),
        they differ, and the law of cosines in Part VI §5 uses this
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
        """True if axis i and axis i+1 are parallel."""
        u = self.axes[i].direction
        v = self.axes[i + 1].direction
        return float(np.linalg.norm(np.cross(u, v))) < _PARALLEL_TOL

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
        if n_norm < _PARALLEL_TOL:
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
        if n_norm < _PARALLEL_TOL:
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

        Only defined for 1 <= i <= 4 (needs both neighbours).
        For i = 0 or i = 5, raises IndexError.
        """
        if i < 1 or i > 4:
            raise IndexError(f"axis_shift undefined for joint {i}; needs neighbours")
        foot_prev = self.common_perpendicular_foot(i - 1)
        foot_next = self.common_perpendicular_foot(i)
        u = self.axes[i].direction
        return float(np.dot(foot_next - foot_prev, u))

    def describe(self) -> str:
        """Human-readable summary. Useful in tests and logs."""
        lines = ["ArmGeometry:"]
        for i, ax in enumerate(self.axes):
            lines.append(f"  J{i+1} {ax.name}: point={ax.point.round(6)}, "
                         f"dir={ax.direction.round(6)}")
        lines.append(f"  wrist_center:  {self.wrist_center.round(6)}")
        lines.append(f"  wrist_offset:  {self.wrist_offset:.6f}")
        lines.append(f"  flange_offset: {self.flange_offset:.6f}")
        lines.append(f"  forearm_length (J3->P5): {self.forearm_length():.6f}")
        lines.append(f"  axis_gap(2) (J3->J4):    {self.axis_gap(2):.6f}")
        for i in range(5):
            lines.append(f"  axis_twist[{i}] = {self.axis_twist(i):.6f} rad "
                         f"({self.axis_twist(i)/np.pi:.4f}π)")
            lines.append(f"  axis_gap[{i}]   = {self.axis_gap(i):.6f}")
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
    """
    if base_link is None:
        base_link = model.get_true_root()

    # Get the arm chain: the six revolute joints, in order.
    arm_chain = model.get_arm_chain(base_link_name=base_link)
    if len(arm_chain) != 6:
        raise ValueError(
            f"Expected 6 revolute joints in arm chain, got {len(arm_chain)}: "
            f"{arm_chain}"
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
        T_parent_in_base = T_base_inv @ T_parent_world

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

    return ArmGeometry(
        axes=axes,
        wrist_center=wrist_center,
        flange_offset=flange_offset,
        joint_limits=joint_limits,
        joint_names=list(arm_chain),
    )


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

    if denom < _PARALLEL_TOL:
        # Parallel axes: pick the point on a closest to q, then the
        # corresponding point on b. The midpoint is on the midline.
        return a.project(q)

    s = (np.dot(w, u) - np.dot(w, v) * uv) / denom
    t = (np.dot(w, u) * uv - np.dot(w, v)) / denom
    # s is parameter along a, t along b. Negate t because we defined w = q - p.
    point_a = p + s * u
    point_b = q + t * v
    return 0.5 * (point_a + point_b)
