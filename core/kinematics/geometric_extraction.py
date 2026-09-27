"""
Geometric extraction from a KinematicModel.

Reads the URDF-derived kinematic model and produces a pure geometric
description of the arm: joint axes as lines in space, the wrist center,
and the tool offset. No DH parameters. No frame assignment beyond the
robot base frame that the model already provides.

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
    """A joint axis as a line in the robot base frame."""
    name: str
    point: np.ndarray      # (3,) a point on the axis, in base frame
    direction: np.ndarray  # (3,) unit vector along the axis, in base frame

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

    All quantities are in the robot base frame (the frame of the true root
    link, as returned by KinematicModel.get_true_root()).

    No DH parameters. The solver computes whatever distances and angles
    it needs from these lines directly.
    """
    # The six joint axes, in kinematic order J1..J6.
    axes: List[Axis]

    # The wrist center: the point where J5 and J6 axes intersect (spherical
    # wrist) or come closest (offset wrist). In base frame.
    wrist_center: np.ndarray

    # Distance from wrist center to TCP, along the tool Z axis.
    # This is the "d6" of the derivation, but named for what it is.
    tool_offset: float

    # Joint limits, in kinematic order. (lower, upper) per joint.
    joint_limits: List[Tuple[float, float]]

    # Names of the joints, in kinematic order.
    joint_names: List[str]

    # The perpendicular distance from the J4 axis to the wrist center.
    # This is the radius of the circle the J4 end traces around P5.
    # Zero for a spherical wrist; nonzero for UR and (possibly) FR.
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
        lines.append(f"  wrist_center: {self.wrist_center.round(6)}")
        lines.append(f"  wrist_offset: {self.wrist_offset:.6f}")
        lines.append(f"  tool_offset:  {self.tool_offset:.6f}")
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

    # All transforms at zero configuration. compute_fk does not mutate state.
    q_zero = np.zeros(len([j for j in model.joints.values()
                           if j['type'] != 'fixed']))
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

    # Tool offset: distance from wrist center to TCP along the tool Z axis.
    # The tool transform is the model's current tool; TCP = tool_mount @ tool.
    # At zero config, TCP is:
    tcp_pose = model.forward_kinematics(q_zero)
    tcp_in_base = T_base_inv @ tcp_pose
    tcp_pos = tcp_in_base[:3, 3]
    tool_offset = float(np.linalg.norm(tcp_pos - wrist_center))

    # Joint limits, in arm chain order.
    joint_limits = []
    for joint_name in arm_chain:
        lim = model.joints[joint_name]['limit']
        joint_limits.append((lim['lower'], lim['upper']))

    return ArmGeometry(
        axes=axes,
        wrist_center=wrist_center,
        tool_offset=tool_offset,
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

    Returns a point in the base frame.
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
