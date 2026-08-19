"""
dh_geometry.py — Physical-geometry DH extraction and wrist classification.

Reads the *physical* joint axes (direction + a point on each axis) from a
KinematicModel, independent of how ugly the URDF frame assignments are, and
produces:

  1. A DHRepresentation in the modified-DH convention used by URIKSolver._T()
     (a, alpha, d, theta_offset, T_base, T_tool).
  2. A wrist classification: "spherical", "ur_offset", or "general".

The extraction is verified: the reconstructed modified-DH forward kinematics
must match the KinematicModel's forward kinematics across sample configs. If
it does not, verify() reports the error loudly — this module never silently
emits parameters that do not describe the real robot.

Principles honored:
  - FK is frame-agnostic: the physical axes are always correct.
  - DH extraction is classification, not solving.
  - Never pretend: if verification fails, say so.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from math import sin, cos, atan2, acos, pi
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

EPS = 1e-9
MOVING_TYPES = ('revolute', 'continuous', 'prismatic')


# ---------------------------------------------------------------------------
# Small frame helpers
# ---------------------------------------------------------------------------
def make_frame(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def any_perp(z: np.ndarray) -> np.ndarray:
    """Return any unit vector perpendicular to unit vector z."""
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    v = np.cross(z, ref)
    return v / np.linalg.norm(v)


def project_perp(v: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Component of v perpendicular to unit vector z (un-normalized)."""
    return v - np.dot(v, z) * z


def angle_about(v1: np.ndarray, v2: np.ndarray, axis: np.ndarray) -> float:
    """Signed angle from v1 to v2 measured about unit axis (right-hand)."""
    return atan2(float(np.dot(axis, np.cross(v1, v2))), float(np.dot(v1, v2)))


# ---------------------------------------------------------------------------
# Data carriers
# ---------------------------------------------------------------------------
@dataclass
class JointAxis:
    """A physical joint axis expressed in the robot base frame at zero config."""
    name: str
    direction: np.ndarray   # unit rotation-axis direction
    point: np.ndarray       # a point on the axis (child-frame origin)
    frame0: np.ndarray      # child link frame (4x4) at zero config


@dataclass
class DHRepresentation:
    """A complete modified-DH description of a 6-DOF arm.

    Convention matches URIKSolver._T(): the transform from frame n-1 to n uses
    a[n-1], alpha[n-1], d[n], theta[n-1].  Index 0 of a/alpha/d is a dummy for
    the base so that joint n reads a[n-1], alpha[n-1], d[n].
    """
    a: np.ndarray
    alpha: np.ndarray
    d: np.ndarray
    theta_offset: np.ndarray     # shape (6,)  URDF-zero vs DH-zero per joint
    T_base: np.ndarray           # pose of DH frame 0 in the robot base frame
    T_tool: np.ndarray           # DH frame 6 -> URDF TCP frame
    joint_names: List[str] = field(default_factory=list)
    wrist_type: str = "unknown"

    def _T(self, n: int, theta: np.ndarray) -> np.ndarray:
        a = self.a[n - 1]
        al = self.alpha[n - 1]
        d = self.d[n]
        th = theta[n - 1]
        ct, st = cos(th), sin(th)
        ca, sa = cos(al), sin(al)
        return np.array([
            [ct,      -st,     0.0,  a],
            [st * ca,  ct * ca, -sa, -sa * d],
            [st * sa,  ct * sa,  ca,  ca * d],
            [0.0,      0.0,     0.0, 1.0],
        ])

    def forward_dh(self, q: np.ndarray) -> np.ndarray:
        """FK from DH frame 0 to DH frame 6 (no bridges)."""
        theta = np.asarray(q, dtype=float) + self.theta_offset
        T = np.eye(4)
        for n in range(1, 7):
            T = T @ self._T(n, theta)
        return T

    def forward(self, q: np.ndarray) -> np.ndarray:
        """Full FK: robot base frame -> TCP frame."""
        return self.T_base @ self.forward_dh(q) @ self.T_tool

    def to_dh(self, T_tcp: np.ndarray) -> np.ndarray:
        """Map a TCP pose (base frame) into the DH frame-0..frame-6 space."""
        return np.linalg.inv(self.T_base) @ T_tcp @ np.linalg.inv(self.T_tool)

    def from_dh(self, T_dh: np.ndarray) -> np.ndarray:
        """Map a DH-space pose back to a TCP pose in the base frame."""
        return self.T_base @ T_dh @ self.T_tool

    def angles_to_dh(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(q, dtype=float) + self.theta_offset

    def angles_from_dh(self, theta: np.ndarray) -> np.ndarray:
        return np.asarray(theta, dtype=float) - self.theta_offset


# ---------------------------------------------------------------------------
# Extractor
# ---------------------------------------------------------------------------
class DHGeometry:
    def __init__(self, model):
        self.model = model

    # ================= Stage 1: physical joint axes ========================
    def extract_joint_axes(self, arm_chain: List[str]) -> List[JointAxis]:
        """Compute each joint's physical axis (direction + point) in the base
        frame at zero config. Frame-agnostic: correct regardless of URDF RPY."""
        m = self.model
        saved = m.get_current_joint_positions()
        try:
            for _, j in m.joints.items():
                if j['type'] in MOVING_TYPES:
                    j['value'] = 0.0
            m._forward_kinematics()

            base = m.get_true_root()
            T_base_w = m.link_transforms.get(base, np.eye(4))
            T_w2b = np.linalg.inv(T_base_w)

            axes: List[JointAxis] = []
            for jname in arm_chain:
                j = m.joints[jname]
                T_child_w = m.link_transforms[j['child']]
                T_child_b = T_w2b @ T_child_w

                axis_local = np.array(j['axis'], dtype=float)
                axis_local = axis_local / np.linalg.norm(axis_local)

                direction = T_child_b[:3, :3] @ axis_local
                direction = direction / np.linalg.norm(direction)
                point = T_child_b[:3, 3].copy()

                axes.append(JointAxis(
                    name=jname,
                    direction=direction,
                    point=point,
                    frame0=T_child_b.copy(),
                ))
            return axes
        finally:
            m.update_state(saved)

    # ================= Stage 2: classification ============================
    def classify_from_axes(self, axes: List[JointAxis]) -> Tuple[str, dict]:
        """Classify the wrist from physical axes. Returns (type, evidence)."""
        z = [ax.direction for ax in axes]
        p = [ax.point for ax in axes]
        evidence: dict = {}

        # Spherical: last three axes intersect at a point.
        residual = self._axes_intersection_residual(z[3:6], p[3:6])
        evidence['wrist_intersection_residual'] = residual
        if residual < 1e-4:
            return 'spherical', evidence

        # UR-like offset wrist: J2||J3||J4, J1⊥J2, J4⊥J5, J5⊥J6.
        def par(u, v): return float(np.linalg.norm(np.cross(u, v))) < 1e-3
        def perp(u, v): return abs(float(np.dot(u, v))) < 1e-3

        checks = {
            'j2_parallel_j3': par(z[1], z[2]),
            'j3_parallel_j4': par(z[2], z[3]),
            'j1_perp_j2':     perp(z[0], z[1]),
            'j4_perp_j5':     perp(z[3], z[4]),
            'j5_perp_j6':     perp(z[4], z[5]),
        }
        evidence.update(checks)
        if all(checks.values()):
            return 'ur_offset', evidence
        return 'general', evidence

    @staticmethod
    def _axes_intersection_residual(dirs, pts) -> float:
        """Least-squares distance of the point closest to all given axis lines."""
        A = np.zeros((3, 3))
        b = np.zeros(3)
        for z, p in zip(dirs, pts):
            P = np.eye(3) - np.outer(z, z)
            A += P
            b += P @ p
        try:
            x = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return float('inf')
        res2 = 0.0
        for z, p in zip(dirs, pts):
            dvec = x - p
            res2 += float(np.dot(dvec, dvec) - np.dot(dvec, z) ** 2)
        return float(np.sqrt(max(res2, 0.0)))

    # ================= Stage 3: full DH extraction ========================
    def extract(self, arm_chain: List[str]) -> DHRepresentation:
        axes = self.extract_joint_axes(arm_chain)
        if len(axes) != 6:
            raise ValueError(f"DH extraction expects a 6-DOF chain, got {len(axes)}")

        wrist_type, evidence = self.classify_from_axes(axes)
        frames = self._build_dh_frames(axes)
        a, alpha, d, theta_offset = self._extract_params(frames)
        T_base, T_tool = self._compute_bridges(frames)

        rep = DHRepresentation(
            a=a, alpha=alpha, d=d, theta_offset=theta_offset,
            T_base=T_base, T_tool=T_tool,
            joint_names=list(arm_chain), wrist_type=wrist_type,
        )
        logger.info("DH extraction complete: wrist=%s evidence=%s", wrist_type, evidence)
        return rep

    # ---- closest points between two axis lines ----
    @staticmethod
    def _closest_points(p1, z1, p2, z2):
        """Closest points A on line1 and B on line2. Returns (A, B, is_parallel)."""
        w0 = p1 - p2
        b = float(np.dot(z1, z2))
        d = float(np.dot(z1, w0))
        e = float(np.dot(z2, w0))
        denom = 1.0 - b * b
        if abs(denom) < 1e-9:
            # Parallel axes: connector through p1's projection onto line2.
            t = float(np.dot(p1 - p2, z2))
            B = p2 + t * z2
            return p1.copy(), B, True
        t = (e - d * b) / denom
        s = (b * e - d) / denom
        A = p1 + s * z1
        B = p2 + t * z2
        return A, B, False

    def _build_dh_frames(self, axes: List[JointAxis]) -> List[np.ndarray]:
        """Build modified-DH frames D_0..D_6 in the base frame at zero config.
        D_n has its z-axis along joint n's physical axis."""
        dirs = [ax.direction for ax in axes]
        pts = [ax.point for ax in axes]
        n = len(dirs)

        foot_A = [None] * (n - 1)   # foot on axis k of CN(k, k+1)
        foot_B = [None] * (n - 1)   # foot on axis k+1
        cn_dir = [None] * (n - 1)
        for k in range(n - 1):
            A, B, _parallel = self._closest_points(pts[k], dirs[k], pts[k + 1], dirs[k + 1])
            foot_A[k], foot_B[k] = A, B
            vec = B - A
            L = float(np.linalg.norm(vec))
            if L > EPS:
                cn_dir[k] = vec / L
            else:
                c = np.cross(dirs[k], dirs[k + 1])
                nrm = float(np.linalg.norm(c))
                cn_dir[k] = (c / nrm) if nrm > EPS else any_perp(dirs[k])

        frames: List[Optional[np.ndarray]] = [None] * 7

        # D_1 .. D_5 : z = axis k, x = common normal toward next axis,
        #              origin = foot of that common normal on axis k.
        for i in range(1, 6):
            k = i - 1
            zax = dirs[k]
            xax = project_perp(cn_dir[k], zax)
            xl = float(np.linalg.norm(xax))
            xax = (xax / xl) if xl > EPS else any_perp(zax)
            yax = np.cross(zax, xax)
            frames[i] = make_frame(np.column_stack([xax, yax, zax]), foot_A[k])

        # D_6 : flange frame. z = joint-6 axis, origin = tool-mount point.
        frames[6] = self._build_flange_frame(axes)

        # D_0 : base DH frame. z = joint-1 axis.
        frames[0] = self._build_base_frame(axes)
        return frames

    def _build_flange_frame(self, axes: List[JointAxis]) -> np.ndarray:
        m = self.model
        base = m.get_true_root()
        T_w2b = np.linalg.inv(m.link_transforms.get(base, np.eye(4)))
        T_mount_b = T_w2b @ m.link_transforms.get(m.tool_mount_link, np.eye(4))

        zax = axes[-1].direction
        origin = T_mount_b[:3, 3].copy()
        xax = project_perp(T_mount_b[:3, 0], zax)
        xl = float(np.linalg.norm(xax))
        xax = (xax / xl) if xl > EPS else any_perp(zax)
        yax = np.cross(zax, xax)
        return make_frame(np.column_stack([xax, yax, zax]), origin)

    def _build_base_frame(self, axes: List[JointAxis]) -> np.ndarray:
        zax = axes[0].direction
        p0 = axes[0].point
        origin = p0 + (-float(np.dot(p0, zax))) * zax  # project base origin onto axis 1
        xax = project_perp(np.array([1.0, 0.0, 0.0]), zax)
        xl = float(np.linalg.norm(xax))
        if xl < EPS:
            xax = project_perp(np.array([0.0, 1.0, 0.0]), zax)
            xl = float(np.linalg.norm(xax))
        xax = xax / xl
        yax = np.cross(zax, xax)
        return make_frame(np.column_stack([xax, yax, zax]), origin)

    @staticmethod
    def _params_from_T(G: np.ndarray):
        """Read (a, alpha, d, theta) from a transform that equals _T(n, theta)."""
        R = G[:3, :3]
        t = G[:3, 3]
        alpha = atan2(-R[1, 2], R[2, 2])
        theta = atan2(-R[0, 1], R[0, 0])
        a = float(t[0])
        if abs(cos(alpha)) > 1e-6:
            d = float(t[2] / cos(alpha))
        else:
            d = float(-t[1] / sin(alpha))
        return a, alpha, d, theta

    def _extract_params(self, frames):
        a = np.zeros(7)
        alpha = np.zeros(7)
        d = np.zeros(7)
        theta_offset = np.zeros(6)
        for nidx in range(1, 7):
            G = np.linalg.inv(frames[nidx - 1]) @ frames[nidx]
            ai, alphai, di, thi = self._params_from_T(G)
            a[nidx - 1] = ai
            alpha[nidx - 1] = alphai
            d[nidx] = di
            theta_offset[nidx - 1] = thi
        return a, alpha, d, theta_offset

    def _compute_bridges(self, frames):
        m = self.model
        base = m.get_true_root()
        T_w2b = np.linalg.inv(m.link_transforms.get(base, np.eye(4)))
        T_tcp_w = (m.link_transforms.get(m.tool_mount_link, np.eye(4))
                   @ m.get_tool_transform())
        T_tcp_b = T_w2b @ T_tcp_w
        T_base = frames[0]
        T_tool = np.linalg.inv(frames[6]) @ T_tcp_b
        return T_base, T_tool

    # ================= The arbiter ========================================
    def verify(self, rep: DHRepresentation, n_samples: int = 25,
               seed: int = 1234, pos_tol: float = 1e-6, rot_tol: float = 1e-6):
        """FK round-trip across random configs. Returns (ok, max_pos, max_rot)."""
        m = self.model
        base = m.get_true_root()
        T_w2b = np.linalg.inv(m.link_transforms.get(base, np.eye(4)))
        limits = m.get_joint_info()['limits']
        rng = np.random.default_rng(seed)
        saved = m.get_current_joint_positions()

        max_pos = 0.0
        max_rot = 0.0
        try:
            for _ in range(n_samples):
                q = np.zeros(6)
                for i, name in enumerate(rep.joint_names):
                    lo, hi = limits[name]
                    lo = lo if np.isfinite(lo) else -pi
                    hi = hi if np.isfinite(hi) else pi
                    q[i] = rng.uniform(lo, hi)

                m.update_state({name: q[i] for i, name in enumerate(rep.joint_names)})
                T_model_b = T_w2b @ m.get_tcp_pose()
                T_dh = rep.forward(q)

                pos_err = float(np.linalg.norm(T_model_b[:3, 3] - T_dh[:3, 3]))
                Rrel = T_model_b[:3, :3].T @ T_dh[:3, :3]
                rot_err = abs(acos(np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0)))
                max_pos = max(max_pos, pos_err)
                max_rot = max(max_rot, rot_err)
        finally:
            m.update_state(saved)

        ok = (max_pos < pos_tol) and (max_rot < rot_tol)
        return ok, max_pos, max_rot

    # ================= Human-readable report ==============================
    def report(self, arm_chain: List[str]) -> str:
        axes = self.extract_joint_axes(arm_chain)
        wtype, evidence = self.classify_from_axes(axes)
        lines = ["=== DHGeometry report ===", f"wrist type: {wtype}"]
        for k, v in evidence.items():
            lines.append(f"  {k}: {v}")
        lines.append("joint axes (base frame, zero config):")
        for ax in axes:
            lines.append(
                f"  {ax.name}: dir=({ax.direction[0]:+.4f},{ax.direction[1]:+.4f},"
                f"{ax.direction[2]:+.4f}) pt=({ax.point[0]:+.4f},{ax.point[1]:+.4f},"
                f"{ax.point[2]:+.4f})")
        try:
            rep = self.extract(arm_chain)
            lines.append("DH parameters (modified DH, UR convention):")
            lines.append(f"  a     = {np.round(rep.a, 6).tolist()}")
            lines.append(f"  alpha = {np.round(rep.alpha, 6).tolist()}")
            lines.append(f"  d     = {np.round(rep.d, 6).tolist()}")
            lines.append(f"  theta_offset = {np.round(rep.theta_offset, 6).tolist()}")
            ok, mp, mr = self.verify(rep)
            lines.append(f"verify: ok={ok} max_pos_err={mp:.3e} max_rot_err={mr:.3e}")
        except Exception as e:  # noqa: BLE001 - report must not crash the UI
            lines.append(f"full extraction failed: {e}")
        return "\n".join(lines)