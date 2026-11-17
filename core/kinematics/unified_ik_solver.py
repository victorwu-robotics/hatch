"""
Unified analytical inverse kinematics for arm-plane 6R manipulators.

One derivation for offset and spherical wrists alike, following
docs/unified_IK.md Parts 5-9. No DH table, no per-arm branches, no
numerical iteration.

The whole solver rests on one formula (Part 7):

    A = P5 - rho * a4 - delta * a5

where A is the wrist root (the arm chain's target), P5 the wrist centre,
rho = |AB| the wrist normal offset, delta = |BC| the wrist radial offset,
and a4, a5 the directions of J4's and J5's axes.

Each term needs a direction, and a direction is available once the joints
that fix it are known. The solving order is therefore *forced by the
machine's geometry*, not chosen by a wrist-type label (Part 9.3):

    J4 normal to arm plane, delta > 0   ->  theta1, wrist, A, arm
    J4 normal to arm plane, delta = 0   ->  theta1, A, arm, wrist
    J4 in the arm plane,    delta = 0   ->  theta1, A, arm, wrist
    J4 in the arm plane,    delta > 0   ->  circular dependency: refuse

UR10 and FR5 occupy the first row; Elfin E15 Pro the third. The spherical
wrist is not a special case: it is the value at delta = 0, where a term
vanishes and a direction arrives later.

Every joint angle is obtained the same way — by placing the chain and
measuring a signed angle between two vectors about a known axis. No
Euler-angle extraction, no rotation-matrix inversion, no hardcoded
per-arm constants.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from math import acos, atan2, cos, hypot, sin, sqrt
from typing import Dict, List, Optional, Tuple

import numpy as np

from .geometric_extraction import (
    ArmGeometry,
    UnsupportedArmError,
    extract_arm_geometry,
)

logger = logging.getLogger(__name__)

# Tolerances. FR5 and Elfin URDFs carry CAD rounding of ~4e-6 in their axis
# directions, producing residuals up to ~2e-6 m. Nothing here may be tighter
# than that, or Elfin is misclassified as an offset wrist (doc Appendix C.4).
GEOM_TOL = 1e-5      # metres / direction cosines: "is this zero?"
ANGLE_TOL = 1e-5     # radians
SINGULAR_TOL = 1e-7  # |a4 x a6| below this: wrist singularity
VERIFY_POS_TOL = 1e-4   # metres, FK round-trip acceptance
VERIFY_ROT_TOL = 1e-4   # radians, FK round-trip acceptance
DEDUP_TOL = 1e-6     # radians, solution identity


# ----------------------------------------------------------------------
# Small geometric helpers. Each is one operation on vectors, not matrices.
# ----------------------------------------------------------------------

def _unit(v: np.ndarray) -> np.ndarray:
    n = _norm(v)
    if n < 1e-15:
        raise ValueError("cannot normalise a zero vector")
    return v / n


def _norm(v: np.ndarray) -> float:
    """Length of a 3-vector. np.linalg.norm costs ~10x this on 3 elements."""
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _dot(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product of two 3-vectors. np.cross routes through moveaxis."""
    return np.array([a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]])


def _perp(v: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Component of v perpendicular to unit vector n."""
    return v - _dot(v, n) * n


def _signed_angle(u: np.ndarray, v: np.ndarray, n: np.ndarray) -> float:
    """
    Signed angle from u to v measured about the unit axis n, right-handed.

    u and v are projected perpendicular to n first, so the result is the
    angle in the plane normal to n regardless of small out-of-plane
    components. This is the only angle primitive the solver uses.
    """
    up = _perp(u, n)
    vp = _perp(v, n)
    nu, nv = _norm(up), _norm(vp)
    if nu < 1e-12 or nv < 1e-12:
        raise ValueError("reference or target is parallel to the rotation axis")
    up = up / nu
    vp = vp / nv
    return atan2(_dot(_cross(up, vp), n), _dot(up, vp))


def _rz(theta: float) -> np.ndarray:
    c, s = cos(theta), sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _wrap(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return float(atan2(sin(a), cos(a)))


def _rot_error(R1: np.ndarray, R2: np.ndarray) -> float:
    """Geodesic angle between two rotations, in radians."""
    c = (float(np.trace(R1.T @ R2)) - 1.0) / 2.0
    return float(acos(np.clip(c, -1.0, 1.0)))


# ----------------------------------------------------------------------
# Result type
# ----------------------------------------------------------------------

@dataclass
class IKSolution:
    """
    One joint vector, with the geometric choices that produced it.

    The tags are *geometric* names, not root indices: they keep their
    meaning as the arm moves, which is what makes configuration
    continuity checkable (doc Part 8.6).
    """
    q: np.ndarray
    shoulder: str                  # 'right' | 'left'  (which tangent)
    elbow: str                     # 'up' | 'down'
    wrist: str                     # 'no-flip' | 'flip'
    wrist_singular: bool = False   # a4 parallel a6: theta4/theta6 coupled
    shoulder_singular: bool = False
    elbow_singular: bool = False
    fk_pos_error: float = 0.0
    fk_rot_error: float = 0.0

    def tags(self) -> Tuple[str, str, str]:
        return (self.shoulder, self.elbow, self.wrist)

    def __str__(self) -> str:
        flags = []
        if self.wrist_singular:
            flags.append("wrist-singular")
        if self.shoulder_singular:
            flags.append("shoulder-singular")
        if self.elbow_singular:
            flags.append("elbow-singular")
        tail = ("  [" + ", ".join(flags) + "]") if flags else ""
        return (f"{self.shoulder:>5}/{self.elbow:>4}/{self.wrist:>7}  "
                f"q={np.round(self.q, 6)}{tail}")


# ----------------------------------------------------------------------
# The solver
# ----------------------------------------------------------------------

class UnifiedIKSolver:
    """
    Analytical IK for one arm, driven entirely by its URDF.

    Args:
        model: a loaded KinematicModel.
        geometry: optional pre-extracted ArmGeometry (must be in the
            model's true-root frame).
        q_reference: joint vector used to resolve the wrist singularity
            by continuity. May be updated between calls.

    Raises:
        UnsupportedArmError: if the arm is outside the supported class.
            The message names the violated condition and its measured
            value. The solver never approximates a non-class arm.
    """

    def __init__(self,
                 model,
                 geometry: Optional[ArmGeometry] = None,
                 q_reference: Optional[np.ndarray] = None):
        self.model = model
        self.root = model.get_true_root()
        self.flange = model.get_flange()
        self.chain: List[str] = list(model.get_arm_chain(self.root))
        if len(self.chain) != 6:
            raise UnsupportedArmError(
                f"expected 6 revolute joints in the arm chain, got "
                f"{len(self.chain)}: {self.chain}")

        self.geom = geometry if geometry is not None else extract_arm_geometry(model)
        self.q_reference = (np.zeros(6) if q_reference is None
                            else np.asarray(q_reference, dtype=float).copy())

        # Parent link of each joint, and the joint's own origin transform.
        self._parent = [model.joints[j]['parent'] for j in self.chain]
        self._origin = [model._compose_transform(model.joints[j]['origin_xyz'],
                                                 model.joints[j]['origin_rpy'])
                        for j in self.chain]
        self._axis_local = []
        for j in self.chain:
            a = np.asarray(model.joints[j]['axis'], dtype=float)
            self._axis_local.append(_unit(a))

        # The fast chain walk assumes each joint's child is the next
        # joint's parent. True for all three arms; if a URDF interleaves
        # fixed joints between arm joints, fall back to the scene FK.
        self._chain_adjacent = all(
            model.joints[self.chain[i]]['child']
            == model.joints[self.chain[i + 1]]['parent'] for i in range(5))
        tr0 = model.compute_fk({name: 0.0 for name in self.chain})
        Ti0 = np.linalg.inv(tr0[self.root])
        self._T_pre = Ti0 @ tr0[self._parent[0]]
        last_child = model.joints[self.chain[-1]]['child']
        self._T_post = np.linalg.inv(tr0[last_child]) @ tr0[self.flange]

        # Split the fixed transforms once, and precompute each joint's
        # rotation generator, so placing the chain is 3x3 products only.
        self._R_pre = self._T_pre[:3, :3].copy()
        self._p_pre = self._T_pre[:3, 3].copy()
        self._R_post = self._T_post[:3, :3].copy()
        self._p_post = self._T_post[:3, 3].copy()
        self._Ro = [T[:3, :3].copy() for T in self._origin]
        self._to = [T[:3, 3].copy() for T in self._origin]
        self._place_cache = {}
        self._I3 = np.eye(3)
        self._K, self._KK = [], []
        for a in self._axis_local:
            K = np.array([[0.0, -a[2], a[1]],
                          [a[2], 0.0, -a[0]],
                          [-a[1], a[0], 0.0]])
            self._K.append(K)
            self._KK.append(K @ K)

        self._extract_constants()
        self._check_class()

    # ------------------------------------------------------------------
    # Placement: the only forward-kinematics the solver performs
    # ------------------------------------------------------------------

    def _place(self, q) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], np.ndarray]:
        """
        Place the chain at the given (possibly partial) joint vector.

        Returns the six joint axes as (point, direction) pairs and the
        flange pose, all in the true-root frame. Unsolved joints are zero.

        This walks the six arm joints directly rather than calling the
        scene-wide FK. The solver places the chain ~20 times per solve,
        and rebuilding every link in the scene each time made an
        analytical solver cost milliseconds. The two fixed transforms
        bookending the chain are measured once from the model, so no
        geometry is hardcoded here.
        """
        key = (float(q[0]), float(q[1]), float(q[2]),
               float(q[3]), float(q[4]), float(q[5]))
        hit = self._place_cache.get(key)
        if hit is not None:
            return hit
        out = self._place_uncached(q)
        self._place_cache[key] = out
        return out

    def _place_uncached(self, q):
        if not self._chain_adjacent:
            return self._place_via_model(q)
        R = self._R_pre
        p = self._p_pre
        joints = []
        for i in range(6):
            Rj = R @ self._Ro[i]
            pj = p + R @ self._to[i]
            joints.append((pj, _unit(Rj @ self._axis_local[i])))
            th = float(q[i])
            R = Rj @ (self._I3 + sin(th) * self._K[i]
                      + (1.0 - cos(th)) * self._KK[i])
            p = pj
        Tf = np.eye(4)
        Tf[:3, :3] = R @ self._R_post
        Tf[:3, 3] = p + R @ self._p_post
        return joints, Tf

    def _place_via_model(self, q):
        """
        The same placement through the scene-wide FK. Kept as the
        independent reference that the fast walk is tested against.

        Joint values are passed by *name*, never by position: compute_fk
        maps a list onto every non-fixed joint in document order, which
        is only the arm chain by coincidence.
        """
        qd = {name: float(q[i]) for i, name in enumerate(self.chain)}
        tr = self.model.compute_fk(qd)
        Ti = np.linalg.inv(tr[self.root])
        joints = []
        for i in range(6):
            Tj = Ti @ tr[self._parent[i]] @ self._origin[i]
            joints.append((Tj[:3, 3].copy(), _unit(Tj[:3, :3] @ self._axis_local[i])))
        return joints, Ti @ tr[self.flange]

    def _arm_plane_normal(self, theta1: float) -> np.ndarray:
        return _unit(_rz(theta1) @ self._n0)

    def _project_to_arm_plane(self, p: np.ndarray, n: np.ndarray) -> np.ndarray:
        """The arm plane passes through the root origin (rho_arm = 0)."""
        return p - _dot(p, n) * n

    # ------------------------------------------------------------------
    # Extraction of the machine constants
    # ------------------------------------------------------------------

    def _extract_constants(self) -> None:
        g = self.geom
        J0, Tf0 = self._place(np.zeros(6))
        p2_0, a2_0 = J0[1]
        p3_0, a3_0 = J0[2]
        p4_0, a4_0 = J0[3]
        p5_0, a5_0 = J0[4]
        p6_0, a6_0 = J0[5]

        self._n0 = a2_0
        self._az2 = float(np.arctan2(a2_0[1], a2_0[0]))
        self._n0_xy = float(hypot(a2_0[0], a2_0[1]))

        # Wrist triangle, from axes as lines (origin-independent).
        B0 = p4_0 + _dot(p5_0 - p4_0, a4_0) * a4_0
        P5_0 = p5_0 + _dot(p6_0 - p5_0, a5_0) * a5_0
        A0 = self._project_to_arm_plane(B0, a2_0)

        self.rho = float(_norm(B0 - A0))
        self.delta = float(_norm(P5_0 - B0))

        # Signed h: the perpendicular distance from P5 to the arm plane.
        # A machine constant, not a branch (doc Part 5.2).
        self.h = _dot(P5_0, a2_0)

        # Signed flange offset. The sign depends on which link the model
        # designates as the flange, so it is measured, never assumed.
        self.d6 = _dot(Tf0[:3, 3] - P5_0, a6_0)

        # Shoulder offset: does the arm plane contain J1's axis?
        self.rho_arm = max(abs(_dot(p2_0, a2_0)),
                           abs(_dot(p3_0, a2_0)))

        # Is J4 normal to the arm plane (pitch) or in it (roll)?
        self._a4_dot_n = abs(_dot(a4_0, a2_0))
        self.pitch_wrist = self._a4_dot_n > 1.0 - GEOM_TOL
        # Sign relating a4 to the plane normal, for pitch wrists.
        self._a4_sign = float(np.sign(_dot(a4_0, a2_0))) if self.pitch_wrist else 0.0

        # Arm chain lengths: both constants of the machine.
        E0 = self._project_to_arm_plane(p3_0, a2_0)
        self.L2 = float(_norm(E0 - p2_0))
        self.L3 = float(_norm(A0 - E0))

        # Wrist angle constants: the angle each joint's geometric measure
        # reads at the URDF zero pose. Measured, not hardcoded (doc 6.6).
        self._c5 = self._angle_or_zero(a4_0, a6_0, a5_0)
        self._c6 = self._angle_or_zero(a5_0, Tf0[:3, :3][:, 0], a6_0)

        # How inconsistent is this URDF with its own class conditions?
        # A direction cannot be resolved more finely than the model
        # defines it, so this sets the singularity threshold. Using a
        # fixed 1e-7 would call FR5's and Elfin's singular wrists
        # resolvable, because their axes are tilted by 3.7e-6 rad.
        self.model_tilt = max(
            abs(_dot(a4_0, a5_0)),
            abs(_dot(a5_0, a6_0)),
            abs(abs(_dot(a4_0, a6_0)) - 1.0),
            abs(abs(_dot(a2_0, a3_0)) - 1.0),
            abs(float(a2_0[2])))
        self.singular_tol = max(SINGULAR_TOL, 10.0 * self.model_tilt)

        # Joint limits.
        self.limits = list(g.joint_limits)

        self.diagnostics: Dict[str, object] = {}

    @staticmethod
    def _angle_or_zero(u, v, n) -> float:
        """Signed angle, or 0 when the reference is parallel to the axis."""
        try:
            return _signed_angle(u, v, n)
        except ValueError:
            return 0.0

    def _check_class(self) -> None:
        """
        The class conditions of doc Part 9.2, measured here rather than
        trusted. Refusal names the violation and the value.
        """
        J0, Tf0 = self._place(np.zeros(6))
        a = [d for _, d in J0]

        if abs(_dot(a[3], a[4])) > GEOM_TOL:
            raise UnsupportedArmError(
                f"J4 and J5 are not perpendicular: |a4.a5| = "
                f"{abs(_dot(a[3], a[4])):.3e} > {GEOM_TOL:.1e}")
        if abs(_dot(a[4], a[5])) > GEOM_TOL:
            raise UnsupportedArmError(
                f"J5 and J6 are not perpendicular: |a5.a6| = "
                f"{abs(_dot(a[4], a[5])):.3e} > {GEOM_TOL:.1e}")
        if abs(abs(_dot(a[3], a[5])) - 1.0) > GEOM_TOL:
            raise UnsupportedArmError(
                f"J4 and J6 are not parallel at the zero pose: |a4.a6| = "
                f"{abs(_dot(a[3], a[5])):.6f}, expected 1. "
                f"This is a structural check on the frame conventions, "
                f"not on a live configuration.")
        if self.rho_arm > GEOM_TOL:
            raise UnsupportedArmError(
                f"shoulder offset rho_arm = {self.rho_arm:.6f} m > "
                f"{GEOM_TOL:.1e}: the arm plane does not contain J1's axis. "
                f"theta_1 would require common tangents to two circles "
                f"(doc Appendix B.3), which is not implemented.")
        # The flange Z must be J6's axis, or P5 = P6 - d6*a6 is wrong.
        z_err = float(_norm(Tf0[:3, :3][:, 2] - a[5]))
        if z_err > GEOM_TOL:
            raise UnsupportedArmError(
                f"flange Z is not J6's axis (|difference| = {z_err:.3e}). "
                f"The flange back-off P5 = P6 - d6*a6 requires it.")
        # The circular-dependency refusal (doc Appendix B.1).
        if self.delta > GEOM_TOL and not self.pitch_wrist:
            raise UnsupportedArmError(
                f"delta = {self.delta:.6f} m > 0 but J4 lies in the arm "
                f"plane (|a4.n| = {self._a4_dot_n:.6f}). Locating A needs "
                f"a5, which needs a4, which needs the arm chain, which "
                f"needs A: the dependency closes into a loop, so no "
                f"closed-form solution exists in this order.")
        # h must be a machine constant, not a function of configuration.
        for probe in (np.array([0.4, -0.7, 0.9, 0.6, 1.1, -0.3]),
                      np.array([-0.8, -1.1, 1.3, -0.5, 0.9, 0.7])):
            Jp, Tfp = self._place(probe)
            ap = [d for _, d in Jp]
            P5p = Tfp[:3, 3] - self.d6 * Tfp[:3, :3][:, 2]
            hp = _dot(P5p, ap[1])
            if abs(hp - self.h) > 1e-4:
                raise UnsupportedArmError(
                    f"h is not a machine constant: {self.h:.6f} at zero "
                    f"pose vs {hp:.6f} at a probe pose. The perpendicular "
                    f"distance from P5 to the arm plane must not vary "
                    f"with configuration (doc Part 5.3).")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(self,
              T_flange: np.ndarray,
              q_reference: Optional[np.ndarray] = None,
              verify: bool = True,
              respect_limits: bool = True) -> List[IKSolution]:
        """
        All analytical solutions for a flange pose in the true-root frame.

        Args:
            T_flange: 4x4 target pose of the *flange*, not the TCP. Remove
                the tool transform first — a bent welding torch puts the
                TCP off J6's axis, and the back-off would be wrong.
            q_reference: joint vector for continuity at the wrist
                singularity. Defaults to the solver's stored reference.
            verify: drop solutions whose FK round-trip exceeds tolerance.
                The count dropped is reported in self.diagnostics.
            respect_limits: drop solutions outside the joint limits
                (after one 2*pi wrap attempt per joint).

        Returns:
            Up to 8 tagged solutions. Fewer at singular loci.
        """
        if q_reference is not None:
            self.q_reference = np.asarray(q_reference, dtype=float).copy()

        # Placements repeat across branches: the theta_1-only placement is
        # shared by both wrist branches, and theta_4 depends on only four
        # distinct arm configurations. Memoise for the duration of a solve.
        self._place_cache = {}

        T = np.asarray(T_flange, dtype=float)
        P6 = T[:3, 3].copy()
        R = T[:3, :3].copy()
        a6 = _unit(R[:, 2])
        Xf = _unit(R[:, 0])

        # Step 1 — the wrist centre.
        P5 = P6 - self.d6 * a6

        solutions: List[IKSolution] = []
        rejected_fk = 0
        rejected_limits = 0

        for th1, shoulder, sh_singular in self._solve_theta1(P5):
            n = self._arm_plane_normal(th1)

            if self.delta > GEOM_TOL:
                # Wrist before arm: a5 is needed to locate A.
                a4 = self._a4_sign * n
                for a5, th5, th6, wrist, singular in self._wrist_branches(
                        a4, a6, Xf, th1, None):
                    A = P5 - self.rho * a4 - self.delta * a5
                    for th2, th3, elbow, el_singular in self._solve_arm(th1, A):
                        th4 = self._solve_theta4(th1, th2, th3, a5)
                        if th4 is None:
                            continue
                        solutions.append(IKSolution(
                            q=np.array([_wrap(v) for v in
                                        (th1, th2, th3, th4, th5, th6)]),
                            shoulder=shoulder, elbow=elbow, wrist=wrist,
                            wrist_singular=singular,
                            shoulder_singular=sh_singular,
                            elbow_singular=el_singular))
            else:
                # delta = 0: A is the projection of P5, no wrist needed.
                A = self._project_to_arm_plane(P5, n)
                for th2, th3, elbow, el_singular in self._solve_arm(th1, A):
                    a4 = self._a4_after_arm(th1, th2, th3)
                    for a5, th5, th6, wrist, singular in self._wrist_branches(
                            a4, a6, Xf, th1, (th2, th3)):
                        th4 = self._solve_theta4(th1, th2, th3, a5)
                        if th4 is None:
                            continue
                        solutions.append(IKSolution(
                            q=np.array([_wrap(v) for v in
                                        (th1, th2, th3, th4, th5, th6)]),
                            shoulder=shoulder, elbow=elbow, wrist=wrist,
                            wrist_singular=singular,
                            shoulder_singular=sh_singular,
                            elbow_singular=el_singular))

        solutions = self._deduplicate(solutions)

        if respect_limits:
            kept = []
            for s in solutions:
                q = self._fit_limits(s.q)
                if q is None:
                    rejected_limits += 1
                else:
                    s.q = q
                    kept.append(s)
            solutions = kept

        if verify:
            kept = []
            for s in solutions:
                _, Tf = self._place(s.q)
                s.fk_pos_error = float(_norm(Tf[:3, 3] - P6))
                s.fk_rot_error = _rot_error(Tf[:3, :3], R)
                if (s.fk_pos_error <= VERIFY_POS_TOL
                        and s.fk_rot_error <= VERIFY_ROT_TOL):
                    kept.append(s)
                else:
                    rejected_fk += 1
                    logger.debug("rejected solution %s: pos %.3e rot %.3e",
                                 s.tags(), s.fk_pos_error, s.fk_rot_error)
            solutions = kept

        self.diagnostics = {
            "n_solutions": len(solutions),
            "rejected_fk": rejected_fk,
            "rejected_limits": rejected_limits,
            "wrist_singular": any(s.wrist_singular for s in solutions),
        }
        if rejected_fk:
            logger.warning("%d candidate solution(s) failed FK verification "
                           "and were dropped", rejected_fk)
        return solutions

    def solve_closest(self,
                      T_flange: np.ndarray,
                      q_current: np.ndarray,
                      **kwargs) -> Optional[IKSolution]:
        """
        The solution nearest q_current, preferring the same configuration.

        Configuration tags dominate the metric: a branch change is what
        sweeps the arm through the workpiece, so it is penalised far more
        than joint travel. This is the mechanism that makes "the arm shall
        not change shape unless commanded to" enforceable.
        """
        sols = self.solve(T_flange, q_reference=q_current, **kwargs)
        if not sols:
            return None
        qc = np.asarray(q_current, dtype=float)
        ref_tags = self._tags_of(qc)

        def cost(s: IKSolution) -> float:
            d = np.arctan2(np.sin(s.q - qc), np.cos(s.q - qc))
            travel = float(np.sum(d ** 2))
            branch_penalty = 0.0
            if ref_tags is not None:
                for a, b in zip(s.tags(), ref_tags):
                    if a != b:
                        branch_penalty += 100.0
            return travel + branch_penalty

        return min(sols, key=cost)

    def fk(self, q: np.ndarray) -> np.ndarray:
        """Flange pose in the true-root frame. The solver's own FK contract."""
        return self.model.get_flange_pose(np.asarray(q, dtype=float))

    def describe(self) -> str:
        kind = "pitch (normal to arm plane)" if self.pitch_wrist else "roll (in arm plane)"
        order = ("theta1 -> wrist -> A -> arm" if self.delta > GEOM_TOL
                 else "theta1 -> A -> arm -> wrist")
        return (
            f"UnifiedIKSolver on {self.root} -> {self.flange}\n"
            f"  rho   (|AB|)          = {self.rho:.6f} m\n"
            f"  delta (|BC|)          = {self.delta:.6f} m\n"
            f"  |P5 A|                = {hypot(self.rho, self.delta):.6f} m\n"
            f"  h     (signed)        = {self.h:+.6f} m\n"
            f"  d6    (signed)        = {self.d6:+.6f} m\n"
            f"  rho_arm               = {self.rho_arm:.3e} m\n"
            f"  L2 (upper arm)        = {self.L2:.6f} m\n"
            f"  L3 (elbow to wrist)   = {self.L3:.6f} m\n"
            f"  az2                   = {np.degrees(self._az2):+.4f} deg\n"
            f"  c5, c6                = {self._c5:+.6f}, {self._c6:+.6f} rad\n"
            f"  model axis tilt       = {self.model_tilt:.3e} rad "
            f"(singular_tol {self.singular_tol:.1e})\n"
            f"  J4                    = {kind}, |a4.n| = {self._a4_dot_n:.6f}\n"
            f"  forced order          = {order}"
        )

    # ------------------------------------------------------------------
    # Step 2 — theta_1 (doc Part 5)
    # ------------------------------------------------------------------

    def _solve_theta1(self, P5: np.ndarray):
        """
        The arm plane is a tangent from the root origin to the circle of
        radius |h| centred at P5's projection. Two tangents, two roots.

        The constraint is signed, so there is no sigma and no +pi family.

        Written out exactly, with n0 = (s*cos(az2), s*sin(az2), nz):

            dot(P5, Rz(t1) n0) = s*r*cos(t1 + az2 - alpha) + nz*P5z = h

        so cos(t1 + az2 - alpha) = (h - nz*P5z) / (s*r). The nz*P5z term
        is not optional: FR5 and Elfin tilt J2 out of horizontal by
        3.7e-6 rad, and near the shoulder singularity that term is
        amplified by 1/sin(beta) into a visible angle error.
        """
        r = float(hypot(P5[0], P5[1]))
        if r < 1e-12:
            return                      # P5 on the J1 axis: azimuth undefined
        alpha = float(np.arctan2(P5[1], P5[0]))
        denom = self._n0_xy * r
        c = (self.h - self._n0[2] * P5[2]) / denom
        if abs(c) > 1.0 + GEOM_TOL:
            return                      # inside the cylindrical hole
        beta = float(acos(np.clip(c, -1.0, 1.0)))
        sin_beta = float(sin(beta))
        # Two distinct causes of ill-conditioning, both geometric:
        # the tangents merge (sin beta -> 0), or the azimuth of P5 is
        # itself ill-defined (r -> 0).
        singular = (sin_beta < 1e-4) or (r < 1e-4)
        for sign, name in ((-1.0, "right"), (+1.0, "left")):
            yield _wrap(alpha - self._az2 + sign * beta), name, singular

    # ------------------------------------------------------------------
    # Steps 4/6 — the wrist branches (doc Part 6)
    # ------------------------------------------------------------------

    def _wrist_branches(self, a4, a6, Xf, th1, arm):
        """
        a5 = +-normalize(a4 x a6). The sign IS the wrist branch; delta is
        a positive length and carries no sign of its own.

        At the singularity a4 || a6 the cross product vanishes and a5
        cannot be recovered from the pose. One branch is emitted, with
        theta_4 taken from the reference configuration, and it is flagged.
        """
        cr = _cross(a4, a6)
        nrm = float(_norm(cr))
        cos5 = float(np.clip(_dot(a4, a6), -1.0, 1.0))

        if nrm > self.singular_tol:
            base = cr / nrm
            for sign, name in ((+1.0, "no-flip"), (-1.0, "flip")):
                a5 = sign * base
                th5 = _wrap(_signed_angle(a4, a6, a5) - self._c5)
                th6 = _wrap(self._angle_or_zero(a5, Xf, a6) - self._c6)
                yield a5, th5, th6, name, False
        else:
            # Singular: pick theta_4 by continuity, then read a5 from the
            # placed chain. theta_6 absorbs the remainder.
            th4_ref = float(self.q_reference[3])
            if arm is None:
                a5 = self._a5_at_singular_pitch(th1, a4)
            else:
                a5 = self._a5_at_singular_roll(th1, arm[0], arm[1], th4_ref)
            if a5 is None:
                return
            th5 = 0.0 if cos5 > 0 else _wrap(np.pi)
            th5 = _wrap(th5 - self._c5)
            th6 = _wrap(self._angle_or_zero(a5, Xf, a6) - self._c6)
            yield a5, th5, th6, "singular", True

    def _a5_at_singular_pitch(self, th1, a4):
        """
        a5 at the wrist singularity, for a pitch wrist solved before the arm.

        Here A is not a point but a circle of radius delta about the
        projection of P5, so one free parameter must be supplied. theta_4
        alone does not supply it: for a pitch wrist a4 is fixed by theta_1,
        but a5's direction *within* the arm plane also depends on theta_2
        and theta_3. Placing the chain with the arm at zero would therefore
        pick the wrong member of the family.

        Instead take a5 from the whole reference configuration and rotate
        it about Z into the current arm plane. Perpendicularity to a4 is
        preserved, because a4 is itself the rotated plane normal.
        """
        qr = self.q_reference
        J, _ = self._place(qr)
        a5 = _rz(th1 - float(qr[0])) @ J[4][1]
        return a5 if abs(_dot(a5, a4)) < 1e-3 else None

    def _a5_at_singular_roll(self, th1, th2, th3, th4_ref):
        q = np.zeros(6)
        q[0], q[1], q[2], q[3] = th1, th2, th3, th4_ref
        J, _ = self._place(q)
        return J[4][1]

    def _a4_after_arm(self, th1, th2, th3):
        q = np.zeros(6)
        q[0], q[1], q[2] = th1, th2, th3
        J, _ = self._place(q)
        return J[3][1]

    # ------------------------------------------------------------------
    # Step 6 — the arm chain (doc Part 8)
    # ------------------------------------------------------------------

    def _solve_arm(self, th1: float, A: np.ndarray):
        """
        Planar 2R from J2's origin to A, with two *constant* lengths.

        The elbow point E is found by intersecting two circles in the arm
        plane; the joint angles are then the signed rotations that take
        the placed zero-pose link directions onto the solved ones. This
        handles Elfin's anti-parallel J2/J3 without a special case: the
        measured axis direction carries the sign.
        """
        n = self._arm_plane_normal(th1)
        q = np.zeros(6)
        q[0] = th1
        J, _ = self._place(q)
        p2, a2 = J[0 + 1]
        p3 = J[2][0]
        E0 = self._project_to_arm_plane(p3, n)
        u0 = E0 - p2
        if _norm(u0) < 1e-12:
            return
        u0 = _unit(u0)

        d_vec = A - p2
        D = float(_norm(d_vec))
        if D < 1e-12:
            return
        if not (abs(self.L2 - self.L3) - GEOM_TOL <= D <= self.L2 + self.L3 + GEOM_TOL):
            return
        uhat = d_vec / D
        vhat = _cross(a2, uhat)
        if _norm(vhat) < 1e-12:
            return
        vhat = _unit(vhat)

        cos_psi = (D * D + self.L2 * self.L2 - self.L3 * self.L3) / (2.0 * D * self.L2)
        cos_psi = float(np.clip(cos_psi, -1.0, 1.0))
        sin_psi = float(np.sqrt(max(0.0, 1.0 - cos_psi ** 2)))
        el_singular = sin_psi <= 1e-7

        for sign, name in ((+1.0, "up"), (-1.0, "down")):
            E = p2 + self.L2 * (cos_psi * uhat + sign * sin_psi * vhat)
            upper = _unit(E - p2)
            th2 = _wrap(_signed_angle(u0, upper, a2))

            # Place with theta_2 solved, theta_3 = 0, to read the forearm's
            # zero direction in this configuration.
            q2 = np.zeros(6)
            q2[0], q2[1] = th1, th2
            J2, _ = self._place(q2)
            p3b, a3b = J2[2]
            p4b, a4b = J2[3]
            p5b, a5b = J2[4]
            Bb = p4b + _dot(p5b - p4b, a4b) * a4b
            Ab = self._project_to_arm_plane(Bb, n)
            Eb = self._project_to_arm_plane(p3b, n)
            w_vec = Ab - Eb
            if _norm(w_vec) < 1e-12:
                continue
            w0 = _unit(w_vec)

            fore = A - E
            if _norm(fore) < 1e-12:
                continue
            th3 = _wrap(_signed_angle(w0, _unit(fore), a3b))
            yield th2, th3, name, el_singular
            if el_singular:
                break

    # ------------------------------------------------------------------
    # Step 6 — theta_4 (doc Part 8.5)
    # ------------------------------------------------------------------

    def _solve_theta4(self, th1, th2, th3, a5) -> Optional[float]:
        """
        theta_4 is the roll about a4 that carries a5 from where it sits at
        theta_4 = 0 to where the pose requires it.

        The reference is a5 *itself*, read from the chain placed with
        theta_4 = 0 and the arm joints already solved. Using the parent
        link's X axis instead would be wrong by a per-arm constant
        (pi/2 on UR10) — the reference must be the thing being rotated,
        not an arbitrary direction in the parent frame. No constant, and
        no degenerate case when a4 lies along the forearm as on Elfin.
        """
        q = np.zeros(6)
        q[0], q[1], q[2] = th1, th2, th3
        J, _ = self._place(q)
        a4 = J[3][1]
        ref = J[4][1]          # a5 at theta_4 = 0
        try:
            return _wrap(_signed_angle(ref, a5, a4))
        except ValueError:
            return None

    # ------------------------------------------------------------------
    # Limits, identity, tags
    # ------------------------------------------------------------------

    def _fit_limits(self, q: np.ndarray) -> Optional[np.ndarray]:
        out = q.copy()
        for i, (lo, hi) in enumerate(self.limits):
            if lo is None or hi is None:
                continue
            if lo - ANGLE_TOL <= out[i] <= hi + ANGLE_TOL:
                out[i] = float(np.clip(out[i], lo, hi))
                continue
            for shift in (2 * np.pi, -2 * np.pi):
                v = out[i] + shift
                if lo - ANGLE_TOL <= v <= hi + ANGLE_TOL:
                    out[i] = float(np.clip(v, lo, hi))
                    break
            else:
                return None
        return out

    @staticmethod
    def _deduplicate(sols: List[IKSolution]) -> List[IKSolution]:
        out: List[IKSolution] = []
        for s in sols:
            dup = False
            for e in out:
                d = np.arctan2(np.sin(s.q - e.q), np.cos(s.q - e.q))
                if np.max(np.abs(d)) < DEDUP_TOL:
                    dup = True
                    break
            if not dup:
                out.append(s)
        return out

    def _tags_of(self, q: np.ndarray) -> Optional[Tuple[str, str, str]]:
        """The configuration tags of a known joint vector, by re-solving."""
        Tf = self.fk(q)
        for s in self.solve(Tf, q_reference=q, respect_limits=False):
            d = np.arctan2(np.sin(s.q - q), np.cos(s.q - q))
            if np.max(np.abs(d)) < 1e-4:
                return s.tags()
        return None
