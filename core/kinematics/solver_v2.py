"""
Unified IK solver — v2, origin-included form (corrected).

Algorithm:
    1. P5 = P_flange - d6 * a_flange
    2. theta1 from phi - phi_0 +/- alpha (or {phi, phi+pi} spherical)
    3. P_J4 = B + rotated(A_0 - B_0), B = P5 - wrist_offset * z4
    4. theta2, theta3 from 2-link chain shoulder -> P_J4
    5. R03_fk = prod_{i=1..3} R_origin_i * R(a_i_local, theta_i)
    6. R_wrist_target = R03_fk^T * R_flange
    7. (theta4, theta5, theta6) from inverting
           R_origin_4*R(a4,theta4)*R_origin_5*R(a5,theta5)*R_origin_6*R(a6,theta6)
       = R_wrist_target
    8. Filter by limits (mod 2pi), tag, return

Fixes from previous version:
  - a1_base and a4_base are computed from URDF joint frames at zero pose,
    not from geometric_extraction.axes (which returns the wrong J4 axis
    direction for UR10 and FR5).
  - phi_0 is computed from the J1 frame's x-axis in the base frame,
    projected onto the (e_x, e_y) basis. Generalizes to any URDF.
  - theta1 candidates use phi - phi_0 directly (no a1_sign factor).
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional

import logging
logger = logging.getLogger(__name__)

_REACH_TOL = 1e-6
_WRIST_TOL = 1e-6
_GRID = 180


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    return np.array([
        [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)],
    ])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _joint_origin_rot(model, joint):
    return model._compose_transform(joint['origin_xyz'],
                                    joint['origin_rpy'])[:3, :3]


def _joint_axis_local(joint):
    a = np.array(joint['axis'], dtype=float)
    n = np.linalg.norm(a)
    return a / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])


@dataclass
class Solution:
    q: np.ndarray
    tags: Dict[str, str]

    def __repr__(self):
        tags = ", ".join(f"{k}={v}" for k, v in self.tags.items())
        return f"Solution(q={np.array2string(self.q, precision=4)}, tags=({tags}))"


class UnifiedIKSolverV2:

    def __init__(self, model, geom):
        self.model = model
        self.geom = geom
        self.arm_chain = model.get_arm_chain(base_link_name=model.get_true_root())
        if len(self.arm_chain) != 6:
            raise RuntimeError(f"Expected 6 revolute joints, got {len(self.arm_chain)}")

        # Per-joint origin rotations and local axes (J1..J6).
        self.R_o = []
        self.a_local = []
        for jname in self.arm_chain:
            joint = model.joints[jname]
            self.R_o.append(_joint_origin_rot(model, joint))
            self.a_local.append(_joint_axis_local(joint))

        self.wrist_offset = float(geom.wrist_offset)
        self.flange_offset = float(geom.flange_offset)
        self.limits = geom.joint_limits

        # Extract zero-pose quantities (also sets a1_base, a4_base, phi_0).
        self._extract_zero_pose()

        self._precheck()

    def _precheck(self):
        twist_12 = self.geom.axis_twist(1)
        if not (abs(twist_12) < 1e-4 or abs(abs(twist_12) - np.pi) < 1e-4):
            raise RuntimeError(f"J2/J3 not parallel (twist={twist_12:.6f})")
        gap_56 = self.geom.axis_gap(4)
        if gap_56 > 1e-4:
            raise RuntimeError(f"J5/J6 do not intersect (gap={gap_56:.6f})")

    # --------------------------------------------------------------
    # Zero-pose extraction
    # --------------------------------------------------------------

    def _extract_zero_pose(self):
        transforms = self.model.compute_fk(np.zeros(6))
        base = self.model.get_true_root()
        T_base_inv = np.linalg.inv(transforms[base])

        def joint_frame(idx):
            jname = self.arm_chain[idx]
            j = self.model.joints[jname]
            T_p = transforms[j['parent']]
            T_o = self.model._compose_transform(j['origin_xyz'], j['origin_rpy'])
            return T_base_inv @ T_p @ T_o

        T1 = joint_frame(0)
        T2 = joint_frame(1)
        T3 = joint_frame(2)
        T4 = joint_frame(3)
        T5 = joint_frame(4)

        self.J1_origin = T1[:3, 3]
        self.J2_origin_0 = T2[:3, 3]
        self.J3_origin_0 = T3[:3, 3]
        self.A_0 = T4[:3, 3].copy()

        # Axis directions from URDF joint frames (not geom.axes — wrong for UR10/FR5).
        self.a1_base = T1[:3, :3] @ self.a_local[0]
        self.a1_base /= np.linalg.norm(self.a1_base)

        self.a4_base = T4[:3, :3] @ self.a_local[3]
        self.a4_base /= np.linalg.norm(self.a4_base)

        # B_0 = projection of J5 origin onto the J4 line.
        p5_pt = T5[:3, 3]
        self.B_0 = self.A_0 + np.dot(p5_pt - self.A_0, self.a4_base) * self.a4_base

        # Basis perpendicular to a1_base.
        e_x = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(e_x, self.a1_base)) > 0.9:
            e_x = np.array([0.0, 1.0, 0.0])
        e_x = e_x - np.dot(e_x, self.a1_base) * self.a1_base
        self.e_x = e_x / np.linalg.norm(e_x)
        self.e_y = np.cross(self.a1_base, self.e_x)
        self.e_y = self.e_y / np.linalg.norm(self.e_y)

        # phi_0: azimuth of the J1 frame's x-axis in the (e_x, e_y) basis.
        # For identity R_o1 this is 0 (as for UR10, FR5, Elfin). Generalizes
        # to any URDF with a non-identity J1 origin rotation.
        x_axis_j1 = T1[:3, :3] @ np.array([1.0, 0.0, 0.0])
        x_perp = x_axis_j1 - np.dot(x_axis_j1, self.a1_base) * self.a1_base
        if np.linalg.norm(x_perp) < 1e-9:
            self.phi_0 = 0.0
        else:
            self.phi_0 = float(np.arctan2(np.dot(x_perp, self.e_y),
                                          np.dot(x_perp, self.e_x)))

    # --------------------------------------------------------------
    # Step 1: P5
    # --------------------------------------------------------------

    def _compute_p5(self, T_flange):
        return T_flange[:3, 3] - self.flange_offset * T_flange[:3, 2]

    # --------------------------------------------------------------
    # Step 2: theta1 candidates (joint-angle space)
    # --------------------------------------------------------------

    def _compute_theta1_candidates(self, P5):
        P5_rel = P5 - self.J1_origin
        P5_perp = P5_rel - np.dot(P5_rel, self.a1_base) * self.a1_base
        r = float(np.linalg.norm(P5_perp))
        if r < _REACH_TOL:
            return []
        phi = float(np.arctan2(np.dot(P5_perp, self.e_y),
                               np.dot(P5_perp, self.e_x)))
        phi_target = wrap(phi - self.phi_0)

        if self.wrist_offset < _REACH_TOL:
            return [(wrap(phi_target), +1, "right"),
                    (wrap(phi_target + np.pi), -1, "left")]

        ratio = self.wrist_offset / r
        if abs(ratio) > 1.0 + _REACH_TOL:
            return []
        ratio = max(-1.0, min(1.0, ratio))
        alpha = float(np.arcsin(ratio))
        return [
            (wrap(phi_target - alpha), +1, "right"),
            (wrap(phi_target + alpha), -1, "left"),
        ]

    # --------------------------------------------------------------
    # Step 3: P_J4
    # --------------------------------------------------------------

    def _compute_P_J4(self, P5, theta1):
        z4 = rodrigues(self.a1_base, theta1) @ self.a4_base
        z4 = z4 / np.linalg.norm(z4)
        B = P5 - self.wrist_offset * z4
        BA_0 = self.A_0 - self.B_0
        BA_current = rodrigues(self.a1_base, theta1) @ BA_0
        P_J4 = B + BA_current
        return P_J4, z4, B

    # --------------------------------------------------------------
    # Step 4: theta2, theta3 from 2-link chain
    # --------------------------------------------------------------

    def _solve_arm_chain(self, P_J4, theta1, z4):
        n_hat = z4 / np.linalg.norm(z4)

        # J3 at theta1-rotated position.
        j3_rot = (rodrigues(self.a1_base, theta1)
                  @ (self.J3_origin_0 - self.J1_origin)
                  + self.J1_origin)

        # Shoulder = J2 origin (on a1 axis, at height h2 above J1_origin).
        h2 = float(np.dot(self.J2_origin_0 - self.J1_origin, self.a1_base))
        shoulder = self.J1_origin + h2 * self.a1_base

        # Arm-plane basis at theta1.
        v_ref = j3_rot - shoulder
        v_ref = v_ref - np.dot(v_ref, n_hat) * n_hat
        if np.linalg.norm(v_ref) < 1e-9:
            v_ref = np.array([0.0, 0.0, 1.0])
            v_ref = v_ref - np.dot(v_ref, n_hat) * n_hat
        u_hat = v_ref / np.linalg.norm(v_ref)
        w_hat = np.cross(n_hat, u_hat)
        w_hat = w_hat / np.linalg.norm(w_hat)

        delta = P_J4 - shoulder
        u = float(np.dot(delta, u_hat))
        w = float(np.dot(delta, w_hat))
        D = float(np.sqrt(u*u + w*w))

        a2 = float(self.geom.axis_gap(1))
        a3 = float(np.linalg.norm(self.A_0 - self.J3_origin_0))

        if D < 1e-9:
            return []
        if D > a2 + a3 + _REACH_TOL or D < abs(a2 - a3) - _REACH_TOL:
            return []

        cos_elbow = (D*D - a2*a2 - a3*a3) / (2 * a2 * a3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        theta3_abs = float(np.arccos(cos_elbow))

        psi = float(np.arctan2(w, u))
        sin_beta = a3 * np.sin(theta3_abs) / D
        sin_beta = max(-1.0, min(1.0, sin_beta))
        beta = float(np.arcsin(sin_beta))

        theta2_up = wrap(psi - beta)
        theta2_down = wrap(psi + beta)
        theta3_up = wrap(+theta3_abs)
        theta3_down = wrap(-theta3_abs)

        return [
            (theta2_up, theta3_up, "up"),
            (theta2_down, theta3_down, "down"),
        ]

    def _joint_origin_in_root(self, idx):
        transforms = self.model.compute_fk(np.zeros(6))
        base = self.model.get_true_root()
        T_base_inv = np.linalg.inv(transforms[base])
        jname = self.arm_chain[idx]
        joint = self.model.joints[jname]
        T_p = transforms[joint['parent']]
        T_o = self.model._compose_transform(joint['origin_xyz'],
                                            joint['origin_rpy'])
        return (T_base_inv @ T_p @ T_o)[:3, 3]

    # --------------------------------------------------------------
    # Step 5: R03_fk
    # --------------------------------------------------------------

    def _compute_R03_fk(self, theta1, theta2, theta3):
        q = [theta1, theta2, theta3]
        R = np.eye(3)
        for i in range(3):
            R = R @ self.R_o[i] @ rodrigues(self.a_local[i], q[i])
        return R

    # --------------------------------------------------------------
    # Step 7: wrist inversion
    # --------------------------------------------------------------

    def _invert_wrist(self, R_target, grid=_GRID):
        R_o4, R_o5, R_o6 = self.R_o[3], self.R_o[4], self.R_o[5]
        a4, a5, a6 = self.a_local[3], self.a_local[4], self.a_local[5]

        t4g = np.linspace(-np.pi, np.pi, grid, endpoint=False)
        t5g = np.linspace(0, np.pi, grid, endpoint=False)

        best = []
        for t5 in t5g:
            R45 = R_o5 @ rodrigues(a5, t5)
            for t4 in t4g:
                R44 = R_o4 @ rodrigues(a4, t4)
                R_prefix_inv = (R44 @ R45).T
                R6_want = R_prefix_inv @ R_target
                R6_local = R_o6.T @ R6_want
                c = (np.trace(R6_local) - 1.0) / 2.0
                c = max(-1.0, min(1.0, c))
                s_vec = np.array([
                    R6_local[2,1] - R6_local[1,2],
                    R6_local[0,2] - R6_local[2,0],
                    R6_local[1,0] - R6_local[0,1],
                ]) / 2.0
                s = float(np.dot(s_vec, a6))
                t6 = float(np.arctan2(s, c))
                R_pred = R44 @ R45 @ (R_o6 @ rodrigues(a6, t6))
                res = float(np.linalg.norm(R_pred - R_target))
                if res < 0.15:
                    best.append((t4, t5, t6))

        refined = []
        for (t4, t5, t6) in best:
            t4r, t5r, t6r, res = self._refine_wrist(R_target, t4, t5, t6)
            if res < _WRIST_TOL:
                refined.append((t4r, t5r, t6r, res))

        out = []
        for (t4, t5, t6, res) in refined:
            dup = False
            for (u4, u5, u6, _) in out:
                if (abs(wrap(t4 - u4)) < 1e-3
                        and abs(wrap(t5 - u5)) < 1e-3
                        and abs(wrap(t6 - u6)) < 1e-3):
                    dup = True; break
                if (abs(wrap(t4 + np.pi - u4)) < 1e-3
                        and abs(wrap(-t5 - u5)) < 1e-3
                        and abs(wrap(t6 + np.pi - u6)) < 1e-3):
                    dup = True; break
            if not dup:
                out.append((t4, t5, t6, res))
        return out

    def _refine_wrist(self, R_target, t4, t5, t6, iters=30):
        R_o4, R_o5, R_o6 = self.R_o[3], self.R_o[4], self.R_o[5]
        a4, a5, a6 = self.a_local[3], self.a_local[4], self.a_local[5]

        def R_pred(p):
            return (R_o4 @ rodrigues(a4, p[0])
                    @ R_o5 @ rodrigues(a5, p[1])
                    @ R_o6 @ rodrigues(a6, p[2]))

        params = np.array([t4, t5, t6], dtype=float)
        for _ in range(iters):
            Rp = R_pred(params)
            res = (Rp - R_target).flatten()
            J = np.zeros((9, 3))
            eps = 1e-6
            for k in range(3):
                p2 = params.copy(); p2[k] += eps
                J[:, k] = (R_pred(p2) - Rp).flatten() / eps
            try:
                step, *_ = np.linalg.lstsq(J, -res, rcond=None)
            except np.linalg.LinAlgError:
                break
            params = params + step
            if np.linalg.norm(step) < 1e-10:
                break

        Rp = R_pred(params)
        return (float(params[0]), float(params[1]), float(params[2]),
                float(np.linalg.norm(Rp - R_target)))

    # --------------------------------------------------------------
    # Step 8: limits
    # --------------------------------------------------------------

    def _within_limits(self, q):
        for i, (lo, hi) in enumerate(self.limits):
            if lo <= q[i] <= hi:
                continue
            if lo <= q[i] + 2*np.pi <= hi:
                q[i] += 2*np.pi
                continue
            if lo <= q[i] - 2*np.pi <= hi:
                q[i] -= 2*np.pi
                continue
            return False
        return True

    # --------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------

    def solve(self, T_flange):
        return self.solve_with_diagnostics(T_flange)['solutions']

    def solve_with_diagnostics(self, T_flange):
        diag = {'P5': None, 'theta1_candidates': [], 'per_theta1': [],
                'solutions': []}

        P5 = self._compute_p5(T_flange)
        diag['P5'] = P5

        theta1_cands = self._compute_theta1_candidates(P5)
        diag['theta1_candidates'] = theta1_cands

        R_flange = T_flange[:3, :3]

        for theta1, sigma, tag1 in theta1_cands:
            P_J4, z4, B = self._compute_P_J4(P5, theta1)
            entry = {
                'theta1': theta1, 'sigma': sigma, 'tag1': tag1,
                'P_J4': P_J4, 'z4': z4, 'B': B,
                'arm_candidates': [],
                'arm_details': [],
            }

            arm_cands = self._solve_arm_chain(P_J4, theta1, z4)
            entry['arm_candidates'] = arm_cands

            for theta2, theta3, tag_elbow in arm_cands:
                arm_entry = {'theta2': theta2, 'theta3': theta3,
                             'tag_elbow': tag_elbow}

                R03_fk = self._compute_R03_fk(theta1, theta2, theta3)
                R_wrist_target = R03_fk.T @ R_flange
                arm_entry['R03_fk'] = R03_fk
                arm_entry['R_wrist_target'] = R_wrist_target

                wrist_cands = self._invert_wrist(R_wrist_target)
                arm_entry['wrist_candidates'] = wrist_cands

                for (t4, t5, t6, res) in wrist_cands:
                    q = np.array([theta1, theta2, theta3, t4, t5, t6])
                    if not self._within_limits(q):
                        continue
                    sol = Solution(q=q, tags={
                        'shoulder': tag1,
                        'elbow': tag_elbow,
                        'wrist': 'no-flip' if t5 >= 0 else 'flip',
                    })
                    diag['solutions'].append(sol)

                entry['arm_details'].append(arm_entry)

            diag['per_theta1'].append(entry)

        return diag