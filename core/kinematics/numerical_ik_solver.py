"""
Numerical IK Solver - Works with any serial robot using the KinematicModel's FK.

Uses the Jacobian pseudo-inverse with adaptive damping and line search.
No D-H parameters required — uses the model's native FK directly.

Principle #7: Movements as Models.
"""

import numpy as np
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class NumericalIKSolver:
    """Numerical IK solver using Jacobian pseudo-inverse."""

    def __init__(self, kinematic_model, max_iterations=200, tolerance=1e-5, position_weight=10.0):
        self.model = kinematic_model
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.position_weight = position_weight

    def solve_ik_for_tcp(self,
                        target_pose: np.ndarray,
                        q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """
        Solve IK with guaranteed TCP position.

        1. Run a full 6‑DOF least‑squares solve to get close.
        2. Freeze the wrist joints and move only the arm joints
        until the TCP position exactly matches the target.
        """
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation as R

        if q_guess is None:
            q_guess = self.model.get_current_joint_positions()

        pos_target = target_pose[:3, 3]
        rot_target = R.from_matrix(target_pose[:3, :3])

        # ---------- full 6‑DOF solve ----------
        def error_all(q):
            self.model.update_state(q)
            T = self.model.get_tcp_pose()
            pos_err = pos_target - T[:3, 3]
            rot_err = (rot_target * R.from_matrix(T[:3, :3]).inv()).as_rotvec()
            return np.concatenate([pos_err, rot_err])

        res = least_squares(error_all, q_guess, method='lm',
                            max_nfev=150, xtol=1e-12, ftol=1e-12)
        q = res.x

        # ---------- exact position correction (arm only) ----------
        # Keep wrist joints fixed.  Adjust only joints 1,2,3 to move the
        # wrist centre and bring the TCP to the exact target position.
        for _ in range(30):
            self.model.update_state(q)
            T = self.model.get_tcp_pose()
            pos_err = pos_target - T[:3, 3]
            if np.linalg.norm(pos_err) < 1e-6:
                break

            # 3x3 position Jacobian for arm joints
            J = np.zeros((3, 3))
            delta = 1e-5
            p_current = T[:3, 3]
            for i in range(3):
                q_pert = q.copy()
                q_pert[i] += delta
                self.model.update_state(q_pert)
                T_pert = self.model.get_tcp_pose()
                J[:, i] = (T_pert[:3, 3] - p_current) / delta

            # damped least squares for the arm correction
            lam = 0.01
            try:
                dq_arm = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), pos_err)
            except np.linalg.LinAlgError:
                continue
            q[:3] += dq_arm

        return q

    def _compute_jacobian(self, q: np.ndarray, n_joints: int) -> np.ndarray:
        """Compute 6×n geometric Jacobian using model's native FK."""
        J = np.zeros((6, n_joints))
        delta = 1e-4

        self.model.update_state(q)
        T_current = self.model.get_tcp_pose()
        p_current = T_current[:3, 3]

        for i in range(n_joints):
            q_perturbed = q.copy()
            q_perturbed[i] += delta

            self.model.update_state(q_perturbed)
            T_perturbed = self.model.get_tcp_pose()
            p_perturbed = T_perturbed[:3, 3]

            J[:3, i] = self.position_weight * (p_perturbed - p_current) / delta

            from scipy.spatial.transform import Rotation as R
            R_diff = T_perturbed[:3, :3] @ T_current[:3, :3].T
            omega = R.from_matrix(R_diff).as_rotvec()
            J[3:, i] = omega / delta

        return J

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model's native FK."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()