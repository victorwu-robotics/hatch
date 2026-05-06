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

    def __init__(self, kinematic_model, max_iterations=200, tolerance=1e-5):
        self.model = kinematic_model
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose using the model's native FK."""
        n_joints = len(self.model.get_joint_info()['names'])

        if q_guess is None:
            q_guess = self.model.get_current_joint_positions()

        q = np.array(q_guess, dtype=float)

        for iteration in range(self.max_iterations):
            # Use model's native FK (not analytical solver)
            self.model.update_state(q)
            T_current = self.model.get_tcp_pose()

            if T_current is None:
                return None

            # Position error
            pos_error = target_pose[:3, 3] - T_current[:3, 3]

            # Orientation error
            R_error = target_pose[:3, :3] @ T_current[:3, :3].T
            from scipy.spatial.transform import Rotation as R
            ori_error = R.from_matrix(R_error).as_rotvec()

            error = np.concatenate([pos_error, ori_error])
            error_norm = np.linalg.norm(error)

            if error_norm <= self.tolerance:
                return q

            # Compute Jacobian
            J = self._compute_jacobian(q, n_joints)

            # Damped least squares
            lam = 0.5 if iteration < 10 else 0.1
            damping = lam ** 2 * np.eye(6)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + damping, error)
            except np.linalg.LinAlgError:
                return None

            # Line search
            alpha = 1.0
            success = False
            for _ in range(10):
                q_new = q + alpha * dq
                self.model.update_state(q_new)
                T_new = self.model.get_tcp_pose()
                if T_new is None:
                    alpha *= 0.5
                    continue
                R_new_err = target_pose[:3, :3] @ T_new[:3, :3].T
                new_error = np.concatenate([
                    target_pose[:3, 3] - T_new[:3, 3],
                    R.from_matrix(R_new_err).as_rotvec()
                ])
                if np.linalg.norm(new_error) < error_norm:
                    success = True
                    break
                alpha *= 0.5

            if not success:
                if error_norm < 1e-3:
                    return q
                # Try a small random perturbation before giving up
                if iteration == 0:  # Only on first iteration
                    q_perturbed = q + np.random.uniform(-0.01, 0.01, n_joints)
                    self.model.update_state(q_perturbed)
                    T_perturbed = self.model.get_tcp_pose()
                    if T_perturbed is not None:
                        R_per = target_pose[:3, :3] @ T_perturbed[:3, :3].T
                        new_err = np.concatenate([
                            target_pose[:3, 3] - T_perturbed[:3, 3],
                            R.from_matrix(R_per).as_rotvec()
                        ])
                        if np.linalg.norm(new_err) < error_norm:
                            q = q_perturbed
                            continue

            q = q + alpha * dq

        return None

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

            J[:3, i] = (p_perturbed - p_current) / delta

            from scipy.spatial.transform import Rotation as R
            R_diff = T_perturbed[:3, :3] @ T_current[:3, :3].T
            omega = R.from_matrix(R_diff).as_rotvec()
            J[3:, i] = omega / delta

        return J

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model's native FK."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()