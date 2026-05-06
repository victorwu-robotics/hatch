"""
Spherical Wrist IK Solver Wrapper - Standard D‑H parameters for Han's E15‑Pro.

Parameters extracted from the KinematicModel at zero configuration,
so theta offsets are all zero. Uses the standard D‑H convention
consistent with the spherical_wrist_ik.py solver.
"""

import numpy as np
from typing import Optional
from math import pi
import logging

from core.kinematics.spherical_wrist_ik import SphericalWristIKSolver

logger = logging.getLogger(__name__)


class SphericalWristIKSolverWrapper:
    """IK solver wrapper for spherical wrist robots."""

    def __init__(self, kinematic_model=None):
        self.model = kinematic_model

        # Standard D‑H parameters for Han's E15‑Pro
        # Extracted from zero‑configuration transforms — theta offsets all zero
        self.dh_params = [
            {'a': 0.0,   'd': 0.262, 'alpha':  pi/2},   # Joint 1
            {'a': 0.73,  'd': 0.0,   'alpha':  pi/2},   # Joint 2
            {'a': 0.0,   'd': 0.0,   'alpha': -pi/2},   # Joint 3
            {'a': 0.0,   'd': 0.57,  'alpha':  pi/2},   # Joint 4
            {'a': 0.0,   'd': 0.0,   'alpha':  pi/2},   # Joint 5
            {'a': 0.0,   'd': 0.0,   'alpha':  0.0},     # Joint 6
        ]

        self._base_compensation = None
        self._true_base_frame = None
        self._ik_base_frame = None

        if kinematic_model:
            self._setup_base_compensation()

        self.ik = SphericalWristIKSolver(self.dh_params)

        logger.info("SphericalWristIKSolverWrapper initialized")
        logger.info(f"  d1={self.dh_params[0]['d']:.4f}, "
                    f"a2={self.dh_params[1]['a']:.4f}, "
                    f"d4={self.dh_params[3]['d']:.4f}")

    def _setup_base_compensation(self):
        """Compute transform between true base and IK base frame."""
        if not self.model:
            return

        true_base = None
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ('revolute', 'continuous', 'prismatic'):
                true_base = joint['parent']
                break

        if true_base is None:
            return

        self._true_base_frame = true_base

        try:
            arm_chain = self.model.get_arm_chain(true_base)
            if arm_chain:
                first_joint = arm_chain[0]
                ik_base = self.model.joints[first_joint]['parent']
                self._ik_base_frame = ik_base
        except Exception:
            return

        if true_base != self._ik_base_frame:
            if (true_base in self.model.link_transforms and
                self._ik_base_frame in self.model.link_transforms):
                T_true_world = self.model.link_transforms[true_base]
                T_ik_world = self.model.link_transforms[self._ik_base_frame]
                self._base_compensation = np.linalg.inv(T_true_world) @ T_ik_world
                logger.info(f"Base compensation enabled: "
                          f"{true_base} → {self._ik_base_frame}")

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose."""
        if self._base_compensation is not None:
            target_pose = self._base_compensation @ target_pose
        return self.ik.inverse(target_pose, q_guess)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics in true base frame."""
        tcp_ik = self.ik.forward(q)
        if self._base_compensation is not None:
            T_ik_to_true = np.linalg.inv(self._base_compensation)
            return T_ik_to_true @ tcp_ik
        return tcp_ik