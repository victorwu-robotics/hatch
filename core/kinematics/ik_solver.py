"""
IK Solver Dispatcher - Detects wrist type and creates the appropriate solver.

Uses analytical D-H solver for offset wrists (UR-style).
Uses numerical Jacobian solver for spherical wrists.
Selection is automatic based on URDF joint geometry.

Principle: Movements as Models.
Principle: Everything in URDF.
"""

import logging
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class IKSolver:
    """IK solver with automatic wrist-type detection."""

    def __init__(self, kinematic_model=None):
        self.model = kinematic_model
        self._solver = None
        self._wrist_type = None

        if kinematic_model is not None:
            self._detect_and_create()

    def _detect_and_create(self):
        """Detect wrist type and create appropriate solver."""
        try:
            arm_chain = self.model.get_arm_chain(self.model.get_true_root())

            if len(arm_chain) < 6:
                logger.warning(f"Robot has {len(arm_chain)} joints, expected 6")
                self._create_numerical()
                return

            # Spherical wrist: last three joints have zero X offset (a=0)
            last_three = arm_chain[-3:]
            all_a_zero = all(
                abs(self.model.joints[j]['origin_xyz'][0]) < 1e-6
                for j in last_three
            )

            if all_a_zero:
                self._wrist_type = "spherical"
                self._create_analytical()
                logger.info("Detected spherical wrist — using analytical IK solver")
            else:
                self._wrist_type = "offset"
                self._create_offset_wrist()
                logger.info("Detected offset wrist — using analytical IK solver")

        except Exception as e:
            logger.warning(f"Wrist detection failed: {e}")
            self._create_numerical()

    def _create_offset_wrist(self):
        try:
            from core.kinematics.offset_wrist_ik_solver import OffsetWristIKSolver
            self._solver = OffsetWristIKSolver(self.model)
        except ImportError:
            logger.warning("Offset wrist solver not available, falling back to numerical")
            self._create_numerical()

    def _create_analytical(self):
        """Create analytical solver for spherical wrist (official D-H)."""
        try:
            from core.kinematics.analytical_ik_solver import AnalyticalIKSolver
            self._solver = AnalyticalIKSolver(self.model)
            logger.info("Using analytical IK solver (Elfin spherical wrist)")
        except ImportError:
            logger.warning("Analytical solver not available, falling back to numerical")
            self._create_numerical()

    def _create_numerical(self):
        try:
            from core.kinematics.numerical_ik_solver import NumericalIKSolver
            self._solver = NumericalIKSolver(self.model)
        except ImportError:
            raise RuntimeError("No IK solver available.")

    @property
    def wrist_type(self) -> Optional[str]:
        return self._wrist_type

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose."""
        if self._solver is None:
            raise RuntimeError("IK solver not configured.")
        return self._solver.solve_ik_for_tcp(target_pose, q_guess)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics."""
        if self._solver is None:
            raise RuntimeError("FK solver not configured.")
        return self._solver.forward_kinematics(q)