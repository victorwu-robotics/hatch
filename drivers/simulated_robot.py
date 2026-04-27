"""
Simulated Robot - Pure Python robot implementation for simulation mode.

Implements RobotInterface using local FK/IK.
Publishes ROBOT_STATE events on each command.
No Qt dependency — pure Python.

Principle #8: Pure Python.
Principle #7: Movements as Models. Commands produce state, not side effects.
"""

import time
import numpy as np
import logging
from typing import List, Dict, Optional

from drivers.robot_arm.robot_interface import RobotInterface
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.mode import Mode

logger = logging.getLogger(__name__)


class SimulatedRobot(RobotInterface):
    """
    Simulated robot using local FK/IK.

    Updates internal joint state on commands and publishes
    ROBOT_STATE events for the rest of the system to consume.
    Supports three IK modes:
    - SIMULATE_LOCAL: Local IK solver
    - SIMULATE_REAL_IK: Real robot's IK solver (no hardware movement)
    - REAL: Passthrough (controlled by CommandHandler mode switching)
    """

    def __init__(self,
                 kinematic_model,
                 state_channel: StateChannel,
                 real_robot: Optional['RealRobot'] = None):
        """
        Initialize simulated robot.

        Args:
            kinematic_model: KinematicModel for FK/IK (may be None initially,
                            set later via set_kinematic_model when robot loads)
            state_channel: Application event bus
            real_robot: Optional RealRobot for IK passthrough in SIMULATE_REAL_IK mode
        """
        self._model = kinematic_model
        self._channel = state_channel
        self._real_robot = real_robot

        # Joint state
        self._joint_positions: Optional[np.ndarray] = None

        # Mode
        self._current_mode = Mode.SIMULATE_LOCAL
        self._use_real_ik = False

        logger.info("SimulatedRobot initialized")

    # =================================================================
    # Mode and Model Configuration
    # =================================================================

    def set_mode(self, mode: Mode):
        """Set the operating mode for IK source selection."""
        self._current_mode = mode
        self._use_real_ik = mode.uses_real_ik()
        logger.info(f"Mode set to {mode} (use_real_ik={self._use_real_ik})")

    def set_kinematic_model(self, model):
        """
        Set the kinematic model after robot loads.

        Called by RobotManager when ROBOT_LOADED.
        Initializes joint positions from the model's current state.
        """
        self._model = model
        if model is not None:
            self._joint_positions = model.get_current_joint_positions()
            logger.info(f"Kinematic model set, "
                       f"joints initialized to {self._joint_positions}")

    def set_real_robot(self, real_robot: 'RealRobot'):
        """Set the real robot reference for IK passthrough."""
        self._real_robot = real_robot

    # =================================================================
    # RobotInterface Implementation
    # =================================================================

    def move_joints(self, positions: List[float]) -> bool:
        """
        Update internal joint state and publish ROBOT_STATE.

        Args:
            positions: List of joint angles in radians.

        Returns:
            True (always succeeds in simulation).
        """
        self._joint_positions = np.array(positions, dtype=np.float64)
        self._publish_state()
        return True

    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        """
        Solve IK for target pose, update state, publish ROBOT_STATE.

        Uses local IK solver or real robot's IK depending on mode.

        Args:
            pose: 4x4 homogeneous transform for target TCP pose.
            frame: Reference frame ("base" or "world").

        Returns:
            True if IK succeeded and state was updated.
        """
        if self._model is None:
            logger.warning("move_pose called but no kinematic model set")
            return False

        q_current = self._joint_positions

        # Choose IK source
        if self._use_real_ik and self._real_robot and self._real_robot.is_connected():
            q_solution = self._real_robot.solve_ik(pose, q_current)
            ik_source = "real robot IK"
        else:
            try:
                q_solution = self._model.solve_ik_for_tcp(pose, q_current)
                ik_source = "local IK"
            except Exception:
                q_solution = None
                ik_source = "local IK (error)"

        if q_solution is None:
            self._channel.publish(
                EventType.ERROR_OCCURRED,
                data={
                    'error': f'IK failed ({ik_source})',
                    'target_pose': pose.tolist() if hasattr(pose, 'tolist') else str(pose)
                },
                source="simulated_robot"
            )
            return False

        self._joint_positions = q_solution
        self._publish_state()
        return True

    def get_state(self) -> Dict:
        """
        Get current robot state.

        Returns:
            Dictionary with joint_positions, tcp_pose, timestamp, and source.
        """
        if self._model is None or self._joint_positions is None:
            return {
                'joint_positions': [],
                'tcp_pose': None,
                'timestamp': time.time(),
                'source': 'simulated_robot'
            }

        try:
            tcp_pose = self._model.forward_kinematics(self._joint_positions)
            tcp_pose_data = tcp_pose.tolist() if hasattr(tcp_pose, 'tolist') else tcp_pose
        except Exception:
            tcp_pose_data = None

        return {
            'joint_positions': self._joint_positions.tolist(),
            'tcp_pose': tcp_pose_data,
            'timestamp': time.time(),
            'source': 'simulated_robot'
        }

    def is_connected(self) -> bool:
        """Always returns True in simulation."""
        return True

    def connect(self, ip: str = "", **kwargs) -> bool:
        """No-op for simulation. Always returns True."""
        return True

    def disconnect(self) -> None:
        """No-op for simulation."""
        pass

    def stop(self) -> None:
        """No-op for simulation."""
        pass

    # =================================================================
    # State Publishing
    # =================================================================

    def _publish_state(self):
        """Publish current state as ROBOT_STATE event."""
        state = self.get_state()
        self._channel.publish(
            EventType.ROBOT_STATE,
            data=state,
            source="simulated_robot"
        )

    # =================================================================
    # Real Robot Sync
    # =================================================================

    def sync_to_real(self, joint_positions: List[float]):
        """
        Sync virtual robot state to match real robot.

        Called when switching from real to simulate mode
        to maintain visual continuity.

        Args:
            joint_positions: Joint angles from real robot.
        """
        self._joint_positions = np.array(joint_positions, dtype=np.float64)
        self._publish_state()
        logger.info("Synced to real robot position")