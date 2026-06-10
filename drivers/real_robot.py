"""
Real Robot - Wraps URRobotDriver and publishes events via StateChannel.

Uses Qt internally for thread-safe driver communication.
Implements the RobotInterface contract without multiple inheritance.

Principle: Event-Driven. Publishes ROBOT_STATE, CONNECTION_ESTABLISHED, etc.
"""

import time
import numpy as np
import logging
from typing import List, Dict, Optional

from PyQt5.QtCore import QObject, pyqtSlot, Qt
from scipy.spatial.transform import Rotation as R

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from drivers.ur_rtde_bridge import URRobotDriver

logger = logging.getLogger(__name__)


class RealRobot(QObject):
    """
    Real UR robot via RTDE.

    Inherits from QObject for pyqtSlot support — needed to receive
    signals from URRobotDriver's internal thread via Qt.QueuedConnection.

    Implements the RobotInterface contract directly (no ABC inheritance)
    to avoid metaclass conflict between QObject (sip metaclass) and ABC.
    """

    def __init__(self, state_channel: StateChannel):
        """
        Initialize real robot.

        Args:
            state_channel: Application event bus for publishing state.
        """
        super().__init__()

        self._channel = state_channel
        self._driver = URRobotDriver()
        self._connected = False

        # Connect driver signals (Qt handles thread safety)
        self._driver.signals.state_signal.connect(
            self._on_driver_state,
            type=Qt.QueuedConnection
        )
        self._driver.signals.connection_signal.connect(
            self._on_driver_connection,
            type=Qt.QueuedConnection
        )
        self._driver.signals.error_signal.connect(
            self._on_driver_error,
            type=Qt.QueuedConnection
        )

        logger.info("RealRobot initialized")

    # =================================================================
    # Driver Signal Handlers (Qt slots -> StateChannel events)
    # =================================================================

    @pyqtSlot(dict)
    def _on_driver_state(self, state: Dict):
        """Convert driver state signal to ROBOT_STATE event."""
        logger.debug(f"[RealRobot] Received state from driver: {state.get('joint_positions', 'N/A')}")
        joint_positions = state.get('joint_positions')
        if joint_positions is None:
            return

        self._channel.publish(
            EventType.ROBOT_STATE,
            data={
                'joint_positions': joint_positions,
                'tcp_pose': state.get('tcp_pose'),
                'timestamp': time.time(),
                'source': 'real_robot'
            },
            source="real_robot"
        )
        logger.debug(f"[RealRobot] Published ROBOT_STATE")

    @pyqtSlot(bool, str)
    def _on_driver_connection(self, connected: bool, message: str):
        """Convert driver connection signal to StateChannel event."""
        self._connected = connected

        event_type = (
            EventType.CONNECTION_ESTABLISHED if connected
            else EventType.CONNECTION_LOST
        )

        self._channel.publish(
            event_type,
            data={'message': message},
            source="real_robot"
        )
        logger.info(f"RealRobot: {message}")

    @pyqtSlot(str, object)
    def _on_driver_error(self, error_msg: str, exception):
        """Convert driver error signal to ERROR_OCCURRED event."""
        self._channel.publish(
            EventType.ERROR_OCCURRED,
            data={'error': error_msg},
            source="real_robot"
        )
        logger.error(f"RealRobot error: {error_msg}")

    # =================================================================
    # RobotInterface Contract (implemented directly, no ABC inheritance)
    # =================================================================

    def move_joints(self, positions: List[float]) -> bool:
        """Send joint position command to real robot."""
        if not self._connected:
            logger.warning("move_joints called while disconnected")
            return False
        return self._driver.send_joint_command(positions)

    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        """Send Cartesian move command to real robot."""
        if not self._connected:
            logger.warning("move_pose called while disconnected")
            return False
        pose_list = self._transform_to_pose_list(pose)
        return self._driver.send_cartesian_command(pose_list)

    def get_state(self) -> Dict:
        """Get latest cached robot state."""
        state = self._driver.get_current_state()
        return state if state else {}

    def is_connected(self) -> bool:
        """Return True if connected to hardware."""
        return self._connected

    def connect(self, ip: str, **kwargs) -> bool:
        """Connect to robot hardware."""
        frequency = kwargs.get('frequency', 125)
        ur_cap_port = kwargs.get('ur_cap_port', 50002)
        max_retries = kwargs.get('max_retries', 10)
        return self._driver.connect(ip, frequency, ur_cap_port, max_retries)

    def disconnect(self) -> None:
        """Disconnect from hardware."""
        self._driver.disconnect()
        self._connected = False
        logger.info("Disconnected from robot")

    def stop(self) -> None:
        """Emergency stop."""
        if self._connected:
            self._driver.stop_motion()

    # =================================================================
    # Additional Methods
    # =================================================================

    def solve_ik(self,
                 target_pose: np.ndarray,
                 q_guess: Optional[List[float]] = None) -> Optional[np.ndarray]:
        """Solve IK using real robot's controller."""
        if not self._connected:
            logger.warning("solve_ik called while disconnected")
            return None

        pose_list = self._transform_to_pose_list(target_pose)

        if q_guess is None:
            current_state = self.get_state()
            q_guess = current_state.get('joint_positions')

        result = self._driver.get_inverse_kinematics(pose_list, q_guess)
        return np.array(result) if result is not None else None

    def get_joint_names(self) -> List[str]:
        """Get list of joint names in order."""
        return self._driver.joint_names

    # =================================================================
    # Internal Helpers
    # =================================================================

    @staticmethod
    def _transform_to_pose_list(T: np.ndarray) -> List[float]:
        """Convert 4x4 homogeneous transform to [x, y, z, rx, ry, rz]."""
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        rot_vec = R.from_matrix(T[:3, :3]).as_rotvec()
        return [x, y, z, rot_vec[0], rot_vec[1], rot_vec[2]]