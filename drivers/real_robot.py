"""
Real Robot - Wraps URRobotDriver and publishes ROBOT_STATE events.
"""

import time
import numpy as np
from typing import List, Dict, Optional, Any

from PyQt5.QtCore import pyqtSlot, Qt

from drivers.robot_arm.robot_interface import RobotInterface
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from drivers.robot_arm.ur_rtde_bridge import URRobotDriver


class RealRobot(RobotInterface):
    """
    Real UR robot via RTDE.
    
    Wraps URRobotDriver and converts its Qt signals to StateChannel events.
    """
    
    def __init__(self, state_channel: StateChannel, parent=None):
        super().__init__(parent)
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
        
        print("[RealRobot] Initialized")
    
    # ===== Driver Signal Handlers =====
    
    @pyqtSlot(dict)
    def _on_driver_state(self, state: Dict[str, Any]):
        """Convert driver state to ROBOT_STATE event."""
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
    
    @pyqtSlot(bool, str)
    def _on_driver_connection(self, connected: bool, message: str):
        """Convert connection status to StateChannel event."""
        self._connected = connected
        event_type = EventType.ROBOT_CONNECTED if connected else EventType.ROBOT_DISCONNECTED
        self._channel.publish(
            event_type,
            data={'message': message},
            source="real_robot"
        )
        print(f"[RealRobot] {message}")
    
    @pyqtSlot(str, object)
    def _on_driver_error(self, error_msg: str, exception):
        """Convert error to ROBOT_ERROR event."""
        self._channel.publish(
            EventType.ROBOT_ERROR,
            data={'error': error_msg},
            source="real_robot"
        )
        print(f"[RealRobot] Error: {error_msg}")
    
    # ===== RobotInterface Implementation =====
    
    def move_joints(self, positions: List[float]) -> bool:
        if not self._connected:
            return False
        return self._driver.send_joint_command(positions)
    
    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        if not self._connected:
            return False
        pose_list = self._transform_to_pose_list(pose)
        return self._driver.send_cartesian_command(pose_list)
    
    def get_state(self) -> Dict[str, Any]:
        state = self._driver.get_current_state()
        return state if state else {}
    
    def is_connected(self) -> bool:
        return self._connected
    
    def connect(self, ip: str, **kwargs) -> bool:
        frequency = kwargs.get('frequency', 125)
        ur_cap_port = kwargs.get('ur_cap_port', 50002)
        max_retries = kwargs.get('max_retries', 10)
        return self._driver.connect(ip, frequency, ur_cap_port, max_retries)
    
    def disconnect(self) -> None:
        self._driver.disconnect()
        self._connected = False
    
    def stop(self) -> None:
        if self._connected:
            self._driver.stop_motion()
    
    # ===== Additional Methods =====
    
    def solve_ik(self, target_pose: np.ndarray, q_guess: Optional[List[float]] = None) -> Optional[np.ndarray]:
        """Solve IK using real robot's solver (no hardware movement)."""
        if not self._connected:
            return None
        
        pose_list = self._transform_to_pose_list(target_pose)
        
        if q_guess is None:
            current_state = self.get_state()
            q_guess = current_state.get('joint_positions')
        
        result = self._driver.get_inverse_kinematics(pose_list, q_guess)
        return np.array(result) if result is not None else None
    
    def get_joint_names(self) -> List[str]:
        return self._driver.joint_names
    
    def _transform_to_pose_list(self, T: np.ndarray) -> List[float]:
        """Convert 4x4 transform to [x,y,z,rx,ry,rz]."""
        from scipy.spatial.transform import Rotation as R
        
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        rot_vec = R.from_matrix(T[:3, :3]).as_rotvec()
        return [x, y, z, rot_vec[0], rot_vec[1], rot_vec[2]]