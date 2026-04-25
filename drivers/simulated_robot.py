"""
Simulated Robot - Pure Python, no hardware.
Publishes ROBOT_STATE events.
"""

import time
import numpy as np
from typing import List, Dict, Optional

from drivers.robot_arm.robot_interface import RobotInterface
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.kinematics.kinematic_model import KinematicModel
from core.mode import Mode


class SimulatedRobot(RobotInterface):
    """
    Simulated robot using local FK/IK.
    Updates internal state and publishes ROBOT_STATE on each command.
    """
    
    def __init__(self, kinematic_model: KinematicModel, state_channel: StateChannel,
                 real_robot: Optional['RealRobot'] = None, parent=None):
        super().__init__(parent)
        self._model = kinematic_model   # May be None initially
        self._channel = state_channel
        self._real_robot = real_robot
        self._connected = True
        # self._joint_positions = self._model.get_current_joint_positions()
        self._current_mode = Mode.SIMULATE_LOCAL
        self._use_real_ik = False

    def set_mode(self, mode: Mode):
        """Set the operating mode for this simulated robot."""
        self._current_mode = mode
        self._use_real_ik = mode.uses_real_ik()
        print(f"[SimulatedRobot] Mode set to {mode}, use_real_ik={self._use_real_ik}")

    def set_kinematic_model(self, model):
        """Set kinematic model when robot loads."""
        self._model = model
        self._joint_positions = self._model.get_current_joint_positions()

    def set_use_real_ik(self, enabled: bool):
        """Enable or disable using real robot's IK solver."""
        self._use_real_ik = enabled
        mode = "REAL IK" if enabled else "LOCAL IK"
        print(f"[SimulatedRobot] Using {mode} for Cartesian commands")
    
    def sync_to_real(self, joint_positions: List[float]):
        """Sync virtual robot state to match real robot."""
        self._joint_positions = np.array(joint_positions)
        self._publish_state()
        print(f"[SimulatedRobot] Synced to real robot position")
    
    def move_joints(self, positions: List[float]) -> bool:
        """Update internal joint state and publish."""
        print(f"[SIM] move_joints called with {positions}")
        self._joint_positions = np.array(positions)
        print(f"[SIM] Stored: {self._joint_positions}")
        self._publish_state()
        return True
    
    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        """Move to target pose using selected IK solver."""
        q_current = self._joint_positions
        
        if self._use_real_ik and self._real_robot and self._real_robot.is_connected():
            q_solution = self._real_robot.solve_ik(pose, q_current)
            if q_solution is None:
                self._channel.publish(
                    EventType.ROBOT_ERROR,
                    data={'error': 'Real robot IK failed', 'target_pose': pose.tolist()},
                    source="simulated_robot"
                )
                return False
        else:
            q_solution = self._model.solve_ik_for_tcp(pose, q_current)
            if q_solution is None:
                self._channel.publish(
                    EventType.ROBOT_ERROR,
                    data={'error': 'Local IK failed', 'target_pose': pose.tolist()},
                    source="simulated_robot"
                )
                return False
        
        self._joint_positions = q_solution
        self._publish_state()
        return True
    
    def get_state(self) -> Dict:
        """Return current state."""
        tcp_pose = self._model.forward_kinematics(self._joint_positions)
        return {
            'joint_positions': self._joint_positions.tolist(),
            'tcp_pose': tcp_pose.tolist() if hasattr(tcp_pose, 'tolist') else tcp_pose,
            'timestamp': time.time(),
            'source': 'simulated_robot'
        }
    
    def _publish_state(self):
        """Publish ROBOT_STATE event."""
        print(f"[SIM] Publishing ROBOT_STATE with positions: {self._joint_positions}")
        self._channel.publish(
            EventType.ROBOT_STATE,
            data=self.get_state(),
            source="simulated_robot"
        )
    
    def is_connected(self) -> bool:
        return self._connected
    
    def connect(self, ip: str = "", **kwargs) -> bool:
        return True
    
    def disconnect(self) -> None:
        pass
    
    def stop(self) -> None:
        pass