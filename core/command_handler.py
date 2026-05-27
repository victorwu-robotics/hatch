"""
Command Handler - Routes UI commands to the active robot.
"""

from typing import Optional
import numpy as np
import time

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.mode import Mode
from drivers.robot_interface import RobotInterface
from drivers.simulated_robot import SimulatedRobot
from drivers.real_robot import RealRobot


class CommandHandler:
    """
    Routes JOINT_COMMAND and CARTESIAN_COMMAND to the active robot.
    
    Maintains three modes:
    - SIMULATE_LOCAL:    Local IK, virtual robot only
    - SIMULATE_REAL_IK:  Real robot's IK, virtual robot only
    - REAL:              Real robot's IK, real movement
    """
    
    def __init__(self, 
                 state_channel: StateChannel,
                 simulated_robot: SimulatedRobot,
                 real_robot: RealRobot):
        """
        Initialize command handler.
        
        Args:
            state_channel: The application's event bus
            simulated_robot: Simulated robot instance
            real_robot: Real robot instance
        """
        self._channel = state_channel
        self._simulated_robot = simulated_robot
        self._real_robot = real_robot
        
        # Initial state: safe simulation mode
        self._current_mode = Mode.SIMULATE_LOCAL
        self._active_robot: RobotInterface = simulated_robot

        # Set mode on simulated robot
        self._simulated_robot.set_mode(self._current_mode)

        # Subscribe to commands
        self._channel.subscribe(EventType.JOINT_COMMAND, self._on_joint_command)
        self._channel.subscribe(EventType.CARTESIAN_COMMAND, self._on_cartesian_command)
        self._channel.subscribe(EventType.MODE_SWITCH_REQUEST, self._on_mode_switch_request)

        print(f"[CommandHandler] Initialized in mode: {self._current_mode}")

    def _on_joint_command(self, event):
        """
        Handle JOINT_COMMAND event.
        
        Event data format:
            {'positions': List[float], 'names': Optional[List[str]]}
        """
        # print(f"[CH] Received JOINT_COMMAND, active_robot={self._active_robot}")
        if self._active_robot is None:
            # print(f"[CH] No active robot!")
            return
        
        positions = event.data.get('positions')
        print(f"[CMD RECEIVED] {time.time():.3f} | {[f'{p:.4f}' for p in positions]}")
        if positions is None:
            return
        
        # Forward to active robot (simulated or real)
        # print(f"[CH] Calling move_joints with {positions}")
        self._active_robot.move_joints(positions)
    
    def _on_cartesian_command(self, event):
        """
        Handle CARTESIAN_COMMAND event.
        
        Event data format:
            {'pose': np.ndarray (4x4), 'frame': str ('base' or 'world')}
        """
        # print(f"[CH] Received CARTESIAN_COMMAND, active_robot={self._active_robot}")
        if self._active_robot is None:
            return
        
        pose = event.data.get('pose')
        frame = event.data.get('frame', 'base')
        
        if pose is None:
            return
        
        # Forward to active robot
        if self._active_robot:
            self._active_robot.move_pose(pose, frame)

    def _on_mode_switch_request(self, event):
        """
        Handle MODE_SWITCH_REQUEST from UI.
        """
        mode_str = event.data.get('mode')   # "simulate" or "real"
        
        if mode_str == "real":
            if not self._real_robot.is_connected():
                self._channel.publish(
                    EventType.ERROR_OCCURRED,
                    data={'error': 'Cannot switch to REAL mode: Robot not connected'},
                    source="command_handler"
                )
                return
            
            # ===== ADD THIS SECTION =====
            # Get current real robot position and sync virtual robot
            real_state = self._real_robot.get_state()
            real_positions = real_state.get('joint_positions') if real_state else None
            
            if real_positions:
                self._simulated_robot.sync_to_real(real_positions)
                print(f"[CommandHandler] Synced virtual robot to real robot position")
            else:
                print(f"[CommandHandler] Warning: Could not get real robot position")
            # ===========================
            
            self._current_mode = Mode.REAL
            self._active_robot = self._real_robot
            
        else:  # "simulate"
            # If real robot is connected, use its IK for better accuracy
            if self._real_robot.is_connected():
                self._current_mode = Mode.SIMULATE_REAL_IK
            else:
                self._current_mode = Mode.SIMULATE_LOCAL
            self._active_robot = self._simulated_robot
        
        # Update simulated robot's mode (affects IK source)
        self._simulated_robot.set_mode(self._current_mode)
        
        # Publish confirmation
        self._channel.publish(
            EventType.MODE_SWITCHED,
            data={'mode': self._current_mode.name.lower()},
            source="command_handler"
        )
        
        print(f"[CommandHandler] Mode switched to: {self._current_mode}")

    @property
    def current_mode(self) -> Mode:
        """Get current operating mode."""
        return self._current_mode
    
    @property
    def active_robot(self) -> Optional[RobotInterface]:
        """Get currently active robot."""
        return self._active_robot
    
    def get_mode_string(self) -> str:
        """Get mode as string for UI display."""
        return self._current_mode.name.lower()