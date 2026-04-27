"""
Base robot arm driver interface.
Pure driver logic with NO threading - follows same pattern as base_camera.py.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Callable, Any


class BaseRobotArmDriver(ABC):
    """
    Abstract base class for robot arm drivers.
    This class handles the robot-specific communication logic only.
    Threading (if needed) is handled externally by the application.
    """
    
    def __init__(self):
        # Connection state
        self.is_connected = False
        self.robot_ip = None
        
        # Callbacks (set by application)
        self._state_callback = None      # Called when new robot state received
        self._connection_callback = None # Called when connection status changes
        self._error_callback = None      # Called on errors
        
        # Robot info (populated by implementation)
        self.joint_names: List[str] = []
        self.urdf_path: Optional[str] = None
        
    @abstractmethod
    def connect(self, ip: str, **kwargs) -> bool:
        """
        Connect to the real robot.
        
        Args:
            ip: Robot IP address
            **kwargs: Driver-specific connection parameters
            
        Returns:
            bool: True if connection successful
        """
        pass
    
    @abstractmethod
    def disconnect(self):
        """
        Disconnect from the real robot.
        Clean shutdown of all communication.
        """
        pass
    
    @abstractmethod
    def send_joint_command(self, positions: List[float]) -> bool:
        """
        Send joint position command to real robot.
        
        Args:
            positions: Joint positions in radians (order must match joint_names)
            
        Returns:
            bool: True if command sent successfully
        """
        pass
    
    @abstractmethod
    def get_current_state(self) -> Optional[Dict[str, Any]]:
        """
        Get the latest received robot state.
        
        Returns:
            Dict with keys:
                - 'joint_positions': List[float] current joint positions (radians)
                - 'joint_velocities': Optional[List[float]] joint velocities
                - 'joint_currents': Optional[List[float]] joint currents
                - 'timestamp': float timestamp in seconds
                - 'tcp_pose': Optional[List[float]] TCP pose [x,y,z,rx,ry,rz]
                - 'tcp_force': Optional[List[float]] TCP force/torque
                - Any other robot-specific data
        """
        pass
    
    @abstractmethod
    def is_robot_moving(self) -> bool:
        """
        Check if robot is currently in motion.
        
        Returns:
            bool: True if robot is moving
        """
        pass
    
    @abstractmethod
    def stop_motion(self):
        """
        Emergency stop - halt all robot motion.
        """
        pass
    
    # ===== Callback setters =====
    
    def set_state_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Set callback for when new robot state is received.
        
        IMPORTANT: This callback will be called from RTDE's internal thread.
        The application MUST handle thread safety (e.g., using Qt signals).
        
        Args:
            callback: Function that takes state dict as argument
        """
        self._state_callback = callback
    
    def set_connection_callback(self, callback: Callable[[bool, str], None]):
        """
        Set callback for connection status changes.
        
        Args:
            callback: Function that takes (is_connected, message)
        """
        self._connection_callback = callback
    
    def set_error_callback(self, callback: Callable[[str, Exception], None]):
        """
        Set callback for errors.
        
        Args:
            callback: Function that takes (error_message, exception)
        """
        self._error_callback = callback
    
    # ===== Helper methods =====
    
    def get_joint_count(self) -> int:
        """Get number of joints."""
        return len(self.joint_names)
    
    def validate_joint_positions(self, positions: List[float]) -> bool:
        """
        Validate that joint positions are within limits.
        Override in implementation if limits are known.
        """
        return len(positions) == self.get_joint_count()