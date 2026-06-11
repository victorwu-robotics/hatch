"""
Robot Interface - Abstract base class for all robot implementations.

Defines the contract that all robots (real and simulated) must fulfill.
Uses Python's abc module — no Qt dependency.

Principle: Pure Python. No Qt in core abstractions.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Any
import numpy as np


class RobotInterface(ABC):
    """
    Abstract base class for all robot implementations.

    Subclasses must implement all abstract methods.
    RealRobot and SimulatedRobot both conform to this interface,
    allowing CommandHandler to treat them identically.

    No Qt dependency — this is a plain Python ABC.
    """

    @abstractmethod
    def move_joints(self, positions: List[float]) -> bool:
        """
        Command robot to move to target joint positions.

        Args:
            positions: List of joint angles in radians,
                       ordered according to the robot's joint_names.

        Returns:
            True if command was accepted successfully.
        """
        ...

    @abstractmethod
    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        """
        Command robot to move TCP to target pose.

        Args:
            pose: 4x4 homogeneous transform representing target TCP pose.
            frame: Reference frame for the pose ("base" or "world").

        Returns:
            True if command was accepted successfully.
        """
        ...

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """
        Get current robot state.

        Returns:
            Dictionary with keys:
            - 'joint_positions': List[float] current joint angles
            - 'tcp_pose': np.ndarray or List[float] current TCP pose
            - 'timestamp': float time of state acquisition
            - 'source': str identifier ("simulated_robot" or "real_robot")
        """
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if robot is connected and ready.

        Returns:
            True for simulated robots (always ready).
            True for real robots only when hardware connection is active.
        """
        ...

    @abstractmethod
    def connect(self, ip: str, **kwargs) -> bool:
        """
        Connect to robot hardware.

        Args:
            ip: Robot IP address.
            **kwargs: Driver-specific connection parameters.

        Returns:
            True if connection successful.
            For simulated robots, always returns True.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """
        Disconnect from robot hardware.

        For simulated robots, this is a no-op.
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """
        Emergency stop — halt all motion immediately.
        """
        ...