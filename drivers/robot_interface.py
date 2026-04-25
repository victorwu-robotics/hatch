"""
Robot Interface - Base class for all robots.
Subclasses must implement these methods.
"""

from typing import List, Dict, Optional, Any
import numpy as np
from PyQt5.QtCore import QObject


class RobotInterface(QObject):
    """
    Base class for all robots (real and simulated).
    Subclasses must override all methods.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
    
    def move_joints(self, positions: List[float]) -> bool:
        """Command robot to move to target joint positions."""
        raise NotImplementedError("Subclass must implement move_joints()")
    
    def move_pose(self, pose: np.ndarray, frame: str = "base") -> bool:
        """Command robot to move TCP to target pose."""
        raise NotImplementedError("Subclass must implement move_pose()")
    
    def get_state(self) -> Dict[str, Any]:
        """Get current robot state (joint positions, tcp pose, timestamp)."""
        raise NotImplementedError("Subclass must implement get_state()")
    
    def is_connected(self) -> bool:
        """Return True if connected to hardware (or always True for simulation)."""
        raise NotImplementedError("Subclass must implement is_connected()")
    
    def connect(self, ip: str, **kwargs) -> bool:
        """Connect to real robot (no-op for simulation)."""
        raise NotImplementedError("Subclass must implement connect()")
    
    def disconnect(self) -> None:
        """Disconnect from real robot (no-op for simulation)."""
        raise NotImplementedError("Subclass must implement disconnect()")
    
    def stop(self) -> None:
        """Emergency stop - halt all motion."""
        raise NotImplementedError("Subclass must implement stop()")