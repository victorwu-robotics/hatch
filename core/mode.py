"""
Operating modes for robot control.
"""

from enum import Enum, auto


class Mode(Enum):
    """
    Three distinct operating modes.
    
    SIMULATE_LOCAL:    Use local IK solver, move virtual robot only
    SIMULATE_REAL_IK:  Use real robot's IK solver, move virtual robot only
    REAL:              Use real robot's IK solver, move real robot
    """
    
    SIMULATE_LOCAL = auto()
    SIMULATE_REAL_IK = auto()
    REAL = auto()
    
    def __str__(self):
        return self.name
    
    def is_simulate(self) -> bool:
        """Return True if mode is any simulate variant."""
        return self in (Mode.SIMULATE_LOCAL, Mode.SIMULATE_REAL_IK)
    
    def uses_real_ik(self) -> bool:
        """Return True if mode uses real robot's IK solver."""
        return self in (Mode.SIMULATE_REAL_IK, Mode.REAL)
    
    def moves_real_robot(self) -> bool:
        """Return True if mode actually moves hardware."""
        return self == Mode.REAL