"""
Core package for RoboPlatform.

Contains world state management, kinematics, and event system.
"""

# Export key classes for easier imports
from . import world_state
from . import kinematics

__all__ = [
    'world_state',
    'kinematics'
]
