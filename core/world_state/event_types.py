"""
Event types for StateChannel.
Separated into commands, state, and status.
"""

from enum import Enum

class EventType(Enum):
    # ===== Robot lifecycle =====
    ROBOT_LOAD_REQUEST = "robot_load_request"
    ROBOT_LOADED = "robot_loaded"

    # ===== Robot Control =====
    JOINT_COMMAND = "joint_command"
    CARTESIAN_COMMAND = "cartesian_command"
    MODE_SWITCH_REQUEST = "mode_switch_request"
    MODE_SWITCHED = "mode_switched"

    # ===== Robot State =====
    ROBOT_STATE = "robot_state"

    # ===== Connection Events =====
    CONNECTION_REQUEST = "connection_request"
    CONNECTION_ESTABLISHED = "connection_established"
    CONNECTION_LOST = "connection_lost"
    DISCONNECTION_REQUEST = "disconnection_request"

    # ===== Existing events =====
    JOINT_UPDATE = "joint_update"
    CAMERA_STARTED = "camera_started"
    CAMERA_STOPPED = "camera_stopped"
    UI_VISIBILITY_CHANGED = "ui_visibility_changed"
    ERROR_OCCURRED = "error_occurred"
    STATUS_UPDATED = "status_updated"
    
    # Robot arm events
    ROBOT_STATE_UPDATE = "robot_state_update"
    ROBOT_MODE_CHANGED = "robot_mode_changed"
    ROBOT_CONNECTED = "robot_connected"
    ROBOT_DISCONNECTED = "robot_disconnected"
    ROBOT_COMMAND_SENT = "robot_command_sent"
    ROBOT_ERROR = "robot_error"
    
    
