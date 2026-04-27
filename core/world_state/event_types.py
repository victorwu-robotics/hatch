"""
Event types for StateChannel.

Canonical event list for Hatch (孵).
All components publish and subscribe using ONLY these event types.

Events are organized into three categories:
- Commands: UI → System (requests to do something)
- State: System → UI (reports of what happened)
- Lifecycle: System → System (internal coordination)

Principle #6: Time = StateChannel. All events flow through here.
Principle #2: Event-Driven, No Polling. No component polls for changes.
"""

from enum import Enum


class EventType(Enum):
    """
    Canonical event types for the Hatch platform.

    Naming convention: NOUN_VERB (e.g., ROBOT_LOADED, MODE_SWITCHED).
    Commands use imperative nouns (REQUEST, COMMAND).
    State events use past-tense verbs (LOADED, SWITCHED, ESTABLISHED).
    """

    # ===== Robot Lifecycle =====
    ROBOT_LOAD_REQUEST = "robot_load_request"       # UI → System: Load a URDF
    ROBOT_LOADED = "robot_loaded"                   # System → All: URDF loaded, model ready
    ROBOT_UNLOAD_REQUEST = "robot_unload_request"   # UI → System: Unload current robot

    # ===== Robot Commands =====
    JOINT_COMMAND = "joint_command"                 # UI → System: Move joints to positions
    CARTESIAN_COMMAND = "cartesian_command"         # UI → System: Move TCP to pose

    # ===== Mode Control =====
    MODE_SWITCH_REQUEST = "mode_switch_request"     # UI → System: Request mode change
    MODE_SWITCHED = "mode_switched"                 # System → All: Mode changed (simulate/real)

    # ===== Connection Control =====
    CONNECTION_REQUEST = "connection_request"       # UI → System: Connect to hardware
    CONNECTION_ESTABLISHED = "connection_established"  # System → All: Connected successfully
    CONNECTION_LOST = "connection_lost"             # System → All: Connection dropped
    DISCONNECTION_REQUEST = "disconnection_request" # UI → System: Disconnect from hardware

    # ===== Robot State =====
    ROBOT_STATE = "robot_state"                     # Robot → All: Joint positions, TCP pose

    # ===== Errors =====
    ERROR_OCCURRED = "error_occurred"               # Any → UI: Something went wrong
    
