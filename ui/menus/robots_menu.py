"""
Robots Menu - Shows current robot (read-only).
One robot per session.
"""

from PyQt5.QtWidgets import QMenu, QAction
from core.world_state.event_types import EventType


class RobotsMenu(QMenu):
    """Menu showing current robot (read-only)."""
    
    def __init__(self, parent_window, state_channel):
        super().__init__("&Robot", parent_window)
        self.parent = parent_window
        self.state_channel = state_channel
        
        # Current robot display (read-only)
        self.current_robot_action = QAction("No robot loaded", self)
        self.current_robot_action.setEnabled(False)
        self.addAction(self.current_robot_action)
        
        # Subscribe to events
        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)
    
    def _on_robot_loaded(self, event):
        print(f"[DEBUG] RobotsMenu received: {event.data}")
        robot_id = event.data.get('asset_id')
        self.current_robot_action.setText(f"Robot: {robot_id}")
