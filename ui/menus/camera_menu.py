"""
Camera Menu - Handles camera controls.
"""

from PyQt5.QtWidgets import QAction


class CameraMenu:
    """Camera menu for camera controls."""
    
    def __init__(self, parent_window, camera_manager):
        """
        Initialize camera menu.
        
        Args:
            parent_window: The main window
            camera_manager: The camera manager instance
        """
        self.parent = parent_window
        self.camera_manager = camera_manager
        self.actions = []
        
        self._create_actions()
    
    def _create_actions(self):
        """Create all camera menu actions."""
        # Show Camera Panel
        show_panel_action = QAction("&Show Camera Panel", self.parent)
        show_panel_action.setShortcut("Ctrl+Shift+C")
        show_panel_action.triggered.connect(self.camera_manager.show_panel)
        self.actions.append(show_panel_action)
        
        # Transform to World Frame
        self.transform_action = QAction("Transform to World Frame", self.parent)
        self.transform_action.setCheckable(True)
        self.transform_action.setChecked(True)
        self.transform_action.triggered.connect(self.camera_manager.set_transform_to_world)
        self.actions.append(self.transform_action)
        
        # Show Camera Frames
        self.show_frames_action = QAction("Show Camera Frames", self.parent)
        self.show_frames_action.setCheckable(True)
        self.show_frames_action.setChecked(True)
        self.show_frames_action.triggered.connect(self.camera_manager.set_show_frames)
        self.actions.append(self.show_frames_action)
    
    def get_actions(self):
        """Get all camera menu actions."""
        return self.actions