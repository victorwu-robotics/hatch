"""
Camera Menu - Handles camera controls for the active camera.
"""

from PyQt5.QtWidgets import QAction


class CameraMenu:
    """Camera menu for controlling the active camera."""
    
    def __init__(self, parent_window, camera_manager):
        self.parent = parent_window
        self.camera_manager = camera_manager
        self.actions = []
        self._create_actions()
    
    def _create_actions(self):
        # Show Camera Panel
        show_panel = QAction("&Show Camera Panel", self.parent)
        show_panel.setShortcut("Ctrl+Shift+C")
        show_panel.triggered.connect(self.camera_manager.show_panel)
        self.actions.append(show_panel)
        
        # Separator
        sep = QAction(self.parent)
        sep.setSeparator(True)
        self.actions.append(sep)
        
        # Start Camera
        start_action = QAction("&Start Camera", self.parent)
        start_action.triggered.connect(self._on_start)
        self.actions.append(start_action)
        
        # Stop Camera
        stop_action = QAction("&Stop Camera", self.parent)
        stop_action.triggered.connect(self._on_stop)
        self.actions.append(stop_action)
    
    def _on_start(self):
        cam_id = self.camera_manager.active_camera_id
        if cam_id:
            self.camera_manager.start_camera(cam_id)
    
    def _on_stop(self):
        cam_id = self.camera_manager.active_camera_id
        if cam_id:
            self.camera_manager.stop_camera(cam_id)
    
    def get_actions(self):
        return self.actions


