"""
File Menu - Handles file operations.
"""

from pathlib import Path
from PyQt5.QtWidgets import QAction, QFileDialog, QMessageBox
from core.world_state.event_types import EventType


class FileMenu:
    """File menu actions and handlers."""
    
    def __init__(self, parent_window, robot_manager, state_channel):
        """
        Initialize file menu.
        
        Args:
            parent_window: The main window
            mesh_loader: The asset manager instance
            robot_manager
        """
        self.parent = parent_window
        self.robot_manager = robot_manager
        self.state_channel = state_channel
        self.actions = []
        
        self._create_actions()

        # Subscribe to robot events to update UI state
        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)

        # Initially, no robot loaded
        self._update_close_action_enabled(False)
    
    def _create_actions(self):
        """Create all file menu actions."""
        # Load URDF
        self.load_urdf_action = QAction("&Load URDF...", self.parent)
        self.load_urdf_action.setShortcut("Ctrl+O")
        self.load_urdf_action.triggered.connect(self._on_load_urdf)
        self.actions.append(self.load_urdf_action)
        
        # Separator
        separator = QAction(self.parent)
        separator.setSeparator(True)
        self.actions.append(separator)
        
        # Save Screenshot
        self.screenshot_action = QAction("&Save Screenshot...", self.parent)
        self.screenshot_action.setShortcut("Ctrl+S")
        self.screenshot_action.triggered.connect(self._save_screenshot)
        self.actions.append(self.screenshot_action)
        
        # Separator
        separator2 = QAction(self.parent)
        separator2.setSeparator(True)
        self.actions.append(separator2)
        
        # Exit
        self.exit_action = QAction("E&xit", self.parent)
        self.exit_action.setShortcut("Ctrl+Q")
        self.exit_action.triggered.connect(self.parent.close)
        self.actions.append(self.exit_action)
    
    def _on_load_urdf(self):
        """Load a URDF file from disk."""
        # Check if a robot is already loaded
        if self.robot_manager and self.robot_manager.current_asset_id:
            QMessageBox.information(
                self.parent,
                "Robot Already Loaded",
                "This platform supports one robot per session.\n"
                "Please restart the application to load a different robot."
            )
            return

        from PyQt5.QtWidgets import QFileDialog
        
        file_path, _ = QFileDialog.getOpenFileName(
            self.parent,
            "Load URDF File",
            "",
            "URDF Files (*.urdf);;All Files (*)"
        )
        
        if not file_path:
            return
        
        try:
            # Use robot_manager to load robot
            if self.robot_manager:
                asset_id = self.robot_manager.load_robot(file_path)
                if asset_id:
                    print(f"✅ Robot loaded from {file_path}")
                    if hasattr(self.parent, 'statusBar'):
                        self.parent.statusBar().showMessage(f"Loaded robot: {asset_id}", 3000)
                else:
                    print(f"❌ Failed to load robot from {file_path}")
            else:
                print("❌ RobotArmManager not available")                
        except Exception as e:
            print(f"❌ Error loading URDF: {e}")
            import traceback
            traceback.print_exc()
    
        # The UI manager will handle disabling the close action via callback

    def _on_robot_loaded(self, event):
        """Enable close action when robot loads."""
        self._update_close_action_enabled(True)
    
    def _on_robot_unloaded(self, event):
        """Disable close action when robot unloads."""
        self._update_close_action_enabled(False)
    
    def _update_close_action_enabled(self, enabled: bool):
        """Enable or disable close action."""
        if hasattr(self, 'close_robot_action'):
            self.close_robot_action.setEnabled(enabled)
    
    def _on_close_robot(self):
        """Close current robot."""
        if self.robot_manager and self.robot_manager.current_asset_id:
            self.state_channel.publish(
                EventType.ROBOT_UNLOAD_REQUEST,
                data={'robot_id': self.robot_manager.current_asset_id},
                source="file_menu"
            )

    def _save_screenshot(self):
        """Save screenshot of current view."""
        file_path, _ = QFileDialog.getSaveFileName(
            self.parent,
            "Save Screenshot",
            str(Path.home() / "robot_screenshot.png"),
            "PNG Images (*.png);;All Files (*.*)"
        )
        
        if file_path:
            try:
                self.parent.engine.save_screenshot(file_path)
                print(f"MainWindow: Screenshot saved to {file_path}")
                QMessageBox.information(self.parent, "Screenshot Saved", f"Screenshot saved to:\n{file_path}")
            except Exception as e:
                QMessageBox.warning(self.parent, "Screenshot Error", f"Failed to save screenshot:\n\n{str(e)}")
    
    def get_actions(self):
        """Get all file menu actions."""
        return self.actions