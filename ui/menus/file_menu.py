"""
File Menu - Handles file operations (Load URDF, Save Screenshot, Exit).

Communicates via StateChannel events and RobotManager method calls.
Principle: UI Separate from Services.
"""

from pathlib import Path
from PyQt5.QtWidgets import QAction, QFileDialog, QMessageBox
from core.world_state.event_types import EventType


class FileMenu:
    """File menu with Load URDF, Save Screenshot, and Exit."""

    def __init__(self, parent_window, robot_manager, state_channel):
        """
        Initialize file menu.

        Args:
            parent_window: The main window.
            robot_manager: RobotManager for loading robots.
            state_channel: Application event bus.
        """
        self.parent = parent_window
        self.robot_manager = robot_manager
        self.state_channel = state_channel
        self.actions = []

        self._create_actions()

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

    # =================================================================
    # Handlers
    # =================================================================

    def _on_load_urdf(self):
        """Load a URDF file from disk."""
        # One robot per session (Principle)
        if self.robot_manager and self.robot_manager.current_asset_id:
            QMessageBox.information(
                self.parent,
                "Robot Already Loaded",
                "This platform supports one robot per session.\n"
                "Please restart the application to load a different robot."
            )
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self.parent,
            "Load URDF File",
            "",
            "URDF Files (*.urdf *.xacro);;All Files (*)"
        )

        if not file_path:
            return

        try:
            asset_id = self.robot_manager.load_robot(file_path)
            if asset_id:
                if hasattr(self.parent, 'statusBar'):
                    self.parent.statusBar().showMessage(
                        f"Loaded robot: {asset_id}", 3000
                    )
            else:
                QMessageBox.warning(
                    self.parent,
                    "Load Failed",
                    "Failed to load robot. See console for details."
                )
        except Exception as e:
            QMessageBox.warning(
                self.parent,
                "Load Error",
                f"Error loading URDF:\n\n{str(e)}"
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
                QMessageBox.information(
                    self.parent,
                    "Screenshot Saved",
                    f"Screenshot saved to:\n{file_path}"
                )
            except Exception as e:
                QMessageBox.warning(
                    self.parent,
                    "Screenshot Error",
                    f"Failed to save screenshot:\n\n{str(e)}"
                )

    def get_actions(self):
        """Get all file menu actions."""
        return self.actions