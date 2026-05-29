"""
Container for motion control panels.
Robot connection + Joint/Cartesian control tabs.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTabWidget, QButtonGroup,
    QStackedWidget
)
from PyQt5.QtCore import Qt

from ui.panels.robot_connection_panel import RobotConnectionPanel
from ui.panels.joint_control_panel import JointControlPanel
from ui.panels.cartesian_control_panel import CartesianControlPanel


class MotionContainer(QWidget):
    """
    Motion control panel combining:
    - Robot connection (top, always visible)
    - Joint/Cartesian control (tabs below)

    All panels communicate via StateChannel events.
    No direct manager calls, no Qt signal connections.
    """

    def __init__(self, kinematic_model, state_channel, robot_manager,
                 transform_registry, asset_id, parent=None):
        super().__init__(parent)

        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager
        self.transform_registry = transform_registry
        self.asset_id = asset_id

        self._setup_ui()

        # Disable panels if no model loaded yet
        if kinematic_model is None:
            self._set_panels_enabled(False)

    def _set_panels_enabled(self, enabled: bool):
        """Enable or disable motion panels."""
        if hasattr(self, 'joint_panel'):
            self.joint_panel.setEnabled(enabled)
        if hasattr(self, 'cartesian_panel'):
            self.cartesian_panel.setEnabled(enabled)

    def _setup_ui(self):
        """Create the motion control UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        # ===== 1. Robot Connection Panel (top, always visible) =====
        self.connection_panel = RobotConnectionPanel(
            kinematic_model=self.kinematic_model,
            state_channel=self.state_channel,
            robot_manager=self.robot_manager,
            parent=self
        )
        layout.addWidget(self.connection_panel)

        # ===== 2. Mode Selector Buttons =====
        selector_widget = QWidget()
        selector_layout = QHBoxLayout(selector_widget)
        selector_layout.setContentsMargins(5, 5, 5, 5)

        selector_layout.addStretch()

        self.joint_btn = QPushButton("🔧 Joint Control")
        self.joint_btn.setCheckable(True)
        self.joint_btn.setChecked(True)
        self.joint_btn.setStyleSheet("""
            QPushButton {
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #4CAF50;
                color: white;
            }
        """)
        selector_layout.addWidget(self.joint_btn)

        self.cartesian_btn = QPushButton("🎯 Cartesian Control")
        self.cartesian_btn.setCheckable(True)
        self.cartesian_btn.setStyleSheet("""
            QPushButton {
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #2196F3;
                color: white;
            }
        """)
        selector_layout.addWidget(self.cartesian_btn)

        selector_layout.addStretch()

        # Group buttons
        self.btn_group = QButtonGroup()
        self.btn_group.addButton(self.joint_btn)
        self.btn_group.addButton(self.cartesian_btn)
        self.btn_group.buttonClicked.connect(self._on_mode_changed)

        layout.addWidget(selector_widget)

        # ===== 3. Stacked Widget for Joint/Cartesian Panels =====
        self.stacked = QStackedWidget()

        # Joint Control Panel
        self.joint_panel = JointControlPanel(
            self.kinematic_model,
            self.state_channel,
            robot_manager = self.robot_manager,
            parent=self
        )
        self.stacked.addWidget(self.joint_panel)

        # Cartesian Control Panel
        self.cartesian_panel = CartesianControlPanel(
            kinematic_model=self.kinematic_model,
            state_channel=self.state_channel,
            robot_manager=self.robot_manager,
            transform_registry=self.transform_registry,
            asset_id=self.asset_id,
            parent=self
        )
        self.stacked.addWidget(self.cartesian_panel)

        layout.addWidget(self.stacked)

    def _on_mode_changed(self, button):
        """Switch between joint and cartesian control panels."""
        if button == self.joint_btn:
            self.stacked.setCurrentWidget(self.joint_panel)
        else:
            self.stacked.setCurrentWidget(self.cartesian_panel)
            # Force Cartesian panel to sync with current robot state
            if hasattr(self, 'cartesian_panel'):
                # self.cartesian_panel._update_current_display()
                # if self.cartesian_panel.target_pose is None:
                self.cartesian_panel._reset_target()


    def get_widget(self):
        """Return self for docking."""
        return self