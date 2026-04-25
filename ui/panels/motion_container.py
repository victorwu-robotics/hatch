"""
Container for motion control panels.
Robot connection + Joint/Cartesian control tabs.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QTabWidget, QButtonGroup,
                             QStackedWidget)
from PyQt5.QtCore import Qt

from ui.panels.robot_connection_panel import RobotConnectionPanel
from ui.panels.joint_control_panel import JointControlPanel
from ui.panels.cartesian_control_panel import CartesianControlPanel

class MotionContainer(QWidget):
    """
    Motion control panel combining:
    - Robot connection (top, always visible)
    - Joint/Cartesian control (tabs below)
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

        # If no model loaded yet, disable panels or show placeholder
        if kinematic_model is None:
            self._set_panels_enabled(False)

    def _set_panels_enabled(self, enabled: bool):
        """Enable or disable motion panels."""
        if hasattr(self, 'joint_panel'):
            self.joint_panel.setEnabled(enabled)
        if hasattr(self, 'cartesian_panel'):
            self.cartesian_panel.setEnabled(enabled)

    def _setup_ui(self):
        """Create the motion control UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        # ===== 1. Robot Connection Panel (top, always visible) =====
        self.connection_panel = RobotConnectionPanel(
            kinematic_model=self.kinematic_model,
            state_channel=self.state_channel,
            parent=self
        )
        self.connection_panel.set_robot_manager(self.robot_manager)
        layout.addWidget(self.connection_panel)

        # ===== 2. Tab Widget for Joint/Cartesian Control =====
        self.tab_widget = QTabWidget()

        # Joint Control Tab
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
        self.cartesian_btn.setEnabled(True)  # Disable until implemented
        self.cartesian_btn.setStyleSheet("""
            QPushButton {
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #2196F3;
                color: white;
            }
            QPushButton:disabled {
                color: #999;
                background-color: #eee;
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
            robot_manager=self.robot_manager,
            parent=self
        )
        self.stacked.addWidget(self.joint_panel)
        
        # Cartesian Control Panel (now real, not placeholder)
        print("Creating CartesianControlPanel...")
        self.cartesian_panel = CartesianControlPanel(
            kinematic_model=self.kinematic_model,
            state_channel=self.state_channel,
            robot_manager=self.robot_manager,
            transform_registry=self.transform_registry,  # Add this
            asset_id=self.asset_id,  # Also pass asset_id for frame namespacing
            parent=self
        )
        print(f"CartesianPanel created: {self.cartesian_panel}")
        self.stacked.addWidget(self.cartesian_panel)
        print(f"Stacked widget now has {self.stacked.count()} panels")

        layout.addWidget(self.stacked)
    
    def _on_mode_changed(self, button):
        """Switch between control modes."""
        print(f"\n=== Control Mode Changed ===")
        print(f"Button clicked: {button.text()}")
        print(f"Joint btn checked: {self.joint_btn.isChecked()}")
        print(f"Cartesian btn checked: {self.cartesian_btn.isChecked()}")

        if button == self.joint_btn:
            print("Switching to Joint panel")
            self.stacked.setCurrentWidget(self.joint_panel)
        else:
            print("Switching to Cartesian panel")
            self.stacked.setCurrentWidget(self.stacked.widget(1))  # Cartesian placeholder

        print(f"Current widget: {self.stacked.currentWidget()}")
    
    def set_robot_manager(self, manager):
        """Pass robot manager to child panels."""
        self.robot_manager = manager
        if hasattr(self, 'joint_panel'):
            self.joint_panel.robot_manager = manager
            manager.state_received.connect(self.joint_panel._on_robot_state_received)
    
        if hasattr(self, 'cartesian_panel'):
            self.cartesian_panel.robot_manager = manager
            manager.state_received.connect(self.cartesian_panel._on_robot_state_received)
            print(f"===== Just connected to Robot Manager =====")

    def get_widget(self):
        """Return self for docking."""
        return self