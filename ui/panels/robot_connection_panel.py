"""
Robot connection panel - always visible.
Handles IP connection, mode switching, and status display.
"""

import numpy as np
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QDoubleSpinBox,
                             QLabel, QPushButton, QGroupBox,
                             QLineEdit, QSpinBox, QComboBox,
                             QMessageBox, QFrame, QGridLayout, QWidget)
from PyQt5.QtCore import pyqtSignal, QTimer, Qt

from core.world_state.event_types import EventType

class RobotConnectionPanel(QWidget):
    """
    Panel for robot connection and mode control.
    Always visible, independent from motion control panels.
    """
    
    mode_changed = pyqtSignal(str)  # "simulate" or "real"
    
    def __init__(self, kinematic_model, state_channel, parent=None):
        super().__init__(parent)
        
        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = None
        
        self._setup_ui()

        # Set initial UI state
        self.mode_combo.setCurrentText("Simulate")
        self._update_connection_status(False)

        # Subscribe to events
        self.state_channel.subscribe(EventType.CONNECTION_ESTABLISHED, self._on_connected)
        self.state_channel.subscribe(EventType.CONNECTION_LOST, self._on_disconnected)
        self.state_channel.subscribe(EventType.MODE_SWITCHED, self._on_mode_switched)

        # Status update timer
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status_display)
        self._status_timer.start(100)

    def _on_connected(self, event):
        """Handle CONNECTION_ESTABLISHED event."""
        print("[RobotConnectionPanel] Connected to real robot")
        self._update_connection_status(True)
    
    def _on_disconnected(self, event):
        """Handle CONNECTION_LOST event."""
        print("[RobotConnectionPanel] Disconnected from real robot")
        self._update_connection_status(False)
    
    def _on_mode_switched(self, event):
        """Handle MODE_SWITCHED event."""
        mode = event.data.get('mode')
        print(f"[RobotConnectionPanel] Mode switched to: {mode}")
        
        # Update UI to match (block signals to avoid feedback loop)
        self.mode_combo.blockSignals(True)
        if mode == "real":
            self.mode_combo.setCurrentText("Real")
        else:
            self.mode_combo.setCurrentText("Simulate")
        self.mode_combo.blockSignals(False)
        
        # Update connection group visibility based on mode
        if mode == "real":
            self.connection_group.setVisible(False)
        else:
            self.connection_group.setVisible(True)
    
    def _update_connection_status(self, connected: bool):
        """Update UI based on connection status."""
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green;")
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Connected")
            self.disconnect_btn.setVisible(True)
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red;")
            self.connect_btn.setEnabled(True)
            self.connect_btn.setText("Connect to Robot")
            self.disconnect_btn.setVisible(False)

    def _update_connection_status(self, connected: bool):
        """Update UI based on connection status."""
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green;")
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Connected")
            self.disconnect_btn.setVisible(True)
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red;")
            self.connect_btn.setEnabled(True)
            self.connect_btn.setText("Connect to Robot")
            self.disconnect_btn.setVisible(False)

    def _setup_ui(self):
        """Create the connection control UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # ===== MODE SELECTOR =====
        mode_widget = QWidget()
        mode_layout = QHBoxLayout(mode_widget)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        
        mode_layout.addWidget(QLabel("Mode:"))
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Simulate", "Real"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_selected)
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addStretch()
        
        layout.addWidget(mode_widget)
        
        # ===== CONNECTION SECTION =====
        self.connection_group = QGroupBox("Robot Connection")
        conn_layout = QVBoxLayout()
        
        # IP Address
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("IP Address:"))
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.1.10")
        ip_layout.addWidget(self.ip_input)
        conn_layout.addLayout(ip_layout)
        
        # RTDE Frequency
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("RTDE Frequency:"))
        self.freq_spin = QSpinBox()
        self.freq_spin.setRange(10, 500)
        self.freq_spin.setValue(125)
        self.freq_spin.setSingleStep(10)
        self.freq_spin.setSuffix(" Hz")
        freq_layout.addWidget(self.freq_spin)
        freq_layout.addStretch()
        conn_layout.addLayout(freq_layout)
        
        # Connect button
        self.connect_btn = QPushButton("Connect to Robot")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        conn_layout.addWidget(self.connect_btn)
        
        self.connection_group.setLayout(conn_layout)
        layout.addWidget(self.connection_group)
        
        # ===== STATUS SECTION =====
        self.status_group = QGroupBox("Robot Status")
        self.status_group.setVisible(False)
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        status_layout.addWidget(self.status_label)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_btn.setVisible(False)
        self.disconnect_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        status_layout.addWidget(self.disconnect_btn)
        
        self.status_group.setLayout(status_layout)
        layout.addWidget(self.status_group)
        
        # ===== THRESHOLD SETTINGS =====
        self.threshold_group = QGroupBox("Event Thresholds")
        threshold_layout = QGridLayout()
        
        # Joint movement threshold
        threshold_layout.addWidget(QLabel("Joint movement:"), 0, 0)
        self.joint_threshold_spin = QDoubleSpinBox()
        self.joint_threshold_spin.setRange(0.0001, 0.1)
        self.joint_threshold_spin.setValue(0.001)
        self.joint_threshold_spin.setSingleStep(0.0005)
        self.joint_threshold_spin.setSuffix(" rad")
        self.joint_threshold_spin.valueChanged.connect(self._on_threshold_changed)
        threshold_layout.addWidget(self.joint_threshold_spin, 0, 1)
        
        # TCP position threshold
        threshold_layout.addWidget(QLabel("TCP position:"), 1, 0)
        self.pos_threshold_spin = QDoubleSpinBox()
        self.pos_threshold_spin.setRange(0.0001, 0.01)
        self.pos_threshold_spin.setValue(0.0005)
        self.pos_threshold_spin.setSingleStep(0.0001)
        self.pos_threshold_spin.setSuffix(" m")
        self.pos_threshold_spin.valueChanged.connect(self._on_threshold_changed)
        threshold_layout.addWidget(self.pos_threshold_spin, 1, 1)
        
        self.threshold_group.setLayout(threshold_layout)
        layout.addWidget(self.threshold_group)
        
        layout.addStretch()
    
    def set_robot_manager(self, manager):
        """Connect to robot manager."""
        self.robot_manager = manager
        if manager:
            manager.connection_changed.connect(self._on_connection_changed)
            manager.error_occurred.connect(self._on_robot_error)
            manager.mode_changed.connect(self._on_mode_changed_from_manager)
    
    def _on_mode_selected(self, mode_text):
        """Handle mode selection."""
        if not self.robot_manager:
            self.mode_combo.setCurrentText("Simulate")
            return
        self.robot_manager.set_mode(mode_text.lower())
    
    def _on_mode_changed_from_manager(self, mode):
        """Update UI when manager mode changes."""
        self.mode_combo.blockSignals(True)
        self.mode_combo.setCurrentText(mode.capitalize())
        self.mode_combo.blockSignals(False)
        self._update_mode_ui()
    
    def _update_mode_ui(self):
        """Update UI based on current mode."""
        if not self.robot_manager:
            return
        
        mode = self.robot_manager.current_mode
        
        if mode == "simulate":
            self.connection_group.setVisible(True)
            if self.robot_manager.is_connected:
                self.status_group.setVisible(True)
                self.disconnect_btn.setVisible(True)
            else:
                self.status_group.setVisible(False)
        else:  # real
            self.connection_group.setVisible(False)
            if self.robot_manager.is_connected:
                self.status_group.setVisible(True)
                self.disconnect_btn.setVisible(True)
            else:
                self.status_group.setVisible(False)
    
    def _on_connect_clicked(self):
        """Connect to robot."""
        print("\n=== CONNECT BUTTON CLICKED ===")
        
        if not self.robot_manager:
            print("❌ No robot manager available")
            QMessageBox.warning(self, "Error", "Robot manager not available")
            return
        
        ip = self.ip_input.text().strip()
        if not ip:
            print("❌ No IP address entered")
            QMessageBox.warning(self, "Error", "Please enter IP address")
            return
        
        print(f"📡 Attempting to connect to {ip} at {self.freq_spin.value()} Hz")
        print(f"   Robot manager: {self.robot_manager}")
        print(f"   Driver exists: {self.robot_manager.driver is not None}")
        
        success = self.robot_manager.connect_robot(ip, frequency=self.freq_spin.value())
        
        print(f"✅ Connection success: {success}")
        
        if success:
            self.status_label.setText(f"Connected to {ip}")
            self.status_group.setVisible(True)
            self.disconnect_btn.setVisible(True)
            self.connect_btn.setVisible(False)
        else:
            self.status_label.setText("Connection failed")
            self.status_group.setVisible(True)
            self.disconnect_btn.setVisible(False)
    
    def _on_disconnect_clicked(self):
        """Disconnect from robot."""
        reply = QMessageBox.question(self, "Confirm", "Disconnect from robot?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.robot_manager.disconnect_robot()
            self.status_group.setVisible(False)
            self.connect_btn.setVisible(True)
    
    def _on_threshold_changed(self):
        """Update driver thresholds."""
        if self.robot_manager and self.robot_manager.driver:
            self.robot_manager.driver.set_thresholds(
                joint_rad=self.joint_threshold_spin.value(),
                pos_m=self.pos_threshold_spin.value()
            )
    
    def _on_connection_changed(self, connected, message):
        """Handle connection status changes."""
        self.status_label.setText(f"{'Connected' if connected else 'Disconnected'} - {message}")
        self.status_label.setStyleSheet("QLabel { color: %s; }" % ("green" if connected else "#666"))
    
    def _on_robot_error(self, error_msg):
        """Handle robot errors."""
        self.status_label.setText(f"Error: {error_msg}")
        self.status_label.setStyleSheet("QLabel { color: red; }")
        QMessageBox.warning(self, "Robot Error", error_msg)
    
    def _update_status_display(self):
        """Periodic status updates (can be overridden)."""
        pass
    
    def get_widget(self):
        """Return self for docking."""
        return self