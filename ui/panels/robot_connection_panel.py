"""
Robot Connection Panel - Pure UI for connection and mode control.

Handles IP connection, mode switching, and status display.
Communicates via StateChannel events and direct RobotManager method calls.
Does NOT connect to Qt signals from RobotManager.

Principle #9: UI Separate from Services.
Principle #2: Event-Driven. Subscribes to state, publishes commands.
"""

from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QDoubleSpinBox,
    QLabel, QPushButton, QGroupBox,
    QLineEdit, QSpinBox, QComboBox,
    QMessageBox, QGridLayout, QWidget
)
from PyQt5.QtCore import QTimer, Qt

from core.world_state.event_types import EventType


class RobotConnectionPanel(QWidget):
    """
    Panel for robot connection and mode control.

    Publishes:
    - MODE_SWITCH_REQUEST when user changes mode
    - CONNECTION_REQUEST when user clicks connect
    - DISCONNECTION_REQUEST when user clicks disconnect

    Subscribes to:
    - CONNECTION_ESTABLISHED to update connection status
    - CONNECTION_LOST to update connection status
    - MODE_SWITCHED to update mode display
    - ERROR_OCCURRED to show errors
    """

    def __init__(self, kinematic_model, state_channel, robot_manager, parent=None):
        super().__init__(parent)

        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager

        self._setup_ui()

        # Initial UI state
        self.mode_combo.setCurrentText("Simulate")
        self._update_connection_status(False)

        # Subscribe to events via StateChannel
        self.state_channel.subscribe(
            EventType.CONNECTION_ESTABLISHED, self._on_connected
        )
        self.state_channel.subscribe(
            EventType.CONNECTION_LOST, self._on_disconnected
        )
        self.state_channel.subscribe(
            EventType.MODE_SWITCHED, self._on_mode_switched
        )
        self.state_channel.subscribe(
            EventType.ERROR_OCCURRED, self._on_error
        )

    # =================================================================
    # UI Setup
    # =================================================================

    def _setup_ui(self):
        """Create the connection control UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # Mode Selector
        layout.addWidget(self._create_mode_selector())

        # Connection Section
        self.connection_group = self._create_connection_group()
        layout.addWidget(self.connection_group)

        # Status Section
        self.status_group = self._create_status_group()
        layout.addWidget(self.status_group)

        layout.addStretch()

    def _create_mode_selector(self):
        """Create the mode selection row."""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(QLabel("Mode:"))

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Simulate", "Real"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_selected)
        layout.addWidget(self.mode_combo)
        layout.addStretch()

        return widget

    def _create_connection_group(self):
        """Create the connection settings group."""
        group = QGroupBox("Robot Connection")
        layout = QVBoxLayout()

        # IP Address
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("IP Address:"))
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.1.10")
        ip_layout.addWidget(self.ip_input)
        layout.addLayout(ip_layout)

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
        layout.addLayout(freq_layout)

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
        layout.addWidget(self.connect_btn)

        group.setLayout(layout)
        return group

    def _create_status_group(self):
        """Create the status display group."""
        group = QGroupBox("Robot Status")
        layout = QVBoxLayout()

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        layout.addWidget(self.status_label)

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
        layout.addWidget(self.disconnect_btn)

        group.setLayout(layout)
        group.setVisible(False)
        return group

    # =================================================================
    # User Actions → Commands
    # =================================================================

    def _on_mode_selected(self, mode_text: str):
        """Handle mode selection from combo box."""
        if not self.robot_manager:
            return

        mode = mode_text.lower()
        self.robot_manager.set_mode(mode)

    def _on_connect_clicked(self):
        """Handle connect button click."""
        if not self.robot_manager:
            QMessageBox.warning(self, "Error", "Robot manager not available")
            return

        ip = self.ip_input.text().strip()
        if not ip:
            QMessageBox.warning(self, "Error", "Please enter IP address")
            return

        success = self.robot_manager.connect_robot(
            ip, frequency=self.freq_spin.value()
        )

        if not success:
            self.status_label.setText("Connection failed")
            self.status_label.setStyleSheet("QLabel { color: red; }")
            self.status_group.setVisible(True)

    def _on_disconnect_clicked(self):
        """Handle disconnect button click."""
        reply = QMessageBox.question(
            self, "Confirm", "Disconnect from robot?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes and self.robot_manager:
            self.robot_manager.disconnect_robot()

    # =================================================================
    # StateChannel Events → UI Updates
    # =================================================================

    def _on_connected(self, event):
        """Handle CONNECTION_ESTABLISHED event."""
        message = event.data.get('message', 'Connected')
        self.status_label.setText(message)
        self.status_label.setStyleSheet("QLabel { color: green; }")
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("Connected")
        self.disconnect_btn.setVisible(True)
        self.status_group.setVisible(True)

    def _on_disconnected(self, event):
        """Handle CONNECTION_LOST event."""
        message = event.data.get('message', 'Disconnected')
        self.status_label.setText(message)
        self.status_label.setStyleSheet("QLabel { color: red; }")
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Connect to Robot")
        self.disconnect_btn.setVisible(False)
        self.status_group.setVisible(True)
        self.connection_group.setVisible(True)

    def _on_mode_switched(self, event):
        """Handle MODE_SWITCHED event."""
        mode = event.data.get('mode', '')

        # Update combo box without triggering signal
        self.mode_combo.blockSignals(True)
        if mode == "real":
            self.mode_combo.setCurrentText("Real")
            self.connection_group.setVisible(False)
        else:
            self.mode_combo.setCurrentText("Simulate")
            self.connection_group.setVisible(True)
        self.mode_combo.blockSignals(False)

    def _on_error(self, event):
        """Handle ERROR_OCCURRED event."""
        error_data = event.data
        if isinstance(error_data, dict):
            message = error_data.get('error', str(error_data))
        else:
            message = str(error_data)

        self.status_label.setText(f"Error: {message}")
        self.status_label.setStyleSheet("QLabel { color: red; }")

    # =================================================================
    # Helpers
    # =================================================================

    def _update_connection_status(self, connected: bool):
        """Update UI for connection state."""
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("QLabel { color: green; }")
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Connected")
            self.disconnect_btn.setVisible(True)
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("QLabel { color: red; }")
            self.connect_btn.setEnabled(True)
            self.connect_btn.setText("Connect to Robot")
            self.disconnect_btn.setVisible(False)