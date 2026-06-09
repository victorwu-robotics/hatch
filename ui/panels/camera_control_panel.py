"""
Camera Control Panel - Controls multiple cameras discovered from URDF.

Single panel that switches between cameras. Start/Stop controls
the selected camera. Multiple cameras can run simultaneously.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QLineEdit, QGroupBox, QCheckBox, QSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class CameraControlPanel(QWidget):
    """Panel for controlling cameras discovered from the URDF."""

    # Signals
    start_requested = pyqtSignal(str, dict)   # camera_id, config
    stop_requested = pyqtSignal(str)           # camera_id
    camera_selected = pyqtSignal(str)          # camera_id
    transform_toggled = pyqtSignal(bool)
    show_frames_toggled = pyqtSignal(bool)
    visible_toggled = pyqtSignal(bool)

    def __init__(self, camera_manager, parent=None):
        super().__init__(parent)
        self.camera_manager = camera_manager
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Title
        title = QLabel("Camera Control")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Camera Selector
        select_group = QGroupBox("Camera")
        select_layout = QVBoxLayout()

        self.camera_combo = QComboBox()
        self.camera_combo.currentIndexChanged.connect(self._on_camera_selected)
        select_layout.addWidget(self.camera_combo)

        # Camera info label
        self.camera_info = QLabel("")
        self.camera_info.setStyleSheet("color: #666; font-size: 10px;")
        select_layout.addWidget(self.camera_info)

        select_group.setLayout(select_layout)
        layout.addWidget(select_group)

        # Connection Settings (changes based on camera type)
        self.connection_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout()

        # IP address (for network cameras)
        self.ip_layout = QHBoxLayout()
        self.ip_layout.addWidget(QLabel("IP:"))
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.1.100")
        self.ip_layout.addWidget(self.ip_input)
        conn_layout.addLayout(self.ip_layout)

        # Port (for network cameras)
        self.port_layout = QHBoxLayout()
        self.port_layout.addWidget(QLabel("Port:"))
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(24691)
        self.port_layout.addWidget(self.port_input)
        conn_layout.addLayout(self.port_layout)

        # Serial Number (for USB cameras)
        self.sn_layout = QHBoxLayout()
        self.sn_layout.addWidget(QLabel("Serial:"))
        self.sn_input = QLineEdit()
        self.sn_input.setPlaceholderText("Auto-detect if empty")
        self.sn_layout.addWidget(self.sn_input)
        conn_layout.addLayout(self.sn_layout)

        self.connection_group.setLayout(conn_layout)
        layout.addWidget(self.connection_group)

        # Start/Stop buttons
        btn_layout = QHBoxLayout()

        self.start_btn = QPushButton("Start Camera")
        self.start_btn.clicked.connect(self._on_start)
        self.start_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px; }"
        )
        btn_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop Camera")
        self.stop_btn.clicked.connect(self._on_stop)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 6px; }"
        )
        btn_layout.addWidget(self.stop_btn)

        layout.addLayout(btn_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #666;")
        layout.addWidget(self.status_label)

        # Display toggles
        display_group = QGroupBox("Display")
        display_layout = QVBoxLayout()

        self.visible_check = QCheckBox("Show Point Cloud")
        self.visible_check.setChecked(True)
        self.visible_check.toggled.connect(self.visible_toggled.emit)
        display_layout.addWidget(self.visible_check)

        self.transform_check = QCheckBox("Transform to World Frame")
        self.transform_check.setChecked(True)
        self.transform_check.toggled.connect(self.transform_toggled.emit)
        display_layout.addWidget(self.transform_check)

        self.frames_check = QCheckBox("Show Camera Frames")
        self.frames_check.setChecked(False)
        self.frames_check.toggled.connect(self.show_frames_toggled.emit)
        display_layout.addWidget(self.frames_check)

        display_group.setLayout(display_layout)
        layout.addWidget(display_group)

        layout.addStretch()

    # =================================================================
    # Public Methods
    # =================================================================

    def refresh_camera_list(self):
        """Update the camera dropdown from the manager."""
        self.camera_combo.blockSignals(True)
        self.camera_combo.clear()

        cameras = self.camera_manager.get_available_cameras()
        for cam in cameras:
            status = "●" if cam['is_running'] else "○"
            label = f"{status} {cam['id']} ({cam['type']})"
            self.camera_combo.addItem(label, cam['id'])

        self.camera_combo.blockSignals(False)

        # Update active camera display
        if cameras:
            self._update_for_camera(cameras[0]['id'])

    def update_status(self, message: str, is_error: bool = False):
        """Update the status label."""
        self.status_label.setText(message)
        if is_error:
            self.status_label.setStyleSheet("color: red;")
        else:
            self.status_label.setStyleSheet("color: #666;")

    def set_running_state(self, camera_id: str, is_running: bool):
        """Update button states when a camera starts/stops."""
        self.start_btn.setEnabled(not is_running)
        self.stop_btn.setEnabled(is_running)
        # Update the dropdown indicator for this camera only — no full refresh
        for i in range(self.camera_combo.count()):
            if self.camera_combo.itemData(i) == camera_id:
                cam = next((c for c in self.camera_manager.get_available_cameras() 
                        if c['id'] == camera_id), None)
                if cam:
                    status = "●" if cam['is_running'] else "○"
                    label = f"{status} {cam['id']} ({cam['type']})"
                    self.camera_combo.setItemText(i, label)
                break

    # =================================================================
    # Internal Handlers
    # =================================================================

    def _on_camera_selected(self, index):
        """User selected a different camera from the dropdown."""
        if index < 0:
            return
        camera_id = self.camera_combo.itemData(index)
        self.camera_manager.set_active_camera(camera_id)
        self._update_for_camera(camera_id)
        self.camera_selected.emit(camera_id)

    def _update_for_camera(self, camera_id):
        """Show the correct config fields for the selected camera type."""
        cameras = self.camera_manager.get_available_cameras()
        cam = next((c for c in cameras if c['id'] == camera_id), None)
        if cam is None:
            return

        cam_type = cam['type']
        is_network = (cam_type == "keyence")

        self.ip_input.setVisible(is_network)
        self.port_input.setVisible(is_network)
        self.sn_input.setVisible(not is_network)
        self.camera_info.setText(f"Frame: {cam['frame_name']}")

        # Update buttons (no list refresh)
        self.start_btn.setEnabled(not cam['is_running'])
        self.stop_btn.setEnabled(cam['is_running'])

    def _on_start(self):
        """User clicked Start."""
        camera_id = self.camera_combo.currentData()
        if not camera_id:
            return

        config = {}
        cam_type = self._get_current_type()

        if cam_type == "keyence":
            ip = self.ip_input.text().strip()
            if not ip:
                self.update_status("Please enter IP address", is_error=True)
                return
            config['ip'] = ip
            config['port'] = self.port_input.value()
        else:
            sn = self.sn_input.text().strip()
            if sn:
                config['device_sn'] = sn

        self.start_requested.emit(camera_id, config)

    def _on_stop(self):
        """User clicked Stop."""
        camera_id = self.camera_combo.currentData()
        if camera_id:
            self.stop_requested.emit(camera_id)

    def _get_current_type(self):
        """Get the type of the currently selected camera."""
        cameras = self.camera_manager.get_available_cameras()
        camera_id = self.camera_combo.currentData()
        cam = next((c for c in cameras if c['id'] == camera_id), None)
        return cam['type'] if cam else None
