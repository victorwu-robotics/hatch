"""
Camera Control Panel for multiple depth cameras (Orbbec, RealSense, etc.)
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QComboBox,
                             QDoubleSpinBox, QGroupBox, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal


class CameraControlPanel(QWidget):
    """Control panel for multiple camera types with frame visualization"""
    
    # Define signals
    start_streaming = pyqtSignal()
    stop_streaming = pyqtSignal()
    transform_toggled = pyqtSignal(bool)
    show_frames_toggled = pyqtSignal(bool)
    camera_type_changed = pyqtSignal(str)  # New signal for camera switching
    resolution_changed = pyqtSignal(str)  # ← ADD THIS
    min_x_changed = pyqtSignal(float)          # X min
    max_x_changed = pyqtSignal(float)          # X max
    min_y_changed = pyqtSignal(float)          # Y min
    max_y_changed = pyqtSignal(float)          # Y max
    min_z_changed = pyqtSignal(float)          # Z min
    max_z_changed = pyqtSignal(float)          # Z max

    def __init__(self, camera_manager, parent=None):
        super().__init__(parent)
        self.camera_manager = camera_manager
        self.setup_ui()
        self.connect_signals()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # ===== CAMERA SELECTION GROUP =====
        selection_group = QGroupBox("Camera Selection")
        selection_layout = QVBoxLayout()
        
        # Camera type dropdown
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("Camera Type:"))
        self.camera_type_combo = QComboBox()
        
        # Populate from camera manager
        for cam_type, cam_name in self.camera_manager.available_cameras.items():
            self.camera_type_combo.addItem(cam_name, cam_type)
        
        type_layout.addWidget(self.camera_type_combo)
        selection_layout.addLayout(type_layout)
        
        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)
        
        # ===== CAMERA CONTROLS GROUP =====
        control_group = QGroupBox("Camera Controls")
        control_layout = QVBoxLayout()
        
        # Resolution selector (will be updated when camera changes)
        res_layout = QHBoxLayout()
        res_layout.addWidget(QLabel("Resolution:"))
        self.resolution_combo = QComboBox()
        self._update_resolutions()  # Populate based on current camera
        res_layout.addWidget(self.resolution_combo)
        control_layout.addLayout(res_layout)
        
        # ===== 3D ROI CONTROLS =====
        roi_group = QGroupBox("Region of Interest (ROI)")
        roi_layout = QVBoxLayout()
        
        # X Range
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X Range:"))
        self.min_x_spin = QDoubleSpinBox()
        self.min_x_spin.setRange(-5.0, 5.0)
        self.min_x_spin.setValue(-2.0)
        self.min_x_spin.setSingleStep(0.1)
        self.min_x_spin.setDecimals(1)
        self.min_x_spin.setPrefix("min: ")
        self.min_x_spin.setSuffix(" m")
        x_layout.addWidget(self.min_x_spin)
        
        self.max_x_spin = QDoubleSpinBox()
        self.max_x_spin.setRange(-5.0, 5.0)
        self.max_x_spin.setValue(2.0)
        self.max_x_spin.setSingleStep(0.1)
        self.max_x_spin.setDecimals(1)
        self.max_x_spin.setPrefix("max: ")
        self.max_x_spin.setSuffix(" m")
        x_layout.addWidget(self.max_x_spin)
        roi_layout.addLayout(x_layout)
        
        # Y Range
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y Range:"))
        self.min_y_spin = QDoubleSpinBox()
        self.min_y_spin.setRange(-5.0, 5.0)
        self.min_y_spin.setValue(-2.0)
        self.min_y_spin.setSingleStep(0.1)
        self.min_y_spin.setDecimals(1)
        self.min_y_spin.setPrefix("min: ")
        self.min_y_spin.setSuffix(" m")
        y_layout.addWidget(self.min_y_spin)
        
        self.max_y_spin = QDoubleSpinBox()
        self.max_y_spin.setRange(-5.0, 5.0)
        self.max_y_spin.setValue(2.0)
        self.max_y_spin.setSingleStep(0.1)
        self.max_y_spin.setDecimals(1)
        self.max_y_spin.setPrefix("max: ")
        self.max_y_spin.setSuffix(" m")
        y_layout.addWidget(self.max_y_spin)
        roi_layout.addLayout(y_layout)
        
        # Z Range (existing, but enhanced)
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z Range:"))
        self.min_z_spin = QDoubleSpinBox()  # Add min Z
        self.min_z_spin.setRange(0.1, 10.0)
        self.min_z_spin.setValue(0.1)
        self.min_z_spin.setSingleStep(0.1)
        self.min_z_spin.setDecimals(1)
        self.min_z_spin.setPrefix("min: ")
        self.min_z_spin.setSuffix(" m")
        z_layout.addWidget(self.min_z_spin)
        
        self.max_z_spin = QDoubleSpinBox()  # Rename from max_depth_spin
        self.max_z_spin.setRange(0.5, 10.0)
        self.max_z_spin.setValue(2.0)
        self.max_z_spin.setSingleStep(0.1)
        self.max_z_spin.setDecimals(1)
        self.max_z_spin.setPrefix("max: ")
        self.max_z_spin.setSuffix(" m")
        z_layout.addWidget(self.max_z_spin)
        roi_layout.addLayout(z_layout)
        
        # Reset button
        self.reset_roi_btn = QPushButton("Reset ROI")
        roi_layout.addWidget(self.reset_roi_btn)
        
        roi_group.setLayout(roi_layout)
        control_layout.addWidget(roi_group)

        # Start/Stop buttons
        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Camera")
        self.stop_btn = QPushButton("Stop Camera")
        self.stop_btn.setEnabled(False)
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        control_layout.addLayout(button_layout)
        
        # Status
        self.status_label = QLabel("Camera stopped")
        self.status_label.setAlignment(Qt.AlignCenter)
        control_layout.addWidget(self.status_label)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # ===== FRAME VISUALIZATION GROUP =====
        frame_group = QGroupBox("Coordinate Frames")
        frame_layout = QVBoxLayout()
        
        # Show frames toggle
        self.show_frames_check = QCheckBox("Show Camera Frames")
        self.show_frames_check.setChecked(True)
        frame_layout.addWidget(self.show_frames_check)
        
        # Transform to world toggle
        self.transform_check = QCheckBox("Transform to World Frame")
        self.transform_check.setChecked(True)
        frame_layout.addWidget(self.transform_check)
        
        # Info label about frames
        frame_info = QLabel(
            "🟡 Body frame: RGB axes\n"
            "🟢 Optical frame: Cyan axes (Z forward)"
        )
        frame_info.setStyleSheet("color: gray; font-size: 10px;")
        frame_layout.addWidget(frame_info)
        
        frame_group.setLayout(frame_layout)
        layout.addWidget(frame_group)
        
        # ===== VISIBILITY =====
        self.visible_check = QCheckBox("Show Point Cloud")
        self.visible_check.setChecked(True)
        layout.addWidget(self.visible_check)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def connect_signals(self):
        """Connect all UI signals"""
        # Camera type selection
        self.camera_type_combo.currentIndexChanged.connect(self._on_camera_type_changed)

        # ← ADD THIS: Resolution changes
        self.resolution_combo.currentTextChanged.connect(self.resolution_changed.emit)

        # Start/Stop buttons
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self._on_stop)

        # ROI signals
        self.min_x_spin.valueChanged.connect(self.min_x_changed.emit)
        self.max_x_spin.valueChanged.connect(self.max_x_changed.emit)
        self.min_y_spin.valueChanged.connect(self.min_y_changed.emit)
        self.max_y_spin.valueChanged.connect(self.max_y_changed.emit)
        self.min_z_spin.valueChanged.connect(self.min_z_changed.emit)
        self.max_z_spin.valueChanged.connect(self.max_z_changed.emit)

        # Toggles
        self.show_frames_check.toggled.connect(self.show_frames_toggled.emit)
        self.transform_check.toggled.connect(self.transform_toggled.emit)

        # Reset button
        self.reset_roi_btn.clicked.connect(self._reset_roi)

    def _reset_roi(self):
        """Reset ROI to default values"""
        self.min_x_spin.setValue(-2.0)
        self.max_x_spin.setValue(2.0)
        self.min_y_spin.setValue(-2.0)
        self.max_y_spin.setValue(2.0)
        self.min_z_spin.setValue(0.1)
        self.max_z_spin.setValue(2.0)

    def _on_camera_type_changed(self, index):
        """Handle camera type selection change"""
        camera_type = self.camera_type_combo.currentData()
        self.camera_type_changed.emit(camera_type)
        
        # Update resolution list for new camera type
        self._update_resolutions()
    
    def _update_resolutions(self):
        """Update resolution combo box based on current camera"""
        self.resolution_combo.clear()
        resolutions = self.camera_manager.get_resolutions_for_current()
        self.resolution_combo.addItems(resolutions)
    
    def _on_start(self):
        """Handle start button click"""
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status_label.setText("Starting camera...")
        self.start_streaming.emit()
    
    def _on_stop(self):
        """Handle stop button click"""
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Camera stopped")
        self.stop_streaming.emit()
    
    def update_status(self, status: str, is_error: bool = False):
        """Update status label"""
        self.status_label.setText(status)
        if is_error:
            self.status_label.setStyleSheet("color: red")
        else:
            self.status_label.setStyleSheet("color: black")
    
    def set_camera_type(self, camera_type: str):
        """Set the camera type in the combo box"""
        index = self.camera_type_combo.findData(camera_type)
        if index >= 0:
            self.camera_type_combo.setCurrentIndex(index)

    def update_resolutions(self, resolutions):
        """Public method to update resolution dropdown based on camera type."""
        self.resolution_combo.clear()
        self.resolution_combo.addItems(resolutions)
