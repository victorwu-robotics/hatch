"""
Cartesian control panel - Pure UI.
Publishes CARTESIAN_COMMAND events. No business logic.
"""

import numpy as np
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                             QGroupBox, QGridLayout, QDoubleSpinBox, QWidget,
                             QSlider, QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QEvent

from scipy.spatial.transform import Rotation as R

from core.world_state.event_types import EventType


class CartesianControlPanel(QWidget):
    """
    Cartesian control with sliders for each axis.
    Pure UI — publishes CARTESIAN_COMMAND, subscribes to ROBOT_STATE.
    """
    
    def __init__(self, kinematic_model, state_channel, robot_manager=None,
                 transform_registry=None, asset_id=None, parent=None):
        super().__init__(parent)
        
        # Core references (for display only, not business logic)
        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager
        self.transform_registry = transform_registry
        self.asset_id = asset_id
        
        # Target pose (local to UI, published when changed)
        self.target_pose = None
        self._initialized = False
        
        # Slider ranges
        self.position_limits = (-1.5, 1.5)  # meters
        self.orientation_limits = (-np.pi, np.pi)  # radians
        
        # Wheel accumulator for smooth scrolling
        self._wheel_accumulator = {i: 0 for i in range(6)}
        
        # Debounce timer (to avoid flooding events)
        self.move_timer = QTimer()
        self.move_timer.setSingleShot(True)
        self.move_timer.timeout.connect(self._publish_command)
        
        self._setup_ui()
        self._connect_signals()
        
        # Install event filters for wheel events
        for slider in self.sliders:
            slider.installEventFilter(self)
        
        # Subscribe to robot state for display updates
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state_received)
        
        # Timer for periodic display updates (for simulation mode)
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_displays)
        self._update_timer.start(33)  # 30 Hz
    
    def _setup_ui(self):
        """Create the slider-based UI."""
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Title
        title = QLabel("Cartesian Control")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # ===== Current Pose Display =====
        current_group = QGroupBox("Current TCP (robot base frame)")
        current_layout = QGridLayout()
        current_layout.setHorizontalSpacing(20)
        
        headers = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            current_layout.addWidget(label, 0, col)
        
        self.current_values = []
        for col in range(6):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #0066cc; font-family: monospace; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(60)
            current_layout.addWidget(value, 1, col)
            self.current_values.append(value)
        
        current_group.setLayout(current_layout)
        main_layout.addWidget(current_group)
        
        # ===== Target Controls with Sliders =====
        target_group = QGroupBox("Target Pose (robot base frame)")
        target_layout = QVBoxLayout()
        
        # Axis labels row
        axis_labels = QHBoxLayout()
        axis_labels.addWidget(QLabel("Axis"))
        axis_labels.addWidget(QLabel("Slider"))
        axis_labels.addWidget(QLabel("Value"))
        axis_labels.addStretch()
        target_layout.addLayout(axis_labels)
        
        # Create scroll area for sliders
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMaximumHeight(250)
        
        slider_container = QWidget()
        slider_layout = QVBoxLayout(slider_container)
        slider_layout.setSpacing(8)
        
        # Create slider for each axis
        self.axes = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        self.sliders = []
        self.slider_labels = []
        
        for i, axis in enumerate(self.axes):
            row = QHBoxLayout()
            row.setSpacing(10)
            
            # Axis name
            axis_label = QLabel(axis)
            axis_label.setFixedWidth(40)
            axis_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row.addWidget(axis_label)
            
            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(500)
            slider.setMinimumWidth(200)
            slider.setToolTip(f"Adjust {axis} target (robot moves automatically)")
            slider.valueChanged.connect(
                lambda value, idx=i: self._on_slider_changed(idx, value)
            )
            row.addWidget(slider)
            self.sliders.append(slider)
            
            # Value label
            value_label = QLabel("0.000")
            value_label.setFixedWidth(80)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet("QLabel { color: #e67e22; font-weight: bold; }")
            row.addWidget(value_label)
            self.slider_labels.append(value_label)
            
            # Unit label
            unit = "m" if axis in ['X', 'Y', 'Z'] else "rad"
            unit_label = QLabel(unit)
            unit_label.setFixedWidth(30)
            row.addWidget(unit_label)
            
            slider_layout.addLayout(row)
        
        slider_layout.addStretch()
        scroll.setWidget(slider_container)
        target_layout.addWidget(scroll)
        
        target_group.setLayout(target_layout)
        main_layout.addWidget(target_group)

        slider.sliderReleased.connect(self._on_slider_released)

        # ===== Step Size =====
        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("Step:"))
        
        self.step_combo = QDoubleSpinBox()
        self.step_combo.setRange(0.0001, 0.1)
        self.step_combo.setValue(0.001)
        self.step_combo.setSingleStep(0.0005)
        self.step_combo.setDecimals(4)
        self.step_combo.setSuffix(" m/rad")
        step_layout.addWidget(self.step_combo)
        
        self.step_mm = QPushButton("1mm")
        self.step_mm.setCheckable(True)
        self.step_mm.setChecked(True)
        self.step_mm.clicked.connect(lambda: self.step_combo.setValue(0.001))
        step_layout.addWidget(self.step_mm)
        
        self.step_cm = QPushButton("1cm")
        self.step_cm.setCheckable(True)
        self.step_cm.clicked.connect(lambda: self.step_combo.setValue(0.01))
        step_layout.addWidget(self.step_cm)
        
        self.step_deg = QPushButton("1°")
        self.step_deg.setCheckable(True)
        self.step_deg.clicked.connect(lambda: self.step_combo.setValue(0.01745))
        step_layout.addWidget(self.step_deg)
        
        step_layout.addStretch()
        main_layout.addLayout(step_layout)
        
        # ===== Action Buttons =====
        action_layout = QHBoxLayout()
        
        self.reset_btn = QPushButton("Reset to Current")
        self.reset_btn.clicked.connect(self._reset_target)
        action_layout.addWidget(self.reset_btn)
        
        action_layout.addStretch()
        main_layout.addLayout(action_layout)
        
        # Status
        self.status_label = QLabel("Auto-move enabled (scroll or drag slider)")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        main_layout.addWidget(self.status_label)
        
        main_layout.addStretch()
    
    def _connect_signals(self):
        """Connect UI signals."""
        self.step_combo.valueChanged.connect(self._on_step_changed)
    
    # ===== Slider Methods =====
    
    def _slider_to_value(self, axis_idx, slider_value):
        """Convert slider position (0-1000) to actual value."""
        if axis_idx < 3:
            low, high = self.position_limits
        else:
            low, high = self.orientation_limits
        fraction = slider_value / 1000.0
        return low + fraction * (high - low)
    
    def _value_to_slider(self, axis_idx, value):
        """Convert actual value to slider position (0-1000)."""
        if axis_idx < 3:
            low, high = self.position_limits
        else:
            low, high = self.orientation_limits
        fraction = (value - low) / (high - low)
        fraction = np.clip(fraction, 0.0, 1.0)
        return int(fraction * 1000)
    
    def _on_slider_changed(self, axis_idx, slider_value):
        """Handle slider movement - updates target pose and schedules command."""
        if self.target_pose is None:
            return
        
        value = self._slider_to_value(axis_idx, slider_value)
        
        if axis_idx == 0:
            self.target_pose[0, 3] = value
        elif axis_idx == 1:
            self.target_pose[1, 3] = value
        elif axis_idx == 2:
            self.target_pose[2, 3] = value
        elif axis_idx >= 3:
            self._update_target_transform_from_sliders()
        
        self.slider_labels[axis_idx].setText(f"{value:.3f}")
        
        # Debounce: restart timer
        self.move_timer.start(100)
    
    def _update_target_transform_from_sliders(self):
        """Rebuild target transform from current slider positions."""
        if self.target_pose is None:
            return
        
        x = self._slider_to_value(0, self.sliders[0].value())
        y = self._slider_to_value(1, self.sliders[1].value())
        z = self._slider_to_value(2, self.sliders[2].value())
        rx = self._slider_to_value(3, self.sliders[3].value())
        ry = self._slider_to_value(4, self.sliders[4].value())
        rz = self._slider_to_value(5, self.sliders[5].value())
        
        self.target_pose = self._build_transform(x, y, z, rx, ry, rz)
    
    def _build_transform(self, x, y, z, rx, ry, rz):
        """Build 4x4 transform from position and rotation vector."""
        transform = np.eye(4)
        transform[0, 3] = x
        transform[1, 3] = y
        transform[2, 3] = z
        
        rot_vec = np.array([rx, ry, rz])
        rotation = R.from_rotvec(rot_vec)
        transform[:3, :3] = rotation.as_matrix()
        
        return transform
    
    def _update_sliders_from_target(self):
        """Update slider positions from target pose."""
        if self.target_pose is None:
            return
        
        x = self.target_pose[0, 3]
        y = self.target_pose[1, 3]
        z = self.target_pose[2, 3]
        
        rotation = R.from_matrix(self.target_pose[:3, :3])
        rot_vec = rotation.as_rotvec()
        rx, ry, rz = rot_vec[0], rot_vec[1], rot_vec[2]
        
        # Block signals to avoid recursion
        for i in range(6):
            self.sliders[i].blockSignals(True)
        
        self.sliders[0].setValue(self._value_to_slider(0, x))
        self.sliders[1].setValue(self._value_to_slider(1, y))
        self.sliders[2].setValue(self._value_to_slider(2, z))
        self.sliders[3].setValue(self._value_to_slider(3, rx))
        self.sliders[4].setValue(self._value_to_slider(4, ry))
        self.sliders[5].setValue(self._value_to_slider(5, rz))
        
        for i in range(6):
            self.sliders[i].blockSignals(False)
        
        # Update labels
        self.slider_labels[0].setText(f"{x:.3f}")
        self.slider_labels[1].setText(f"{y:.3f}")
        self.slider_labels[2].setText(f"{z:.3f}")
        self.slider_labels[3].setText(f"{rx:.3f}")
        self.slider_labels[4].setText(f"{ry:.3f}")
        self.slider_labels[5].setText(f"{rz:.3f}")
    
    def _on_slider_released(self):
        """Publish command immediately when user finishes dragging."""
        self.move_timer.stop()
        self._publish_command()

    # ===== Command Publishing =====
    
    def _publish_command(self):
        """Publish CARTESIAN_COMMAND event."""
        if self.target_pose is None:
            return
        
        print("[Cartesian] Going to publish CARTESIAN_COMMAND")
        self.state_channel.publish(
            EventType.CARTESIAN_COMMAND,
            data={'pose': self.target_pose.copy(), 'frame': 'base'},
            source="cartesian_control_panel"
        )
    
    def _reset_target(self):
        """Reset target to current TCP pose."""
        T_tcp_base = self._get_tcp_pose_in_base()
        self.target_pose = T_tcp_base.copy()
        self._update_sliders_from_target()
        self.status_label.setText("Target reset to current")
        QTimer.singleShot(1000, lambda: self.status_label.setText("Auto-move enabled"))
    
    # ===== Display Updates (from ROBOT_STATE) =====
    
    def _get_tcp_pose_in_base(self) -> np.ndarray:
        """Get TCP pose in robot base coordinates."""
        T_tcp_world = self.kinematic_model.get_tcp_pose()
        
        if self.transform_registry and self.asset_id:
            base_frame = self.transform_registry.get_asset_base_frame(self.asset_id)
            T_tcp_base = self.transform_registry.transform_pose(
                T_tcp_world,
                from_frame="world",
                to_frame=base_frame
            )
            return T_tcp_base
        return T_tcp_world
    
    def _update_displays(self):
        """Update current pose display from kinematic model."""
        try:
            T_tcp_base = self._get_tcp_pose_in_base()
            
            self.current_values[0].setText(f"{T_tcp_base[0, 3]:.3f}")
            self.current_values[1].setText(f"{T_tcp_base[1, 3]:.3f}")
            self.current_values[2].setText(f"{T_tcp_base[2, 3]:.3f}")
            
            R_mat = T_tcp_base[:3, :3]
            pitch = np.arctan2(-R_mat[2, 0], np.sqrt(R_mat[0, 0]**2 + R_mat[1, 0]**2))
            
            if np.abs(pitch - np.pi/2) < 1e-6:
                roll = 0
                yaw = np.arctan2(R_mat[0, 1], R_mat[1, 1])
            elif np.abs(pitch + np.pi/2) < 1e-6:
                roll = 0
                yaw = -np.arctan2(R_mat[0, 1], R_mat[1, 1])
            else:
                roll = np.arctan2(R_mat[2, 1], R_mat[2, 2])
                yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])
            
            self.current_values[3].setText(f"{roll:.3f}")
            self.current_values[4].setText(f"{pitch:.3f}")
            self.current_values[5].setText(f"{yaw:.3f}")
            
            if not self._initialized and self.target_pose is None:
                self.target_pose = T_tcp_base.copy()
                self._update_sliders_from_target()
                self._initialized = True
                
        except Exception as e:
            print(f"Update displays error: {e}")
    
    def _on_robot_state_received(self, event):
        """Update display from robot state."""
        positions = event.data.get('joint_positions')
        if positions is not None:
            self.kinematic_model.update_state(positions)
            self._update_displays()
    
    def _on_step_changed(self, value):
        """Update step size parameters."""
        self.step_mm.setChecked(abs(value - 0.001) < 0.0001)
        self.step_cm.setChecked(abs(value - 0.01) < 0.001)
        self.step_deg.setChecked(abs(value - 0.01745) < 0.001)
    
    # ===== Wheel Event Handling =====
    
    def eventFilter(self, obj, event):
        """Handle wheel events for smooth scrolling."""
        if event.type() == QEvent.Wheel:
            for idx, slider in enumerate(self.sliders):
                if obj == slider:
                    delta = event.angleDelta().y()
                    self._wheel_accumulator[idx] += delta
                    
                    # Get step size from UI
                    step_size = self.step_combo.value()
                    
                    if idx < 3:
                        range_min, range_max = self.position_limits
                    else:
                        range_min, range_max = self.orientation_limits
                    total_range = range_max - range_min
                    
                    step_in_slider_units = int((step_size / total_range) * 1000)
                    step_in_slider_units = max(1, step_in_slider_units)
                    
                    NOTCH_SIZE = 120
                    while abs(self._wheel_accumulator[idx]) >= NOTCH_SIZE:
                        if self._wheel_accumulator[idx] > 0:
                            new_val = min(slider.value() + step_in_slider_units, slider.maximum())
                            self._wheel_accumulator[idx] -= NOTCH_SIZE
                        else:
                            new_val = max(slider.value() - step_in_slider_units, slider.minimum())
                            self._wheel_accumulator[idx] += NOTCH_SIZE
                        slider.setValue(new_val)
                    
                    self.move_timer.start(100)
                    return True
        return super().eventFilter(obj, event)