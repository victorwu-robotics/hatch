"""
Cartesian Control Panel - Pure UI for TCP pose control.

Publishes CARTESIAN_COMMAND events when the user moves sliders.
Subscribes to ROBOT_STATE for display updates only.
Does NOT update kinematic models or transform registries directly.

Principle: UI Separate from Services. Pure presentation.
Principle: Event-Driven. Publishes events, does not call managers.
"""

import numpy as np
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QDoubleSpinBox, QWidget,
    QSlider, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, QEvent
from scipy.spatial.transform import Rotation as R

from core.world_state.event_types import EventType

import logging
logger = logging.getLogger(__name__)

class CartesianControlPanel(QWidget):
    """
    Cartesian control with sliders for each axis.

    Flow:
        1. User moves slider → publishes CARTESIAN_COMMAND
        2. CommandHandler routes to active robot
        3. Robot publishes ROBOT_STATE
        4. This panel receives ROBOT_STATE and updates display labels ONLY
    """

    def __init__(self, kinematic_model, state_channel, robot_manager=None,
                 transform_registry=None, asset_id=None, parent=None):
        super().__init__(parent)

        # References (for display queries only, not for business logic)
        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager
        self.transform_registry = transform_registry
        self.asset_id = asset_id

        # Target pose (local UI state)
        self.target_pose = None
        self._initialized = False

        # Slider ranges
        self.position_limits = (-1.5, 1.5)
        self.orientation_limits = (-np.pi, np.pi)

        # Wheel accumulator for smooth scrolling
        self._wheel_accumulator = {i: 0 for i in range(6)}

        # Debounce timer
        self._move_timer = QTimer()
        self._move_timer.setSingleShot(True)
        self._move_timer.timeout.connect(self._publish_command)

        self._setup_ui()

        # Check if IK is available for this robot
        self._ik_available = self._check_ik_available()
        if not self._ik_available:
            self._show_no_ik_message()

        # Install event filters for wheel events
        for slider in self.sliders:
            slider.installEventFilter(self)

        # Subscribe to robot state for display updates ONLY
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)
        # Subscribe to mode switched
        self.state_channel.subscribe(EventType.MODE_SWITCHED, self._on_mode_switched)

    # =================================================================
    # UI Setup
    # =================================================================

    def _setup_ui(self):
        """Create the slider-based Cartesian control UI."""
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # Title
        title = QLabel("Cartesian Control")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # IK mode indicator
        self.ik_mode_label = QLabel("IK: unknown")
        self.ik_mode_label.setAlignment(Qt.AlignCenter)
        self.ik_mode_label.setStyleSheet("QLabel { color: #888; font-size: 10px; }")
        main_layout.addWidget(self.ik_mode_label)

        # Current Pose Display
        main_layout.addWidget(self._create_current_pose_group())

        # Target Pose Sliders
        main_layout.addWidget(self._create_target_pose_group())

        # Step Size Selector
        main_layout.addLayout(self._create_step_size_selector())

        # Action Buttons
        main_layout.addLayout(self._create_action_buttons())

        # Status
        self.status_label = QLabel("Auto-move enabled (scroll or drag slider)")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        main_layout.addWidget(self.status_label)

        main_layout.addStretch()

    def _create_current_pose_group(self):
        """Create the current TCP pose display group."""
        group = QGroupBox("Current TCP (robot base frame)")
        layout = QGridLayout()
        layout.setHorizontalSpacing(20)

        headers = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label, 0, col)

        self.current_values = []
        for col in range(6):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #0066cc; font-family: monospace; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(60)
            layout.addWidget(value, 1, col)
            self.current_values.append(value)

        group.setLayout(layout)
        return group

    def _create_target_pose_group(self):
        """Create the target pose sliders group."""
        group = QGroupBox("Target Pose (robot base frame)")
        layout = QVBoxLayout()

        # Header
        axis_labels = QHBoxLayout()
        axis_labels.addWidget(QLabel("Axis"))
        axis_labels.addWidget(QLabel("Slider"))
        axis_labels.addWidget(QLabel("Value"))
        axis_labels.addStretch()
        layout.addLayout(axis_labels)

        # Scrollable slider area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMaximumHeight(250)

        slider_container = QWidget()
        slider_layout = QVBoxLayout(slider_container)
        slider_layout.setSpacing(8)

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
            slider.setToolTip(f"Adjust {axis} target")
            slider.valueChanged.connect(
                lambda value, idx=i: self._on_slider_changed(idx, value)
            )
            slider.sliderReleased.connect(self._on_slider_released)
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
        layout.addWidget(scroll)

        group.setLayout(layout)
        return group

    def _create_step_size_selector(self):
        """Create step size selection row."""
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Step:"))

        self.step_combo = QDoubleSpinBox()
        self.step_combo.setRange(0.0001, 0.1)
        self.step_combo.setValue(0.001)
        self.step_combo.setSingleStep(0.0005)
        self.step_combo.setDecimals(4)
        self.step_combo.setSuffix(" m/rad")
        self.step_combo.valueChanged.connect(self._on_step_changed)
        layout.addWidget(self.step_combo)

        self.step_mm = QPushButton("1mm")
        self.step_mm.setCheckable(True)
        self.step_mm.setChecked(True)
        self.step_mm.clicked.connect(lambda: self.step_combo.setValue(0.001))
        layout.addWidget(self.step_mm)

        self.step_cm = QPushButton("1cm")
        self.step_cm.setCheckable(True)
        self.step_cm.clicked.connect(lambda: self.step_combo.setValue(0.01))
        layout.addWidget(self.step_cm)

        self.step_deg = QPushButton("1°")
        self.step_deg.setCheckable(True)
        self.step_deg.clicked.connect(lambda: self.step_combo.setValue(0.01745))
        layout.addWidget(self.step_deg)

        layout.addStretch()
        return layout

    def _create_action_buttons(self):
        """Create action button row."""
        layout = QHBoxLayout()

        self.reset_btn = QPushButton("Reset to Current")
        self.reset_btn.clicked.connect(self._reset_target)
        layout.addWidget(self.reset_btn)

        layout.addStretch()
        return layout

    # =================================================================
    # Slider Handling
    # =================================================================

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
        """Handle slider movement — updates target pose, schedules publish."""
        if self.target_pose is None:
            return

        value = self._slider_to_value(axis_idx, slider_value)

        if axis_idx < 3:
            self.target_pose[axis_idx, 3] = value
        else:
            self._update_target_from_orientation_sliders()

        self.slider_labels[axis_idx].setText(f"{value:.3f}")

        # Debounce: restart timer
        self._move_timer.start(100)

    def _update_target_from_orientation_sliders(self):
        """Rebuild target pose rotation from current slider values."""
        if self.target_pose is None:
            return

        rx = self._slider_to_value(3, self.sliders[3].value())
        ry = self._slider_to_value(4, self.sliders[4].value())
        rz = self._slider_to_value(5, self.sliders[5].value())

        self.target_pose[:3, :3] = R.from_rotvec([rx, ry, rz]).as_matrix()

    def _on_slider_released(self):
        """Publish command immediately when user finishes dragging."""
        self._move_timer.stop()
        self._publish_command()

    def _update_sliders_from_target(self):
        """Update slider positions from target pose."""
        if self.target_pose is None:
            return

        x, y, z = self.target_pose[0, 3], self.target_pose[1, 3], self.target_pose[2, 3]
        rot_vec = R.from_matrix(self.target_pose[:3, :3]).as_rotvec()
        rx, ry, rz = rot_vec[0], rot_vec[1], rot_vec[2]

        values = [x, y, z, rx, ry, rz]

        for i in range(6):
            self.sliders[i].blockSignals(True)

        for i, val in enumerate(values):
            self.sliders[i].setValue(self._value_to_slider(i, val))

        for i in range(6):
            self.sliders[i].blockSignals(False)

        self.slider_labels[0].setText(f"{x:.3f}")
        self.slider_labels[1].setText(f"{y:.3f}")
        self.slider_labels[2].setText(f"{z:.3f}")
        self.slider_labels[3].setText(f"{rx:.3f}")
        self.slider_labels[4].setText(f"{ry:.3f}")
        self.slider_labels[5].setText(f"{rz:.3f}")

    # =================================================================
    # Command Publishing
    # =================================================================

    def _publish_command(self):
        """Publish CARTESIAN_COMMAND event via StateChannel."""
        if self.target_pose is None:
            return

        self.state_channel.publish(
            EventType.CARTESIAN_COMMAND,
            data={
                'pose': self.target_pose.copy(),
                'frame': 'base'
            },
            source="cartesian_control_panel"
        )

    def _reset_target(self):
        """Reset target sliders to current TCP pose."""
        tcp_pose = self._get_tcp_pose_in_base()
        if tcp_pose is not None:
            self.target_pose = tcp_pose.copy()
            self._update_sliders_from_target()
            self.status_label.setText("Target reset to current")
            QTimer.singleShot(1000, lambda: self.status_label.setText("Auto-move enabled"))

    # =================================================================
    # Display Updates (from ROBOT_STATE — read-only)
    # =================================================================

    def _on_robot_state(self, event):
        """
        Handle ROBOT_STATE event.

        Updates the display labels to show current TCP pose.
        Does NOT update the kinematic model — that is StateHandler's job.
        Never update sliders from state.
        """

        # Re-check IK if panels were disabled
        if hasattr(self, '_ik_available') and not self._ik_available:
            self._ik_available = self._check_ik_available()
            if self._ik_available:
                self._enable_controls()

        # Do NOT call kinematic_model.update_state() here.
        # StateHandler already updated the model.
        # We just refresh our display.
        self._update_current_display()

    def _enable_controls(self):
        """Re-enable sliders and buttons after IK becomes available."""
        for slider in self.sliders:
            slider.setEnabled(True)
        self.reset_btn.setEnabled(True)
        self.status_label.setText("Auto-move enabled (scroll or drag slider)")
        self.status_label.setStyleSheet("QLabel { color: #666; }")

    def _update_current_display(self):
        """Refresh the current pose display from the kinematic model."""
        try:
            tcp_pose = self._get_tcp_pose_in_base()
            '''
            if tcp_pose is None:
                return
            '''

            if not self._initialized and self.target_pose is None:
                self.target_pose = tcp_pose.copy()
                self._initialized = True

            # Position
            self.current_values[0].setText(f"{tcp_pose[0, 3]:.3f}")
            self.current_values[1].setText(f"{tcp_pose[1, 3]:.3f}")
            self.current_values[2].setText(f"{tcp_pose[2, 3]:.3f}")

            R_tcp_to_base = tcp_pose[:3, :3]
            R_base_to_tcp = R_tcp_to_base
            # Orientation as Rx, Ry, Rz
            rotvec = R.from_matrix(R_base_to_tcp).as_rotvec()
            self.current_values[3].setText(f"{rotvec[0]:.3f}")
            self.current_values[4].setText(f"{rotvec[1]:.3f}")
            self.current_values[5].setText(f"{rotvec[2]:.3f}")

            # Initialize target on first display update
            if not self._initialized and self.target_pose is None:
                self.target_pose = tcp_pose.copy()
                self._update_sliders_from_target()
                self._initialized = True

        except Exception as e:
            pass  # Model not ready yet — silent

    def _get_tcp_pose_in_base(self):
        """
        Get TCP pose in robot base coordinates.

        Reads from kinematic model and transform registry.
        Pure query — no mutation.
        """
        if self.kinematic_model is None:
            logger.debug("[CARTESIAN] _get_tcp_pose_in_base: kinematic_model is None")
            return None

        try:
            tcp_world = self.kinematic_model.get_tcp_pose()

            if self.robot_manager and self.asset_id:
                base_frame = self.robot_manager.get_asset_base_frame(self.asset_id)
                result = self.transform_registry.transform_frame_pose(
                    tcp_world,
                    from_frame="world",
                    to_frame=base_frame
                )
                logger.debug(f"[CARTESIAN] _get_tcp_pose_in_base: success, pos=({result[0,3]:.4f}, {result[1,3]:.4f}, {result[2,3]:.4f})")
                return result
            return tcp_world
        except Exception as e:
            logger.debug(f"[CARTESIAN] _get_tcp_pose_in_base: exception {e}")
            return None

    @staticmethod
    def _matrix_to_rpy(R_mat):
        """Convert 3x3 rotation matrix to roll, pitch, yaw (radians)."""
        pitch = np.arctan2(-R_mat[2, 0], np.sqrt(R_mat[0, 0]**2 + R_mat[1, 0]**2))

        if np.abs(pitch - np.pi/2) < 1e-6:
            roll = 0.0
            yaw = np.arctan2(R_mat[0, 1], R_mat[1, 1])
        elif np.abs(pitch + np.pi/2) < 1e-6:
            roll = 0.0
            yaw = -np.arctan2(R_mat[0, 1], R_mat[1, 1])
        else:
            roll = np.arctan2(R_mat[2, 1], R_mat[2, 2])
            yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])

        return roll, pitch, yaw

    # =================================================================
    # Step Size
    # =================================================================

    def _on_step_changed(self, value):
        """Update step size preset buttons."""
        self.step_mm.setChecked(abs(value - 0.001) < 0.0001)
        self.step_cm.setChecked(abs(value - 0.01) < 0.001)
        self.step_deg.setChecked(abs(value - 0.01745) < 0.001)

    # ==================================================================

    def _on_mode_switched(self, event):
        mode = event.data.get('mode', '')
        
        # Update IK mode display
        mode_display = {
            'simulate_local': 'IK: Local (simulated)',
            'simulate_real_ik': 'IK: Real robot solver',
            'real': 'IK: Real robot solver',
        }
        self.ik_mode_label.setText(mode_display.get(mode, f'IK: {mode}'))
        
        # Sync sliders when switching to Real mode
        if mode == "real" and self.robot_manager and self.robot_manager._real_robot:
            state = self.robot_manager._real_robot.get_state()
            if state:
                tcp_pose = self._get_tcp_pose_in_base()
                if tcp_pose is not None:
                    self.target_pose = tcp_pose.copy()
                    self._update_sliders_from_target()
                    self._initialized = True

    def _check_ik_available(self) -> bool:
        """Check if the kinematic model has a working IK solver."""
        if self.kinematic_model is None:
            return False
        try:
            # Try a simple IK call at current position
            current_pose = self.kinematic_model.get_tcp_pose()
            logger.debug(f"[CARTESIAN] current pose: {current_pose}")
            result = self.kinematic_model.solve_ik_for_tcp(
                current_pose, 
                self.kinematic_model.get_current_joint_positions()
            )
            logger.debug(f"[CARTESIAN] result: {result}")
            return result is not None
        except Exception:
            return False

    def _show_no_ik_message(self):
        """Show a message that IK is not available for this robot."""
        # Replace the slider area with a message
        for slider in self.sliders:
            slider.setEnabled(False)
        
        self.status_label.setText(
            "Inverse kinematics not available for this robot.\n"
            "Joint control is still available."
        )
        self.status_label.setStyleSheet(
            "QLabel { color: #e67e22; font-weight: bold; padding: 20px; }"
        )
        self.reset_btn.setEnabled(False)

    # =================================================================
    # Wheel Event Handling
    # =================================================================

    def eventFilter(self, obj, event):
        """Handle wheel events for smooth scrolling on sliders."""
        if event.type() == QEvent.Wheel:
            for idx, slider in enumerate(self.sliders):
                if obj == slider:
                    delta = event.angleDelta().y()
                    self._wheel_accumulator[idx] += delta

                    step_size = self.step_combo.value()

                    if idx < 3:
                        range_min, range_max = self.position_limits
                    else:
                        range_min, range_max = self.orientation_limits
                    total_range = range_max - range_min

                    step_in_units = max(1, int((step_size / total_range) * 1000))
                    notch = 120

                    while abs(self._wheel_accumulator[idx]) >= notch:
                        if self._wheel_accumulator[idx] > 0:
                            new_val = min(slider.value() + step_in_units, slider.maximum())
                            self._wheel_accumulator[idx] -= notch
                        else:
                            new_val = max(slider.value() - step_in_units, slider.minimum())
                            self._wheel_accumulator[idx] += notch
                        slider.setValue(new_val)

                    self._move_timer.start(100)
                    return True
        return super().eventFilter(obj, event)

    # =================================================================
    # Public Methods
    # =================================================================

    def setEnabled(self, enabled: bool):
        """Enable or disable all controls."""
        super().setEnabled(enabled)
        for slider in self.sliders:
            slider.setEnabled(enabled)
        if hasattr(self, 'reset_btn'):
            self.reset_btn.setEnabled(enabled)

    def cleanup(self):
        """Unsubscribe from events before destruction."""
        self.state_channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self._display_timer.stop()