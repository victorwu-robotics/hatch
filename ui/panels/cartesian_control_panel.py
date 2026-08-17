"""
Cartesian Control Panel - Pure UI for TCP pose control.

Publishes CARTESIAN_COMMAND events when the user moves sliders or clicks buttons.
Subscribes to ROBOT_STATE for display updates only.
Does NOT update kinematic models or transform registries directly.

Principle: UI Separate from Services. Pure presentation.
Principle: Event-Driven. Publishes events, does not call managers.

Orientation Control:
    - Base frame: Absolute Euler angles (extrinsic XYZ)
    - Tool frame: Incremental rotations around current TCP axes
    - Moving a slider rotates around that single axis only
    - Position remains unchanged
"""

import numpy as np
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QDoubleSpinBox, QWidget,
    QSlider, QScrollArea, QComboBox, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QTimer, QEvent
from scipy.spatial.transform import Rotation as R

from core.world_state.event_types import EventType

import logging
logger = logging.getLogger(__name__)


class CartesianControlPanel(QWidget):
    """
    Cartesian control with sliders and step buttons for each axis.

    Linear axes (X, Y, Z): always in Base frame, in meters.
    Angular axes (RX, RY, RZ): 
        - Base frame: absolute Euler angles (extrinsic XYZ)
        - Tool frame: incremental rotations around current TCP axes
    """

    def __init__(self, kinematic_model, state_channel, robot_manager=None,
                 transform_registry=None, asset_id=None, parent=None):
        super().__init__(parent)

        # References
        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager
        self.transform_registry = transform_registry
        self.asset_id = asset_id

        # Target pose
        self.target_pose = None
        self._initialized = False

        # Slider value tracking for Tool-frame delta calculations
        self._last_orient_slider_values = [500, 500, 500]

        # Slider ranges
        self.position_limits = (-1.5, 1.5)
        self.orientation_limits = (-np.pi, np.pi)

        # Wheel accumulator
        self._wheel_accumulator = {}

        # Debounce timer
        self._move_timer = QTimer()
        self._move_timer.setSingleShot(True)
        self._move_timer.timeout.connect(self._publish_command)

        self._setup_ui()

        # Check IK availability
        self._ik_available = self._check_ik_available()
        if not self._ik_available:
            self._show_no_ik_message()

        # Install event filters
        for slider in self.position_sliders:
            slider.installEventFilter(self)
        for slider in self.orient_sliders:
            slider.installEventFilter(self)

        # Subscribe to events
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self.state_channel.subscribe(EventType.MODE_SWITCHED, self._on_mode_switched)
        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)

    # =================================================================
    # UI Setup (unchanged from previous version)
    # =================================================================

    def _setup_ui(self):
        """Create the Cartesian control UI."""
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

        # Step Size Controls
        main_layout.addLayout(self._create_step_controls())

        # Action Buttons
        main_layout.addLayout(self._create_action_buttons())

        # Status
        self.status_label = QLabel("Auto-move enabled (scroll, drag, or click buttons)")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        main_layout.addWidget(self.status_label)

        main_layout.addStretch()

    def _create_current_pose_group(self):
        """Create the current TCP pose display group."""
        group = QGroupBox("Current TCP")
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
        """Create the target pose sliders group with [-] and [+] buttons."""
        group = QGroupBox("Target Pose")
        layout = QVBoxLayout()
        layout.setSpacing(8)

        # ===== Position Section =====
        pos_header = QLabel("Position (Base Frame)")
        pos_header.setStyleSheet("font-weight: bold; color: #555;")
        layout.addWidget(pos_header)

        self.position_sliders = []
        self.position_labels = []
        self.position_minus_buttons = []
        self.position_plus_buttons = []

        pos_axes = ['X', 'Y', 'Z']

        for i in range(3):
            row = QHBoxLayout()
            row.setSpacing(6)

            minus_btn = QPushButton("-")
            minus_btn.setFixedWidth(30)
            minus_btn.setFixedHeight(24)
            minus_btn.setToolTip(f"Decrease {pos_axes[i]} by step size")
            minus_btn.clicked.connect(lambda checked, idx=i: self._on_position_step(idx, -1))
            row.addWidget(minus_btn)
            self.position_minus_buttons.append(minus_btn)

            axis_label = QLabel(pos_axes[i])
            axis_label.setFixedWidth(35)
            axis_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
            row.addWidget(axis_label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(500)
            slider.setMinimumWidth(180)
            slider.valueChanged.connect(
                lambda value, idx=i: self._on_position_slider_changed(idx, value)
            )
            slider.sliderReleased.connect(self._on_slider_released)
            row.addWidget(slider, 6)
            self.position_sliders.append(slider)

            plus_btn = QPushButton("+")
            plus_btn.setFixedWidth(30)
            plus_btn.setFixedHeight(24)
            plus_btn.setToolTip(f"Increase {pos_axes[i]} by step size")
            plus_btn.clicked.connect(lambda checked, idx=i: self._on_position_step(idx, +1))
            row.addWidget(plus_btn)
            self.position_plus_buttons.append(plus_btn)

            value_label = QLabel("0.000 m")
            value_label.setFixedWidth(80)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet("QLabel { color: #e67e22; font-weight: bold; font-family: monospace; }")
            row.addWidget(value_label, 3)
            self.position_labels.append(value_label)

            layout.addLayout(row)

        # ===== Orientation Section =====
        orient_header_row = QHBoxLayout()
        orient_header = QLabel("Orientation")
        orient_header.setStyleSheet("font-weight: bold; color: #555;")
        orient_header_row.addWidget(orient_header)

        orient_header_row.addStretch()

        frame_label = QLabel("Frame:")
        orient_header_row.addWidget(frame_label)

        self.frame_selector = QComboBox()
        self.frame_selector.addItems(["Base", "Tool"])
        self.frame_selector.setToolTip(
            "Base: Absolute Euler angles around fixed base axes\n"
            "Tool: Incremental rotations around current TCP axes"
        )
        self.frame_selector.currentIndexChanged.connect(self._on_frame_changed)
        orient_header_row.addWidget(self.frame_selector)

        layout.addLayout(orient_header_row)

        # Tool frame hint label
        self.tool_hint_label = QLabel("")
        self.tool_hint_label.setStyleSheet("QLabel { color: #888; font-size: 10px; }")
        self.tool_hint_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.tool_hint_label)

        self.orient_sliders = []
        self.orient_labels = []
        self.orient_minus_buttons = []
        self.orient_plus_buttons = []

        rot_axes = ['RX', 'RY', 'RZ']

        for i in range(3):
            row = QHBoxLayout()
            row.setSpacing(6)

            minus_btn = QPushButton("-")
            minus_btn.setFixedWidth(30)
            minus_btn.setFixedHeight(24)
            minus_btn.setToolTip(f"Decrease {rot_axes[i]} by step size")
            minus_btn.clicked.connect(lambda checked, idx=i: self._on_orientation_step(idx, -1))
            row.addWidget(minus_btn)
            self.orient_minus_buttons.append(minus_btn)

            axis_label = QLabel(rot_axes[i])
            axis_label.setFixedWidth(35)
            axis_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
            row.addWidget(axis_label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(500)
            slider.setMinimumWidth(180)
            slider.valueChanged.connect(
                lambda value, idx=i: self._on_orientation_slider_changed(idx, value)
            )
            slider.sliderReleased.connect(self._on_slider_released)
            row.addWidget(slider, 6)
            self.orient_sliders.append(slider)

            plus_btn = QPushButton("+")
            plus_btn.setFixedWidth(30)
            plus_btn.setFixedHeight(24)
            plus_btn.setToolTip(f"Increase {rot_axes[i]} by step size")
            plus_btn.clicked.connect(lambda checked, idx=i: self._on_orientation_step(idx, +1))
            row.addWidget(plus_btn)
            self.orient_plus_buttons.append(plus_btn)

            value_label = QLabel("0.000 rad")
            value_label.setFixedWidth(80)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet("QLabel { color: #e67e22; font-weight: bold; font-family: monospace; }")
            row.addWidget(value_label, 3)
            self.orient_labels.append(value_label)

            layout.addLayout(row)

        group.setLayout(layout)
        return group

    def _create_step_controls(self):
        """Create linear and angular step size controls with unit toggle."""
        layout = QVBoxLayout()
        layout.setSpacing(6)

        linear_row = QHBoxLayout()
        linear_row.addWidget(QLabel("Linear Step:"))
        self.linear_step_combo = QComboBox()
        self.linear_step_combo.addItems(["0.0001", "0.001", "0.01", "0.1"])
        self.linear_step_combo.setCurrentText("0.001")
        linear_row.addWidget(self.linear_step_combo)
        linear_row.addWidget(QLabel("m"))
        linear_row.addStretch()
        layout.addLayout(linear_row)

        angular_row = QHBoxLayout()
        angular_row.addWidget(QLabel("Angular Step:"))
        self.angular_step_combo = QComboBox()
        self.angular_step_combo.addItems(["0.001", "0.01", "0.1", "1"])
        self.angular_step_combo.setCurrentText("0.1")
        angular_row.addWidget(self.angular_step_combo)

        self.degrees_radio = QRadioButton("degrees")
        self.degrees_radio.setChecked(True)
        self.radians_radio = QRadioButton("radians")
        self.degrees_radio.toggled.connect(self._on_unit_changed)
        self.radians_radio.toggled.connect(self._on_unit_changed)

        angular_row.addWidget(self.degrees_radio)
        angular_row.addWidget(self.radians_radio)
        angular_row.addStretch()
        layout.addLayout(angular_row)

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
    # Unit Conversion & Display
    # =================================================================

    def _is_degrees(self):
        return self.degrees_radio.isChecked()

    def _get_position_step(self):
        return float(self.linear_step_combo.currentText())

    def _get_angular_step(self):
        step = float(self.angular_step_combo.currentText())
        if self._is_degrees():
            return np.radians(step)
        return step

    def _display_angle(self, angle_rad):
        if self._is_degrees():
            return f"{np.degrees(angle_rad):.1f}°"
        return f"{angle_rad:.3f} rad"

    # =================================================================
    # Slider Value Conversion
    # =================================================================

    def _position_slider_to_value(self, slider_value):
        low, high = self.position_limits
        fraction = slider_value / 1000.0
        return low + fraction * (high - low)

    def _position_value_to_slider(self, value):
        low, high = self.position_limits
        fraction = (value - low) / (high - low)
        fraction = np.clip(fraction, 0.0, 1.0)
        return int(fraction * 1000)

    def _angle_slider_to_value(self, slider_value):
        """Convert slider position to angle. For Tool frame, center (500) = 0."""
        low, high = self.orientation_limits
        fraction = slider_value / 1000.0
        return low + fraction * (high - low)

    def _angle_value_to_slider(self, value_rad):
        low, high = self.orientation_limits
        fraction = (value_rad - low) / (high - low)
        fraction = np.clip(fraction, 0.0, 1.0)
        return int(fraction * 1000)

    # =================================================================
    # Position Slider Handling (unchanged)
    # =================================================================

    def _on_position_slider_changed(self, axis_idx, slider_value):
        if self.target_pose is None:
            return

        value = self._position_slider_to_value(slider_value)
        self.target_pose[axis_idx, 3] = value
        self.position_labels[axis_idx].setText(f"{value:.3f} m")
        self._move_timer.start(100)

    def _on_position_step(self, axis_idx, direction):
        if self.target_pose is None:
            return

        step = self._get_position_step()
        current_val = self.target_pose[axis_idx, 3]
        new_val = current_val + direction * step
        low, high = self.position_limits
        self.target_pose[axis_idx, 3] = np.clip(new_val, low, high)

        self._update_position_slider(axis_idx)
        self._publish_command()

    def _update_position_slider(self, axis_idx):
        value = self.target_pose[axis_idx, 3]
        self.position_sliders[axis_idx].blockSignals(True)
        self.position_sliders[axis_idx].setValue(self._position_value_to_slider(value))
        self.position_sliders[axis_idx].blockSignals(False)
        self.position_labels[axis_idx].setText(f"{value:.3f} m")

    # =================================================================
    # Frame Selection
    # =================================================================

    def _get_selected_frame(self):
        return self.frame_selector.currentText().lower()

    # =================================================================
    # Orientation — Base Frame (Absolute Euler Angles)
    # =================================================================

    def _extract_euler_angles_base(self):
        """Extract Euler angles (extrinsic XYZ) from target pose."""
        if self.target_pose is None:
            return np.zeros(3)
        R_matrix = self.target_pose[:3, :3]
        return R.from_matrix(R_matrix).as_euler('xyz')

    def _apply_euler_angles_base(self, rx, ry, rz):
        """Apply Euler angles (extrinsic XYZ) to target pose. Position unchanged."""
        if self.target_pose is None:
            return
        R_new = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
        self.target_pose[:3, :3] = R_new

    # =================================================================
    # Orientation — Tool Frame (Incremental Rotations)
    # =================================================================

    def _apply_tool_incremental_rotation(self, axis_idx, angle_rad):
        if self.target_pose is None:
            return

        logger.debug(f"[Tool] Applying {angle_rad:.4f} rad around axis {axis_idx}")

        if axis_idx == 0:
            R_inc = R.from_euler('x', angle_rad).as_matrix()
        elif axis_idx == 1:
            R_inc = R.from_euler('y', angle_rad).as_matrix()
        else:
            R_inc = R.from_euler('z', angle_rad).as_matrix()

        self.target_pose[:3, :3] = self.target_pose[:3, :3] @ R_inc

    # =================================================================
    # Orientation Slider Handling
    # =================================================================

    def _on_orientation_slider_changed(self, axis_idx, slider_value):
        """Handle orientation slider movement."""
        if self.target_pose is None:
            return

        frame = self._get_selected_frame()

        if frame == 'base':
            # Absolute Euler angle
            new_angle = self._angle_slider_to_value(slider_value)
            euler = self._extract_euler_angles_base()
            euler[axis_idx] = new_angle
            self._apply_euler_angles_base(euler[0], euler[1], euler[2])
            self.orient_labels[axis_idx].setText(self._display_angle(new_angle))
        else:
            # Tool frame: incremental rotation using delta from last value
            prev_value = self._last_orient_slider_values[axis_idx]
            prev_angle = self._angle_slider_to_value(prev_value)
            curr_angle = self._angle_slider_to_value(slider_value)
            delta_angle = curr_angle - prev_angle

            logger.debug(
                f"[Tool] axis={axis_idx}, slider={slider_value}, "
                f"prev={prev_value}, delta={delta_angle:.4f}"
            )

            # Apply only the delta since the last event
            self._apply_tool_incremental_rotation(axis_idx, delta_angle)

            # Update tracking
            self._last_orient_slider_values[axis_idx] = slider_value

            # Label shows total displacement from center for user feedback
            self.orient_labels[axis_idx].setText(self._display_angle(curr_angle))

        self._move_timer.start(100)

    def _on_orientation_step(self, axis_idx, direction):
        """
        Handle [-] or [+] button click for orientation axis.
        
        Base frame: change absolute Euler angle by step
        Tool frame: apply incremental rotation by step
        """
        if self.target_pose is None:
            return

        step = self._get_angular_step()
        frame = self._get_selected_frame()

        if frame == 'base':
            euler = self._extract_euler_angles_base()
            new_angle = euler[axis_idx] + direction * step
            low, high = self.orientation_limits
            euler[axis_idx] = np.clip(new_angle, low, high)
            self._apply_euler_angles_base(euler[0], euler[1], euler[2])
            self._update_orientation_slider(axis_idx)
        else:
            # Tool frame: apply incremental rotation
            incremental = direction * step
            self._apply_tool_incremental_rotation(axis_idx, incremental)
            # Update label to show incremental angle (momentary)
            self.orient_labels[axis_idx].setText(self._display_angle(incremental))
            # Schedule label reset
            QTimer.singleShot(500, lambda idx=axis_idx: self._reset_tool_label(idx))

        self._publish_command()

    def _reset_tool_label(self, axis_idx):
        """Reset Tool frame label to 0 after showing incremental angle."""
        if self._get_selected_frame() == 'tool':
            self.orient_labels[axis_idx].setText(self._display_angle(0.0))

    # =================================================================
    # Slider Released — Spring-back for Tool Frame
    # =================================================================

    def _on_slider_released(self):
        """Publish command and spring back orientation sliders if in Tool frame."""
        self._move_timer.stop()
        self._publish_command()

        # Spring back orientation sliders in Tool frame
        if self._get_selected_frame() == 'tool':
            self._last_orient_slider_values = [500, 500, 500]
            for i in range(3):
                self.orient_sliders[i].blockSignals(True)
                self.orient_sliders[i].setValue(500)  # Center
                self.orient_sliders[i].blockSignals(False)
                self.orient_labels[i].setText(self._display_angle(0.0))

    # =================================================================
    # Frame Selector Handling
    # =================================================================

    def _on_frame_changed(self):
        """Handle frame selector change."""
        if self.target_pose is None:
            return

        frame = self._get_selected_frame()

        if frame == 'base':
            # Show hint
            self.tool_hint_label.setText("")
            # Update orientation sliders to show absolute Euler angles
            self._update_all_orientation_sliders()
        else:
            # Tool frame: show hint
            self.tool_hint_label.setText(
                "Tool frame: sliders are incremental (spring back to center after release)"
            )
            # Reset slider tracking
            self._last_orient_slider_values = [500, 500, 500]
            # Reset sliders to center
            for i in range(3):
                self.orient_sliders[i].blockSignals(True)
                self.orient_sliders[i].setValue(500)
                self.orient_sliders[i].blockSignals(False)
                self.orient_labels[i].setText(self._display_angle(0.0))

        logger.debug(f"[Cartesian] Frame changed to: {frame}")

    # =================================================================
    # Slider Update Helpers
    # =================================================================

    def _update_orientation_slider(self, axis_idx):
        """Update orientation slider and label (Base frame only)."""
        if self._get_selected_frame() != 'base':
            return

        euler = self._extract_euler_angles_base()
        angle = euler[axis_idx]

        self.orient_sliders[axis_idx].blockSignals(True)
        self.orient_sliders[axis_idx].setValue(self._angle_value_to_slider(angle))
        self.orient_sliders[axis_idx].blockSignals(False)
        self.orient_labels[axis_idx].setText(self._display_angle(angle))

    def _update_all_orientation_sliders(self):
        """Update all orientation sliders (Base frame only)."""
        if self.target_pose is None:
            return

        if self._get_selected_frame() != 'base':
            return

        euler = self._extract_euler_angles_base()

        for i in range(3):
            self.orient_sliders[i].blockSignals(True)
            self.orient_sliders[i].setValue(self._angle_value_to_slider(euler[i]))
            self.orient_sliders[i].blockSignals(False)
            self.orient_labels[i].setText(self._display_angle(euler[i]))

    # =================================================================
    # Unit Changed
    # =================================================================

    def _on_unit_changed(self):
        """Handle degrees/radians toggle."""
        if self.target_pose is not None:
            if self._get_selected_frame() == 'base':
                euler = self._extract_euler_angles_base()
                for i in range(3):
                    self.orient_labels[i].setText(self._display_angle(euler[i]))
            else:
                for i in range(3):
                    self.orient_labels[i].setText(self._display_angle(0.0))

        self._update_current_display()

    # =================================================================
    # Command Publishing
    # =================================================================

    def _publish_command(self):
        """Publish CARTESIAN_COMMAND event."""
        if self.target_pose is None:
            return

        self.state_channel.publish(
            EventType.CARTESIAN_COMMAND,
            data={
                'pose': self.target_pose.copy(),
                'frame': self._get_selected_frame()
            },
            source="cartesian_control_panel"
        )

    # =================================================================
    # Reset Target
    # =================================================================

    def _reset_target(self):
        """Reset target sliders to current TCP pose."""
        tcp_pose = self._get_tcp_pose_in_base()
        if tcp_pose is not None:
            self.target_pose = tcp_pose.copy()
            self._update_all_sliders_from_target()
            self.status_label.setText("Target reset to current")
            QTimer.singleShot(1000, lambda: self.status_label.setText(
                "Auto-move enabled (scroll, drag, or click buttons)"))

    def _update_all_sliders_from_target(self):
        """Update all sliders and labels from target pose."""
        if self.target_pose is None:
            return

        # Position sliders
        for i in range(3):
            self._update_position_slider(i)

        # Orientation sliders
        if self._get_selected_frame() == 'base':
            self._update_all_orientation_sliders()
        else:
            # Tool frame: reset to center
            self._last_orient_slider_values = [500, 500, 500]
            for i in range(3):
                self.orient_sliders[i].blockSignals(True)
                self.orient_sliders[i].setValue(500)
                self.orient_sliders[i].blockSignals(False)
                self.orient_labels[i].setText(self._display_angle(0.0))

    # =================================================================
    # Display Updates (from ROBOT_STATE)
    # =================================================================

    def _on_robot_state(self, event):
        """Handle ROBOT_STATE event."""
        if hasattr(self, '_ik_available') and not self._ik_available:
            self._ik_available = self._check_ik_available()
            if self._ik_available:
                self._enable_controls()

        self._update_current_display()

    def _on_robot_loaded(self, event):
        """Handle robot loaded event."""
        self._ik_available = self._check_ik_available()
        if self._ik_available:
            self._enable_controls()
        else:
            self._show_no_ik_message()

        tcp_pose = self._get_tcp_pose_in_base()
        if tcp_pose is not None:
            self.target_pose = tcp_pose.copy()
            self._update_all_sliders_from_target()
            self._initialized = True

    def _enable_controls(self):
        """Re-enable controls."""
        for slider in self.position_sliders:
            slider.setEnabled(True)
        for slider in self.orient_sliders:
            slider.setEnabled(True)
        for btn in self.position_minus_buttons + self.position_plus_buttons:
            btn.setEnabled(True)
        for btn in self.orient_minus_buttons + self.orient_plus_buttons:
            btn.setEnabled(True)
        self.reset_btn.setEnabled(True)
        self.status_label.setText("Auto-move enabled (scroll, drag, or click buttons)")
        self.status_label.setStyleSheet("QLabel { color: #666; }")

    def _update_current_display(self):
        """Refresh the current pose display."""
        try:
            tcp_pose = self._get_tcp_pose_in_base()

            if tcp_pose is None:
                return

            if self.target_pose is None:
                self.target_pose = tcp_pose.copy()
                self._update_all_sliders_from_target()
                self._initialized = True

            self.current_values[0].setText(f"{tcp_pose[0, 3]:.3f}")
            self.current_values[1].setText(f"{tcp_pose[1, 3]:.3f}")
            self.current_values[2].setText(f"{tcp_pose[2, 3]:.3f}")

            # Always show true current Euler angles regardless of control frame
            R_matrix = tcp_pose[:3, :3]
            euler = R.from_matrix(R_matrix).as_euler('xyz')

            for i in range(3):
                self.current_values[3 + i].setText(self._display_angle(euler[i]))

        except Exception as e:
            logger.debug(f"[Cartesian] _update_current_display exception: {e}")

    def _get_tcp_pose_in_base(self):
        """Get TCP pose in robot base coordinates."""
        if self.kinematic_model is None:
            return None

        try:
            tcp_world = self.kinematic_model.get_tcp_pose()

            if self.robot_manager and self.asset_id and self.transform_registry is not None:
                base_frame = self.robot_manager.get_asset_base_frame(self.asset_id)
                result = self.transform_registry.transform_frame_pose(
                    tcp_world,
                    from_frame="world",
                    to_frame=base_frame
                )
                return result
            return tcp_world
        except Exception as e:
            logger.debug(f"[Cartesian] _get_tcp_pose_in_base exception: {e}")
            return None

    # =================================================================
    # Mode & IK Handling
    # =================================================================

    def _on_mode_switched(self, event):
        mode = event.data.get('mode', '')

        mode_display = {
            'simulate_local': 'IK: Local (simulated)',
            'simulate_real_ik': 'IK: Real robot solver',
            'real': 'IK: Real robot solver',
        }
        self.ik_mode_label.setText(mode_display.get(mode, f'IK: {mode}'))

        if mode == "real" and self.robot_manager:
            real_robot = getattr(self.robot_manager, '_real_robot', None)
            if real_robot:
                state = real_robot.get_state()
                if state:
                    tcp_pose = self._get_tcp_pose_in_base()
                    if tcp_pose is not None:
                        self.target_pose = tcp_pose.copy()
                        self._update_all_sliders_from_target()
                        self._initialized = True

    def _check_ik_available(self) -> bool:
        """Check if IK is available."""
        if self.kinematic_model is None:
            return False
        try:
            current_pose = self.kinematic_model.get_tcp_pose()
            result = self.kinematic_model.solve_ik_for_tcp(
                current_pose,
                self.kinematic_model.get_current_joint_positions()
            )
            return result is not None
        except Exception:
            return False

    def _show_no_ik_message(self):
        """Show IK not available message."""
        for slider in self.position_sliders:
            slider.setEnabled(False)
        for slider in self.orient_sliders:
            slider.setEnabled(False)
        for btn in self.position_minus_buttons + self.position_plus_buttons:
            btn.setEnabled(False)
        for btn in self.orient_minus_buttons + self.orient_plus_buttons:
            btn.setEnabled(False)

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
        """Handle wheel events."""
        if event.type() == QEvent.Wheel:
            for idx, slider in enumerate(self.position_sliders):
                if obj == slider:
                    self._handle_wheel(slider, idx, event, is_position=True)
                    return True

            for idx, slider in enumerate(self.orient_sliders):
                if obj == slider:
                    self._handle_wheel(slider, idx, event, is_position=False)
                    return True

        return super().eventFilter(obj, event)

    def _handle_wheel(self, slider, idx, event, is_position):
        """Handle wheel event for a slider."""
        delta = event.angleDelta().y()
        key = ('pos' if is_position else 'rot', idx)
        if key not in self._wheel_accumulator:
            self._wheel_accumulator[key] = 0
        self._wheel_accumulator[key] += delta

        if is_position:
            step_size = self._get_position_step()
            range_min, range_max = self.position_limits
        else:
            step_size = self._get_angular_step()
            range_min, range_max = self.orientation_limits

        total_range = range_max - range_min
        step_in_units = max(1, int((step_size / total_range) * 1000))
        notch = 120

        while abs(self._wheel_accumulator[key]) >= notch:
            if self._wheel_accumulator[key] > 0:
                new_val = min(slider.value() + step_in_units, slider.maximum())
                self._wheel_accumulator[key] -= notch
            else:
                new_val = max(slider.value() - step_in_units, slider.minimum())
                self._wheel_accumulator[key] += notch
            slider.setValue(new_val)

        self._move_timer.start(100)

    # =================================================================
    # Public Methods
    # =================================================================

    def setEnabled(self, enabled: bool):
        """Enable or disable all controls."""
        super().setEnabled(enabled)
        for slider in self.position_sliders:
            slider.setEnabled(enabled)
        for slider in self.orient_sliders:
            slider.setEnabled(enabled)
        for btn in self.position_minus_buttons + self.position_plus_buttons:
            btn.setEnabled(enabled)
        for btn in self.orient_minus_buttons + self.orient_plus_buttons:
            btn.setEnabled(enabled)
        if hasattr(self, 'reset_btn'):
            self.reset_btn.setEnabled(enabled)

    def cleanup(self):
        """Unsubscribe from events."""
        self.state_channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self.state_channel.unsubscribe(EventType.MODE_SWITCHED, self._on_mode_switched)
        self.state_channel.unsubscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)
        self._move_timer.stop()