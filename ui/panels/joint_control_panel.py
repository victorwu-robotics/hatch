"""
Joint Control Panel - Pure UI for joint position control.

Publishes JOINT_COMMAND events when the user moves sliders.
Subscribes to ROBOT_STATE for display updates only.
Does NOT update kinematic models or call managers directly.

Principle #9: UI Separate from Services. Pure presentation.
Principle #2: Event-Driven. Publishes events, subscribes to state.
"""

import numpy as np
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QSlider,
    QPushButton, QGroupBox, QScrollArea, QWidget
)
from PyQt5.QtCore import Qt, QEvent

from core.world_state.event_types import EventType


class JointControlPanel(QWidget):
    """
    Panel with joint sliders only.

    Flow:
        1. User drags slider → publishes JOINT_COMMAND
        2. CommandHandler routes to active robot
        3. Robot publishes ROBOT_STATE
        4. StateHandler updates kinematic model + transform registry
        5. This panel receives ROBOT_STATE and updates sliders/labels
    """

    def __init__(self, kinematic_model, state_channel, parent=None):
        super().__init__(parent)

        self.kinematic_model = kinematic_model
        self.state_channel = state_channel

        # Joint information from model
        self.joint_info = kinematic_model.get_joint_info()
        self.joint_names = self.joint_info['names']
        self.joint_limits = self.joint_info['limits']

        # UI state
        self.sliders = {}
        self.labels = {}
        self.name_labels = {}
        self._wheel_accumulator = {name: 0 for name in self.joint_names}

        self._setup_ui()

        # Subscribe to events
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self.state_channel.subscribe(EventType.MODE_SWITCHED, self._on_mode_switched)

        # Install wheel event filters
        for slider in self.sliders.values():
            slider.installEventFilter(self)

    # =================================================================
    # UI Setup
    # =================================================================

    def _setup_ui(self):
        """Create the joint sliders UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # Title
        title = QLabel("Joint Control")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Scrollable joint list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setSpacing(12)

        for joint_name in self.joint_names:
            group = self._create_joint_control(joint_name)
            container_layout.addWidget(group)

        container_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)

        # Home / Zero buttons
        button_layout = QHBoxLayout()

        self.home_btn = QPushButton("Home Position")
        self.home_btn.clicked.connect(self._on_home_clicked)
        button_layout.addWidget(self.home_btn)

        self.zero_btn = QPushButton("Zero All")
        self.zero_btn.clicked.connect(self._on_zero_clicked)
        button_layout.addWidget(self.zero_btn)

        layout.addLayout(button_layout)

    def _create_joint_control(self, joint_name):
        """Create a group box with slider and label for one joint."""
        group = QGroupBox()
        layout = QHBoxLayout(group)

        # Joint name label
        self.name_labels[joint_name] = QLabel(joint_name.replace('_', ' ').title())
        self.name_labels[joint_name].setFixedWidth(170)
        self.name_labels[joint_name].setToolTip(f"Joint: {joint_name}")
        layout.addWidget(self.name_labels[joint_name])

        # Slider
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 1000)
        slider.setValue(500)

        lower, upper = self._get_ui_limits(joint_name)
        slider.joint_name = joint_name
        slider.limits = (lower, upper)

        slider.valueChanged.connect(
            lambda value, name=joint_name: self._on_slider_changed(name, value)
        )
        layout.addWidget(slider)
        self.sliders[joint_name] = slider

        # Value label
        value_label = QLabel("0.000 rad")
        value_label.setAlignment(Qt.AlignRight)
        value_label.setMinimumWidth(80)
        value_label.setToolTip("Current value in radians")
        layout.addWidget(value_label)
        self.labels[joint_name] = value_label

        return group

    def _get_ui_limits(self, joint_name):
        """Get UI-friendly limits for a joint."""
        lower, upper = self.joint_limits[joint_name]
        if np.isfinite(lower) and np.isfinite(upper):
            return (lower, upper)
        return (-2 * np.pi, 2 * np.pi)

    # =================================================================
    # Slider Handling
    # =================================================================

    def _on_slider_changed(self, joint_name, slider_value):
        """
        Handle slider value change.

        Publishes JOINT_COMMAND with ALL current slider positions.
        Does NOT update the kinematic model directly.
        """
        slider = self.sliders[joint_name]
        lower, upper = slider.limits

        # Update label for immediate feedback
        fraction = slider_value / 1000.0
        joint_value = lower + fraction * (upper - lower)
        self.labels[joint_name].setText(f"{joint_value:.3f} rad")

        # Build positions array from ALL sliders
        positions = []
        for name in self.joint_names:
            s = self.sliders[name]
            low, high = s.limits
            frac = s.value() / 1000.0
            positions.append(low + frac * (high - low))

        # Publish JOINT_COMMAND
        self.state_channel.publish(
            EventType.JOINT_COMMAND,
            data={
                'positions': positions,
                'names': self.joint_names
            },
            source="joint_control_panel",
            description=f"Joint {joint_name} → {joint_value:.3f} rad"
        )

    # =================================================================
    # Button Handlers
    # =================================================================

    def _on_home_clicked(self):
        """Reset all joints to neutral (home) position."""
        neutral = self.kinematic_model.neutral_state()
        positions = neutral.tolist() if hasattr(neutral, 'tolist') else list(neutral)

        self._update_ui_from_positions(positions)

        self.state_channel.publish(
            EventType.JOINT_COMMAND,
            data={
                'positions': positions,
                'names': self.joint_names
            },
            source="joint_control_panel",
            description="Home position"
        )

    def _on_zero_clicked(self):
        """Set all joints to zero (or closest valid position)."""
        positions = []
        for name in self.joint_names:
            lower, upper = self.joint_limits[name]
            if lower <= 0 <= upper:
                positions.append(0.0)
            else:
                positions.append(lower if abs(lower) < abs(upper) else upper)

        self._update_ui_from_positions(positions)

        self.state_channel.publish(
            EventType.JOINT_COMMAND,
            data={
                'positions': positions,
                'names': self.joint_names
            },
            source="joint_control_panel",
            description="Zero position"
        )

    def _update_ui_from_positions(self, positions):
        """Update all sliders and labels from a positions list."""
        for i, name in enumerate(self.joint_names):
            if i >= len(positions):
                break

            slider = self.sliders[name]
            lower, upper = slider.limits
            pos = positions[i]

            fraction = (pos - lower) / (upper - lower)
            fraction = max(0.0, min(1.0, fraction))
            slider_value = int(fraction * 1000)

            slider.blockSignals(True)
            slider.setValue(slider_value)
            slider.blockSignals(False)

            self.labels[name].setText(f"{pos:.3f} rad")

    # =================================================================
    # Event Handlers (display only — no model mutation)
    # =================================================================

    def _on_robot_state(self, event):
        """
        Handle ROBOT_STATE event.

        Updates UI sliders and labels from the authoritative robot state.
        Does NOT update the kinematic model — that is StateHandler's job.
        """
        positions = event.data.get('joint_positions')
        if positions:
            self._update_ui_from_positions(positions)

    def _on_mode_switched(self, event):
        """Handle MODE_SWITCHED — update UI indicators."""
        mode = event.data.get('mode', '')
        is_real = (mode == "real")
        self._set_mode_indicators(is_real)

    def _set_mode_indicators(self, is_real):
        """Update joint name labels to show (actual) in Real mode."""
        for joint_name in self.joint_names:
            if joint_name in self.name_labels:
                base_text = joint_name.replace('_', ' ').title()
                if is_real:
                    self.name_labels[joint_name].setText(f"{base_text} (actual)")
                    self.name_labels[joint_name].setStyleSheet(
                        "QLabel { color: #666; font-style: italic; }"
                    )
                else:
                    self.name_labels[joint_name].setText(base_text)
                    self.name_labels[joint_name].setStyleSheet("")

    # =================================================================
    # Wheel Event Handling
    # =================================================================

    def eventFilter(self, obj, event):
        """Handle wheel events for smooth scrolling on sliders."""
        if event.type() == QEvent.Wheel:
            for name, slider in self.sliders.items():
                if obj == slider:
                    delta = event.angleDelta().y()
                    self._wheel_accumulator[name] += delta

                    notch = 120
                    while abs(self._wheel_accumulator[name]) >= notch:
                        if self._wheel_accumulator[name] > 0:
                            new_val = min(slider.value() + 1, slider.maximum())
                            self._wheel_accumulator[name] -= notch
                        else:
                            new_val = max(slider.value() - 1, slider.minimum())
                            self._wheel_accumulator[name] += notch
                        slider.setValue(new_val)
                    return True
        return super().eventFilter(obj, event)

    # =================================================================
    # Public Methods
    # =================================================================

    def setEnabled(self, enabled: bool):
        """Enable or disable all controls."""
        super().setEnabled(enabled)
        for slider in self.sliders.values():
            slider.setEnabled(enabled)
        if hasattr(self, 'home_btn'):
            self.home_btn.setEnabled(enabled)
        if hasattr(self, 'zero_btn'):
            self.zero_btn.setEnabled(enabled)

    def cleanup(self):
        """Unsubscribe from events before destruction."""
        self.state_channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self.state_channel.unsubscribe(EventType.MODE_SWITCHED, self._on_mode_switched)