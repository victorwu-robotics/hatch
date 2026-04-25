"""
Joint control panel with sliders for each joint.
Pure joint control - publishes JOINT_COMMAND and listens to ROBOT_STATE.
"""

from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QLabel, QSlider,
                             QPushButton, QGroupBox, QScrollArea, QWidget)
from PyQt5.QtCore import Qt, pyqtSignal, QEvent
import numpy as np

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType


class JointControlPanel(QWidget):
    """
    Panel with joint sliders only.
    
    Flow:
        1. User drags slider → publishes JOINT_COMMAND
        2. CommandHandler routes to active robot
        3. Robot publishes ROBOT_STATE
        4. This panel receives ROBOT_STATE and updates sliders/labels
    """
    
    # Keep this signal for backward compatibility with MainWindow
    # (MainWindow may still listen to it)
    state_changed = pyqtSignal()
    
    def __init__(self, kinematic_model, state_channel: StateChannel, 
                 robot_manager=None, parent=None):
        super().__init__(parent)
        
        # Core references
        self.kinematic_model = kinematic_model
        self.state_channel = state_channel
        self.robot_manager = robot_manager  # Kept for compatibility, but not used for commands
        
        # Get joint information from model
        self.joint_info = kinematic_model.get_joint_info()
        self.joint_names = self.joint_info['names']
        self.joint_limits = self.joint_info['limits']
        
        # UI state
        self.sliders = {}        # joint_name -> QSlider
        self.labels = {}         # joint_name -> QLabel (value)
        self.name_labels = {}    # joint_name -> QLabel (joint name)
        self._wheel_accumulator = {name: 0 for name in self.joint_names}
        
        # Current joint state (for display only, not authoritative)
        # self.current_q = kinematic_model.neutral_state().copy()
        
        # Build the UI
        self._setup_ui()
        
        # ===== NEW: Subscribe to ROBOT_STATE instead of robot_manager signals =====
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state_received)
        
        # Also subscribe to mode changes for UI indicators (optional)
        self.state_channel.subscribe(EventType.ROBOT_MODE_CHANGED, self._on_mode_changed)
        
        # Install event filters for wheel events
        for slider in self.sliders.values():
            slider.installEventFilter(self)
        
        print("[JointControlPanel] Initialized - publishing JOINT_COMMAND")
    
    def _setup_ui(self):
        """Create the joint sliders UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # Title
        title = QLabel("Joint Control")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Scroll area for joints
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setSpacing(12)
        
        # Create controls for each joint
        for joint_name in self.joint_names:
            group = self._create_joint_control(joint_name)
            container_layout.addWidget(group)
        
        container_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)
        
        # Home/Zero buttons
        button_layout = QHBoxLayout()
        
        home_btn = QPushButton("Home Position")
        home_btn.clicked.connect(self._on_home_clicked)
        button_layout.addWidget(home_btn)
        
        zero_btn = QPushButton("Zero All")
        zero_btn.clicked.connect(self._on_zero_clicked)
        button_layout.addWidget(zero_btn)
        
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
        
        # Store slider limits for this joint
        lower, upper = self._get_ui_limits(joint_name)
        slider.joint_name = joint_name
        slider.limits = (lower, upper)
        
        # Connect signal
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
        return (-2*np.pi, 2*np.pi)  # Default for unlimited joints
    
    # ===== Core Logic: Slider Changed =====
    
    def _on_slider_changed(self, joint_name, slider_value):
        """
        Handle slider value changes.
        Publishes JOINT_COMMAND — no local state storage.
        """
        slider = self.sliders[joint_name]
        lower, upper = slider.limits
        
        # Convert slider value to joint angle
        fraction = slider_value / 1000.0
        joint_value = lower + fraction * (upper - lower)
        
        # Update local label (immediate feedback)
        self.labels[joint_name].setText(f"{joint_value:.3f} rad")
        
        # Build positions array from ALL current slider values
        positions = []
        for name in self.joint_names:
            s = self.sliders[name]
            low, high = s.limits
            frac = s.value() / 1000.0
            pos = low + frac * (high - low)
            positions.append(pos)
        
        # Publish JOINT_COMMAND
        self.state_channel.publish(
            EventType.JOINT_COMMAND,
            data={
                'positions': positions,
                'names': self.joint_names
            },
            source="joint_control_panel",
            description=f"Joint {joint_name} moved to {joint_value:.3f} rad"
        )
    
    # ===== Home and Zero Buttons =====
    
    def _on_home_clicked(self):
        """Reset all joints to neutral position."""
        neutral = self.kinematic_model.neutral_state()
        positions = neutral.tolist() if hasattr(neutral, 'tolist') else list(neutral)
        
        # Update UI immediately
        self._update_ui_from_positions(positions)
        
        # Publish JOINT_COMMAND
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
        """Set all joints to zero (if within limits)."""
        positions = []
        for name in self.joint_names:
            lower, upper = self.joint_limits[name]
            if lower <= 0 <= upper:
                positions.append(0.0)
            else:
                # Use the limit closer to zero
                positions.append(lower if abs(lower) < abs(upper) else upper)
        
        # Update UI immediately
        self._update_ui_from_positions(positions)
        
        # Publish JOINT_COMMAND
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
            
            # Convert joint angle to slider value
            fraction = (pos - lower) / (upper - lower)
            fraction = max(0.0, min(1.0, fraction))
            slider_value = int(fraction * 1000)
            
            # Update slider (block signals to avoid recursion)
            slider.blockSignals(True)
            slider.setValue(slider_value)
            slider.blockSignals(False)
            
            # Update label
            self.labels[name].setText(f"{pos:.3f} rad")

    # ===== ROBOT STATE FEEDBACK (from StateHandler) =====
    
    def _on_robot_state_received(self, event):
        """
        Update UI from robot state.
        This is the ONLY source of truth for joint positions.
        """
        positions = event.data.get('joint_positions')
        if not positions:
            return
        
        # Update UI from received positions
        self._update_ui_from_positions(positions)
    
    # ===== Mode Change (for UI visual feedback) =====
    
    def _on_mode_changed(self, event):
        """Handle mode changes to update UI appearance."""
        mode = event.data.get('mode')
        is_real = (mode == "real")
        self.set_real_mode_indicators(is_real)
    
    def set_real_mode_indicators(self, is_real):
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
    
    # ===== Wheel Event Handling (unchanged) =====
    
    def eventFilter(self, obj, event):
        """Handle wheel events for smooth scrolling."""
        if event.type() == QEvent.Wheel:
            for name, slider in self.sliders.items():
                if obj == slider:
                    delta = event.angleDelta().y()
                    self._wheel_accumulator[name] += delta
                    
                    NOTCH_SIZE = 120
                    while abs(self._wheel_accumulator[name]) >= NOTCH_SIZE:
                        if self._wheel_accumulator[name] > 0:
                            new_val = min(slider.value() + 1, slider.maximum())
                            self._wheel_accumulator[name] -= NOTCH_SIZE
                        else:
                            new_val = max(slider.value() - 1, slider.minimum())
                            self._wheel_accumulator[name] += NOTCH_SIZE
                        slider.setValue(new_val)
                    return True
        return super().eventFilter(obj, event)

    def cleanup(self):
        """Unsubscribe from state channel before panel is destroyed."""
        self.state_channel.unsubscribe(EventType.ROBOT_STATE, self._on_robot_state_received)
        self.state_channel.unsubscribe(EventType.ROBOT_MODE_CHANGED, self._on_mode_changed)
        print("[JointControlPanel] Cleaned up subscriptions")

    def setEnabled(self, enabled: bool):
        """Enable or disable all controls in the panel."""
        super().setEnabled(enabled)
        for slider in self.sliders.values():
            slider.setEnabled(enabled)
        for btn in [self.home_btn, self.zero_btn] if hasattr(self, 'home_btn') else []:
            btn.setEnabled(enabled)

    # ===== Compatibility Methods (kept for MainWindow) =====
    
    def set_robot_manager(self, manager):
        """
        Set robot manager (kept for compatibility).
        The panel no longer uses robot_manager for commands,
        but keeps reference for mode indicators.
        """
        self.robot_manager = manager