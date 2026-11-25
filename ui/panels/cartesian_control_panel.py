"""
Cartesian Control Panel - TCP pose control with proper coordinate semantics.
Publishes CARTESIAN_COMMAND events when the user moves sliders or clicks buttons.
Subscribes to ROBOT_STATE for display updates only.
Does NOT update kinematic models or transform registries directly.

Design: See docs/cartesian_jog_design.md
- Position and orientation have independent frame selectors
- Position: Base=absolute, Tool=incremental jog
- Orientation: incremental jog in both frames (Base: pre-multiply, Tool: post-multiply)
- Anchor-at-press semantics for all jogs (exactly reversible, immune to event quantization)
- Readout: rotation vector (primary), RPY secondary, no quaternions
- URDF-derived position ranges
- Compact UI: Current/Target readouts integrated into Position/Orientation groups
"""
import numpy as np
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QWidget,
    QSlider, QComboBox, QRadioButton
)
from PyQt5.QtCore import Qt, QTimer, QEvent
from scipy.spatial.transform import Rotation as R
from core.world_state.event_types import EventType
import logging

logger = logging.getLogger(__name__)


class CartesianControlPanel(QWidget):
    """
    Cartesian control with proper coordinate semantics.

    Position and orientation each have their own frame selector.
    Position: Base frame = absolute values, Tool frame = incremental jog
    Orientation: Both frames = incremental jog (Base: R=A(d)*R0, Tool: R=R0*A(d))

    All jogs use anchor-at-press semantics: the pose is computed from the anchor
    (captured when drag starts) and the current delta, not by accumulating deltas.
    This makes drags exactly reversible and immune to event quantization.
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

        # Target pose (4x4 homogeneous matrix)
        self.target_pose = None
        self._last_achieved_pose = None  # For rollback on IK refusal
        self._initialized = False

        # Anchor state for jog semantics.
        # PER AXIS: an anchor belongs to one gesture on one axis. A single
        # shared anchor let axis Y inherit axis X's origin and silently
        # discard X's motion.
        self._pos_anchor = [None, None, None]     # (p0, R0) per axis, Tool jog
        self._orient_anchor = [None, None, None]  # R0 per axis, both frames

        # Motion bookkeeping: divergence must mean "refused", not "in flight".
        self._motion_in_flight = False
        self._ik_refused = False

        # Wheel state
        self._wheel_accumulator = {}
        self._wheel_active = {}  # Track which sliders are wheel-active
        self._wheel_burst = {}   # Physical delta accumulated in this burst
        self._wheel_idle_timer = QTimer()
        self._wheel_idle_timer.setSingleShot(True)
        self._wheel_idle_timer.timeout.connect(self._on_wheel_idle_timeout)

        # Debounce timer for publishing
        self._move_timer = QTimer()
        self._move_timer.setSingleShot(True)
        self._move_timer.timeout.connect(self._publish_command)

        # Compute URDF-derived ranges
        self._urdf_reach = self._compute_urdf_reach()
        self.position_limits = (-self._urdf_reach, self._urdf_reach)
        self._pos_jog_full_scale = 0.1 * self._urdf_reach  # 10% of reach
        self._orient_jog_full_scale = np.radians(30)  # 30 degrees

        # Setup UI
        self._setup_ui()

        # Check IK availability
        self._ik_available = self._check_ik_available()
        if not self._ik_available:
            self._show_no_ik_message()

        # Install event filters for wheel handling
        for slider in self.position_sliders:
            slider.installEventFilter(self)
        for slider in self.orient_sliders:
            slider.installEventFilter(self)

        # Subscribe to events
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)
        self.state_channel.subscribe(EventType.MODE_SWITCHED, self._on_mode_switched)
        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)
        self.state_channel.subscribe(EventType.ERROR_OCCURRED, self._on_error)

    # =================================================================
    # URDF Reach Computation
    # =================================================================

    def _compute_urdf_reach(self):
        """
        Maximum planar reach from the loaded URDF: L2 + L3 + |d6|.

        Measured from the arm's axis lines, not from DH parameters:
          L2 = axis_gap(2)      J3 -> J4 common-perpendicular distance
          L3 = elbow_to_wrist() works for roll and pitch wrists alike
          d6 = |flange_offset|  wrist centre -> flange

        Verified: UR10 1.2765 m, FR5 0.92001 m, Elfin 1.5000 m.

        This is an UPPER BOUND on reach from the shoulder axis, not the
        reachable set. Reachability is decided by IK refusal, never by this
        number (see docs/cartesian_jog_design.md section 8).
        """
        fallback = 1.5
        if self.kinematic_model is None:
            return fallback
        try:
            from core.kinematics.geometric_extraction import extract_arm_geometry
            geom = extract_arm_geometry(self.kinematic_model)
            reach = (geom.axis_gap(2)
                     + geom.elbow_to_wrist()
                     + abs(geom.flange_offset))
            if not np.isfinite(reach) or reach <= 0.05:
                logger.warning("[Cartesian] implausible URDF reach %s; using %s m",
                               reach, fallback)
                return fallback
            logger.info("[Cartesian] URDF-derived reach %.5f m", reach)
            return float(reach)
        except Exception as e:
            # Out-of-class arms refuse extraction. Say so rather than pretend
            # the range is URDF-derived.
            logger.warning("[Cartesian] reach not URDF-derived (%s: %s); "
                           "using %s m", type(e).__name__, e, fallback)
            return fallback

    def _style_slider(self, slider, is_jog):
        """Jog sliders get a centre detent; value sliders do not."""
        if is_jog:
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(500)
        else:
            slider.setTickPosition(QSlider.NoTicks)

    # =================================================================
    # UI Setup
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

        # Position Control Group (with integrated readout)
        main_layout.addWidget(self._create_position_group())

        # Orientation Control Group (with integrated readout)
        main_layout.addWidget(self._create_orientation_group())

        # Step Size Controls (with Reset button on Linear Step row)
        main_layout.addLayout(self._create_step_controls())

        # Status
        self.status_label = QLabel("Auto-move enabled (scroll, drag, or click buttons)")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { color: #666; }")
        main_layout.addWidget(self.status_label)

        main_layout.addStretch()

    def _create_position_group(self):
        """Create position control group with frame selector and integrated readout."""
        group = QGroupBox("Position")
        layout = QVBoxLayout()
        layout.setSpacing(8)

        # --- Integrated Readout Row: Current [Left] | Target [Right] ---
        readout_container = QWidget()
        readout_layout = QHBoxLayout(readout_container)
        readout_layout.setContentsMargins(0, 0, 0, 0)
        readout_layout.setSpacing(10)

        # Current Position (X, Y, Z)
        curr_box = QGroupBox("Current")
        curr_layout = QGridLayout()
        curr_layout.setHorizontalSpacing(5)
        curr_layout.setVerticalSpacing(2)
        headers = ['X', 'Y', 'Z']
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QLabel { font-size: 9px; color: #666; }")
            curr_layout.addWidget(label, 0, col)

        self.pos_current_vals = []
        for col in range(3):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #0066cc; font-family: monospace; font-size: 10px; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(50)
            curr_layout.addWidget(value, 1, col)
            self.pos_current_vals.append(value)
        curr_box.setLayout(curr_layout)
        readout_layout.addWidget(curr_box, 1)

        # Target Position (X, Y, Z)
        tgt_box = QGroupBox("Target")
        tgt_layout = QGridLayout()
        tgt_layout.setHorizontalSpacing(5)
        tgt_layout.setVerticalSpacing(2)
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QLabel { font-size: 9px; color: #666; }")
            tgt_layout.addWidget(label, 0, col)

        self.pos_target_vals = []
        for col in range(3):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #e67e22; font-family: monospace; font-size: 10px; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(50)
            tgt_layout.addWidget(value, 1, col)
            self.pos_target_vals.append(value)
        tgt_box.setLayout(tgt_layout)
        readout_layout.addWidget(tgt_box, 1)

        layout.addWidget(readout_container)

        # Frame selector header
        header_row = QHBoxLayout()
        header = QLabel("Frame:")
        header_row.addWidget(header)
        header_row.addStretch()

        self.pos_frame_selector = QComboBox()
        self.pos_frame_selector.addItems(["Base", "Tool"])
        self.pos_frame_selector.setToolTip(
            "Base: Absolute position values\n"
            "Tool: Incremental jog along current TCP axes"
        )
        self.pos_frame_selector.currentIndexChanged.connect(self._on_pos_frame_changed)
        header_row.addWidget(self.pos_frame_selector)
        layout.addLayout(header_row)

        # Mode hint
        self.pos_mode_hint = QLabel("")
        self.pos_mode_hint.setStyleSheet("QLabel { color: #888; font-size: 10px; }")
        self.pos_mode_hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.pos_mode_hint)

        # Sliders
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
            minus_btn.setToolTip(f"Decrease along {pos_axes[i]} by step size")
            minus_btn.clicked.connect(lambda checked, idx=i: self._on_position_step(idx, -1))
            row.addWidget(minus_btn)
            self.position_minus_buttons.append(minus_btn)

            axis_label = QLabel(f"along {pos_axes[i]}")
            axis_label.setFixedWidth(70)
            axis_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
            row.addWidget(axis_label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(500)
            slider.setMinimumWidth(180)
            slider.setSingleStep(1)
            slider.setPageStep(1)
            slider.sliderPressed.connect(lambda idx=i: self._on_pos_slider_pressed(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._on_position_slider_changed(idx, value))
            slider.sliderReleased.connect(lambda idx=i: self._on_pos_slider_released(idx))
            row.addWidget(slider, 6)
            self.position_sliders.append(slider)

            plus_btn = QPushButton("+")
            plus_btn.setFixedWidth(30)
            plus_btn.setFixedHeight(24)
            plus_btn.setToolTip(f"Increase along {pos_axes[i]} by step size")
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

        group.setLayout(layout)
        return group

    def _create_orientation_group(self):
        """Create orientation control group with frame selector and integrated readout."""
        group = QGroupBox("Orientation")
        layout = QVBoxLayout()
        layout.setSpacing(8)

        # --- Integrated Readout Row: Current [Left] | Target [Right] ---
        readout_container = QWidget()
        readout_layout = QHBoxLayout(readout_container)
        readout_layout.setContentsMargins(0, 0, 0, 0)
        readout_layout.setSpacing(10)

        # Current Orientation (RV-X, RV-Y, RV-Z)
        curr_box = QGroupBox("Current")
        curr_layout = QGridLayout()
        curr_layout.setHorizontalSpacing(5)
        curr_layout.setVerticalSpacing(2)
        headers = ['RV-X', 'RV-Y', 'RV-Z']
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QLabel { font-size: 9px; color: #666; }")
            curr_layout.addWidget(label, 0, col)

        self.orient_current_vals = []
        for col in range(3):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #0066cc; font-family: monospace; font-size: 10px; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(50)
            curr_layout.addWidget(value, 1, col)
            self.orient_current_vals.append(value)
        curr_box.setLayout(curr_layout)
        readout_layout.addWidget(curr_box, 1)

        # Target Orientation (RV-X, RV-Y, RV-Z)
        tgt_box = QGroupBox("Target")
        tgt_layout = QGridLayout()
        tgt_layout.setHorizontalSpacing(5)
        tgt_layout.setVerticalSpacing(2)
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QLabel { font-size: 9px; color: #666; }")
            tgt_layout.addWidget(label, 0, col)

        self.orient_target_vals = []
        for col in range(3):
            value = QLabel("0.000")
            value.setStyleSheet("QLabel { color: #e67e22; font-family: monospace; font-size: 10px; }")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumWidth(50)
            tgt_layout.addWidget(value, 1, col)
            self.orient_target_vals.append(value)
        tgt_box.setLayout(tgt_layout)
        readout_layout.addWidget(tgt_box, 1)

        layout.addWidget(readout_container)

        # RPY secondary readout (derived, extrinsic XYZ). Rotation vector above
        # is the primary truth; RPY is offered for familiarity and labelled with
        # its convention because it is a chart, not independent axis amounts.
        rpy_row = QHBoxLayout()
        rpy_row.setSpacing(10)
        self.orient_current_rpy = QLabel("RPY —")
        self.orient_target_rpy = QLabel("RPY —")
        for w, col in ((self.orient_current_rpy, "#0066cc"), (self.orient_target_rpy, "#e67e22")):
            w.setAlignment(Qt.AlignCenter)
            w.setStyleSheet(f"QLabel {{ color: {col}; font-family: monospace; font-size: 9px; }}")
            w.setToolTip("Euler RPY, extrinsic XYZ (R = Rz·Ry·Rx). Derived display only.\n"
                         "Degenerate at RY = ±90°, where the numbers can jump.")
            rpy_row.addWidget(w, 1)
        layout.addLayout(rpy_row)

        # Frame selector header
        header_row = QHBoxLayout()
        header = QLabel("Frame:")
        header_row.addWidget(header)
        header_row.addStretch()

        self.orient_frame_selector = QComboBox()
        self.orient_frame_selector.addItems(["Base", "Tool"])
        self.orient_frame_selector.setToolTip(
            "Base: Incremental rotation about fixed base axes (pre-multiply)\n"
            "Tool: Incremental rotation about current TCP axes (post-multiply)"
        )
        self.orient_frame_selector.currentIndexChanged.connect(self._on_orient_frame_changed)
        header_row.addWidget(self.orient_frame_selector)
        layout.addLayout(header_row)

        # Mode hint
        self.orient_mode_hint = QLabel("Incremental jog (springs back to center after release)")
        self.orient_mode_hint.setStyleSheet("QLabel { color: #888; font-size: 10px; }")
        self.orient_mode_hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.orient_mode_hint)

        # Sliders
        self.orient_sliders = []
        self.orient_labels = []
        self.orient_minus_buttons = []
        self.orient_plus_buttons = []

        rot_axes = ['X', 'Y', 'Z']
        for i in range(3):
            row = QHBoxLayout()
            row.setSpacing(6)

            minus_btn = QPushButton("-")
            minus_btn.setFixedWidth(30)
            minus_btn.setFixedHeight(24)
            minus_btn.setToolTip(f"Decrease about {rot_axes[i]} by step size")
            minus_btn.clicked.connect(lambda checked, idx=i: self._on_orientation_step(idx, -1))
            row.addWidget(minus_btn)
            self.orient_minus_buttons.append(minus_btn)

            axis_label = QLabel(f"about {rot_axes[i]}")
            axis_label.setFixedWidth(70)
            axis_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
            row.addWidget(axis_label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            slider.setValue(500)
            slider.setMinimumWidth(180)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(500)
            slider.setSingleStep(1)
            slider.setPageStep(1)
            slider.sliderPressed.connect(lambda idx=i: self._on_orient_slider_pressed(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._on_orientation_slider_changed(idx, value))
            slider.sliderReleased.connect(lambda idx=i: self._on_orient_slider_released(idx))
            row.addWidget(slider, 6)
            self.orient_sliders.append(slider)

            plus_btn = QPushButton("+")
            plus_btn.setFixedWidth(30)
            plus_btn.setFixedHeight(24)
            plus_btn.setToolTip(f"Increase about {rot_axes[i]} by step size")
            plus_btn.clicked.connect(lambda checked, idx=i: self._on_orientation_step(idx, +1))
            row.addWidget(plus_btn)
            self.orient_plus_buttons.append(plus_btn)

            value_label = QLabel("Δ0.0 deg")
            value_label.setFixedWidth(80)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet("QLabel { color: #e67e22; font-weight: bold; font-family: monospace; }")
            row.addWidget(value_label, 3)
            self.orient_labels.append(value_label)

            layout.addLayout(row)

        group.setLayout(layout)
        self.orient_group = group
        return group

    def _create_step_controls(self):
        """Create linear and angular step size controls with Reset button."""
        layout = QVBoxLayout()
        layout.setSpacing(6)

        # Linear Step row with Reset button on the right
        linear_row = QHBoxLayout()
        linear_row.addWidget(QLabel("Linear Step:"))
        self.linear_step_combo = QComboBox()
        self.linear_step_combo.addItems(["0.0001", "0.001", "0.01", "0.1"])
        self.linear_step_combo.setCurrentText("0.001")
        linear_row.addWidget(self.linear_step_combo)
        linear_row.addWidget(QLabel("m"))
        linear_row.addStretch()

        # Reset button on the right side of Linear Step row
        self.reset_btn = QPushButton("Reset to Current")
        self.reset_btn.setToolTip("Reset target pose to match current TCP position")
        self.reset_btn.clicked.connect(self._reset_target)
        linear_row.addWidget(self.reset_btn)

        layout.addLayout(linear_row)

        # Angular Step row
        angular_row = QHBoxLayout()
        angular_row.addWidget(QLabel("Angular Step:"))
        self.angular_step_combo = QComboBox()
        self.angular_step_combo.addItems(["0.1", "1", "5", "10"])
        self.angular_step_combo.setCurrentText("1")
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
            return f"{np.degrees(angle_rad):.1f} deg"
        return f"{angle_rad:.3f} rad"

    # =================================================================
    # Slider Value Conversion
    # =================================================================

    def _position_slider_to_value(self, slider_value, is_absolute=True):
        """Convert slider position to position value."""
        if is_absolute:
            # Absolute: map [0, 1000] to position_limits
            low, high = self.position_limits
            fraction = slider_value / 1000.0
            return low + fraction * (high - low)
        else:
            # Jog: map [0, 1000] to [-full_scale, +full_scale]
            fraction = (slider_value - 500) / 500.0
            return fraction * self._pos_jog_full_scale

    def _position_value_to_slider(self, value, is_absolute=True):
        """Convert position value to slider position."""
        if is_absolute:
            low, high = self.position_limits
            fraction = (value - low) / (high - low)
            fraction = np.clip(fraction, 0.0, 1.0)
            return int(fraction * 1000)
        else:
            fraction = value / self._pos_jog_full_scale
            fraction = np.clip(fraction, -1.0, 1.0)
            return int(500 + fraction * 500)

    def _angle_slider_to_delta(self, slider_value):
        """Convert slider position to angle delta for jog."""
        fraction = (slider_value - 500) / 500.0
        return fraction * self._orient_jog_full_scale

    # =================================================================
    # Frame Selection
    # =================================================================

    def _get_pos_frame(self):
        return self.pos_frame_selector.currentText().lower()

    def _get_orient_frame(self):
        return self.orient_frame_selector.currentText().lower()

    def _on_pos_frame_changed(self):
        """Handle position frame selector change."""
        if self.target_pose is None:
            return

        frame = self._get_pos_frame()
        self._pos_anchor = [None, None, None]
        for i in range(3):
            self._style_slider(self.position_sliders[i], is_jog=(frame == 'tool'))
        if frame == 'base':
            self.pos_mode_hint.setText("Absolute position values")
            # Populate sliders from current target
            for i in range(3):
                self._update_position_slider(i, is_absolute=True)
        else:
            self.pos_mode_hint.setText("Incremental jog (springs back to center after release)")
            # Recentre to 0
            for i in range(3):
                self.position_sliders[i].blockSignals(True)
                self.position_sliders[i].setValue(500)
                self.position_sliders[i].blockSignals(False)
                self.position_labels[i].setText("Δ0.000 m")

        logger.debug(f"[Cartesian] Position frame changed to: {frame}")

    def _on_orient_frame_changed(self):
        """Handle orientation frame selector change."""
        # Both frames are jog. Drop any live anchor so a frame switch cannot
        # compose a Base delta onto a Tool anchor.
        self._orient_anchor = [None, None, None]
        frame = self._get_orient_frame()
        logger.debug(f"[Cartesian] Orientation frame changed to: {frame}")

    # =================================================================
    # Position Slider Handling
    # =================================================================

    def _clip_tool_displacement(self, p0, disp):
        """
        Scale a displacement so p0 + t*disp stays inside position_limits.

        Scaling rather than component clipping preserves the direction, so a
        Tool jog still moves ALONG the chosen tool axis right up to the boundary.
        Returns the (possibly shortened) displacement.
        """
        low, high = self.position_limits
        t = 1.0
        for k in range(3):
            d = disp[k]
            if abs(d) < 1e-15:
                continue
            limit = high if d > 0 else low
            t_k = (limit - p0[k]) / d
            t = min(t, max(0.0, t_k))
        return disp * t

    def _finish_pos_gesture(self, axis_idx):
        """Complete any live jog gesture on this axis before a discrete step."""
        self.position_sliders[axis_idx].blockSignals(True)
        self.position_sliders[axis_idx].setValue(500)
        self.position_sliders[axis_idx].blockSignals(False)
        self._pos_anchor[axis_idx] = None
        self._wheel_burst[('pos', axis_idx)] = 0.0
        self._wheel_active[('pos', axis_idx)] = False

    def _on_pos_slider_pressed(self, axis_idx):
        """Capture anchor when drag starts (for Tool frame jog)."""
        if self.target_pose is None:
            return

        if self._get_pos_frame() == 'tool':
            self._pos_anchor[axis_idx] = (
                self.target_pose[:3, 3].copy(),
                self.target_pose[:3, :3].copy()
            )

    def _on_position_slider_changed(self, axis_idx, slider_value):
        """Handle position slider movement."""
        if self.target_pose is None:
            return

        frame = self._get_pos_frame()

        if frame == 'base':
            # Absolute position
            value = self._position_slider_to_value(slider_value, is_absolute=True)
            self.target_pose[axis_idx, 3] = value
            self.position_labels[axis_idx].setText(f"{value:.3f} m")
        else:
            # Tool frame jog, anchored at press for THIS axis
            if self._pos_anchor[axis_idx] is None:
                self._pos_anchor[axis_idx] = (
                    self.target_pose[:3, 3].copy(),
                    self.target_pose[:3, :3].copy()
                )

            p0, R0 = self._pos_anchor[axis_idx]
            delta_scalar = self._position_slider_to_value(slider_value, is_absolute=False)
            delta = np.zeros(3)
            delta[axis_idx] = delta_scalar

            disp = self._clip_tool_displacement(p0, R0 @ delta)
            self.target_pose[:3, 3] = p0 + disp
            self.position_labels[axis_idx].setText(f"Δ{delta_scalar:.3f} m")

        self._move_timer.start(100)

    def _apply_position_step(self, axis_idx, direction):
        """
        One discrete position step. A complete gesture: acts on the CURRENT
        pose and leaves no anchor, so repeated steps accumulate.
        """
        step = self._get_position_step()
        frame = self._get_pos_frame()

        if frame == 'base':
            current_val = self.target_pose[axis_idx, 3]
            low, high = self.position_limits
            self.target_pose[axis_idx, 3] = np.clip(
                current_val + direction * step, low, high)
            self._update_position_slider(axis_idx, is_absolute=True)
        else:
            self._finish_pos_gesture(axis_idx)
            p0 = self.target_pose[:3, 3].copy()
            R0 = self.target_pose[:3, :3].copy()
            delta = np.zeros(3)
            delta[axis_idx] = direction * step
            disp = self._clip_tool_displacement(p0, R0 @ delta)
            self.target_pose[:3, 3] = p0 + disp
            self.position_sliders[axis_idx].blockSignals(True)
            self.position_sliders[axis_idx].setValue(500)
            self.position_sliders[axis_idx].blockSignals(False)
            self.position_labels[axis_idx].setText(f"Δ{direction * step:.3f} m")
            QTimer.singleShot(500, lambda i=axis_idx: self._reset_pos_label(i))

    def _reset_pos_label(self, axis_idx):
        if self._get_pos_frame() == 'tool':
            self.position_labels[axis_idx].setText("Δ0.000 m")

    def _on_position_step(self, axis_idx, direction):
        """Handle [-] or [+] button click for position axis."""
        if self.target_pose is None:
            return
        self._apply_position_step(axis_idx, direction)
        self._publish_command()

    def _on_pos_slider_released(self, axis_idx):
        """Handle position slider release (spring back for Tool frame)."""
        self._move_timer.stop()
        self._publish_command()

        if self._get_pos_frame() == 'tool':
            # Spring back to center, and clear only THIS axis's anchor
            self.position_sliders[axis_idx].blockSignals(True)
            self.position_sliders[axis_idx].setValue(500)
            self.position_sliders[axis_idx].blockSignals(False)
            self.position_labels[axis_idx].setText("Δ0.000 m")
            self._pos_anchor[axis_idx] = None

    def _update_position_slider(self, axis_idx, is_absolute=True):
        """Update position slider and label."""
        value = self.target_pose[axis_idx, 3]
        self.position_sliders[axis_idx].blockSignals(True)
        self.position_sliders[axis_idx].setValue(self._position_value_to_slider(value, is_absolute))
        self.position_sliders[axis_idx].blockSignals(False)

        if is_absolute:
            self.position_labels[axis_idx].setText(f"{value:.3f} m")
        else:
            self.position_labels[axis_idx].setText(f"Δ{value:.3f} m")

    # =================================================================
    # Orientation Slider Handling
    # =================================================================

    def _rot_about(self, axis_idx, angle_rad):
        """Rotation matrix for a jog about one coordinate axis."""
        return R.from_euler('xyz'[axis_idx], angle_rad).as_matrix()

    def _compose_orient(self, R0, R_inc):
        """Base pre-multiplies (fixed axes); Tool post-multiplies (current axes)."""
        if self._get_orient_frame() == 'base':
            return R_inc @ R0
        return R0 @ R_inc

    def _finish_orient_gesture(self, axis_idx):
        """Complete any live jog gesture on this axis before a discrete step."""
        self.orient_sliders[axis_idx].blockSignals(True)
        self.orient_sliders[axis_idx].setValue(500)
        self.orient_sliders[axis_idx].blockSignals(False)
        self._orient_anchor[axis_idx] = None
        self._wheel_burst[('rot', axis_idx)] = 0.0
        self._wheel_active[('rot', axis_idx)] = False

    def _on_orient_slider_pressed(self, axis_idx):
        """Capture anchor when drag starts, for THIS axis only."""
        if self.target_pose is None:
            return
        self._orient_anchor[axis_idx] = self.target_pose[:3, :3].copy()

    def _on_orientation_slider_changed(self, axis_idx, slider_value):
        """Handle orientation slider movement."""
        if self.target_pose is None:
            return

        if self._orient_anchor[axis_idx] is None:
            self._orient_anchor[axis_idx] = self.target_pose[:3, :3].copy()

        R0 = self._orient_anchor[axis_idx]
        delta_angle = self._angle_slider_to_delta(slider_value)
        self.target_pose[:3, :3] = self._compose_orient(R0, self._rot_about(axis_idx, delta_angle))

        self.orient_labels[axis_idx].setText(f"Δ{self._display_angle(delta_angle)}")
        self._move_timer.start(100)

    def _apply_orientation_step(self, axis_idx, direction):
        """
        One discrete orientation step. A complete gesture: acts on the CURRENT
        orientation and leaves no anchor, so repeated steps accumulate.
        """
        step = self._get_angular_step()
        self._finish_orient_gesture(axis_idx)
        R0 = self.target_pose[:3, :3].copy()
        self.target_pose[:3, :3] = self._compose_orient(R0, self._rot_about(axis_idx, direction * step))
        self.orient_labels[axis_idx].setText(f"Δ{self._display_angle(direction * step)}")
        QTimer.singleShot(500, lambda i=axis_idx: self._reset_orient_label(i))

    def _reset_orient_label(self, axis_idx):
        self.orient_labels[axis_idx].setText(f"Δ{self._display_angle(0.0)}")

    def _on_orientation_step(self, axis_idx, direction):
        """Handle [-] or [+] button click for orientation axis."""
        if self.target_pose is None:
            return
        self._apply_orientation_step(axis_idx, direction)
        self._publish_command()

    def _on_orient_slider_released(self, axis_idx):
        """Handle orientation slider release (spring back)."""
        self._move_timer.stop()
        self._publish_command()

        # Spring back to center, and clear only THIS axis's anchor
        self.orient_sliders[axis_idx].blockSignals(True)
        self.orient_sliders[axis_idx].setValue(500)
        self.orient_sliders[axis_idx].blockSignals(False)
        self.orient_labels[axis_idx].setText(f"Δ{self._display_angle(0.0)}")
        self._orient_anchor[axis_idx] = None

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
        """
        Handle wheel event for a slider.

        Each notch is ONE DISCRETE STEP, applied to the current pose in physical
        units -- exactly like a button click. It is not converted into integer
        slider units: that conversion floored at one unit and made a 0.1 mm step
        move 3 mm (30x) and the default 1 mm step move 3 mm (3x).

        The slider handle is moved for visual feedback only, with signals
        blocked, so it never drives the pose.
        """
        if self.target_pose is None:
            return

        delta = event.angleDelta().y()
        key = ('pos' if is_position else 'rot', idx)
        self._wheel_accumulator[key] = self._wheel_accumulator.get(key, 0) + delta

        notch = 120
        burst = self._wheel_burst.get(key, 0.0)
        applied = 0
        while abs(self._wheel_accumulator[key]) >= notch:
            direction = 1 if self._wheel_accumulator[key] > 0 else -1
            self._wheel_accumulator[key] -= direction * notch
            if is_position:
                self._apply_position_step(idx, direction)
                burst += direction * self._get_position_step()
            else:
                self._apply_orientation_step(idx, direction)
                burst += direction * self._get_angular_step()
            applied += 1

        if applied == 0:
            return

        self._wheel_burst[key] = burst
        jog = (not is_position) or self._get_pos_frame() == 'tool'

        if jog:
            # Visual only: show the burst on the handle, recentre when idle.
            if is_position:
                vis = self._position_value_to_slider(burst, is_absolute=False)
                self.position_sliders[idx].blockSignals(True)
                self.position_sliders[idx].setValue(vis)
                self.position_sliders[idx].blockSignals(False)
                self.position_labels[idx].setText(f"Δ{burst:.3f} m")
            else:
                frac = np.clip(burst / self._orient_jog_full_scale, -1.0, 1.0)
                self.orient_sliders[idx].blockSignals(True)
                self.orient_sliders[idx].setValue(int(500 + frac * 500))
                self.orient_sliders[idx].blockSignals(False)
                self.orient_labels[idx].setText(f"Δ{self._display_angle(burst)}")
            self._wheel_active[key] = True
            self._wheel_idle_timer.start(500)

        self._move_timer.start(100)

    def _on_wheel_idle_timeout(self):
        """Recentre jog sliders after a wheel burst. Per axis, not globally."""
        for key, active in list(self._wheel_active.items()):
            if not active:
                continue
            is_position, idx = key[0] == 'pos', key[1]
            if is_position:
                if self._get_pos_frame() != 'tool':
                    self._wheel_active[key] = False
                    continue
                self.position_sliders[idx].blockSignals(True)
                self.position_sliders[idx].setValue(500)
                self.position_sliders[idx].blockSignals(False)
                self.position_labels[idx].setText("Δ0.000 m")
                self._pos_anchor[idx] = None
            else:
                self.orient_sliders[idx].blockSignals(True)
                self.orient_sliders[idx].setValue(500)
                self.orient_sliders[idx].blockSignals(False)
                self.orient_labels[idx].setText(f"Δ{self._display_angle(0.0)}")
                self._orient_anchor[idx] = None
            self._wheel_burst[key] = 0.0
            self._wheel_active[key] = False

    # =================================================================
    # Command Publishing
    # =================================================================

    def _publish_command(self):
        """Publish CARTESIAN_COMMAND event."""
        if self.target_pose is None:
            return

        # Note: frame argument is kept for logging but unused downstream
        # (move_pose ignores it; solve_ik_for_tcp expects world frame)
        self._motion_in_flight = True
        self.state_channel.publish(
            EventType.CARTESIAN_COMMAND,
            data={
                'pose': self.target_pose.copy(),
                'frame': 'base'  # target_pose is always in base frame
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
            self._last_achieved_pose = tcp_pose.copy()
            self._update_all_sliders_from_target()
            self.status_label.setText("Target reset to current")
            QTimer.singleShot(1000, lambda: self.status_label.setText(
                "Auto-move enabled (scroll, drag, or click buttons)"))

    def _update_all_sliders_from_target(self):
        """Update all sliders and labels from target pose."""
        if self.target_pose is None:
            return

        # Position sliders
        pos_frame = self._get_pos_frame()
        for i in range(3):
            self._style_slider(self.position_sliders[i], is_jog=(pos_frame == 'tool'))
            if pos_frame == 'base':
                self._update_position_slider(i, is_absolute=True)
            else:
                # Tool frame: recentre to 0
                self.position_sliders[i].blockSignals(True)
                self.position_sliders[i].setValue(500)
                self.position_sliders[i].blockSignals(False)
                self.position_labels[i].setText("Δ0.000 m")

        # Orientation sliders: always jog, so recentre to 0
        for i in range(3):
            self._style_slider(self.orient_sliders[i], is_jog=True)
            self.orient_sliders[i].blockSignals(True)
            self.orient_sliders[i].setValue(500)
            self.orient_sliders[i].blockSignals(False)
            self.orient_labels[i].setText(f"Δ{self._display_angle(0.0)}")

        # Clear anchors, per axis
        self._pos_anchor = [None, None, None]
        self._orient_anchor = [None, None, None]
        self._wheel_burst = {}
        self._wheel_active = {}
        self._ik_refused = False

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
        self._update_target_display()

    def _on_robot_loaded(self, event):
        """Handle robot loaded event."""
        self._ik_available = self._check_ik_available()
        if self._ik_available:
            self._enable_controls()
        else:
            self._show_no_ik_message()

        self.target_pose = None
        self._last_achieved_pose = None
        self._initialized = False

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

    def _fmt_rotvec(self, M):
        """Rotation vector components, in the user's chosen angular unit."""
        v = R.from_matrix(M).as_rotvec()
        if self._is_degrees():
            v = np.degrees(v)
        return [f"{x:.3f}" for x in v]

    def _fmt_rpy(self, M):
        """Secondary RPY display, convention named, units honoured."""
        e = R.from_matrix(M).as_euler('xyz')
        unit = "deg" if self._is_degrees() else "rad"
        if self._is_degrees():
            e = np.degrees(e)
        return f"RPY {unit}: {e[0]:.1f} {e[1]:.1f} {e[2]:.1f}"

    def _rot_separation(self, A, B):
        """
        True angle between two orientations, in radians.

        Subtracting rotation vectors is NOT a rotation distance: it wraps at
        +/-180 deg, so two poses 2 deg apart (179 vs -179 about X) reported
        6.248 rad and flagged a false divergence.

        The norm of the RELATIVE rotation vector is used rather than
        arccos((trace-1)/2): arccos is ill-conditioned near 0 and 180 deg, where
        it loses about half the significant digits. Measured: a true 1e-6 deg
        separation reads as exactly 0 via arccos, and 1e-6 deg via rotvec.
        """
        return float(np.linalg.norm(R.from_matrix(A.T @ B).as_rotvec()))

    def _update_current_display(self):
        """Refresh the current TCP pose display."""
        try:
            # Motion has settled once the debounce timer is idle. Clear this
            # BEFORE any early return, or an unavailable pose latches the flag
            # true forever and divergence can never be reported.
            if not self._move_timer.isActive():
                self._motion_in_flight = False

            tcp_pose = self._get_tcp_pose_in_base()
            if tcp_pose is None:
                return

            # If target_pose is None, initialize from current
            if self.target_pose is None:
                self.target_pose = tcp_pose.copy()
                self._last_achieved_pose = tcp_pose.copy()
                self._update_all_sliders_from_target()
                self._initialized = True

            for i in range(3):
                self.pos_current_vals[i].setText(f"{tcp_pose[i, 3]:.3f}")
            for i, txt in enumerate(self._fmt_rotvec(tcp_pose[:3, :3])):
                self.orient_current_vals[i].setText(txt)
            self.orient_current_rpy.setText(self._fmt_rpy(tcp_pose[:3, :3]))

            self._last_achieved_pose = tcp_pose.copy()

        except Exception as e:
            logger.debug(f"[Cartesian] _update_current_display exception: {e}")

    def _update_target_display(self):
        """Refresh the target TCP pose display."""
        if self.target_pose is None:
            return

        try:
            unit = "deg" if self._is_degrees() else "rad"
            self.orient_group.setTitle(f"Orientation  (readout: {unit})")

            for i in range(3):
                self.pos_target_vals[i].setText(f"{self.target_pose[i, 3]:.3f}")
            for i, txt in enumerate(self._fmt_rotvec(self.target_pose[:3, :3])):
                self.orient_target_vals[i].setText(txt)
            self.orient_target_rpy.setText(self._fmt_rpy(self.target_pose[:3, :3]))

            if self._last_achieved_pose is None:
                return

            pos_diff = np.linalg.norm(
                self.target_pose[:3, 3] - self._last_achieved_pose[:3, 3])
            rot_diff = self._rot_separation(
                self._last_achieved_pose[:3, :3], self.target_pose[:3, :3])

            separated = pos_diff > 0.01 or rot_diff > np.radians(5.7)
            # While a move is still in flight the target legitimately leads the
            # arm; that is not a failure and must not look like one.
            diverged = self._ik_refused or (separated and not self._motion_in_flight)

            if diverged:
                style = ("QLabel { color: #e74c3c; font-family: monospace; "
                         "font-size: 10px; font-weight: bold; }")
            else:
                style = ("QLabel { color: #e67e22; font-family: monospace; "
                         "font-size: 10px; }")
            for val in self.pos_target_vals + self.orient_target_vals:
                val.setStyleSheet(style)

            if self._ik_refused:
                self.status_label.setText("IK refused the target — arm did not move")
                self.status_label.setStyleSheet("QLabel { color: #e74c3c; font-weight: bold; }")
            elif not diverged:
                self.status_label.setText("Auto-move enabled (scroll, drag, or click buttons)")
                self.status_label.setStyleSheet("QLabel { color: #666; }")

        except Exception as e:
            logger.debug(f"[Cartesian] _update_target_display exception: {e}")

    def _on_error(self, event):
        """An IK refusal is a fact, not something to infer from divergence."""
        data = getattr(event, 'data', {}) or {}
        src = getattr(event, 'source', '')
        if src in ('simulated_robot', 'real_robot') or 'IK' in str(data.get('error', '')):
            self._ik_refused = True
            self._motion_in_flight = False
            logger.warning("[Cartesian] %s", data.get('error', 'IK error'))
            self._update_target_display()

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
                        self._last_achieved_pose = tcp_pose.copy()
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
    # Unit Changed
    # =================================================================

    def _on_unit_changed(self):
        """Handle degrees/radians toggle."""
        # Readouts and the jog delta labels both carry angular units.
        for i in range(3):
            self.orient_labels[i].setText(f"Δ{self._display_angle(0.0)}")
        self._update_current_display()
        self._update_target_display()

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
        self.state_channel.unsubscribe(EventType.ERROR_OCCURRED, self._on_error)
        self._move_timer.stop()
        self._wheel_idle_timer.stop()
