"""
Joint Frame Panel - Toggle visibility and show pose data for all frames.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QCheckBox, 
                             QPushButton, QLabel, QScrollArea,
                             QDoubleSpinBox, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import Qt, QTimer


class JointFramePanel(QWidget):
    """Panel with checkboxes, pose data, and appearance controls."""

    def __init__(self, joint_display, state_channel=None, parent=None):
        super().__init__(parent)
        self.joint_display = joint_display
        self.state_channel = state_channel
        self.checkboxes = {}
        self.pose_labels = {}
        self._setup_ui()
        
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._refresh_poses)
        self._update_timer.start(100)
        
        if self.state_channel:
            from core.world_state.event_types import EventType
            self.state_channel.subscribe(EventType.ROBOT_STATE, lambda e: self._refresh_poses())

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        title = QLabel("Joint Frames")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Show All / Hide All
        btn_layout = QHBoxLayout()
        show_all = QPushButton("Show All")
        show_all.clicked.connect(self._show_all)
        btn_layout.addWidget(show_all)
        hide_all = QPushButton("Hide All")
        hide_all.clicked.connect(self._hide_all)
        btn_layout.addWidget(hide_all)
        layout.addLayout(btn_layout)

        # Appearance controls
        appearance = QGroupBox("Appearance")
        app_layout = QVBoxLayout()
        
        # Scale
        scale_row = QHBoxLayout()
        scale_row.addWidget(QLabel("Scale:"))
        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setRange(0.01, 1.0)
        self.scale_spin.setValue(0.1)
        self.scale_spin.setSingleStep(0.02)
        self.scale_spin.valueChanged.connect(self._on_scale_changed)
        scale_row.addWidget(self.scale_spin)
        app_layout.addLayout(scale_row)
        
        # Thickness
        thick_row = QHBoxLayout()
        thick_row.addWidget(QLabel("Thickness:"))
        self.thick_spin = QDoubleSpinBox()
        self.thick_spin.setRange(0.002, 0.05)
        self.thick_spin.setValue(0.008)
        self.thick_spin.setSingleStep(0.002)
        self.thick_spin.setDecimals(3)
        self.thick_spin.valueChanged.connect(self._on_thickness_changed)
        thick_row.addWidget(self.thick_spin)
        app_layout.addLayout(thick_row)
        
        appearance.setLayout(app_layout)
        layout.addWidget(appearance)

        # TCP checkbox
        self.tcp_cb = QCheckBox("TCP (tool center point)")
        self.tcp_cb.setChecked(False)
        self.tcp_cb.stateChanged.connect(
            lambda state: self.joint_display.set_tcp_visible(state == Qt.Checked)
        )
        layout.addWidget(self.tcp_cb)
        
        self.tcp_pose_label = QLabel("")
        self.tcp_pose_label.setStyleSheet("font-size: 8px; color: #666; padding-left: 20px;")
        self.tcp_pose_label.setVisible(False)
        layout.addWidget(self.tcp_pose_label)

        # Scrollable frame list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        checkbox_widget = QWidget()
        checkbox_layout = QVBoxLayout(checkbox_widget)
        checkbox_layout.setSpacing(0)

        frame_names = self.joint_display.get_joint_names()
        for display_name in frame_names:
            # Extract the actual name (without J:/L: prefix)
            name = display_name.split(": ", 1)[1] if ": " in display_name else display_name
            
            cb = QCheckBox(display_name)
            cb.setChecked(False)
            cb.stateChanged.connect(
                lambda state, n=name: self._on_toggle(n, state)
            )
            checkbox_layout.addWidget(cb)
            self.checkboxes[name] = cb
            
            pose_label = QLabel("")
            pose_label.setStyleSheet("font-size: 8px; color: #666; padding-left: 20px;")
            pose_label.setVisible(False)
            checkbox_layout.addWidget(pose_label)
            self.pose_labels[name] = pose_label

        checkbox_layout.addStretch()
        scroll.setWidget(checkbox_widget)
        layout.addWidget(scroll)
        layout.addStretch()

    def _on_toggle(self, name, state):
        visible = state == Qt.Checked
        self.joint_display.set_joint_visible(name, visible)
        if name in self.pose_labels:
            self.pose_labels[name].setVisible(visible)
        self.tcp_pose_label.setVisible(self.tcp_cb.isChecked())

    def _show_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(True)
        self.tcp_cb.setChecked(True)

    def _hide_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(False)
        self.tcp_cb.setChecked(False)

    def _on_scale_changed(self, value):
        self.joint_display.set_scale(value)

    def _on_thickness_changed(self, value):
        self.joint_display.set_thickness(value)

    def _refresh_poses(self):
        from scipy.spatial.transform import Rotation as R
        
        poses = self.joint_display.get_frame_poses()
        
        for name, label in self.pose_labels.items():
            if label.isVisible() and name in poses:
                T = poses[name]
                pos = T[:3, 3]
                rotvec = R.from_matrix(T[:3, :3]).as_rotvec()
                rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
                label.setText(
                    f"  pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})\n"
                    f"  rot: ({rotvec[0]:.3f}, {rotvec[1]:.3f}, {rotvec[2]:.3f})\n"
                    f"  rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
                )
        
        if self.tcp_pose_label.isVisible():
            tcp_pose = self.joint_display.get_tcp_pose()
            if tcp_pose is not None:
                pos = tcp_pose[:3, 3]
                rotvec = R.from_matrix(tcp_pose[:3, :3]).as_rotvec()
                rpy = R.from_matrix(tcp_pose[:3, :3]).as_euler('xyz', degrees=True)
                self.tcp_pose_label.setText(
                    f"  pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})\n"
                    f"  rot: ({rotvec[0]:.3f}, {rotvec[1]:.3f}, {rotvec[2]:.3f})\n"
                    f"  rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
                )