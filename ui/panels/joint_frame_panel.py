"""
Joint Frame Panel - Toggle visibility and show pose data.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QCheckBox, 
                             QPushButton, QLabel, QScrollArea)
from PyQt5.QtCore import Qt, QTimer


class JointFramePanel(QWidget):
    """Panel with checkboxes and pose data for each joint frame."""

    def __init__(self, joint_display, parent=None):
        super().__init__(parent)
        self.joint_display = joint_display
        self.checkboxes = {}
        self.pose_labels = {}
        self._setup_ui()
        
        # Timer to refresh pose data
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._refresh_poses)
        self._update_timer.start(100)  # 10 Hz

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        # Title
        title = QLabel("Joint Frames")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Show All / Hide All
        btn_layout = QVBoxLayout()
        show_all = QPushButton("Show All")
        show_all.clicked.connect(self._show_all)
        btn_layout.addWidget(show_all)
        hide_all = QPushButton("Hide All")
        hide_all.clicked.connect(self._hide_all)
        btn_layout.addWidget(hide_all)
        layout.addLayout(btn_layout)

        # TCP checkbox
        self.tcp_cb = QCheckBox("TCP (tool center point)")
        self.tcp_cb.setChecked(False)
        self.tcp_cb.stateChanged.connect(
            lambda state: self.joint_display.set_tcp_visible(state == Qt.Checked)
        )
        layout.addWidget(self.tcp_cb)
        
        # TCP pose label
        self.tcp_pose_label = QLabel("")
        self.tcp_pose_label.setStyleSheet("font-size: 9px; color: #666; padding-left: 20px;")
        self.tcp_pose_label.setVisible(False)
        layout.addWidget(self.tcp_pose_label)

        # Scrollable joint list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        checkbox_widget = QWidget()
        checkbox_layout = QVBoxLayout(checkbox_widget)
        checkbox_layout.setSpacing(1)

        joint_names = self.joint_display.get_joint_names()
        for i, name in enumerate(joint_names):
            # Checkbox
            cb = QCheckBox(f"J{i+1}: {name}")
            cb.setChecked(False)
            cb.stateChanged.connect(
                lambda state, n=name: self._on_toggle(n, state)
            )
            checkbox_layout.addWidget(cb)
            self.checkboxes[name] = cb
            
            # Pose label (hidden until checked)
            pose_label = QLabel("")
            pose_label.setStyleSheet("font-size: 9px; color: #666; padding-left: 20px;")
            pose_label.setVisible(False)
            checkbox_layout.addWidget(pose_label)
            self.pose_labels[name] = pose_label

        checkbox_layout.addStretch()
        scroll.setWidget(checkbox_widget)
        layout.addWidget(scroll)
        layout.addStretch()

    def _on_toggle(self, joint_name, state):
        """Show or hide a joint frame and its pose data."""
        visible = state == Qt.Checked
        self.joint_display.set_joint_visible(joint_name, visible)
        if joint_name in self.pose_labels:
            self.pose_labels[joint_name].setVisible(visible)
        # Also update TCP label visibility
        # self.tcp_pose_label.setVisible(self.tcp_cb.isChecked())
        self.tcp_pose_label.setVisible(True)

    def _show_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(True)
        self.tcp_cb.setChecked(True)

    def _hide_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(False)
        self.tcp_cb.setChecked(False)

    def _refresh_poses(self):
        """Update pose labels for all visible frames."""
        from scipy.spatial.transform import Rotation as R
        
        poses = self.joint_display.get_frame_poses()
        
        for joint_name, label in self.pose_labels.items():
            if label.isVisible() and joint_name in poses:
                T = poses[joint_name]
                pos = T[:3, 3]
                rotvec = R.from_matrix(T[:3, :3]).as_rotvec()
                rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
                label.setText(
                    f"  pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})\n"
                    f"  rot: ({rotvec[0]:.3f}, {rotvec[1]:.3f}, {rotvec[2]:.3f})\n"
                    f"  rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
                )
        
        # TCP pose
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