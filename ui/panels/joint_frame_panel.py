"""
Joint Frame Panel - Toggle visibility of joint coordinate frames.

Lists all joints in the kinematic chain with checkboxes.
Joint frames are hidden by default.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QCheckBox, 
                             QPushButton, QLabel, QScrollArea)
from PyQt5.QtCore import Qt


class JointFramePanel(QWidget):
    """Panel with checkboxes for each joint frame."""

    def __init__(self, joint_display, parent=None):
        super().__init__(parent)
        self.joint_display = joint_display
        self.checkboxes = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Title
        title = QLabel("Joint Frames")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Show All / Hide All buttons
        btn_layout = QVBoxLayout()

        show_all = QPushButton("Show All")
        show_all.clicked.connect(self._show_all)
        btn_layout.addWidget(show_all)

        hide_all = QPushButton("Hide All")
        hide_all.clicked.connect(self._hide_all)
        btn_layout.addWidget(hide_all)

        layout.addLayout(btn_layout)

        # TCP checkbox (separate from joint list)
        self.tcp_cb = QCheckBox("TCP (tool center point)")
        self.tcp_cb.setChecked(False)
        self.tcp_cb.stateChanged.connect(
            lambda state: self.joint_display.set_tcp_visible(state == Qt.Checked)
        )
        layout.addWidget(self.tcp_cb)

        # Scrollable checkbox list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        checkbox_widget = QWidget()
        checkbox_layout = QVBoxLayout(checkbox_widget)
        checkbox_layout.setSpacing(2)

        joint_names = self.joint_display.get_joint_names()
        for i, name in enumerate(joint_names):
            cb = QCheckBox(f"J{i+1}: {name}")
            cb.setChecked(False)
            cb.stateChanged.connect(
                lambda state, n=name: self._on_toggle(n, state)
            )
            checkbox_layout.addWidget(cb)
            self.checkboxes[name] = cb

        checkbox_layout.addStretch()
        scroll.setWidget(checkbox_widget)
        layout.addWidget(scroll)
        layout.addStretch()

    def _on_toggle(self, joint_name, state):
        """Show or hide a single joint frame."""
        self.joint_display.set_joint_visible(joint_name, state == Qt.Checked)

    def _show_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(True)

    def _hide_all(self):
        for cb in self.checkboxes.values():
            cb.setChecked(False)