"""
View Controls Panel - Provides buttons for standard camera views.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout,
                             QPushButton, QGroupBox, QLabel,
                             QToolBar, QAction)
from PyQt5.QtCore import Qt, pyqtSignal

class ViewControlsPanel(QWidget):
    """Panel with buttons for standard camera views."""
    
    # Signals for different view changes
    view_requested = pyqtSignal(str)  # View name: 'top', 'front', 'side', 'isometric'
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # View controls group
        view_group = QGroupBox("Camera Views")
        view_layout = QVBoxLayout()
        
        # Top view button
        self.top_btn = QPushButton("🔝 Top View")
        self.top_btn.setToolTip("View from top (Z-axis)")
        self.top_btn.clicked.connect(lambda: self.view_requested.emit('top'))
        self.top_btn.setStyleSheet(self._get_button_style())
        view_layout.addWidget(self.top_btn)
        
        # Front view button
        self.front_btn = QPushButton("⬆️ Front View")
        self.front_btn.setToolTip("View from front (Y-axis)")
        self.front_btn.clicked.connect(lambda: self.view_requested.emit('front'))
        self.front_btn.setStyleSheet(self._get_button_style())
        view_layout.addWidget(self.front_btn)
        
        # Side view button
        self.side_btn = QPushButton("⬅️ Side View")
        self.side_btn.setToolTip("View from side (X-axis)")
        self.side_btn.clicked.connect(lambda: self.view_requested.emit('side'))
        self.side_btn.setStyleSheet(self._get_button_style())
        view_layout.addWidget(self.side_btn)
        
        # Isometric view button
        self.iso_btn = QPushButton("🔲 Isometric View")
        self.iso_btn.setToolTip("Isometric (3/4) view")
        self.iso_btn.clicked.connect(lambda: self.view_requested.emit('isometric'))
        self.iso_btn.setStyleSheet(self._get_button_style())
        view_layout.addWidget(self.iso_btn)
        
        view_group.setLayout(view_layout)
        layout.addWidget(view_group)
        
        # Camera controls group
        camera_group = QGroupBox("Camera Controls")
        camera_layout = QVBoxLayout()
        
        # Reset view button
        self.reset_btn = QPushButton("🔄 Reset View")
        self.reset_btn.setToolTip("Reset camera to default position")
        self.reset_btn.clicked.connect(lambda: self.view_requested.emit('reset'))
        self.reset_btn.setStyleSheet(self._get_button_style(reset=True))
        camera_layout.addWidget(self.reset_btn)
        
        # Zoom to fit button
        self.fit_btn = QPushButton("🔍 Zoom to Fit")
        self.fit_btn.setToolTip("Zoom to fit all objects in view")
        self.fit_btn.clicked.connect(lambda: self.view_requested.emit('fit'))
        self.fit_btn.setStyleSheet(self._get_button_style())
        camera_layout.addWidget(self.fit_btn)
        
        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)
        
        # Help text
        help_label = QLabel("💡 Tip: You can also use the\nView menu for more options")
        help_label.setStyleSheet("color: gray; font-size: 10px; padding: 5px;")
        help_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(help_label)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _get_button_style(self, reset=False):
        """Get button stylesheet."""
        if reset:
            return """
                QPushButton {
                    background-color: #f0f0f0;
                    border: 1px solid #999;
                    border-radius: 3px;
                    padding: 8px;
                    font-weight: bold;
                    color: #333;
                }
                QPushButton:hover {
                    background-color: #e0e0e0;
                    border-color: #666;
                }
                QPushButton:pressed {
                    background-color: #d0d0d0;
                }
            """
        else:
            return """
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    border-radius: 3px;
                    padding: 8px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:pressed {
                    background-color: #3d8b40;
                }
            """


class ViewToolBar(QToolBar):
    """Toolbar with view control buttons."""
    
    view_requested = pyqtSignal(str)
    
    def __init__(self, title="View Controls", parent=None):
        super().__init__(title, parent)
        self.setup_actions()
    
    def setup_actions(self):
        """Create toolbar actions."""
        # Top view
        self.top_action = QAction("🔝 Top", self)
        self.top_action.setToolTip("Top view (Ctrl+1)")
        self.top_action.setShortcut("Ctrl+1")
        self.top_action.triggered.connect(lambda: self.view_requested.emit('top'))
        self.addAction(self.top_action)
        
        # Front view
        self.front_action = QAction("⬆️ Front", self)
        self.front_action.setToolTip("Front view (Ctrl+2)")
        self.front_action.setShortcut("Ctrl+2")
        self.front_action.triggered.connect(lambda: self.view_requested.emit('front'))
        self.addAction(self.front_action)
        
        # Side view
        self.side_action = QAction("⬅️ Side", self)
        self.side_action.setToolTip("Side view (Ctrl+3)")
        self.side_action.setShortcut("Ctrl+3")
        self.side_action.triggered.connect(lambda: self.view_requested.emit('side'))
        self.addAction(self.side_action)
        
        # Isometric view
        self.iso_action = QAction("🔲 Isometric", self)
        self.iso_action.setToolTip("Isometric view (Ctrl+4)")
        self.iso_action.setShortcut("Ctrl+4")
        self.iso_action.triggered.connect(lambda: self.view_requested.emit('isometric'))
        self.addAction(self.iso_action)
        
        self.addSeparator()
        
        # Reset view
        self.reset_action = QAction("🔄 Reset", self)
        self.reset_action.setToolTip("Reset view (Ctrl+0)")
        self.reset_action.setShortcut("Ctrl+0")
        self.reset_action.triggered.connect(lambda: self.view_requested.emit('reset'))
        self.addAction(self.reset_action)
        
        # Zoom to fit
        self.fit_action = QAction("🔍 Fit", self)
        self.fit_action.setToolTip("Zoom to fit (Ctrl+F)")
        self.fit_action.setShortcut("Ctrl+F")
        self.fit_action.triggered.connect(lambda: self.view_requested.emit('fit'))
        self.addAction(self.fit_action)