"""
Grid Control Panel - Advanced grid controls widget.
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, 
                             QGroupBox, QColorDialog)
from PyQt5.QtCore import Qt


class GridControlPanel(QWidget):
    """Advanced grid control panel with size and color controls."""
    
    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Grid size selection
        size_group = QGroupBox("Grid Size")
        size_layout = QVBoxLayout()
        
        self.grid_size_combo = QComboBox()
        self.grid_size_combo.addItems([
            "10 mm",
            "50 mm", 
            "100 mm",
            "500 mm",
            "1.0 m"
        ])
        self.grid_size_combo.setCurrentText("100 mm")  # Default
        self.grid_size_combo.currentTextChanged.connect(self._on_size_changed)
        size_layout.addWidget(self.grid_size_combo)
        size_group.setLayout(size_layout)
        layout.addWidget(size_group)
        
        # Grid color selection
        color_group = QGroupBox("Grid Color")
        color_layout = QVBoxLayout()
        
        # Color preview button
        color_btn_layout = QHBoxLayout()
        self.color_preview = QLabel()
        self.color_preview.setFixedSize(30, 20)
        self.color_preview.setStyleSheet("background-color: lightgray; border: 1px solid black;")
        color_btn_layout.addWidget(self.color_preview)
        
        self.color_btn = QPushButton("Choose Color...")
        self.color_btn.clicked.connect(self._choose_color)
        color_btn_layout.addWidget(self.color_btn)
        color_layout.addLayout(color_btn_layout)
        
        # Preset colors
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("Presets:"))
        
        self.gray_preset = QPushButton("Gray")
        self.gray_preset.setStyleSheet("background-color: #808080; color: white;")
        self.gray_preset.clicked.connect(lambda: self._set_color((0.5, 0.5, 0.5)))
        preset_layout.addWidget(self.gray_preset)
        
        self.white_preset = QPushButton("White")
        self.white_preset.setStyleSheet("background-color: white; color: black;")
        self.white_preset.clicked.connect(lambda: self._set_color((1.0, 1.0, 1.0)))
        preset_layout.addWidget(self.white_preset)
        
        self.black_preset = QPushButton("Black")
        self.black_preset.setStyleSheet("background-color: black; color: white;")
        self.black_preset.clicked.connect(lambda: self._set_color((0.0, 0.0, 0.0)))
        preset_layout.addWidget(self.black_preset)
        
        color_layout.addLayout(preset_layout)
        color_group.setLayout(color_layout)
        layout.addWidget(color_group)
        
        # Apply button
        self.apply_btn = QPushButton("Apply Grid Settings")
        self.apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 8px;
                font-weight: bold;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        self.apply_btn.clicked.connect(self._apply_settings)
        layout.addWidget(self.apply_btn)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _on_size_changed(self, text):
        """Handle grid size combo box change."""
        # Auto-apply if desired, or wait for apply button
        pass
    
    def _set_color(self, color_rgb):
        """Set the color and update preview."""
        r, g, b = color_rgb
        color_hex = f"#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}"
        self.color_preview.setStyleSheet(f"background-color: {color_hex}; border: 1px solid black;")
        self._apply_color(color_rgb)
    
    def _choose_color(self):
        """Open color dialog."""
        color = QColorDialog.getColor(initial=Qt.lightGray, parent=self, title="Choose Grid Color")
        if color.isValid():
            rgb = (color.red()/255.0, color.green()/255.0, color.blue()/255.0)
            self._set_color(rgb)
    
    def _apply_color(self, color_rgb):
        """Apply color to grid."""
        self.engine.set_grid_color(color_rgb)
    
    def _apply_settings(self):
        """Apply all grid settings."""
        # Get size from combo box
        size_text = self.grid_size_combo.currentText()
        size_map = {
            "10 mm": 0.01,
            "50 mm": 0.05,
            "100 mm": 0.1,
            "500 mm": 0.5,
            "1.0 m": 1.0
        }
        size = size_map[size_text]
        
        # Apply size
        self.engine.set_grid_size(size)
        
        # Color is applied immediately via _set_color