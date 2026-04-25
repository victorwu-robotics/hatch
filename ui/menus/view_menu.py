"""
View Menu - Handles view options including grid settings and camera views.
"""

from PyQt5.QtWidgets import QAction, QMenu
from PyQt5.QtCore import Qt


class ViewMenu:
    """View menu with grid controls, visibility toggles, and camera views."""
    
    def __init__(self, parent_window, mesh_loader, camera_manager, engine):
        """
        Initialize view menu.
        
        Args:
            parent_window: The main window
            mesh_loader: The asset manager instance
            camera_manager: The camera manager instance
            engine: The visualizer engine
        """
        self.parent = parent_window
        self.mesh_loader = mesh_loader
        self.camera_manager = camera_manager
        self.engine = engine
        
        # Store menu actions
        self.actions = []
        self.grid_size_actions = {}
        self.grid_color_actions = {}
        
        self._create_actions()
    
    def _create_actions(self):
        """Create all view menu actions."""
        # ===== CAMERA VIEWS SECTION =====
        views_menu = QMenu("Camera Views", self.parent)
        
        top_view = QAction("🔝 Top View (Ctrl+1)", self.parent)
        top_view.setShortcut("Ctrl+1")
        top_view.triggered.connect(lambda: self.engine.set_view('top'))
        views_menu.addAction(top_view)
        
        front_view = QAction("⬆️ Front View (Ctrl+2)", self.parent)
        front_view.setShortcut("Ctrl+2")
        front_view.triggered.connect(lambda: self.engine.set_view('front'))
        views_menu.addAction(front_view)
        
        side_view = QAction("⬅️ Side View (Ctrl+3)", self.parent)
        side_view.setShortcut("Ctrl+3")
        side_view.triggered.connect(lambda: self.engine.set_view('side'))
        views_menu.addAction(side_view)
        
        iso_view = QAction("🔲 Isometric View (Ctrl+4)", self.parent)
        iso_view.setShortcut("Ctrl+4")
        iso_view.triggered.connect(lambda: self.engine.set_view('isometric'))
        views_menu.addAction(iso_view)
        
        views_menu.addSeparator()
        
        reset_view = QAction("🔄 Reset View (Ctrl+0)", self.parent)
        reset_view.setShortcut("Ctrl+0")
        reset_view.triggered.connect(lambda: self.engine.set_view('reset'))
        views_menu.addAction(reset_view)
        
        fit_view = QAction("🔍 Zoom to Fit (Ctrl+F)", self.parent)
        fit_view.setShortcut("Ctrl+F")
        fit_view.triggered.connect(lambda: self.engine.set_view('fit'))
        views_menu.addAction(fit_view)
        
        self.actions.append(views_menu.menuAction())
        
        # Separator
        separator1 = QAction(self.parent)
        separator1.setSeparator(True)
        self.actions.append(separator1)
        
        # ===== GRID SETTINGS SECTION =====
        grid_menu = QMenu("Grid Settings", self.parent)
        
        # Grid Size submenu
        size_menu = grid_menu.addMenu("Grid Size")
        self._create_grid_size_actions(size_menu)
        
        # Grid Color submenu
        color_menu = grid_menu.addMenu("Grid Color")
        self._create_grid_color_actions(color_menu)
        
        # Show Grid Controls
        show_grid_controls = QAction("Show Grid Controls", self.parent)
        show_grid_controls.setCheckable(True)
        show_grid_controls.triggered.connect(
            lambda checked: self.parent.ui_builder.toggle_grid_controls(checked)
        )
        grid_menu.addSeparator()
        grid_menu.addAction(show_grid_controls)
        
        # Add grid menu to main actions
        self.actions.append(grid_menu.menuAction())
        
        # Separator after grid settings
        separator2 = QAction(self.parent)
        separator2.setSeparator(True)
        self.actions.append(separator2)
        
        # ===== FULLSCREEN =====
        self.fullscreen_action = QAction("&Fullscreen (F11)", self.parent)
        self.fullscreen_action.setShortcut("F11")
        self.fullscreen_action.setCheckable(True)
        self.fullscreen_action.triggered.connect(self._toggle_fullscreen)
        self.actions.append(self.fullscreen_action)
    
    def _create_grid_size_actions(self, menu):
        """Create grid size menu items."""
        size_options = {
            "10 mm": 0.01,
            "50 mm": 0.05,
            "100 mm": 0.1,
            "500 mm": 0.5,
            "1.0 m": 1.0
        }
        
        for label, size in size_options.items():
            action = QAction(label, self.parent)
            action.setCheckable(True)
            action.setChecked(size == 0.1)  # Default 100mm
            action.triggered.connect(lambda checked, s=size: self._set_grid_size(s))
            menu.addAction(action)
            self.grid_size_actions[size] = action
    
    def _create_grid_color_actions(self, menu):
        """Create grid color menu items."""
        colors = {
            "Light Gray": (0.8, 0.8, 0.8),
            "Dark Gray": (0.2, 0.2, 0.2),
            "White": (1.0, 1.0, 1.0),
            "Black": (0.0, 0.0, 0.0),
            "Red": (1.0, 0.0, 0.0),
            "Green": (0.0, 1.0, 0.0),
            "Blue": (0.0, 0.0, 1.0),
            "Yellow": (1.0, 1.0, 0.0),
            "Cyan": (0.0, 1.0, 1.0),
            "Magenta": (1.0, 0.0, 1.0)
        }
        
        for label, color in colors.items():
            action = QAction(label, self.parent)
            action.setCheckable(True)
            action.setChecked(label == "Light Gray")  # Default
            action.triggered.connect(lambda checked, c=color: self._set_grid_color(c))
            menu.addAction(action)
            self.grid_color_actions[label] = action
        
        # Custom color option
        menu.addSeparator()
        custom_action = QAction("Custom Color...", self.parent)
        custom_action.triggered.connect(self._choose_grid_color)
        menu.addAction(custom_action)
    
    def _set_grid_size(self, size_meters: float):
        """Set the grid size and update check marks."""
        self.engine.set_grid_size(size_meters)
        
        # Update check marks
        for size, action in self.grid_size_actions.items():
            action.setChecked(size == size_meters)
    
    def _set_grid_color(self, color_rgb: tuple):
        """Set the grid color and update check marks."""
        self.engine.set_grid_color(color_rgb)
        
        # Update check marks
        color_map = {
            (0.8, 0.8, 0.8): "Light Gray",
            (0.2, 0.2, 0.2): "Dark Gray",
            (1.0, 1.0, 1.0): "White",
            (0.0, 0.0, 0.0): "Black",
            (1.0, 0.0, 0.0): "Red",
            (0.0, 1.0, 0.0): "Green",
            (0.0, 0.0, 1.0): "Blue",
            (1.0, 1.0, 0.0): "Yellow",
            (0.0, 1.0, 1.0): "Cyan",
            (1.0, 0.0, 1.0): "Magenta"
        }
        
        for label, action in self.grid_color_actions.items():
            action.setChecked(label == color_map.get(color_rgb, ""))
    
    def _choose_grid_color(self):
        """Open color dialog for custom grid color."""
        from PyQt5.QtWidgets import QColorDialog
        
        color = QColorDialog.getColor(initial=Qt.lightGray, parent=self.parent, title="Choose Grid Color")
        if color.isValid():
            rgb = (color.red()/255.0, color.green()/255.0, color.blue()/255.0)
            self._set_grid_color(rgb)
    
    def _toggle_fullscreen(self, checked):
        """Toggle fullscreen mode."""
        if checked:
            self.parent.showFullScreen()
        else:
            self.parent.showNormal()
    
    def update(self):
        """Update menu based on current state."""
        # Update robot visibility actions
        # This would be implemented to show/hide robot visibility toggles
        pass
    
    def get_actions(self):
        """Get all view menu actions."""
        return self.actions