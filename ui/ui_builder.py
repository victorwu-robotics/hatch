"""
UI Manager - Handles menu creation and UI component management.
"""

from PyQt5.QtWidgets import QAction, QMessageBox, QWidget, QDockWidget
from PyQt5.QtCore import Qt

from ui.menus.file_menu import FileMenu
from ui.menus.view_menu import ViewMenu
from ui.menus.robots_menu import RobotsMenu
from ui.menus.camera_menu import CameraMenu
from ui.panels.grid_control_panel import GridControlPanel
from ui.panels.view_controls_panel import ViewControlsPanel, ViewToolBar


class UIBuilder:
    """
    Manages UI components: menus, dock widgets, and status bar.
    """
    
    def __init__(self, parent_window, state_channel, robot_manager, camera_manager, engine):
        """
        Initialize the UI manager.

        Args:
            parent_window: The main window
            mesh_loader: The mesh loader instance
            camera_manager: The camera manager instance
            engine: The visualizer engine
        """
        self.parent = parent_window
        self.state_channel = state_channel
        self.robot_manager = robot_manager
        self.camera_manager = camera_manager
        self.engine = engine
        
        # Menu instances
        self.file_menu = None
        self.view_menu = None
        self.robots_menu = None
        self.camera_menu = None
        self.help_menu = None
        
        # Grid control panel
        self.grid_control_panel = None
        self.grid_control_dock = None
        
        # View controls
        self.view_toolbar = None
        self.view_controls_dock = None
        
        self._setup_menus()
        self._setup_toolbars()
        self._create_grid_control_dock()
        self._create_view_controls_dock()
        
        print("UIBuilder: Initialized")
    
    def _setup_menus(self):
        """Setup all menus."""
        menubar = self.parent.menuBar()
        
        # File Menu
        self.file_menu = FileMenu(self.parent, self.robot_manager, self.state_channel)  # Still needs it
        file_menu = menubar.addMenu("&File")
        for action in self.file_menu.get_actions():
            file_menu.addAction(action)
        
        # View Menu
        # self.view_menu = ViewMenu(self.parent, self.camera_manager, self.engine)
        self.view_menu = ViewMenu(self.parent, self.engine)
        view_menu = menubar.addMenu("&View")
        for action in self.view_menu.get_actions():
            view_menu.addAction(action)
        
        # Robots Menu
        self.robots_menu = RobotsMenu(self.parent, self.state_channel)
        menubar.addMenu(self.robots_menu)   # Directly add, not separate
        
        # Camera Menu
        self.camera_menu = CameraMenu(self.parent, self.camera_manager)
        camera_menu = menubar.addMenu("&Camera")
        for action in self.camera_menu.get_actions():
            camera_menu.addAction(action)

        # Help Menu
        self.help_menu = menubar.addMenu("&Help")
        about_action = QAction("&About", self.parent)
        about_action.triggered.connect(self._show_about)
        self.help_menu.addAction(about_action)
    
    def _setup_toolbars(self):
        """Setup toolbars."""
        # View toolbar
        self.view_toolbar = ViewToolBar("View Controls", self.parent)
        self.view_toolbar.view_requested.connect(self._on_view_requested)
        self.parent.addToolBar(Qt.TopToolBarArea, self.view_toolbar)
    
    def _create_view_controls_dock(self):
        """Create a dock widget with view control buttons."""
        from PyQt5.QtWidgets import QDockWidget
        
        self.view_controls_panel = ViewControlsPanel(self.parent)
        self.view_controls_panel.view_requested.connect(self._on_view_requested)
        
        self.view_controls_dock = QDockWidget("View Controls", self.parent)
        self.view_controls_dock.setWidget(self.view_controls_panel)
        self.parent.addDockWidget(Qt.LeftDockWidgetArea, self.view_controls_dock)
    
    def _on_view_requested(self, view_name: str):
        """Handle view requests from UI components."""
        self.engine.set_view(view_name)
    
    def _create_grid_control_dock(self):
        """Create a dock widget with advanced grid controls."""
        from PyQt5.QtWidgets import QDockWidget
        
        self.grid_control_panel = GridControlPanel(self.engine, self.parent)
        
        self.grid_control_dock = QDockWidget("Grid Controls", self.parent)
        self.grid_control_dock.setWidget(self.grid_control_panel)
        self.parent.addDockWidget(Qt.RightDockWidgetArea, self.grid_control_dock)
        self.grid_control_dock.hide()
    
    def toggle_grid_controls(self, checked: bool):
        """Show/hide the grid control dock."""
        if self.grid_control_dock:
            self.grid_control_dock.setVisible(checked)
    
    def _on_assets_changed(self):
        """Called when assets are loaded or unloaded."""
        self.update_menus()
    
    def update_menus(self):
        """Update all menus based on current state."""
        self._update_robots_menu()
        self.view_menu.update()
        self.robots_menu.update()
        
        # Update file menu close action
        if hasattr(self.file_menu, 'close_robot_action'):
            self.file_menu.close_robot_action.setEnabled(self.mesh_loader.has_assets())
    
    def _show_about(self):
        """Show about dialog."""
        QMessageBox.about(
            self.parent,
            "About RoboPlatform",
            "<h2>RoboPlatform - Digital Twin Visualizer</h2>"
            "<p>A pure VTK + URDF robot visualization platform.</p>"
            "<p><b>Version:</b> 2.0.0</p>"
            "<p><b>Features:</b><br>"
            "• Three-thread point cloud pipeline<br>"
            "• Transform Registry (no TF overhead)<br>"
            "• Event-driven State Channel<br>"
            "• Direct VTK rendering<br>"
            "• Pure Python URDF parsing<br>"
            "• Multiple robot support<br>"
            "• Customizable grid (size and color)<br>"
            "• Multiple camera views (Top/Front/Side/Isometric)</p>"
        )

    def create_dock(self, name: str, widget: QWidget, area) -> QDockWidget:
        """Create a dock widget and add to main window."""
        dock = QDockWidget(name, self.parent)
        dock.setWidget(widget)
        dock.setObjectName(f"{name}_dock")
        self.parent.addDockWidget(area, dock)
        return dock
    
    def tabify_docks(self, dock1_name: str, dock2_name: str):
        """Tabify two docks together. (Not yet implemented)"""
        # Find docks by object name and tabify
        pass
