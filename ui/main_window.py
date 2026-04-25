"""
PHASE 6 - Main Window Orchestrator (VTK Direct Version)
Goal: Create the main application window that orchestrates all components.
Now with modular architecture for better maintainability and multi-camera support.
"""

import sys
import numpy as np

from PyQt5.QtWidgets import QMainWindow, QApplication, QDockWidget
from PyQt5.QtCore import Qt, QTimer

# Core imports
from core.world_state.state_channel import StateChannel, EventType
from core.world_state.transform_registry import TransformRegistry
from core.mesh_loader import MeshLoader

# Visualization imports
from viz.visualizer_engine import VisualizerEngine, RenderConfig

# Managers
from ui.managers.camera_manager import CameraManager
from ui.ui_builder import UIBuilder


class MainWindow(QMainWindow):
    """
    Primary application window managing the composition of all components.
    Now with multi-camera support and modular architecture.
    """
    
    def __init__(self, parent=None):
        """Initialize window and create all components."""
        super().__init__(parent)
        
        self.setWindowTitle("RoboPlatform - Digital Twin")
        self.setGeometry(100, 100, 1400, 800)

        # Ensure window can be resized and maximized
        self.setWindowFlags(Qt.Window)
        self.setWindowState(Qt.WindowNoState)  # Reset any weird state

        # ===== 1. CORE: Single Source of Truth =====
        self.transform_registry = TransformRegistry()
        self.state_channel = StateChannel(enable_history=True)
        
        # ===== 2. VISUALIZATION =====
        self._setup_visualizer()

        # ===== ENSURE CENTRAL WIDGET EXPANDS =====
        if self.centralWidget():
            from PyQt5.QtWidgets import QSizePolicy
            self.centralWidget().setSizePolicy(
                QSizePolicy.Expanding, 
                QSizePolicy.Expanding
            )
            # Force the central widget to take all available space
            self.setCentralWidget(self.centralWidget())

        # ===== 3. ROBOT Components =====
        from core.robot_manager import RobotManager

        self.robot_manager = RobotManager(
            self.transform_registry,
            self.state_channel,
            self.engine,
            self
        )

        # Mesh loader for robots
        self.mesh_loader = MeshLoader(True)

        # Camera manager for point cloud streaming (now multi-camera)
        self.camera_manager = CameraManager(
            self.transform_registry,
            self.state_channel,
            self.engine,
            self
        )

        # 4. Create simulated and real robots (needed for CommandHandler)
        from drivers.robot_arm.simulated_robot import SimulatedRobot
        from drivers.robot_arm.real_robot import RealRobot
        
        self.simulated_robot = SimulatedRobot(
            kinematic_model=None,  # Will be set when robot loads
            state_channel=self.state_channel,
            real_robot=None,
            parent=self
        )
        
        self.real_robot = RealRobot(self.state_channel, parent=self)
        
        # 5. CommandHandler (depends on both robots)
        from core.command_handler import CommandHandler
        
        self.command_handler = CommandHandler(
            state_channel=self.state_channel,
            simulated_robot=self.simulated_robot,
            real_robot=self.real_robot
        )
        
        # 6. Connect robot_manager to update simulated_robot when robot loads
        # self.robot_manager.set_simulated_robot(self.simulated_robot)

        # UI manager for menus and controls
        self.ui_builder = UIBuilder(
            self,
            self.state_channel,     # UI communicates via events
            self.robot_manager,     # Keep it for the time being
            self.camera_manager,
            self.engine
        )

        # ===== EVENT HANDLERS =====
        self._setup_event_handlers()
        
        # ===== LOAD DEFAULT ASSET =====
        self._load_ur5_asset()

        # ===== PHASE 3 ADDITION =====
        # ===== ADD CONTROL CONTAINER =====
        # self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded_for_motion)

        # ===== FORCE LAYOUT UPDATE =====
        # This ensures the central widget expands properly
        self.layout().setContentsMargins(0, 0, 0, 0)
        self.layout().setSpacing(0)

        # After creating state_channel, add a test subscriber
        # self.state_channel.subscribe(EventType.CARTESIAN_COMMAND, 
        #     lambda e: print(f"[TEST] CARTESIAN_COMMAND received: {e.data.get('pose')}"))

    def _create_motion_container(self, kinematic_model, asset_id):
        """Create the container after robot loads."""
        from ui.panels.motion_container import MotionContainer
        
        # Create container (no model yet - will update when robot loads)
        self.motion_container = MotionContainer(
            kinematic_model,
            state_channel=self.state_channel,
            robot_manager=self.robot_manager,
            transform_registry=self.transform_registry,
            asset_id=None,
            parent=self
        )
        
        # Create dock widget
        dock = QDockWidget("Motion Control", self)
        dock.setWidget(self.motion_container)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        
        # Add to main window (right side)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        
        # Store reference
        self.motion_dock = dock
        
    def _on_robot_loaded(self, event):
        """Create motion container when robot loads
           and set kinematic model for simulated_robot"""
        asset_id = event.data.get('asset_id')
        model = event.data.get('kinematic_model')
        if self.robot_manager and self.robot_manager.current_kinematic_model:
            # Only create motion_container if not already exists
            if not hasattr(self, 'motion_container') or self.motion_container is None:
                self._create_motion_container(
                    self.robot_manager.current_kinematic_model,
                    asset_id
                )
            self.simulated_robot.set_kinematic_model(model)

    def _update_control_display(self):
        """Update displays in the active control panel."""
        if hasattr(self, 'control_container'):
            # Let the container handle which panel to update
            if self.control_container.current_mode == "cartesian":
                if hasattr(self.control_container.cartesian_panel, 'update_current_display'):
                    self.control_container.cartesian_panel.update_current_display()

    def _update_cartesian_display(self):
        """Update current TCP display in Cartesian panel."""
        if hasattr(self, 'cartesian_panel'):
            self.cartesian_panel._update_current_display()

    def _setup_visualizer(self):
        """Setup the visualizer engine."""
        config = RenderConfig(
            width=1280,
            height=720,
            background_color=(1.0, 1.0, 1.0),
            anti_aliasing=False,
            vsync=False,
            use_display_lists=True
        )
        
        # CREATE ENGINE FIRST
        self.engine = VisualizerEngine(
            title="3D Visualization", 
            config=config,
            parent=self
        )
        
        # THEN use it
        render_widget = self.engine.get_render_widget()
        
        # Set size policy
        from PyQt5.QtWidgets import QSizePolicy
        render_widget.setSizePolicy(
            QSizePolicy.Expanding, 
            QSizePolicy.Expanding
        )
        
        self.setCentralWidget(render_widget)
    
    def _setup_event_handlers(self):
        """Setup subscriptions to state channel events."""
        
        # Subscribe to joint updates
        self.state_channel.subscribe(
            EventType.JOINT_UPDATE, 
            self._on_joint_update_event
        )

        # Subscribe to Robot Loaded
        self.state_channel.subscribe(
            EventType.ROBOT_LOADED,
            self._on_robot_loaded
        )

        # Subscribe to camera events (now with camera type info)
        self.state_channel.subscribe(
            EventType.CAMERA_STARTED,
            self._on_camera_event
        )
        
        self.state_channel.subscribe(
            EventType.CAMERA_STOPPED,
            self._on_camera_event
        )
        
        # Subscribe to errors
        self.state_channel.subscribe(
            EventType.ERROR_OCCURRED,
            self._on_error_event
        )

    def _setup_cartesian_panel(self):
        """Add Cartesian control panel as a dock widget."""
        from ui.panels.cartesian_control_panel import CartesianControlPanel
        
        # Get current asset
        current_asset = self.mesh_loader.get_current_asset()
        if not current_asset:
            print("No asset loaded - Cartesian panel will be empty")
            return
        
        # Create panel
        self.cartesian_panel = CartesianControlPanel(
            current_asset['model'],
            parent=self
        )
        
        # Create dock widget
        dock = QDockWidget("Cartesian Control", self)
        dock.setWidget(self.cartesian_panel)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        
        # Add to main window (right side, below joint controls)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        
        # Store reference
        self.cartesian_dock = dock
        
        # Connect to asset changes to update when robot changes
        self.mesh_loader.register_asset_change_callback(self._on_asset_changed)

    def _on_asset_changed(self):
        """Handle asset changes (new robot loaded)."""
        if hasattr(self, 'cartesian_panel'):
            current_asset = self.mesh_loader.get_current_asset()
            if current_asset:
                self.cartesian_panel.kinematic_model = current_asset['model']
                self.cartesian_panel._update_from_current_pose()

    def _on_joint_update_event(self, event):
        """Handle joint update events from the panel."""
        try:
            # Get joint positions from event
            joint_data = event.data
            joint_positions = joint_data.get('positions')
            joint_names = joint_data.get('names')
            
            if joint_positions is None or joint_names is None:
                return
            
            # Get current asset
            current_asset = self.mesh_loader.get_current_asset()
            if not current_asset:
                return
            
            # Update kinematic model
            model = current_asset['model']
            model.update_state(np.array(joint_positions))
            
            # Update transform registry
            if hasattr(model, 'link_transforms'):
                asset_id = self.mesh_loader.current_asset_id
                for link_name, T in model.link_transforms.items():
                    # Determine parent frame
                    if link_name in model.root_links:
                        parent_frame = "world"
                    else:
                        parent_link = model.link_parents.get(link_name, 'world')
                        parent_frame = f"{asset_id}_{parent_link}"
                    
                    # Update transform registry
                    self.transform_registry.set(
                        f"{asset_id}_{link_name}",
                        T,
                        parent=parent_frame
                    )
            
            # Force a render
            self.engine.render()
            
        except Exception as e:
            pass
    
    def _on_camera_event(self, event):
        """Handle camera events - now shows camera type in status bar."""
        if event.type == EventType.CAMERA_STARTED:
            # Extract camera type from event data if available
            camera_type = "Unknown"
            if event.data and 'camera_type' in event.data:
                camera_type = event.data['camera_type']
            self.statusBar().showMessage(f"Camera started: {camera_type}", 2000)
        elif event.type == EventType.CAMERA_STOPPED:
            self.statusBar().showMessage("Camera stopped", 2000)
    
    def _on_error_event(self, event):
        """Handle error events."""
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.warning(
            self,
            "Error",
            str(event.data)
        )
    
    def _load_ur5_asset(self):
        """Load hardcoded UR5 asset with error handling."""
        try:
            from robot_descriptions import ur5_description
            urdf_path = ur5_description.URDF_PATH
            asset_id = self.mesh_loader.load_robot_from_file(urdf_path)
            if asset_id:
                pass
        except ImportError:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(
                self,
                "UR5 Not Available",
                "robot_descriptions package not installed.\n\n"
                "Install it with: pip install robot_descriptions\n\n"
                "You can still load URDF files manually via File → Load URDF."
            )
        except Exception as e:
            pass
    
    def closeEvent(self, event):
        """Handle window close event."""
        
        # Clean up managers
        if hasattr(self, 'camera_manager'):
            self.camera_manager.cleanup()
        
        if hasattr(self, 'robot_manager'):
            self.robot_manager.cleanup()
        
        super().closeEvent(event)

    def changeEvent(self, event):
        """Debug window state changes."""
        if event.type() == event.WindowStateChange:
            print(f"Window state changed to: {self.windowState()}")
        super().changeEvent(event)

    def resizeEvent(self, event):
        """Debug resize events."""
        print(f"Window resized to: {self.width()} x {self.height()}")
        if self.centralWidget():
            print(f"Central widget: {self.centralWidget().width()} x {self.centralWidget().height()}")
        super().resizeEvent(event)

def main():
    """Main entry point for RoboPlatform application."""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()