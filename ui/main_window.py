"""
Hatch Main Window - Application entry point and component orchestrator.

Creates all core services, visualization engine, and UI.
Wires components together via StateChannel events.
Does NOT hold business logic, update models, or bypass the event architecture.

Principle #9: UI Separate from Services.
Principle #2: Event-Driven, No Polling.
"""

import sys
import logging

from PyQt5.QtWidgets import QMainWindow, QApplication, QDockWidget, QMessageBox, QSizePolicy
from PyQt5.QtCore import Qt

# Core services
from core.world_state.state_channel import StateChannel
from core.world_state.transform_registry import TransformRegistry
from core.world_state.event_types import EventType
from core.mesh_loader import MeshLoader

# Visualization
from viz.visualizer_engine import VisualizerEngine, RenderConfig

# Robot components
from core.robot_manager import RobotManager
from core.command_handler import CommandHandler
from drivers.simulated_robot import SimulatedRobot
from drivers.real_robot import RealRobot

# UI
from ui.ui_builder import UIBuilder
#from ui.managers.camera_manager import CameraManager
from ui.panels.motion_container import MotionContainer

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """
    Primary application window.

    Responsibilities:
    - Create and own core services (TransformRegistry, StateChannel)
    - Create and own visualization engine
    - Create and own robot components (RobotManager, CommandHandler, robots)
    - Create UIBuilder for menus, docks, and panels
    - Create MotionContainer for joint/cartesian control
    - Subscribe only to events that require window-level handling

    Does NOT:
    - Update kinematic models directly
    - Update transform registry directly
    - Force renders directly
    - Hold business logic
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Hatch (孵) - Robotics Platform")
        self.setGeometry(100, 100, 1400, 800)
        self.setWindowFlags(Qt.Window)

        # ===== 1. Core Services =====
        self.transform_registry = TransformRegistry()
        self.state_channel = StateChannel(enable_history=True)

        # ===== 2. Visualization Engine =====
        self._setup_visualizer()

        # ===== 3. Mesh Loader =====
        self.mesh_loader = MeshLoader(enable_color_extraction=True)

        # ===== 4. Robot Components =====
        self._setup_robot_components()

        # ===== 5. Camera Manager =====
        '''
        self.camera_manager = CameraManager(
            self.transform_registry,
            self.state_channel,
            self.engine,
            self
        )
        '''

        # ===== 6. UI Builder (menus, toolbars, docks) =====
        self.ui_builder = UIBuilder(
            self,
            self.state_channel,
            self.robot_manager,
            # self.camera_manager,
            self.engine
        )

        # ===== 8. Window-Level Event Handlers =====
        self._setup_event_handlers()

        # ===== 9. Finalize Layout =====
        self._finalize_layout()

        logger.info("MainWindow initialized")

    # =================================================================
    # Setup Methods
    # =================================================================

    def _setup_visualizer(self):
        """Create the VTK visualization engine and set as central widget."""
        config = RenderConfig(
            width=1280,
            height=720,
            background_color=(1.0, 1.0, 1.0),
            anti_aliasing=False,
            vsync=False,
            use_display_lists=True
        )

        self.engine = VisualizerEngine(
            title="Hatch Visualizer",
            config=config,
            parent=self
        )

        render_widget = self.engine.get_render_widget()
        from PyQt5.QtWidgets import QSizePolicy
        render_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCentralWidget(render_widget)

    def _setup_robot_components(self):
        """Create RobotManager, robots, and CommandHandler."""
        # Create RobotManager (orchestrator, no Qt signals)
        self.robot_manager = RobotManager(
            self.transform_registry,
            self.state_channel,
            self.engine
        )

        self.robot_manager.mesh_loader = self.mesh_loader

        # Create robot instances
        self.simulated_robot = SimulatedRobot(
            kinematic_model=None,  # Set when robot loads
            state_channel=self.state_channel,
            real_robot=None
        )

        self.real_robot = RealRobot(self.state_channel)

        # Inject robots into RobotManager
        self.robot_manager.set_simulated_robot(self.simulated_robot)
        self.robot_manager.set_real_robot(self.real_robot)

        # Connect simulated robot to real robot for IK passthrough
        self.simulated_robot.set_real_robot(self.real_robot)

        # Create CommandHandler (routes commands to active robot)
        self.command_handler = CommandHandler(
            state_channel=self.state_channel,
            simulated_robot=self.simulated_robot,
            real_robot=self.real_robot
        )

    def _setup_motion_container(self, kinematic_model, asset_id):
        """Create the motion control container as a dock widget."""
        self.motion_container = MotionContainer(
            kinematic_model=kinematic_model,
            state_channel=self.state_channel,
            robot_manager=self.robot_manager,
            transform_registry=self.transform_registry,
            asset_id=asset_id,
            parent=self
        )

        dock = QDockWidget("Motion Control", self)
        dock.setWidget(self.motion_container)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self.motion_dock = dock

    def _setup_event_handlers(self):
        """Subscribe to events that need window-level handling."""
        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)
        self.state_channel.subscribe(EventType.ERROR_OCCURRED, self._on_error)

    def _finalize_layout(self):
        """Ensure proper layout behavior."""
        if self.centralWidget():
            from PyQt5.QtWidgets import QSizePolicy
            self.centralWidget().setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Expanding
            )

    # =================================================================
    # Event Handlers (window-level only)
    # =================================================================

    def _on_robot_loaded(self, event):
        """Update motion container and simulated robot when a robot loads."""
        asset_id = event.data.get('asset_id')
        model = event.data.get('kinematic_model')

        if model is None:
            return

        # Create StateHandler (single owner of model + registry updates)
        from core.state_handler import StateHandler
        self.state_handler = StateHandler(
            state_channel=self.state_channel,
            kinematic_model=model,
            transform_registry=self.transform_registry,
            asset_id=asset_id
        )

        # Create motion container (only when we have a model)
        self._setup_motion_container(model, asset_id)

        # Update simulated robot with the kinematic model
        self.simulated_robot.set_kinematic_model(model)

        # Create joint frame control panel
        if hasattr(self.robot_manager, '_joint_display'):
            from ui.panels.joint_frame_panel import JointFramePanel
            frame_panel = JointFramePanel(self.robot_manager._joint_display)
            
            dock = QDockWidget("Joint Frames", self)
            dock.setWidget(frame_panel)
            dock.setMinimumWidth(250)
            dock.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
            self.addDockWidget(Qt.LeftDockWidgetArea, dock)

    def _on_error(self, event):
        """Show error dialog for ERROR_OCCURRED events."""
        error_data = event.data
        if isinstance(error_data, dict):
            message = error_data.get('error', str(error_data))
        else:
            message = str(error_data)

        QMessageBox.warning(self, "Error", message)

    # =================================================================
    # Window Lifecycle
    # =================================================================

    def closeEvent(self, event):
        """Clean up resources on window close."""
        if hasattr(self, 'camera_manager'):
            self.camera_manager.cleanup()

        if hasattr(self, 'robot_manager'):
            self.robot_manager.cleanup()

        super().closeEvent(event)


def main():
    """Main entry point for Hatch."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(name)s: %(message)s'
    )

    print(f"Logging level: {logging.getLogger().getEffectiveLevel()}")
    print(f"Geometric solver logger level: {logging.getLogger('core.kinematics.geom_sph_ik_solver').getEffectiveLevel()}")

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()