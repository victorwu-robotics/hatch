"""
Camera Manager - Manages multiple cameras discovered from URDF.

Each camera gets its own PointCloudDisplay and lifecycle.
The control panel switches between cameras; multiple cameras
can be active simultaneously.

Principle #4: Everything in URDF. Cameras are discovered, not added.
"""

import logging
from typing import Dict, Optional, List

from PyQt5.QtWidgets import QDockWidget
from PyQt5.QtCore import Qt

from core.world_state.state_channel import StateChannel
from core.world_state.transform_registry import TransformRegistry
from core.world_state.event_types import EventType
from displays.pointcloud.camera_pipeline import CameraPipeline

logger = logging.getLogger(__name__)


class CameraManager:
    """
    Manages camera discovery, lifecycle, and UI.

    Cameras are discovered from the URDF when a robot is loaded.
    Each camera gets its own PointCloudDisplay pipeline.
    The control panel switches between cameras for configuration.
    """

    def __init__(self,
                 transform_registry: TransformRegistry,
                 state_channel: StateChannel,
                 engine,
                 robot_manager,
                 parent_window):
        self.transform_registry = transform_registry
        self.state_channel = state_channel
        self.engine = engine
        self.robot_manager = robot_manager
        self.parent = parent_window

        self.state_channel.subscribe(EventType.ROBOT_LOADED, self._on_robot_loaded)
        
        # Registered cameras: camera_id -> {display, type, frame_name, is_running, config}
        self.cameras: Dict[str, dict] = {}

        # Which camera the panel is currently showing
        self.active_camera_id: Optional[str] = None

        # Camera-specific IP/SN entries (filled by user in panel)
        self.camera_connections: Dict[str, dict] = {}

        # UI
        self.camera_dock = None
        self.camera_panel = None

        logger.info("CameraManager initialized")

    # =================================================================
    # Camera Registration (called when URDF is loaded)
    # =================================================================

    def add_camera(self, camera_id: str, camera_type: str, frame_name: str):
        """
        Register a camera discovered from the URDF.

        Creates the PointCloudDisplay pipeline but does not start streaming.

        Args:
            camera_id: Unique identifier (e.g., "orbbec_gemini_335").
            camera_type: "orbbec", "realsense", or "keyence".
            frame_name: The camera's depth optical frame name (with asset prefix).
        """
        if camera_id in self.cameras:
            logger.warning(f"Camera '{camera_id}' already registered")
            return

        pipeline = CameraPipeline(self.transform_registry)
        pipeline.attach(self.engine.get_renderer())
        self.engine.register_display(pipeline)

        self.cameras[camera_id] = {
            'pipeline': pipeline,
            'type': camera_type,
            'frame_name': frame_name,
            'is_running': False,
        }

        # Set as active if this is the first camera
        if self.active_camera_id is None:
            self.active_camera_id = camera_id

        logger.info(f"Camera registered: {camera_id} ({camera_type}), "
                   f"frame: {frame_name}")

    # =================================================================
    # Camera Lifecycle
    # =================================================================

    def start_camera(self, camera_id: str, config: dict = None) -> bool:
        """
        Start a camera streaming.

        Args:
            camera_id: Which camera to start.
            config: Connection details (ip/port for network, device_sn for USB).

        Returns:
            True if started successfully.
        """
        if camera_id not in self.cameras:
            logger.error(f"Unknown camera: {camera_id}")
            return False

        cam = self.cameras[camera_id]

        if cam['is_running']:
            logger.info(f"Camera '{camera_id}' is already running")
            return True

        # Store connection config
        if config:
            self.camera_connections[camera_id] = config

        # Get stored config
        conn = self.camera_connections.get(camera_id, {})

        # Get asset info from robot manager
        asset_id = None
        kinematic_model = None
        if self.robot_manager:
            asset_id = self.robot_manager.current_asset_id
            kinematic_model = self.robot_manager.current_kinematic_model

        try:
            pipeline = cam['pipeline']
            success = pipeline.start(
                camera_type=cam['type'],
                asset_id=asset_id,
                kinematic_model=kinematic_model,
                **conn  # Unpacks ip, port, device_sn, width, height, fps
            )

            if success:
                cam['is_running'] = True
                logger.info(f"Camera started: {camera_id}")
            return success

        except Exception as e:
            logger.error(f"Failed to start camera '{camera_id}': {e}")
            return False

    def stop_camera(self, camera_id: str) -> bool:
        """
        Stop a camera streaming.

        Args:
            camera_id: Which camera to stop.

        Returns:
            True if stopped successfully.
        """
        if camera_id not in self.cameras:
            logger.error(f"Unknown camera: {camera_id}")
            return False

        cam = self.cameras[camera_id]

        if not cam['is_running']:
            return True

        try:
            pipeline = cam['pipeline']
            success = pipeline.stop()
            cam['is_running'] = False
            logger.info(f"Camera stopped: {camera_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to stop camera '{camera_id}': {e}")
            return False

    # =================================================================
    # Camera Queries
    # =================================================================

    def get_available_cameras(self) -> List[dict]:
        """
        Get list of all registered cameras with their status.

        Returns:
            List of dicts with keys: id, type, frame_name, is_running.
        """
        return [
            {
                'id': cam_id,
                'type': cam['type'],
                'frame_name': cam['frame_name'],
                'is_running': cam['is_running'],
            }
            for cam_id, cam in self.cameras.items()
        ]

    def set_active_camera(self, camera_id: str):
        """
        Switch the control panel to show a different camera's settings.

        Args:
            camera_id: Which camera to control.
        """
        if camera_id not in self.cameras:
            logger.warning(f"Cannot set active camera: '{camera_id}' not found")
            return
        self.active_camera_id = camera_id
        logger.info(f"Active camera set to: {camera_id}")

    # =================================================================
    # Camera Discovery from URDF
    # =================================================================

    def _on_robot_loaded(self, event):
        """When a robot is loaded, discover cameras from its URDF."""
        logger.debug(f"===== handling ROBOT_LOADED ")
        self.discover_cameras_from_urdf()

    def discover_cameras_from_urdf(self):
        """
        Scan the loaded kinematic model for camera frames and register them.

        Detects cameras by looking for link names containing known patterns:
        - 'depth_optical_frame' → RGB-D camera (Orbbec, RealSense)
        - 'lj_v7200' → Keyence laser scanner

        Camera type is inferred from the frame naming convention.
        """
        logger.debug(f"----- Discovering Cameras from URDF")
        if not self.robot_manager or not self.robot_manager.current_kinematic_model:
            logger.info("No robot loaded — skipping camera discovery")
            return

        model = self.robot_manager.current_kinematic_model
        asset_id = self.robot_manager.current_asset_id

        for link_name in model.link_transforms.keys():
            frame_name = f"{asset_id}_{link_name}"

            if 'depth_optical_frame' in link_name.lower():
                # Determine camera type from package presence
                camera_type = "orbbec"  # Default
                camera_id = "rgbd_camera"
                self.add_camera(camera_id, camera_type, frame_name)

            elif 'lj_v7200_optical_frame' in link_name.lower() and 'frame' in link_name.lower():
                camera_id = "keyence_scanner"
                self.add_camera(camera_id, "keyence", frame_name)

        logger.info(f"Camera discovery complete: {len(self.cameras)} cameras found")

    # =================================================================
    # UI
    # =================================================================

    def show_panel(self):
        """Show the camera control panel, creating it if needed."""
        if self.camera_dock is None:
            from ui.panels.camera_control_panel import CameraControlPanel
            
            self.camera_panel = CameraControlPanel(self)
            
            # Wire panel signals
            self.camera_panel.start_requested.connect(
                lambda cam_id, config: self.start_camera(cam_id, config)
            )
            self.camera_panel.stop_requested.connect(self.stop_camera)
            
            # Wire display toggles
            self.camera_panel.transform_toggled.connect(self._on_transform_toggled)
            self.camera_panel.show_frames_toggled.connect(self._on_frames_toggled)
            self.camera_panel.visible_toggled.connect(self._on_visible_toggled)
            
            self.camera_dock = QDockWidget("Camera Control", self.parent)
            self.camera_dock.setWidget(self.camera_panel)
            self.parent.addDockWidget(Qt.RightDockWidgetArea, self.camera_dock)
            
            # Wire ROI changes
            self.camera_panel.roi_changed.connect(
                lambda xmin, xmax, ymin, ymax, zmin, zmax:
                    self._on_roi_changed(xmin, xmax, ymin, ymax, zmin, zmax)
            )

            # Populate the camera list
            self.camera_panel.refresh_camera_list()
        
        self.camera_dock.show()
        self.camera_dock.raise_()

    def _on_transform_toggled(self, enabled):
        cam_id = self.active_camera_id
        if cam_id and cam_id in self.cameras:
            self.cameras[cam_id]['pipeline'].set_transform_to_world(enabled)

    def _on_frames_toggled(self, enabled):
        cam_id = self.active_camera_id
        if cam_id and cam_id in self.cameras:
            self.cameras[cam_id]['pipeline'].set_show_frames(enabled)

    def _on_visible_toggled(self, enabled):
        cam_id = self.active_camera_id
        if cam_id and cam_id in self.cameras:
            self.cameras[cam_id]['pipeline'].set_visible(enabled)

    def _on_roi_changed(self, x_min, x_max, y_min, y_max, z_min, z_max):
        cam_id = self.active_camera_id
        if cam_id and cam_id in self.cameras:
            self.cameras[cam_id]['pipeline'].set_roi(x_min, x_max, y_min, y_max, z_min, z_max)

    def cleanup(self):
        """Stop all cameras and clean up resources."""
        for camera_id in list(self.cameras.keys()):
            self.stop_camera(camera_id)
        logger.info("CameraManager cleaned up")
