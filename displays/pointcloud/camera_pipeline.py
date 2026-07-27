"""
Camera Pipeline - Single-threaded point cloud pipeline for one camera.

Replaces the fragmented PointCloudDisplay/Processor/Renderer architecture
with a single class. Capture → Process → Render, driven by a 30 FPS timer.

Principle: Event-Driven. The timer is a single centralized check.
Principle: Visualizer as Mind-Prying Tool. Reads state, doesn't control.
"""

import time
import numpy as np
import logging
from typing import Optional
from PyQt5.QtCore import QTimer

from core.world_state.transform_registry import TransformRegistry

logger = logging.getLogger(__name__)


class CameraPipeline:
    """
    Single-threaded pipeline for one camera.

    Owns the driver, processes frames, and manages the VTK actor.
    One pipeline per physical camera. Multiple pipelines can run
    simultaneously — each has its own timer and VTK actor.
    """

    def __init__(self, registry: TransformRegistry):
        self.registry = registry
        self.renderer = None
        self.camera = None

        # State
        self.is_running = False
        self._needs_render = False
        self.camera_type = None
        self.camera_optical_frame = None

        # ROI (disabled by default for scanners, enabled for depth cameras)
        self.roi_enabled = True
        self.x_min, self.x_max = -2.0, 2.0
        self.y_min, self.y_max = -2.0, 2.0
        self.z_min, self.z_max = 0.1, 2.0

        # Pre-allocated mask for ROI clipping
        self._max_points = 640 * 480
        self._mask = np.empty(self._max_points, dtype=bool)

        # Timer
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)

        # Performance tracking
        self._frame_count = 0
        self._capture_times = []
        self._process_times = []

    # =================================================================
    # Lifecycle
    # =================================================================

    def attach(self, renderer):
        """Attach to VTK renderer and create the point cloud actor."""
        from .streaming_pointcloud import StreamingPointCloud
        self.renderer = StreamingPointCloud(renderer, max_points=self._max_points)

    def start(self, camera_type: str, asset_id: str = None,
              kinematic_model=None, **driver_config) -> bool:
        """
        Start the camera pipeline.

        Args:
            camera_type: "orbbec", "realsense", or "keyence".
            asset_id: Asset ID for frame name construction.
            kinematic_model: For optical frame auto-detection.
            **driver_config: Passed to the driver (ip, port, device_sn, etc.).
        """
        self.camera_type = camera_type

        # Create driver
        if camera_type == "keyence":
            from drivers.camera.keyence_scanner import KeyenceScannerDriver
            ip = driver_config.get('ip', '192.168.1.100')
            port = driver_config.get('port', 24691)
            self.camera = KeyenceScannerDriver(ip, port)
            self.roi_enabled = False  # Scanner produces clean profiles

        elif camera_type == "orbbec":
            from drivers.camera.orbbec_capture import OrbbecDriver
            device_sn = driver_config.get('device_sn')
            self.camera = OrbbecDriver(device_sn)
            self.camera.set_resolution(
                depth_width=driver_config.get('width', 640),
                depth_height=driver_config.get('height', 360),
                depth_fps=driver_config.get('fps', 30),
            )

        elif camera_type == "realsense":
            from drivers.camera.realsense_capture import RealSenseDriver
            device_sn = driver_config.get('device_sn')
            self.camera = RealSenseDriver(device_sn)

        else:
            logger.error(f"Unknown camera type: {camera_type}")
            return False

        # Start the driver
        logger.info(f"[DIAG] Going to start streaming")
        self.camera.start_streaming()

        # Auto-detect optical frame
        if kinematic_model and asset_id:
            self.camera_optical_frame = self._find_optical_frame(
                kinematic_model, asset_id, camera_type
            )
            if self.camera_optical_frame:
                logger.info(f"Camera optical frame: {self.camera_optical_frame}")

        # Set scanner point size
        if camera_type == "keyence" and self.renderer:
            self.renderer.actor.GetProperty().SetPointSize(4)

        # Start the timer
        self.is_running = True
        self._timer.start(33)  # 30 FPS
        logger.info(f"Camera pipeline started: {camera_type}")
        return True

    def stop(self):
        """Stop the pipeline and release resources."""
        self.is_running = False
        self._timer.stop()

        if self.camera:
            self.camera.stop_streaming()
            self.camera = None

        if self.renderer:
            self.renderer.clear()
        logger.info("Camera pipeline stopped")

    def detach(self):
        """Remove from renderer."""
        self.stop()
        self.renderer = None

    # =================================================================
    # Main Loop
    # =================================================================

    def _update(self):
        """Called at 30 FPS. Captures, processes, and renders one frame."""
        if not self.is_running or self.camera is None:
            return
        logger.debug(f"[Pipeline] _update called")  # ← Add this

        # 1. Capture
        t0 = time.time()
        points, colors = self.camera.capture_raw_pointcloud()
        if points is None or len(points) == 0:
            return
        self._capture_times.append((time.time() - t0) * 1000)

        # 2. Process
        t1 = time.time()
        points, colors = self._process(points, colors)
        if points is None or len(points) == 0:
            return
        self._process_times.append((time.time() - t1) * 1000)

        # 3. Render
        if self.renderer:
            self.renderer.update(points, colors)
            self._needs_render = True

        # Trim performance buffers
        if len(self._capture_times) > 30:
            self._capture_times.pop(0)
            self._process_times.pop(0)

    # =================================================================
    # Processing
    # =================================================================

    def _process(self, points: np.ndarray, colors: np.ndarray):
        """ROI clipping and world transform."""
        if len(points) == 0:
            return points, colors

        # ROI clipping
        if self.roi_enabled:
            n = len(points)
            mask = self._mask[:n]
            mask[:] = True

            mask &= (points[:, 0] >= self.x_min) & (points[:, 0] <= self.x_max)
            mask &= (points[:, 1] >= self.y_min) & (points[:, 1] <= self.y_max)
            mask &= (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)

            points = points[mask]
            colors = colors[mask]

            if len(points) == 0:
                return points, colors

        # Transform to world frame
        if self.camera_optical_frame:
            try:
                T = self.registry.get_transform("world", self.camera_optical_frame)
                R = T[:3, :3].astype(np.float32)
                t = T[:3, 3].astype(np.float32)
                points = points @ R.T + t
            except ValueError:
                pass  # Frame not in registry yet

        return points, colors

    # =================================================================
    # Optical Frame Detection
    # =================================================================

    def _find_optical_frame(self, kinematic_model, asset_id, camera_type):
        """Find the optical frame for this camera type."""
        patterns = {
            "keyence": "lj_v7200_optical_frame",
            "orbbec": "depth_optical_frame",
            "realsense": "depth_optical_frame",
        }
        pattern = patterns.get(camera_type, "depth_optical_frame")

        for link_name in kinematic_model.link_transforms.keys():
            if pattern in link_name.lower():
                return f"{asset_id}_{link_name}"
        return None

    # =================================================================
    # Public Methods
    # =================================================================

    def set_visible(self, visible: bool):
        """Show/hide the point cloud."""
        if self.renderer:
            self.renderer.set_visible(visible)

    def set_roi(self, x_min, x_max, y_min, y_max, z_min, z_max):
        """Set ROI clipping bounds."""
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max

    def clear(self):
        """Clear the point cloud."""
        if self.renderer:
            self.renderer.clear()