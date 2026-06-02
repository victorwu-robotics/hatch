"""
Camera Manager - Handles camera initialization, streaming, and switching between camera types.
"""

import numpy as np
from PyQt5.QtWidgets import QDockWidget
from PyQt5.QtCore import Qt, QTimer

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.world_state.transform_registry import TransformRegistry, FrameStatus
from displays.pointcloud.pointcloud_display import PointCloudDisplay
from ui.panels.camera_control_panel import CameraControlPanel

import logging; logger = logging.getLogger(__name__)

class CameraManager:
    """
    Manages camera streaming, point cloud display, and camera UI.
    Now supports multiple camera types (Orbbec, RealSense) with runtime switching.
    """
    
    def __init__(self, transform_registry: TransformRegistry,
                 state_channel: StateChannel,
                 engine,
                 robot_manager,
                 parent_window):
        """
        Initialize the camera manager.
        
        Args:
            transform_registry: The transform registry
            state_channel: The event channel
            engine: The visualizer engine
            parent_window: The main window
        """
        self.transform_registry = transform_registry
        self.state_channel = state_channel
        self.engine = engine
        self.robot_manager = robot_manager
        self.parent = parent_window
        
        # Camera components
        self.pointcloud_display = None
        self.camera_panel = None
        self.camera_dock = None
        
        # Camera state
        self.current_camera_type = "orbbec"  # Default
        self.is_running = False
        self.device_sn = None
        
        # Available cameras and their display names
        self.available_cameras = {
            "orbbec": "Orbbec Gemini 335",
            "realsense": "Intel RealSense D435"
        }
        
        # Camera-specific resolution presets
        self.camera_resolutions = {
            "orbbec": [
                "640x360 @30fps",
                "640x480 @30fps",
                "1280x720 @30fps",
                "1280x800 @30fps"
            ],
            "realsense": [
                "640x480 @30fps",
                "848x480 @30fps",
                "1280x720 @30fps"
            ]
        }
        
        self._init_camera_support()
        print("CameraManager: Initialized with multi-camera support")
    
    def _init_camera_support(self):
        """Initialize point cloud display and control panel."""

        # Create point cloud display with transform registry
        self.pointcloud_display = PointCloudDisplay(
            self.transform_registry,
            kinematic_model=self.robot_manager.current_kinematic_model,
            asset_id=self.robot_manager.current_asset_id
            )
        self.pointcloud_display.attach(self.engine.get_renderer())
        self.engine.register_display(self.pointcloud_display)
        
        # Create control panel (now takes camera_manager reference)
        self.camera_panel = CameraControlPanel(self)

        # Connect ROI signals
        self.camera_panel.min_x_changed.connect(lambda v: self.set_roi_axis('x_min', v))
        self.camera_panel.max_x_changed.connect(lambda v: self.set_roi_axis('x_max', v))
        self.camera_panel.min_y_changed.connect(lambda v: self.set_roi_axis('y_min', v))
        self.camera_panel.max_y_changed.connect(lambda v: self.set_roi_axis('y_max', v))
        self.camera_panel.min_z_changed.connect(lambda v: self.set_roi_axis('z_min', v))
        self.camera_panel.max_z_changed.connect(lambda v: self.set_roi_axis('z_max', v))

        # Connect signals
        self.camera_panel.start_streaming.connect(self.start_camera)
        self.camera_panel.stop_streaming.connect(self.stop_camera)
        self.camera_panel.camera_type_changed.connect(self.switch_camera_type)
        # self.camera_panel.max_depth_changed.connect(self.set_max_depth)  # ← NEW

        # ← ADD THIS: Connect resolution changes
        self.camera_panel.resolution_changed.connect(self.set_resolution)

        self.camera_panel.visible_check.toggled.connect(
            self.pointcloud_display.set_visible
        )
        self.camera_panel.show_frames_toggled.connect(
            self.pointcloud_display.set_show_frames
        )
        self.camera_panel.transform_toggled.connect(
            self.pointcloud_display.set_transform_to_world
        )
        
        # Create dock widget
        self.camera_dock = QDockWidget("Camera Stream", self.parent)
        self.camera_dock.setWidget(self.camera_panel)
        self.parent.addDockWidget(Qt.RightDockWidgetArea, self.camera_dock)
        self.camera_dock.hide()
    
    def _is_camera_connected(self, camera_type: str) -> bool:
        """
        Check if a specific camera type is actually connected to the system.
        
        Args:
            camera_type: "orbbec" or "realsense"
        
        Returns:
            True if at least one camera of that type is connected
        """
        try:
            if camera_type == "realsense":
                import pyrealsense2 as rs
                ctx = rs.context()
                devices = ctx.query_devices()
                return len(devices) > 0
                
            elif camera_type == "orbbec":
                import pyorbbecsdk as ob
                # Create a context to query devices
                ctx = ob.Context()
                device_list = ctx.query_devices()
                return device_list.get_count() > 0
                
            else:
                return False
                
        except ImportError as e:
            print(f"⚠️ SDK not installed for {camera_type}: {e}")
            return False
        except Exception as e:
            print(f"⚠️ Error checking {camera_type} connection: {e}")
            return False

    def start_camera(self, device_sn=None):
        """
        Start camera streaming with current camera type.
        
        Args:
            device_sn: Optional device serial number for specific camera
        """
        if not self.pointcloud_display:
            return False

        # Check if camera is actually connected
        if not self._is_camera_connected(self.current_camera_type):
            camera_name = self.available_cameras[self.current_camera_type]
            error_msg = f"{camera_name} not detected. Please check connection."
            print(f"❌ {error_msg}")
            
            # Update panel with error
            self.camera_panel.update_status(error_msg, is_error=True)
            
            # Publish error event
            self.state_channel.publish(
                EventType.ERROR_OCCURRED,
                data=error_msg,
                source="camera_manager",
                description="Camera connection failed"
            )
            return False
        
        self.device_sn = device_sn

        # Get resolution from stored values or panel
        if hasattr(self, '_target_width'):
            width = self._target_width
            height = self._target_height
            fps = self._target_fps
            print(f"📹 Using stored resolution: {width}x{height} @ {fps}fps")
        else:
            # Get from panel
            current_res = self.camera_panel.resolution_combo.currentText()
            try:
                res_part = current_res.split(' @')[0]
                width, height = map(int, res_part.split('x'))
                fps_part = current_res.split('@')[1].replace('fps', '')
                fps = int(fps_part)
                print(f"📹 Using panel resolution: {width}x{height} @ {fps}fps")
            except:
                # Default fallback
                width, height, fps = 640, 360, 30
                print(f"📹 Using default resolution: {width}x{height}")

        # Start the camera with current type and resolution
        success = self.pointcloud_display.start_camera(
            device_sn=device_sn,
            camera_type=self.current_camera_type,
            width=width,
            height=height,
            fps=fps,
            asset_id=self.robot_manager.current_asset_id,
            kinematic_model=self.robot_manager.current_kinematic_model
        )

        if success:
            self.is_running = True
            camera_name = self.available_cameras[self.current_camera_type]
            self.camera_panel.update_status(f"Streaming {camera_name}")

            # Set initial ROI from panel values
            self.set_roi_range(
                self.camera_panel.min_x_spin.value(),
                self.camera_panel.max_x_spin.value(),
                self.camera_panel.min_y_spin.value(),
                self.camera_panel.max_y_spin.value(),
                self.camera_panel.min_z_spin.value(),
                self.camera_panel.max_z_spin.value()
            )

            logger.info(f"Camera started: {camera_name}")
            '''
            self.state_channel.publish(
                EventType.CAMERA_STARTED,
                source="camera_manager",
                data={"camera_type": self.current_camera_type},
                description=f"{camera_name} started successfully"
            )
            '''
        else:
            self.camera_panel.update_status("Failed to start camera", is_error=True)
            self.state_channel.publish(
                EventType.ERROR_OCCURRED,
                data="Failed to start camera",
                source="camera_manager",
                description="Camera start failed"
            )
        
        return success
    
    def stop_camera(self):
        """Stop camera streaming."""
        if self.pointcloud_display:
            self.pointcloud_display.stop_camera()
            self.pointcloud_display.clear()
            self.is_running = False
            logger.info("Camera stopped")
            '''
            self.state_channel.publish(
                EventType.CAMERA_STOPPED,
                source="camera_manager",
                description="Camera stopped"
            )
            '''
            return True
        return False
    
    def switch_camera_type(self, camera_type: str):
        """
        Switch to different camera type at runtime.
        
        Args:
            camera_type: "orbbec" or "realsense"
        """
        if camera_type not in self.available_cameras:
            print(f"❌ Unknown camera type: {camera_type}")
            return False
        
        if camera_type == self.current_camera_type:
            return True  # Already using this camera
        
        old_camera = self.available_cameras[self.current_camera_type]
        new_camera = self.available_cameras[camera_type]
        print(f"🔄 Switching camera: {old_camera} → {new_camera}")
        
        # Store whether we need to restart
        was_running = self.is_running
        
        # Stop current camera if running
        if was_running:
            self.stop_camera()
        
        # Update camera type
        self.current_camera_type = camera_type
        
        # Update panel's resolution list
        self.camera_panel.update_resolutions(
            self.camera_resolutions[camera_type]
        )
        
        # Restart if it was running
        if was_running:
            # Small delay to ensure clean shutdown
            QTimer.singleShot(100, self._restart_after_switch)
        
        return True
    
    def _restart_after_switch(self):
        """Restart camera after type switch."""
        self.start_camera(self.device_sn)

    def set_resolution(self, resolution_str: str):
        """
        Set camera resolution from string like "640x480 @30fps"
        """
        print(f"📹 Resolution changed to: {resolution_str}")
        
        try:
            # Parse resolution string
            res_part = resolution_str.split(' @')[0]
            width, height = map(int, res_part.split('x'))
            fps_part = resolution_str.split('@')[1].replace('fps', '')
            fps = int(fps_part)
            
            # Store for next camera start
            self._target_width = width
            self._target_height = height
            self._target_fps = fps
            
            # If camera is running, restart with new resolution
            if self.is_running:
                print(f"🔄 Restarting camera with new resolution: {width}x{height}")
                was_running = self.is_running
                self.stop_camera()
                
                # Small delay then restart
                from PyQt5.QtCore import QTimer
                QTimer.singleShot(100, lambda: self.start_camera(self.device_sn))
                
        except Exception as e:
            print(f"❌ Failed to parse resolution: {resolution_str}, error: {e}")

    def get_resolutions_for_current(self):
        """Get available resolutions for current camera."""
        return self.camera_resolutions.get(self.current_camera_type, [])
    
    def set_transform_to_world(self, enabled: bool):
        """Toggle point cloud transformation to world frame."""
        if self.pointcloud_display:
            self.pointcloud_display.set_transform_to_world(enabled)
    
    def set_show_frames(self, enabled: bool):
        """Set whether camera frames are shown."""
        if self.pointcloud_display:
            self.pointcloud_display.set_show_frames(enabled)
    
    def show_panel(self):
        """Show the camera control panel."""
        if self.camera_dock:
            self.camera_dock.show()
            self.camera_dock.raise_()
    
    def cleanup(self):
        """Clean up camera resources."""
        if self.pointcloud_display:
            self.pointcloud_display.detach()
        print("CameraManager: Cleaned up")

    def set_roi_axis(self, axis: str, value: float):
        """Set individual ROI axis value."""
        if not self.pointcloud_display:
            return
        
        # Store current ROI values
        if not hasattr(self, '_roi_ranges'):
            self._roi_ranges = {
                'x_min': -2.0, 'x_max': 2.0,
                'y_min': -2.0, 'y_max': 2.0,
                'z_min': 0.1, 'z_max': 2.0
            }
        
        # Update the specific axis
        self._roi_ranges[axis] = value
        
        # Send all ranges to processor
        self.pointcloud_display.set_roi_range(
            self._roi_ranges['x_min'], self._roi_ranges['x_max'],
            self._roi_ranges['y_min'], self._roi_ranges['y_max'],
            self._roi_ranges['z_min'], self._roi_ranges['z_max']
        )

    def set_roi_range(self, x_min: float, x_max: float, 
                    y_min: float, y_max: float,
                    z_min: float, z_max: float):
        """Set complete ROI range all at once."""
        if not self.pointcloud_display:
            return
        
        self.pointcloud_display.set_roi_range(
            x_min, x_max, y_min, y_max, z_min, z_max
        )
