"""
RealSense-specific camera driver.
Thread 1: Capture ONLY - raw frames from Intel RealSense camera.
Matches the interface of OrbbecDriver for drop-in replacement.
"""

import numpy as np
import time
import cv2
from typing import Optional, Tuple
import pyrealsense2 as rs

from .base_camera import BaseCameraDriver, CameraCaptureThread


class RealSenseDriver(BaseCameraDriver):
    """
    Intel RealSense D435/D435i camera implementation.
    Handles ONLY camera hardware interaction.
    Matches OrbbecDriver interface for seamless swapping.
    """
    
    def __init__(self, device_sn: Optional[str] = None):
        super().__init__(device_sn)
        
        # RealSense-specific
        self.pipeline = None
        self.config = None
        self.align = None
        self.colorizer = None
        self.device = None
        self.depth_scale = None
        
        # Frame tracking
        self.frame_count = 0
        self.last_emit_time = time.time()
        
        # Performance tracking
        self.capture_times = []
        
        # Default to D435/D435i compatible settings
        self.depth_width = 640
        self.depth_height = 480
        self.depth_fps = 30
        self.color_width = 640
        self.color_height = 480
        self.color_fps = 30
        
    def start_streaming(self):
        """Initialize RealSense camera pipeline"""
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # If specific device requested, use it
            if self.device_sn:
                self.config.enable_device(self.device_sn)
            
            # Configure streams
            self.config.enable_stream(rs.stream.depth, 
                                      self.depth_width, 
                                      self.depth_height, 
                                      rs.format.z16, 
                                      self.depth_fps)
            self.config.enable_stream(rs.stream.color, 
                                      self.color_width, 
                                      self.color_height, 
                                      rs.format.bgr8, 
                                      self.color_fps)
            
            # Start pipeline
            profile = self.pipeline.start(self.config)
            
            # Get device info
            self.device = profile.get_device()
            print(f"📷 RealSense Device: {self.device.get_info(rs.camera_info.name)}")
            print(f"   Serial: {self.device.get_info(rs.camera_info.serial_number)}")
            print(f"   Firmware: {self.device.get_info(rs.camera_info.firmware_version)}")
            
            # Get depth scale for conversion
            depth_sensor = self.device.first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"   Depth scale: {self.depth_scale:.4f} m/unit")
            
            # Create align object to align depth to color
            self.align = rs.align(rs.stream.color)
            
            # Optional: colorizer for debugging
            self.colorizer = rs.colorizer()
            
            print(f"✅ RealSenseDriver: Camera hardware ready")
            print(f"   Depth: {self.depth_width}x{self.depth_height} @ {self.depth_fps}fps")
            print(f"   Color: {self.color_width}x{self.color_height} @ {self.color_fps}fps")
            
        except Exception as e:
            print(f"❌ Failed to start RealSense camera: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def capture_raw_pointcloud(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture raw point cloud.
        
        Returns:
            points: (N, 3) float32 in OPTICAL frame (meters)
            colors: (N, 3) uint8 RGB
        """
        start_time = time.time()
        
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            if frames is None:
                print("No Frames ....")
                return None, None
            # Align depth to color
            aligned_frames = self.align.process(frames)
            
            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                print("No Frames after alignment ....")
                return None, None
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Convert BGR to RGB
            color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # Get intrinsics for point cloud calculation
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

            # Generate point cloud
            points, colors = self._depth_to_pointcloud(
                depth_image, 
                color_image_rgb,
                depth_intrinsics,
                self.depth_scale
            )
            
            if points is None or len(points) == 0:
                print("No Points ....")
                return None, None
            
            # Track performance
            capture_time = (time.time() - start_time) * 1000
            self.capture_times.append(capture_time)
            if len(self.capture_times) > 30:
                self.capture_times.pop(0)

            # Periodic stats
            self.frame_count += 1
            now = time.time()
            if now - self.last_emit_time > 5.0:
                avg_time = sum(self.capture_times) / len(self.capture_times)
                print(f"📊 RealSense: {self.frame_count/5:.1f} FPS, {avg_time:.1f}ms, {len(points)} points")
                self.frame_count = 0
                self.last_emit_time = now

            return points.astype(np.float32), colors.astype(np.uint8)
            
        except Exception as e:
            # Silent fail for performance
            return None, None
    
    def _depth_to_pointcloud(self, depth_image, color_image, intrinsics, depth_scale):
        """
        Convert depth + color to point cloud.
        Matches the output format of Orbbec's point cloud filter.
        """
        height, width = depth_image.shape
        
        # Create meshgrid of pixel coordinates
        v, u = np.mgrid[0:height, 0:width]
        
        # Flatten arrays
        u = u.flatten()
        v = v.flatten()
        depth = depth_image.flatten().astype(np.float32) * depth_scale
        
        # Filter out zero depth
        valid = depth > 0
        u = u[valid]
        v = v[valid]
        depth = depth[valid]
        
        if len(depth) == 0:
            return None, None
        
        # Convert to 3D points using intrinsics
        points = np.zeros((len(depth), 3), dtype=np.float32)
        
        # X = (u - cx) * depth / fx
        # Y = (v - cy) * depth / fy
        # Z = depth
        
        points[:, 0] = (u - intrinsics.ppx) * depth / intrinsics.fx
        points[:, 1] = (v - intrinsics.ppy) * depth / intrinsics.fy
        points[:, 2] = depth
        
        # Get corresponding colors
        colors = color_image[v, u]  # Shape: (N, 3)
        
        return points, colors
    
    def get_intrinsics(self):
        """Get camera intrinsics (useful for calibration)"""
        if not self.pipeline:
            return None
        
        # Get a frame to extract intrinsics
        frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        if frames:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                return depth_frame.profile.as_video_stream_profile().intrinsics
        return None
    
    def set_laser_power(self, power: float):
        """Set laser power (0.0 to 1.0)"""
        if self.device:
            depth_sensor = self.device.first_depth_sensor()
            if depth_sensor.supports(rs.option.laser_power):
                max_power = depth_sensor.get_option_range(rs.option.laser_power).max
                depth_sensor.set_option(rs.option.laser_power, power * max_power)
    
    def set_resolution(self, depth_width=640, depth_height=480, 
                       color_width=640, color_height=480):
        """Change resolution (must be called before start_streaming)"""
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.color_width = color_width
        self.color_height = color_height
    
    def stop_streaming(self):
        """Clean shutdown of RealSense camera"""
        if self.pipeline:
            try:
                self.pipeline.stop()
                print(f"✅ RealSenseDriver: Pipeline stopped")
            except:
                pass
            self.pipeline = None
            self.config = None
            self.align = None
            self.device = None


# Convenience function to create a capture thread
def create_realsense_capture_thread(device_sn: Optional[str] = None) -> CameraCaptureThread:
    """
    Create a RealSense capture thread.
    Drop-in replacement for create_orbbec_capture_thread.
    """
    driver = RealSenseDriver(device_sn)
    thread = CameraCaptureThread(driver)
    return thread


# For ultra-fast mode (lower resolution)
def create_fast_realsense_thread(device_sn: Optional[str] = None) -> CameraCaptureThread:
    """Create a fast RealSense capture thread with lower resolution"""
    driver = RealSenseDriver(device_sn)
    driver.set_resolution(424, 240, 424, 240)  # Lower resolution for speed
    thread = CameraCaptureThread(driver)
    return thread