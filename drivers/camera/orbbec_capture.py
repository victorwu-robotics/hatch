"""
Orbbec-specific camera driver.
Thread 1: Capture ONLY - raw frames from Orbbec camera.
"""

import numpy as np
import time
from typing import Optional, Tuple

from .base_camera import BaseCameraDriver

import logging
logger = logging.getLogger(__name__)

class OrbbecDriver(BaseCameraDriver):
    """
    Orbbec-specific camera implementation.
    Handles ONLY camera hardware interaction.
    """
    
    def __init__(self, device_sn: Optional[str] = None):
        super().__init__(device_sn)
        
        # Orbbec-specific
        self.pipeline = None
        self.align_filter = None
        self.point_cloud_filter = None

        # Add resolution settings (defaults)
        self.depth_width = 640
        self.depth_height = 360  # Note: Orbbec often uses 360p for depth
        self.depth_fps = 30
        self.color_width = 640
        self.color_height = 360
        self.color_fps = 30

        # Frame tracking
        self.frame_count = 0
        self.last_emit_time = time.time()

    def set_resolution(self, depth_width=640, depth_height=360, depth_fps=30,
                       color_width=640, color_height=360, color_fps=30):
        """Set resolution (must be called before start_streaming)"""
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.depth_fps = depth_fps
        self.color_width = color_width
        self.color_height = color_height
        self.color_fps = color_fps
        logger.info(f"📹 Orbbec resolution set to {depth_width}x{depth_height} @ {depth_fps}fps")

    def start_streaming(self):
        """Initialize Orbbec camera pipeline"""
        try:
            import pyorbbecsdk
            
            self.pipeline = pyorbbecsdk.Pipeline()
            config = pyorbbecsdk.Config()
            
            # Configure depth stream
            depth_profiles = self.pipeline.get_stream_profile_list(
                pyorbbecsdk.OBSensorType.DEPTH_SENSOR
            )
            depth_profile = None

            # Try to find exact match
            for i in range(depth_profiles.get_count()):
                profile = depth_profiles.get_stream_profile_by_index(i)
                if (profile.get_width() == self.depth_width and 
                    profile.get_height() == self.depth_height and 
                    profile.get_format() == pyorbbecsdk.OBFormat.Y16):
                    depth_profile = profile
                    logger.info(f"✅ Found matching depth profile: {self.depth_width}x{self.depth_height} @ {self.depth_fps}fps")
                    break

            # If exact match not found, try any resolution with Y16 format
            if depth_profile is None:
                for i in range(depth_profiles.get_count()):
                    profile = depth_profiles.get_stream_profile_by_index(i)
                    if profile.get_format() == pyorbbecsdk.OBFormat.Y16:
                        depth_profile = profile
                        logger.info(f"⚠️ Using fallback depth profile: {profile.get_width()}x{profile.get_height()} @ {profile.get_fps()}fps")
                        break
                        
            if depth_profile is None:
                raise RuntimeError("No suitable depth profile found")
            config.enable_stream(depth_profile)

            # Configure color stream
            color_profiles = self.pipeline.get_stream_profile_list(
                pyorbbecsdk.OBSensorType.COLOR_SENSOR
            )
            color_profile = None

            # Try exact match
            for i in range(color_profiles.get_count()):
                profile = color_profiles.get_stream_profile_by_index(i)
                if (profile.get_width() == self.color_width and 
                    profile.get_height() == self.color_height and 
                    profile.get_fps() == self.color_fps and
                    profile.get_format() == pyorbbecsdk.OBFormat.RGB):
                    color_profile = profile
                    break
            
            # Fallback
            if color_profile is None:
                for i in range(color_profiles.get_count()):
                    profile = color_profiles.get_stream_profile_by_index(i)
                    if profile.get_format() == pyorbbecsdk.OBFormat.RGB:
                        color_profile = profile
                        logger.info(f"⚠️ Using fallback color profile: {profile.get_width()}x{profile.get_height()}")
                        break
                        
            if color_profile is None:
                raise RuntimeError("No suitable color profile found")
            config.enable_stream(color_profile)

            # Start pipeline
            self.pipeline.enable_frame_sync()
            self.pipeline.start(config)
            
            # Setup filters
            self.align_filter = pyorbbecsdk.AlignFilter(pyorbbecsdk.OBStreamType.COLOR_STREAM)
            self.point_cloud_filter = pyorbbecsdk.PointCloudFilter()
            self.point_cloud_filter.set_camera_param(self.pipeline.get_camera_param())
            self.point_cloud_filter.set_create_point_format(pyorbbecsdk.OBFormat.RGB_POINT)
            
            self.is_streaming = True
            logger.info(f"✅ OrbbecDriver: Camera hardware ready at {self.depth_width}x{self.depth_height}")
            
        except Exception as e:
            logger.info(f"Failed to start Orbbec camera: {e}")
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
        try:
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                return None, None
            
            aligned_frames = self.align_filter.process(frames)
            if aligned_frames is None:
                return None, None
            
            pc_frame = self.point_cloud_filter.process(aligned_frames)
            if pc_frame is None:
                return None, None
            
            # full_points = self.point_cloud_filter.calculate(pc_frame)

            pc_data = self.point_cloud_filter.calculate(pc_frame)
            if pc_data is not None:
                # Ensure correct dtype before conversion
                if hasattr(pc_data, 'dtype') and pc_data.dtype == np.float64:
                    full_points = pc_data.astype(np.float32)
                else:
                    full_points = pc_data
            else:
                return None, None

            if len(full_points) == 0:
                return None, None
            
            # EXTRACT ONLY - NO TRANSFORMATIONS except unit conversion
            coordinates = full_points[:, :3].copy()
            rgb_colors = full_points[:, 3:6].copy()
            
            # Unit conversion only (mm → m)
            coordinates[:, :3] *= 0.001
            
            return coordinates.astype(np.float32), rgb_colors.astype(np.uint8)
            
        except Exception as e:
            return None, None
    
    def stop_streaming(self):
        """Clean shutdown of Orbbec camera"""
        if self.pipeline:
            try:
                self.pipeline.stop()
                logger.info("✅ OrbbecDriver: Pipeline stopped")
            except:
                pass
            self.is_streaming = False
            self.pipeline = None
            self.align_filter = None
            self.point_cloud_filter = None



if __name__ == "__main__":
    # Quick latency test
    import time
    import numpy as np
    
    print("Starting direct driver test...")
    driver = OrbbecDriver()  # Use your actual driver class
    driver.start_streaming()
    
    time.sleep(2)  # Warm up
    
    times = []
    for i in range(100):
        t0 = time.time()
        points, colors = driver.capture_raw_pointcloud()
        t1 = time.time()
        
        if points is not None:
            times.append(t1 - t0)
            
        if i % 10 == 0:
            print(f"Frame {i}: {(t1-t0)*1000:.1f}ms")
    
    driver.stop_streaming()
    
    print(f"\nResults ({len(times)} frames):")
    print(f"  Mean: {np.mean(times)*1000:.1f}ms")
    print(f"  Std:  {np.std(times)*1000:.1f}ms")
    print(f"  Max:  {np.max(times)*1000:.1f}ms")
