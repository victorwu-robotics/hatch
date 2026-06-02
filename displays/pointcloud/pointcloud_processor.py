"""
Thread 2: Point Cloud Processor - OPTIMIZED
Always processes latest frame, drops old ones.
"""

import numpy as np
import time
from PyQt5.QtCore import QObject, pyqtSignal, QMutex

from core.world_state.transform_registry import TransformRegistry


class PointCloudProcessor(QObject):
    """
    Thread 2: Processes raw point clouds - OPTIMIZED for speed
    """
    
    frame_processed = pyqtSignal(np.ndarray, np.ndarray)
    status_updated = pyqtSignal(str)
    processing_time = pyqtSignal(float)
    
    def __init__(self, registry: TransformRegistry):
        super().__init__()
        self.registry = registry
        self.running = True
        
        # ROI ranges
        self.x_min = -2.0
        self.x_max = 2.0
        self.y_min = -2.0
        self.y_max = 2.0
        self.z_min = 0.1
        self.z_max = 2.0
        
        # Transform settings
        self.transform_to_world = True
        self.camera_optical_frame = None
        self.world_frame = "world"
        
        # OPTIMIZATION 1: Cache transform (only update when needed)
        self._cached_transform = np.eye(4)
        self._transform_frame_count = 0
        self._transform_update_interval = 30  # Update transform every 30 frames
        self._transform_mutex = QMutex()
        
        # OPTIMIZATION 2: Pre-allocated arrays (avoid repeated allocation)
        self._homogeneous_buffer = None
        self._buffer_size = 0
        
        # Performance tracking
        self.frame_count = 0
        self.last_log_time = time.time()
        self.last_frame_time = time.time()
        self.queue_size = 0
        self._processing = False

        # Optimizing z-clipping
        # Pre-allocate mask array (reused each frame)
        self._mask = None
        self.max_points = 640 * 480    # Maximum expected points
        # Pre-allocate mask at maximum size ONCE
        self._mask = np.empty(self.max_points, dtype=bool)
        print(f"✅ Pre-allocated mask for {self.max_points} points")
        self._valid_check_counter = 0
        self._check_validity_every = 30 # Check validity every 30 frames

        # Initialize transform cache
        self._cached_R = np.eye(3, dtype=np.float32)
        self._cached_t = np.zeros(3, dtype=np.float32)
        self._transform_frame_count = 0
        
    def process_frame(self, raw_points: np.ndarray, raw_colors: np.ndarray):
        """
        Ultra-optimized point cloud processing with detailed timing.
        Process frame and RETURN the result (no signal)
        """
        # Overall timing
        frame_start = time.perf_counter()
        
        # Step timings
        t0 = time.perf_counter()
        transform_time = 0
        mask_time = 0
        dtype_time = 0
        emit_time = 0
        
        try:
            points = raw_points
            colors = raw_colors
            
            if len(points) == 0:
                return points, colors
            
            # ===== 3D ROI CLIPPING =====
            current_size = len(points)
            mask_view = self._mask[:current_size]
            mask_view[:] = True
            
            # Apply all range filters
            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]
            
            # X range
            np.greater_equal(x, self.x_min, out=mask_view)
            np.logical_and(mask_view, x <= self.x_max, out=mask_view)
            
            # Y range
            temp_mask = np.empty_like(mask_view)
            np.greater_equal(y, self.y_min, out=temp_mask)
            np.logical_and(temp_mask, y <= self.y_max, out=temp_mask)
            np.logical_and(mask_view, temp_mask, out=mask_view)
            
            # Z range
            np.greater_equal(z, self.z_min, out=temp_mask)
            np.logical_and(temp_mask, z <= self.z_max, out=temp_mask)
            np.logical_and(mask_view, temp_mask, out=mask_view)

            # Apply mask (unavoidable copy for result)
            points = points[mask_view]
            colors = colors[mask_view]
            
            if len(points) == 0:
                return points, colors
            
            # ===== TRANSFORM (if enabled) - NOW on already clipped points =====
            if self.transform_to_world:
                self._transform_frame_count += 1
                if self._transform_frame_count % 30 == 0:
                    print(f"world_frame:{self.world_frame}; camera_optical_frame:{self.camera_optical_frame}")
                    T = self.registry.get_transform(self.world_frame, self.camera_optical_frame)
                    self._cached_R = T[:3, :3].astype(np.float32, copy=False)
                    self._cached_t = T[:3, 3].astype(np.float32, copy=False)

                    # DEBUG: Print the transform
                    # print(f"🔍 Camera→World transform:")
                    # print(f"   Rotation:\n{self._cached_R}")
                    # print(f"   Translation: {self._cached_t}")
                
                # Apply transform to already clipped points
                points = points @ self._cached_R.T + self._cached_t
            
            # ===== ENSURE CORRECT DTYPES =====
            if points.dtype != np.float32:
                points = points.astype(np.float32, copy=False)
            if colors.dtype != np.uint8:
                colors = np.clip(colors, 0, 255).astype(np.uint8, copy=False)
            
            return points, colors
            
        except Exception as e:
            print(f"❌ Processor error: {e}")
            return np.array([]), np.array([])
    
    def set_transform_to_world(self, enabled: bool):
        """Enable/disable world transform"""
        self.transform_to_world = enabled
    
    def set_camera_frame(self, frame_name: str):
        """Set the camera optical frame name"""
        self.camera_optical_frame = frame_name
    
    def set_z_range(self, z_min: float, z_max: float):
        """Set depth clipping range"""
        self.z_min = z_min
        self.z_max = z_max
        print(f"📏 Processor z-range: {z_min}m - {z_max}m")

    def set_roi_range(self, x_min: float, x_max: float, 
                    y_min: float, y_max: float,
                    z_min: float, z_max: float):
        """Set 3D region of interest."""
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        print(f"📏 ROI set: X[{x_min:.1f}, {x_max:.1f}], Y[{y_min:.1f}, {y_max:.1f}], Z[{z_min:.1f}, {z_max:.1f}]")

    def stop(self):
        """Stop processing"""
        self.running = False

if __name__ == "__main__":
    import numpy as np
    from core.world_state.transform_registry import TransformRegistry
    
    registry = TransformRegistry()
    # Register a test camera frame at a known transform
    T = np.eye(4)
    T[:3, 3] = [1.0, 0.0, 0.5]  # Camera 1m in X, 0.5m in Z from world
    registry.register_frame("test_camera_frame", T, parent="world")
    
    processor = PointCloudProcessor(registry, camera_optical_frame="camera_depth_optical_frame")
    processor.set_roi_range(-1.0, 1.0, -1.0, 1.0, 0.1, 2.0)
    
    # Synthetic points: some inside ROI, some outside
    test_points = np.array([
        [0.0, 0.0, 0.5],   # Inside
        [2.0, 0.0, 0.5],   # Outside X
        [0.0, 0.0, 0.05],  # Outside Z (too close)
        [0.0, 0.0, 1.5],   # Inside
    ], dtype=np.float32)
    test_colors = np.array([
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
    ], dtype=np.uint8)
    
    print(f"Input: {len(test_points)} points")
    points_out, colors_out = processor.process_frame(test_points, test_colors)
    print(f"Output: {len(points_out)} points")
    print(f"Points:\n{points_out}")
    print(f"Colors:\n{colors_out}")
