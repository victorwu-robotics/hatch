"""
Point Cloud Display - SINGLE THREADED
Combines capture, process, and render in one loop.
No threads, no signal queues, no growing lag.
"""

import time
from typing import Optional
from PyQt5.QtCore import QTimer

from core.world_state.transform_registry import TransformRegistry
from .pointcloud_processor import PointCloudProcessor
from .pointcloud_renderer import PointCloudRenderer


class PointCloudDisplay:
    """
    Single-threaded point cloud display.
    Everything runs in the main Qt thread at 30 FPS.
    """
    
    def __init__(self, registry: TransformRegistry, kinematic_model=None, asset_id=None):
        """
        Initialize point cloud display.
        
        Args:
            registry: Shared transform registry
        """
        self.registry = registry
        self.kinematic_model = kinematic_model
        self.asset_id = asset_id
        
        # Components (no threads)
        self.camera = None
        self.processor = PointCloudProcessor(registry)
        self.renderer = PointCloudRenderer(registry)
        
        # State
        self.is_attached = False
        self.is_running = False
        self.device_sn = None
        self._needs_render = False
        
        # Performance tracking
        self.frame_count = 0
        self.last_stats_time = time.time()
        self.capture_times = []
        self.process_times = []
        self.render_times = []
        self.total_times = []
        
        # Timer for main loop (30 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self._update)
        
    def attach(self, renderer, engine=None):
        """Attach point cloud display to renderer"""
        if self.is_attached:
            return
        
        self.renderer.attach(renderer)
        if engine:
            engine.register_display(self)
        self._needs_render = False
        self.is_attached = True
        print("✅ PointCloudDisplay: Attached (single-threaded)")
    
    def start_camera(self, device_sn: Optional[str] = None, 
                    camera_type: str = "orbbec",
                    width: int = 640, height: int = 360, fps: int = 30,
                    asset_id=None, kinematic_model=None) -> bool:
        """
        Start camera with specified resolution.
        Note: Orbbec default is 640x360, RealSense default is 640x480
        """
        if not self.is_attached:
            print("PointCloudDisplay: Cannot start camera - not attached")
            return False
        
        self.device_sn = device_sn
        
        try:
            if camera_type.lower() == "realsense":
                from drivers.camera.realsense_capture import RealSenseDriver
                self.camera = RealSenseDriver(device_sn)
                # RealSense expects same resolution for depth and color
                self.camera.set_resolution(width, height, width, height)
                print(f"✅ Using RealSense D435 at {width}x{height}")
                
            elif camera_type.lower() == "orbbec":
                from drivers.camera.orbbec_capture import OrbbecDriver
                self.camera = OrbbecDriver(device_sn)
                # Orbbec can have different depth/color resolutions
                self.camera.set_resolution(
                    depth_width=width, depth_height=height, depth_fps=fps,
                    color_width=width, color_height=height, color_fps=fps
                )
                print(f"✅ Using Orbbec Gemini 335 at {width}x{height}")
                
            else:
                print(f"❌ Unknown camera type: {camera_type}")
                return False
                
            self.camera.start_streaming()
            print(f"✅ Camera initialized")

            if kinematic_model and asset_id:
                print(f"✅ Going to find Camera Optical Frame")
                optical_frame = self._find_camera_optical_frame(kinematic_model, asset_id)
                if optical_frame:
                    self.set_camera_frame(optical_frame)
                    print(f"📷 Camera optical frame: {optical_frame}")
                else:
                    print("⚠️ No camera depth optical frame found in URDF")

        except Exception as e:
            print(f"❌ Failed to start camera: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        # ... rest of method (processor setup, etc.) ...
        
        # Reset performance tracking
        self.frame_count = 0
        self.last_stats_time = time.time()
        self.capture_times = []
        self.process_times = []
        self.render_times = []
        self.total_times = []
        
        # Start main loop (30 FPS)
        self.is_running = True
        self.timer.start(33)  # 33ms = ~30 FPS
        
        print("✅ PointCloudDisplay: Single-threaded pipeline started")
        return True

    def _find_camera_optical_frame(self, kinematic_model, asset_id):
        """Find the camera depth optical frame from the kinematic model."""
        for link_name in kinematic_model.link_transforms.keys():
            if 'depth_optical_frame' in link_name.lower():
                return f"{asset_id}_{link_name}"
        return None

    def _update(self):
        """
        Main update loop - called at 30 FPS by QTimer.
        Combines capture, process, and render in one thread.
        """
        if not self.is_running or not self.camera:
            return
        
        frame_start = time.time()
        
        # ===== 1. CAPTURE =====
        t0 = time.time()
        points, colors = self.camera.capture_raw_pointcloud()
        capture_time = (time.time() - t0) * 1000
        
        if points is None or len(points) == 0:
            return  # No frame, try again next cycle
        
        # ===== 2. PROCESS =====
        t1 = time.time()
        # Call processor directly (no signal)
        points, colors = self.processor.process_frame(points, colors)
        process_time = (time.time() - t1) * 1000
        
        if points is None or len(points) == 0:
            return  # Processing filtered out all points
        
        # ===== 3. RENDER =====
        t2 = time.time()
        # Update renderer directly (no signal)
        self.renderer.update_point_cloud(points, colors)
        self._needs_render = True
        render_time = (time.time() - t2) * 1000
        
        # ===== Track Performance =====
        total_time = (time.time() - frame_start) * 1000
        
        # Store times for averaging (keep last 30 frames)
        self.capture_times.append(capture_time)
        self.process_times.append(process_time)
        self.render_times.append(render_time)
        self.total_times.append(total_time)
        
        if len(self.capture_times) > 30:
            self.capture_times.pop(0)
            self.process_times.pop(0)
            self.render_times.pop(0)
            self.total_times.pop(0)
        
        # Statistics every 30 frames
        self.frame_count += 1
        if self.frame_count >= 30:
            # self._print_stats()
            self.frame_count = 0
    
    def _print_stats(self):
        """Print performance statistics"""
        if not self.total_times:
            return
        
        avg_capture = sum(self.capture_times) / len(self.capture_times)
        avg_process = sum(self.process_times) / len(self.process_times)
        avg_render = sum(self.render_times) / len(self.render_times)
        avg_total = sum(self.total_times) / len(self.total_times)
        max_total = max(self.total_times)
        
        fps = 1000 / avg_total if avg_total > 0 else 0
        
        print(f"\n📊 Single-Thread Performance (last {len(self.total_times)} frames):")
        print(f"  Capture:  {avg_capture:.1f}ms")
        print(f"  Process:  {avg_process:.1f}ms")
        print(f"  Render:   {avg_render:.1f}ms")
        print(f"  TOTAL:    {avg_total:.1f}ms (max: {max_total:.1f}ms) → {fps:.1f} FPS")
        print(f"  Points:   {self.renderer.point_count if hasattr(self.renderer, 'point_count') else 0}")
    
    def stop_camera(self):
        """Stop camera and main loop"""
        self.is_running = False
        self.timer.stop()
        
        if self.camera:
            self.camera.stop_streaming()
            self.camera = None
        
        print("✅ PointCloudDisplay: Stopped")
    
    # ===== Forwarding methods =====
    
    def set_visible(self, visible: bool):
        """Toggle point cloud visibility"""
        self.renderer.set_visible(visible)
    
    def set_show_frames(self, show: bool):
        """Toggle frame axes visibility"""
        self.renderer.set_show_frames(show)
    
    def set_transform_to_world(self, enabled: bool):
        """Enable/disable world transform"""
        self.processor.set_transform_to_world(enabled)
    
    def set_camera_frame(self, frame_name: str):
        """Set camera optical frame name"""
        self.processor.set_camera_frame(frame_name)
        self.renderer.camera_optical_frame = frame_name
    
    def set_roi_range(self, x_min: float, x_max: float, 
                    y_min: float, y_max: float,
                    z_min: float, z_max: float):
        """Set 3D region of interest for point cloud clipping."""
        if self.processor:
            self.processor.set_roi_range(x_min, x_max, y_min, y_max, z_min, z_max)

    def clear(self):
        """Clear point cloud"""
        self.renderer.clear()
    
    def detach(self):
        """Clean up all resources"""
        self.stop_camera()
        self.renderer.detach()
        self.is_attached = False
        print("PointCloudDisplay: Detached")