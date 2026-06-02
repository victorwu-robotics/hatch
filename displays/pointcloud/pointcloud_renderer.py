"""
Thread 3: Point Cloud Renderer - BACK TO BASICS
Exactly like the original working version
"""

import numpy as np
import vtk
import time
from vtk.util import numpy_support

from core.world_state.transform_registry import TransformRegistry


class PointCloudRenderer:
    """
    Thread 3: Runs in Qt main thread.
    Simplified version - exactly like original working code.
    """
    
    def __init__(self, registry: TransformRegistry, camera_optical_frame: str = None):
        self.registry = registry
        self.camera_optical_frame = camera_optical_frame
        self.renderer = None
        self.point_cloud = None
        self.optical_frame = None
        self.body_frame = None
        self._is_visible = True
        self._show_frames = True
        self._needs_render = False
        self._has_valid_pointcloud = False
        self.point_count = 0

        self.last_log_time = time.time()
        self.frame_count = 0
        
        self.registry.register_callback(self._on_transform_updated)
        
    def attach(self, renderer: vtk.vtkRenderer):
        """Attach to VTK renderer - exactly like original"""
        self.renderer = renderer
        self.point_cloud = StreamingPointCloud(renderer, max_points=640 * 480)
        
        print("PointCloudRenderer: Attached to renderer")

    def update_point_cloud(self, points: np.ndarray, colors: np.ndarray):
        """
        Update point cloud with processed frame - exactly like original
        """
        if not self._is_visible or not self.point_cloud:
            return
        
        if len(points) > 0:
            # Just update - no fancy visibility settings
            self.point_count = len(points)
            self.point_cloud.update(points, colors)
            self._needs_render = True   # Signal the engine to render
            
            # Signal first frame
            if not self._has_valid_pointcloud:
                self._has_valid_pointcloud = True
                print(f"✅ PointCloud ready: {len(points)} points")
            
            # Logging (every second)
            now = time.time()
            if now - self.last_log_time > 1.0:
                # print(f"🎨 Render: {self.frame_count} FPS, {len(points)} points")
                self.frame_count = 0
                self.last_log_time = now
    
    def _on_transform_updated(self, frame_name: str, transform: np.ndarray):
        """Handle transform updates from registry"""
        # print(f"📢 Transform update: {frame_name}")  # See ALL incoming frames

        if not self._show_frames or not self.renderer:
            return
        
        # Update camera frames
        # if frame_name == "bunker_with_arm_camera_depth_optical_frame":
        if self.camera_optical_frame and frame_name == self.camera_optical_frame:
            # if self.body_frame is None:
                # self.body_frame = FrameAxes(self.renderer, length=0.1)
            self.body_frame.set_transform(transform)
            print(f"🔍 Updated depth optical frame")
        
        elif frame_name == "camera_depth_optical_frame":
            # if self.optical_frame is None:
                # self.optical_frame = FrameAxes(self.renderer, length=0.1)
            self.optical_frame.set_transform(transform)
    
    def set_visible(self, visible: bool):
        """Toggle point cloud visibility"""
        self._is_visible = visible
        if self.point_cloud:
            self.point_cloud.set_visible(visible)
    
    def set_show_frames(self, show: bool):
        """Toggle frame axes visibility"""
        self._show_frames = show
        if self.body_frame:
            self.body_frame.set_visible(show)
        if self.optical_frame:
            self.optical_frame.set_visible(show)
    
    def clear(self):
        """Clear point cloud"""
        if self.point_cloud:
            self.point_cloud.clear()
    
    def detach(self):
        """Clean up resources"""
        self.registry.remove_callback(self._on_transform_updated)
        self.clear()
        self.point_cloud = None
        if self.optical_frame:
            self.optical_frame.detach()
            self.optical_frame = None
        if self.body_frame:
            self.body_frame.detach()
            self.body_frame = None
        self.renderer = None
        print("PointCloudRenderer: Detached")

class StreamingPointCloud:
    """
    Zero-copy VTK point cloud visualization.
    Direct memory overwrite for maximum performance.
    """
    
    def __init__(self, renderer: vtk.vtkRenderer, max_points: int = 640*480):
        self.renderer = renderer
        self.max_points = max_points
        self.current_size = 0
        
        # Points array
        self.points = vtk.vtkPoints()
        points_array = numpy_support.numpy_to_vtk(
            np.zeros((max_points, 3), dtype=np.float32),
            deep=False,
            array_type=vtk.VTK_FLOAT
        )
        self.points.SetData(points_array)
        
        # Colors array
        colors_array = numpy_support.numpy_to_vtk(
            np.zeros((max_points, 3), dtype=np.uint8),
            deep=False,
            array_type=vtk.VTK_UNSIGNED_CHAR
        )
        colors_array.SetNumberOfComponents(3)
        colors_array.SetName("Colors")
        
        # PolyData
        self.polydata = vtk.vtkPolyData()
        self.polydata.SetPoints(self.points)
        self.polydata.GetPointData().SetScalars(colors_array)
        
        # Vertices cell array
        self.vertices = vtk.vtkCellArray()
        self.vertices.Allocate(max_points * 2)
        self.polydata.SetVerts(self.vertices)
        
        # Mapper & Actor
        self.mapper = vtk.vtkPolyDataMapper()
        # self.mapper = vtk.vtkPointGaussianMapper()
        self.mapper.SetInputData(self.polydata)
        # self.mapper.SetEmissive(False)
        # self.mapper.SetScaleFactor(0.001)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.renderer.AddActor(self.actor)
        
        # Numpy views for zero-copy updates
        self.points_view = numpy_support.vtk_to_numpy(self.points.GetData()).reshape(-1, 3)
        self.colors_view = numpy_support.vtk_to_numpy(
            self.polydata.GetPointData().GetScalars()
        ).reshape(-1, 3)
        
        print(f"StreamingPointCloud: Initialized with {max_points} max points")
    
    def update(self, points_np: np.ndarray, colors_np: np.ndarray):
        """ZERO-COPY update - direct memory overwrite."""
        n_points = points_np.shape[0]
        
        if n_points == 0 or n_points > self.max_points:
            return
        
        # Direct memory overwrite
        self.points_view[:n_points] = points_np
        self.colors_view[:n_points] = colors_np
        
        # Update vertices with actual point count
        self.vertices.Reset()
        self.vertices.InsertNextCell(n_points)
        for i in range(n_points):
            self.vertices.InsertCellPoint(i)
        self.polydata.SetVerts(self.vertices)
        
        # Mark as modified
        self.points.Modified()
        self.polydata.GetPointData().GetScalars().Modified()
        self.polydata.Modified()
        
        self.current_size = n_points
    
    def set_visible(self, visible: bool):
        """Toggle visibility"""
        self.actor.SetVisibility(visible)
    
    def clear(self):
        """Remove all points"""
        self.update(
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.uint8)
        )
    
    def get_actor(self):
        """Get the VTK actor"""
        return self.actor