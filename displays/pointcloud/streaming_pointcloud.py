"""
Zero-copy VTK point cloud visualization.
Direct memory overwrite for maximum performance.
"""

import numpy as np
import vtk
import time
from vtk.util import numpy_support


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