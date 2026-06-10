"""
PHASE 1 - Visualizer Engine Core (VTK Direct)
Contract: Creates and configures a VTK render widget properly embedded in Qt.
Now with distance-preserving camera views.
"""

import vtk
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from typing import Tuple
from dataclasses import dataclass

import logging
logger = logging.getLogger(__name__)

@dataclass
class RenderConfig:
    """Performance-critical rendering parameters"""
    width: int = 1280
    height: int = 720
    background_color: Tuple[float, float, float] = (1.0, 1.0, 1.0)  # White
    point_size: float = 2.0
    anti_aliasing: bool = False
    vsync: bool = False
    use_display_lists: bool = True
    # Grid settings
    grid_size: float = 0.1  # Default 100mm in meters
    grid_color: Tuple[float, float, float] = (0.8, 0.8, 0.8)  # Light gray
    grid_extent: float = 2.0  # How far the grid extends in each direction
    grid_cells: int = 20  # Number of grid cells in each direction


class VisualizerEngine:
    """
    A minimal engine to create and manage a VTK-based 3D visualization.
    Configuration: White background, visible grid, visible axes.
    Now with distance-preserving camera views.
    """

    def __init__(self, title="RoboPlatform Visualizer", config: RenderConfig = None, parent=None):
        """
        Creates and configures the VTK render widget.
        
        Args:
            title (str): The title for the window (unused, kept for compatibility).
            config (RenderConfig): Rendering configuration.
            parent (QWidget): Parent Qt widget.
        """
        self.config = config or RenderConfig()
        
        # Create a container widget for VTK
        self.container = QWidget(parent)
        self.container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.layout = QVBoxLayout(self.container)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # Create the VTK widget
        self.vtk_widget = QVTKRenderWindowInteractor(self.container)
        self.vtk_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(self.vtk_widget)
        
        # Get the render window and renderer
        self.render_window = self.vtk_widget.GetRenderWindow()
        self.renderer = vtk.vtkRenderer()
        self.render_window.AddRenderer(self.renderer)
        
        # Configure renderer
        self.renderer.SetBackground(self.config.background_color)
        
        # Configure render window
        self.render_window.SetSize(self.config.width, self.config.height)
        self.render_window.SetWindowName(title)
        
        # Performance settings
        if not self.config.anti_aliasing:
            self.render_window.SetMultiSamples(0)
        if not self.config.vsync:
            self.render_window.SetSwapControl(0)  # Disable VSync
        
        # Get interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        
        # Camera style
        self.camera_style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(self.camera_style)
        
        # Store grid actors
        self.grid_actor = None
        self.grid_lines_actor = None
        
        # Store camera state
        self.default_camera_position = (3, -3, 3)
        self.default_focal_point = (0, 0, 0)
        self.default_view_up = (0, 0, 1)
        self.current_distance = None  # Store current camera distance
        
        # Apply standard configuration
        self._setup_grid()
        self._setup_axes()
        
        # Initialize the interactor
        self.interactor.Initialize()
        
        # Set default view
        self.set_view_isometric()
        
        # Force initial render
        self.render()
        
        # Displays...
        self.displays = []
        self._setup_render_timer()

        logger.info(f"VisualizerEngine: VTK renderer created")
        logger.info(f"VisualizerEngine: Grid size: {self.config.grid_size}m, extent: ±{self.config.grid_extent}m")
        logger.info(f"VisualizerEngine: Embedded VTK widget in Qt container")

    def _get_camera_distance(self) -> float:
        """Get the current distance from camera to focal point."""
        camera = self.renderer.GetActiveCamera()
        pos = np.array(camera.GetPosition())
        focal = np.array(camera.GetFocalPoint())
        return float(np.linalg.norm(pos - focal))

    def _set_camera_position_with_distance(self, direction: Tuple[float, float, float], 
                                           distance: float = None):
        """
        Set camera position along a given direction while preserving distance.
        
        Args:
            direction: Unit vector direction for the camera (from focal point)
            distance: Desired distance (uses current distance if None)
        """
        if distance is None:
            distance = self._get_camera_distance()
        
        # Normalize the direction vector
        direction = np.array(direction)
        direction = direction / np.linalg.norm(direction)
        
        # Calculate new position
        focal = np.array(self.renderer.GetActiveCamera().GetFocalPoint())
        new_position = focal + direction * distance
        
        # Set camera position
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(new_position)
        
        logger.info(f"VisualizerEngine: Camera distance preserved: {distance:.2f}m")

    # ===== DISTANCE-PRESERVING VIEW METHODS =====
    
    def set_view_top(self, preserve_distance: bool = True):
        camera = self.renderer.GetActiveCamera()  # Define at start
        distance = None if not preserve_distance else self._get_camera_distance()
        
        direction = (0, 0, 1)
        
        if preserve_distance:
            self._set_camera_position_with_distance(direction, distance)
        else:
            camera.SetPosition(0, 0, self.config.grid_extent * 3)
            camera.SetFocalPoint(0, 0, 0)
        
        camera.SetViewUp(0, 1, 0)
        self.renderer.ResetCameraClippingRange()
        self.render()

    def set_view_front(self, preserve_distance: bool = True):
        camera = self.renderer.GetActiveCamera()
        distance = None if not preserve_distance else self._get_camera_distance()
        
        direction = (0, -1, 0)
        
        if preserve_distance:
            self._set_camera_position_with_distance(direction, distance)
        else:
            camera.SetPosition(0, -self.config.grid_extent * 3, 0)
            camera.SetFocalPoint(0, 0, 0)
        
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self.render()

    def set_view_side(self, preserve_distance: bool = True):
        camera = self.renderer.GetActiveCamera()
        distance = None if not preserve_distance else self._get_camera_distance()
        
        direction = (1, 0, 0)
        
        if preserve_distance:
            self._set_camera_position_with_distance(direction, distance)
        else:
            camera.SetPosition(self.config.grid_extent * 3, 0, 0)
            camera.SetFocalPoint(0, 0, 0)
        
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self.render()

    def set_view_isometric(self, preserve_distance: bool = True):
        """
        Set camera to isometric view.
        
        Args:
            preserve_distance: If True, maintain current zoom distance
        """
        camera = self.renderer.GetActiveCamera()  # Define camera at the start!
        distance = None if not preserve_distance else self._get_camera_distance()
        
        # Direction for isometric view (equal components)
        direction = (1, -1, 1)  # Isometric direction
        
        if preserve_distance:
            self._set_camera_position_with_distance(direction, distance)
        else:
            distance = self.config.grid_extent * 2.5
            direction_norm = np.array(direction) / np.linalg.norm(direction)
            new_position = direction_norm * distance
            camera.SetPosition(new_position)
            camera.SetFocalPoint(0, 0, 0)
        
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self.render()
        
        mode = "preserving distance" if preserve_distance else "default distance"
        logger.info(f"VisualizerEngine: Set view to ISOMETRIC ({mode})")

    def zoom_to_fit(self, target: str = "all"):
        """
        Zoom camera to fit specified target.
        
        Args:
            target: What to fit - 'all', 'robot', 'grid', or 'selection'
        """
        if target == "grid":
            # Fit just the grid
            bounds = [-self.config.grid_extent, self.config.grid_extent,
                     -self.config.grid_extent, self.config.grid_extent,
                     0, 0]  # Grid is at z=0
            self.renderer.ResetCamera(bounds)
        elif target == "robot" and self._has_robot():
            # Fit the robot (would need to get robot bounds)
            self.renderer.ResetCamera()  # For now, just fit all
        else:
            # Fit everything
            self.renderer.ResetCamera()
        
        self.renderer.ResetCameraClippingRange()
        self.render()
        
        # Store the new distance
        self.current_distance = self._get_camera_distance()
        logger.info(f"VisualizerEngine: Zoomed to fit {target}, new distance: {self.current_distance:.2f}m")

    def _has_robot(self) -> bool:
        """Check if there's a robot in the scene."""
        # This would need to be implemented based on your robot detection
        return True

    def reset_view(self, preserve_distance: bool = False):
        """
        Reset camera to default isometric view.
        
        Args:
            preserve_distance: If True, maintain current zoom distance
        """
        self.set_view_isometric(preserve_distance)

    def set_view(self, view_name: str, preserve_distance: bool = True):
        """
        Set camera to predefined view.
        
        Args:
            view_name: One of 'top', 'front', 'side', 'isometric', 'reset', 'fit', 'fit_grid', 'fit_robot'
            preserve_distance: If True, maintain current zoom distance when switching views
        """
        view_map = {
            'top': lambda: self.set_view_top(preserve_distance),
            'front': lambda: self.set_view_front(preserve_distance),
            'side': lambda: self.set_view_side(preserve_distance),
            'isometric': lambda: self.set_view_isometric(preserve_distance),
            'reset': lambda: self.reset_view(preserve_distance),
            'fit': lambda: self.zoom_to_fit('all'),
            'fit_grid': lambda: self.zoom_to_fit('grid'),
            'fit_robot': lambda: self.zoom_to_fit('robot')
        }
        
        if view_name in view_map:
            view_map[view_name]()
        else:
            logger.debug(f"VisualizerEngine: Unknown view '{view_name}'")

    # ===== GRID METHODS =====
    
    def _setup_grid(self):
        """Add a ground grid to the scene with configurable size and color."""
        extent = self.config.grid_extent
        grid_size = self.config.grid_size
        grid_color = self.config.grid_color
        
        # Calculate number of lines
        num_lines = int(2 * extent / grid_size) + 1
        
        # Create grid lines
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        
        # Generate grid lines
        for i in range(num_lines):
            x = -extent + i * grid_size
            
            # Vertical lines (along Y direction)
            points.InsertNextPoint(x, -extent, 0)
            points.InsertNextPoint(x, extent, 0)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, points.GetNumberOfPoints() - 2)
            line.GetPointIds().SetId(1, points.GetNumberOfPoints() - 1)
            lines.InsertNextCell(line)
            
            # Horizontal lines (along X direction)
            y = -extent + i * grid_size
            
            points.InsertNextPoint(-extent, y, 0)
            points.InsertNextPoint(extent, y, 0)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, points.GetNumberOfPoints() - 2)
            line.GetPointIds().SetId(1, points.GetNumberOfPoints() - 1)
            lines.InsertNextCell(line)
        
        # Create polydata
        grid_polydata = vtk.vtkPolyData()
        grid_polydata.SetPoints(points)
        grid_polydata.SetLines(lines)
        
        # Create mapper
        grid_mapper = vtk.vtkPolyDataMapper()
        grid_mapper.SetInputData(grid_polydata)
        
        # Create grid actor
        self.grid_actor = vtk.vtkActor()
        self.grid_actor.SetMapper(grid_mapper)
        self.grid_actor.GetProperty().SetColor(grid_color)
        self.grid_actor.GetProperty().SetRepresentationToWireframe()
        self.grid_actor.GetProperty().SetLineWidth(1)
        
        self.renderer.AddActor(self.grid_actor)
        
        # Add axis lines
        axes_lines = vtk.vtkAxes()
        axes_lines.SetOrigin(0, 0, 0)
        axes_lines.SetScaleFactor(extent * 0.75)
        
        axes_mapper = vtk.vtkPolyDataMapper()
        axes_mapper.SetInputConnection(axes_lines.GetOutputPort())
        
        self.grid_lines_actor = vtk.vtkActor()
        self.grid_lines_actor.SetMapper(axes_mapper)
        self.grid_lines_actor.GetProperty().SetColor(0.5, 0.5, 0.5)
        self.grid_lines_actor.GetProperty().SetLineWidth(1)
        
        self.renderer.AddActor(self.grid_lines_actor)

    def set_grid_parameters(self, size_meters: float, color_rgb: Tuple[float, float, float]):
        """Set the grid size and color dynamically."""
        if not self.grid_actor:
            logger.info("Warning: No grid actor found")
            return
        
        try:
            # Update config
            self.config.grid_size = size_meters
            self.config.grid_color = color_rgb
            
            # Recreate grid
            self.renderer.RemoveActor(self.grid_actor)
            self._setup_grid()
            
            # Force render
            self.render()
            logger.info(f"VisualizerEngine: Grid updated - size: {size_meters}m, color: {color_rgb}")
            
        except Exception as e:
            logger.info(f"Error updating grid: {e}")
            import traceback
            traceback.print_exc()

    def set_grid_size(self, size_meters: float):
        """Set only grid size"""
        self.set_grid_parameters(size_meters, self.config.grid_color)

    def set_grid_color(self, color_rgb: Tuple[float, float, float]):
        """Set only grid color"""
        self.set_grid_parameters(self.config.grid_size, color_rgb)

    def set_grid_extent(self, extent: float):
        """Change how far the grid extends in each direction"""
        self.config.grid_extent = extent
        self.renderer.RemoveActor(self.grid_actor)
        self._setup_grid()
        self.render()
        logger.info(f"VisualizerEngine: Grid extent set to ±{extent}m")

    def set_grid_preset(self, preset_name: str):
        """
        Set grid to predefined size.
        
        Presets:
            'tiny': ±0.5m, 10mm grid
            'small': ±1.0m, 50mm grid
            'medium': ±2.0m, 100mm grid (default)
            'large': ±5.0m, 500mm grid
            'xlarge': ±10.0m, 1.0m grid
        """
        presets = {
            'tiny': {'extent': 0.5, 'size': 0.01},
            'small': {'extent': 1.0, 'size': 0.05},
            'medium': {'extent': 2.0, 'size': 0.1},
            'large': {'extent': 5.0, 'size': 0.5},
            'xlarge': {'extent': 10.0, 'size': 1.0}
        }
        
        if preset_name in presets:
            preset = presets[preset_name]
            self.config.grid_extent = preset['extent']
            self.config.grid_size = preset['size']
            self.renderer.RemoveActor(self.grid_actor)
            self._setup_grid()
            self.render()
            logger.info(f"VisualizerEngine: Grid set to {preset_name} (±{preset['extent']}m, {preset['size']*1000:.0f}mm grid)")
        else:
            logger.info(f"Unknown preset: {preset_name}")

    def _setup_axes(self):
        """Add coordinate axes widget."""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.5, 0.5, 0.5)
        axes.SetShaftType(0)
        axes.SetCylinderRadius(0.02)
        axes.SetConeRadius(0.05)
        
        widget = vtk.vtkOrientationMarkerWidget()
        widget.SetOrientationMarker(axes)
        widget.SetInteractor(self.interactor)
        widget.SetViewport(0, 0, 0.2, 0.2)
        widget.EnabledOn()
        widget.InteractiveOff()
        
        self.axes_widget = widget

    def _setup_render_timer(self):
        from PyQt5.QtCore import QTimer
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self._on_render_timer)
        self.render_timer.start(16)

    def _on_render_timer(self):
        """Called 60 times per second - renders ONLY if needed."""
        logger.debug(f"[VIZ] Timer tick, checking {len(self.displays)} displays")
        needs_render = False
        for display in self.displays:
            if display._needs_render:
                logger.debug(f"[VIZ] Display needs render")
                needs_render = True
                break
        
        if needs_render:
            self.vtk_widget.GetRenderWindow().Render()
            for display in self.displays:
                display._needs_render = False
    
    def register_display(self, display):
        """Register a display that can request renders."""
        self.displays.append(display)
        display._needs_render = False

    def get_render_widget(self) -> QWidget:
        """Get the Qt widget containing the 3D render view."""
        return self.container

    def get_renderer(self) -> vtk.vtkRenderer:
        """Get the VTK renderer for adding actors."""
        return self.renderer

    def get_interactor(self) -> vtk.vtkRenderWindowInteractor:
        """Get the VTK interactor."""
        return self.interactor

    def render(self):
        """Render a single frame."""
        self.render_window.Render()

    def save_screenshot(self, filename: str):
        """Save current view to PNG."""
        window_to_image = vtk.vtkWindowToImageFilter()
        window_to_image.SetInput(self.render_window)
        window_to_image.SetScale(2)
        window_to_image.Update()
        
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(filename)
        writer.SetInputConnection(window_to_image.GetOutputPort())
        writer.Write()

