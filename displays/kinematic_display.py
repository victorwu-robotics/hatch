"""
PHASE 4 - Kinematic Display (Adapter) - VTK Direct Version
Optimized for performance: transforms created once, modified in place.
"""

import vtk
import numpy as np
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import pyassimp
from vtk.util import numpy_support
from collections import deque
from statistics import mean

from core.world_state.transform_registry import TransformRegistry


class KinematicDisplay:
    """
    A visual display that renders a KinematicModel in a 3D scene using direct VTK.
    Optimized for performance: transforms created once, modified in place.
    """
    
    def __init__(self, kinematic_model, registry: TransformRegistry, asset_id: str = None):
        """
        Initialize the display with a data source and transform registry.
        
        Args:
            kinematic_model (KinematicModel): The data source providing mesh paths and kinematics.
            registry (TransformRegistry): Registry to listen for transform updates.
            asset_id (str): The asset ID for this robot (for frame name matching)
        """
        self.kinematic_model = kinematic_model
        self.registry = registry
        self.asset_id = asset_id
        self.renderer = None
        
        # Visual state
        self.link_actors: Dict[str, dict] = {}  # actor_key -> {actor, link_name, filter, transform_key}
        self.transform_filters: Dict[str, vtk.vtkTransformPolyDataFilter] = {}  # filter_key -> filter
        self.link_transforms: Dict[str, vtk.vtkTransform] = {}  # transform_key -> transform
        self.base_transforms: Dict[str, vtk.vtkTransform] = {}  # transform_key -> base_transform
        self.visual_geometries: Dict[str, List[Dict]] = {}
        
        self.is_attached = False
        self._is_visible = True
        
        # Performance optimization: cache VTK readers
        self.mesh_readers: Dict[str, vtk.vtkAlgorithm] = {}
        
        # Get visual geometries from model
        self.visual_geometries = self.kinematic_model.get_visual_geometries()
        
        # Subscribe to transform registry updates
        self.registry.register_callback(self._on_transform_updated)
        print(f"KinematicDisplay: Subscribed to transform registry with asset_id: {asset_id}")
    
        # Add profiling attributes
        self.profile_data = {
            'total': deque(maxlen=30),
            'transform_get': deque(maxlen=30),
            'geometry_updates': deque(maxlen=30),
            'render': deque(maxlen=30),
            'link_times': {}  # Per-link timing
        }
        self.profile_counter = 0
        self.last_profile_print = time.time()

        self._needs_render = False      # Flag managed by engine

    def attach(self, renderer):
        """Attach display to VTK renderer."""
        if self.is_attached:
            return

        self.renderer = renderer
        print(f"KinematicDisplay: Attaching to VTK renderer...")
        
        # Load all visual geometries
        for link_name, geometries in self.visual_geometries.items():
            # Case 1: Virtual link - no visuals at all
            if not geometries:
                print(f"    Skipping virutal link: {link_name}")
                continue

            # Case 2: Link has visuals - try to load them
            for i, geom in enumerate(geometries):
                self._load_geometry(link_name, geom, i)
        
        self.is_attached = True
        loaded_count = len(self.link_actors)
        print(f"KinematicDisplay: Loaded {loaded_count} visual geometries")
        
        # Force initial update
        if self.asset_id:
            for link_name in self.kinematic_model.link_transforms.keys():
                frame_name = f"{self.asset_id}_{link_name}"
                T = self.kinematic_model.link_transforms[link_name]
                self._on_transform_updated(frame_name, T)
    
    def _load_geometry(self, link_name: str, geom: Dict, index: int):
        """Load a geometry and create VTK actor for a link."""
        geom_type = geom['type']
        
        if geom_type == 'mesh':
            self._load_mesh_geometry(link_name, geom, index)
        elif geom_type == 'box':
            self._load_box_geometry(link_name, geom, index)
        elif geom_type == 'cylinder':
            self._load_cylinder_geometry(link_name, geom, index)
        elif geom_type == 'sphere':
            self._load_sphere_geometry(link_name, geom, index)

    def _create_actor_from_polydata(self, link_name: str, geom: Dict, polydata: vtk.vtkPolyData, index: int):
        """Create VTK actor from PolyData with optimized transform handling."""
        
        # 1. Create transform filter (once)
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputData(polydata)
        
        # 2. Create base transform (scale + visual origin) - this is STATIC, never changes
        base_transform = vtk.vtkTransform()
        
        # Apply scale
        scale = geom.get('scale', [1, 1, 1])
        if scale != [1, 1, 1]:
            base_transform.Scale(scale)
        
        # Apply visual origin transform
        origin_transform_matrix = geom.get('origin_transform', np.eye(4))
        origin_transform = self._matrix_to_vtk_transform(origin_transform_matrix)
        if origin_transform:
            base_transform.Concatenate(origin_transform)
        
        # 3. Create chained transform (kinematic + base) - this will be UPDATED each frame
        transform_key = f"{link_name}_{index}"
        chained_transform = vtk.vtkTransform()
        
        # Initialize with current kinematic transform
        kinematic_transform = self.kinematic_model.get_vtk_transform(link_name)
        if kinematic_transform:
            chained_transform.DeepCopy(kinematic_transform)
            chained_transform.Concatenate(base_transform)
        
        # 4. Set the transform on the filter
        transform_filter.SetTransform(chained_transform)
        transform_filter.Update()  # One-time update to initialize
        
        # 5. Store everything
        filter_key = f"{link_name}_{index}"
        self.transform_filters[filter_key] = transform_filter
        self.link_transforms[transform_key] = chained_transform
        self.base_transforms[transform_key] = base_transform
        
        # 6. Create mapper (no need to update this ever)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        
        # 7. Create actor (static)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Check if the polydata already has vertex colors (from embedded materials)
        has_vertex_colors = polydata.GetPointData().GetScalars() is not None

        if has_vertex_colors:

            # print(f"    🎨 Found VTK scalars: {scalars.GetNumberOfTuples()} values, {scalars.GetNumberOfComponents()} components")
            # print(f"    First few: {[scalars.GetTuple(i) for i in range(min(3, scalars.GetNumberOfTuples()))]}")

            # Polydata has embedded colors - use them
            mapper.ScalarVisibilityOn()
            mapper.SetScalarModeToUsePointData()
            # Don't set actor color - let the scalar colors show through
            # But still apply opacity if specified
            actor.GetProperty().SetOpacity(geom.get('opacity', 1.0))
            actor.GetProperty().LightingOn()
            # print(f"    🎨 Using embedded colors for {link_name}[{index}]")
        else:
            print(f"    ⚠ No VTK scalars found in polydata")
            # No embedded colors - use URDF color
            color = geom.get('color', [0.7, 0.7, 0.7])
            actor.GetProperty().SetColor(color)
            actor.GetProperty().SetOpacity(geom.get('opacity', 1.0))
            actor.GetProperty().LightingOn()
            # print(f"    🎨 Using URDF color {color} for {link_name}[{index}]")
        
        # 8. Store actor info
        actor_key = f"{link_name}_{index}"
        self.link_actors[actor_key] = {
            'actor': actor,
            'link_name': link_name,
            'filter_key': filter_key,
            'transform_key': transform_key
        }
        
        # 9. Add to renderer
        self.renderer.AddActor(actor)

    def _numpy_to_vtk_transform(self, matrix: np.ndarray) -> vtk.vtkTransform:
        """Convert 4x4 numpy array to vtkTransform."""
        vtk_transform = vtk.vtkTransform()
        flat_matrix = matrix.flatten(order='C')
        vtk_transform.SetMatrix(flat_matrix)
        return vtk_transform

    def _on_transform_updated(self, frame_name: str, transform: np.ndarray):
        """Callback for transform registry updates."""
        print(f"[KD] {frame_name} transform position: ({transform[0,3]:.3f}, {transform[1,3]:.3f}, {transform[2,3]:.3f})")
        if not self.is_attached or not self._is_visible or not self.asset_id:
            return
        
        if not frame_name.startswith(f"{self.asset_id}_"):
            return
        
        link_name = frame_name[len(self.asset_id)+1:]
        
        # ===== FIX: Get WORLD transform from registry =====
        try:
            T_world = self.registry.get_world_transform(frame_name)
        except Exception as e:
            print(f"[KD] Could not get world transform for {frame_name}: {e}")
            return
        
        kinematic_transform = self._numpy_to_vtk_transform(T_world)

        
        # Update ALL geometries for this link
        for actor_key, actor_info in self.link_actors.items():
            if actor_info['link_name'] != link_name:
                continue
            
            transform_key = actor_info['transform_key']
            if transform_key not in self.link_transforms:
                continue
            
            chained_transform = self.link_transforms[transform_key]
            base_transform = self.base_transforms[transform_key]
            
            # Modify transform in place
            chained_transform.Identity()
            chained_transform.Concatenate(kinematic_transform)
            chained_transform.Concatenate(base_transform)

        # SINGLE RENDER CALL - Time it properly
        if self.renderer:
            # render_start = time.time()
            # self.renderer.GetRenderWindow().Render()
            self._needs_render = True
            # self.profile_data['render'].append(time.time() - render_start)
        
        # Record total time
        # self.profile_data['total'].append(time.time() - total_start)
        # self.profile_counter += 1
        
        # Print profile every 30 frames or every 2 seconds
        # now = time.time()
        # if self.profile_counter >= 30 or (now - self.last_profile_print) > 2.0:
            # self._print_profile()
            # self.profile_counter = 0
            # self.last_profile_print = now

    '''
    def render_if_needed(self):
        if self._needs_render and self.renderer:
            self.renderer.GetRenderWindow().Render()
            self._needs_render = False
    '''
            
    def _print_profile(self):
        """Print timing profile statistics."""
        print("\n" + "="*60)
        print("🔍 PERFORMANCE PROFILE")
        print("="*60)
        
        if not self.profile_data['total']:
            print("No profile data yet")
            return
        
        # Calculate averages
        avg_total = mean(self.profile_data['total']) * 1000  # Convert to ms
        avg_transform = mean(self.profile_data['transform_get']) * 1000
        avg_geometry = mean(self.profile_data['geometry_updates']) * 1000
        avg_render = mean(self.profile_data['render']) * 1000
        
        print(f"\n📊 Averages per frame:")
        print(f"  Total time:          {avg_total:.2f} ms ({1000/avg_total:.1f} FPS)")
        print(f"  ├─ Get transform:    {avg_transform:.2f} ms")
        print(f"  ├─ Update geometries: {avg_geometry:.2f} ms")
        print(f"  └─ Render:           {avg_render:.2f} ms")
        
        # Check where time is spent
        if avg_render > 30:
            print(f"\n⚠️  Render is slow ({avg_render:.1f} ms) - VTK rendering is the bottleneck")
        if avg_geometry > 10:
            print(f"⚠️  Geometry updates are slow ({avg_geometry:.1f} ms) - Too many transforms?")
        if avg_transform > 5:
            print(f"⚠️  Getting transforms is slow ({avg_transform:.1f} ms) - Check kinematic_model")
        
        # Print per-link timing
        if self.profile_data['link_times']:
            print("\n📈 Per-link geometry update times:")
            for link_name, times in self.profile_data['link_times'].items():
                if times:
                    avg_link = mean(times) * 1000
                    print(f"  {link_name}: {avg_link:.2f} ms")
        
        # Check for outliers
        if self.profile_data['total']:
            max_total = max(self.profile_data['total']) * 1000
            min_total = min(self.profile_data['total']) * 1000
            if max_total > avg_total * 2:
                print(f"\n⚠️  High variability: {min_total:.1f} - {max_total:.1f} ms")
        
        print("="*60 + "\n")

    def _load_mesh_geometry(self, link_name: str, geom: Dict, index: int):
        """
        Load a mesh geometry and create VTK actor for a link.
        Supports STL, OBJ, DAE (via trimesh), PLY, VTP formats.
        
        Args:
            link_name: Name of the link.
            geom: Mesh geometry dictionary.
            index: Index for multiple geometries on same link.
        """
        mesh_path = geom['mesh_path']
        
        if mesh_path is None:
            # Case 2a: Mesh path missing - create place holder
            print(f"  WARNING: Mesh path is None for {link_name}")
            self._create_placeholder_geometry(link_name, geom, index, "no_mesh")
            return
        
        if not mesh_path.exists():
            # Case 2b: Mesh file not found - create placeholder
            print(f"  WARNING: Mesh not found: {mesh_path}")
            self._create_placeholder_geometry(link_name, geom, index, "missing")
            return
        
        try:
            # Check file extension
            ext = mesh_path.suffix.lower()
            reader_key = f"{link_name}_{index}"
            polydata = None

            print(f"** LOADING MESH: {str(mesh_path)}")   #...

            # VTK native formats
            if ext in ['.stl', '.stla', '.stlb']:
                reader = vtk.vtkSTLReader()
                reader.SetFileName(str(mesh_path))
                reader.Update()
                polydata = reader.GetOutput()
                self.mesh_readers[reader_key] = reader
                print(f"    Loaded STL: {mesh_path.name}")
                
            elif ext in ['.obj']:
                reader = vtk.vtkOBJReader()
                reader.SetFileName(str(mesh_path))
                reader.Update()
                polydata = reader.GetOutput()
                self.mesh_readers[reader_key] = reader
                print(f"    Loaded OBJ: {mesh_path.name}")
                
            elif ext in ['.ply']:
                reader = vtk.vtkPLYReader()
                reader.SetFileName(str(mesh_path))
                reader.Update()
                polydata = reader.GetOutput()
                self.mesh_readers[reader_key] = reader
                print(f"    Loaded PLY: {mesh_path.name}")
                
            elif ext in ['.vtp']:
                reader = vtk.vtkXMLPolyDataReader()
                reader.SetFileName(str(mesh_path))
                reader.Update()
                polydata = reader.GetOutput()
                self.mesh_readers[reader_key] = reader
                print(f"    Loaded VTP: {mesh_path.name}")
                
            elif ext in ['.dae']:
                # Use trimesh for COLLADA files
                polydata = self._load_collada_with_trimesh(mesh_path)
                if polydata is None:
                    print(f"  ⚠ {link_name}[{index}]: Failed to load COLLADA, creating placeholder")
                    self._create_placeholder_geometry(link_name, geom, index, "dae_load_failed")
                    return
                    
            else:
                print(f"  ⚠ {link_name}[{index}]: Unsupported format {ext}, using placeholder")
                self._create_placeholder_geometry(link_name, geom, index, f"unsupported_{ext}")
                return
            
            if polydata is None or polydata.GetNumberOfPoints() == 0:
                print(f"  WARNING: No points in mesh for {link_name}")
                self._create_placeholder_geometry(link_name, geom, index, "empty")
                return
            
            self._create_actor_from_polydata(link_name, geom, polydata, index)
            print(f"  ✓ {link_name}[{index}]: {mesh_path.name} ({polydata.GetNumberOfPoints()} points)")
            
        except Exception as e:
            print(f"  ✗ {link_name}[{index}]: {e}")
            import traceback
            traceback.print_exc()
            self._create_placeholder_geometry(link_name, geom, index, "error")
    
    def _load_collada_with_trimesh(self, mesh_path: Path) -> Optional[vtk.vtkPolyData]:
        """
        Load COLLADA using trimesh.
        
        Args:
            mesh_path: Path to the COLLADA file.
            
        Returns:
            vtk.vtkPolyData: Combined mesh data, or None if loading fails.
        """
        try:
            import trimesh
        except ImportError:
            print(f"    ERROR: trimesh not installed. Install with: pip install trimesh")
            return None
        
        try:
            # Load mesh with trimesh
            scene = trimesh.load(str(mesh_path))
            
            append_filter = vtk.vtkAppendPolyData()
            mesh_count = 0
            
            # Handle both single mesh and scene
            if isinstance(scene, trimesh.Trimesh):
                meshes = [scene]
            elif isinstance(scene, trimesh.Scene):
                meshes = scene.dump()
            else:
                meshes = []
            
            for mesh in meshes:
                if not isinstance(mesh, trimesh.Trimesh):
                    continue
                    
                # Get vertices and faces
                vertices_np = mesh.vertices.astype(np.float32)
                faces_np = mesh.faces.astype(np.int32)
                
                # Create VTK points
                points = vtk.vtkPoints()
                vtk_points = numpy_support.numpy_to_vtk(
                    vertices_np, 
                    deep=True, 
                    array_type=vtk.VTK_FLOAT
                )
                points.SetData(vtk_points)
                
                # Create VTK cells
                cells = vtk.vtkCellArray()
                for face in faces_np:
                    cells.InsertNextCell(3, face)
                
                # Create PolyData
                polydata = vtk.vtkPolyData()
                polydata.SetPoints(points)
                polydata.SetPolys(cells)
                
                # Add vertex colors if available
                if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'vertex_colors'):
                    colors_np = mesh.visual.vertex_colors[:, :3].astype(np.uint8)
                    # print(f"    🎨 Found vertex colors! Shape: {colors_np.shape}")
                    # print(f"    🎨 First few colors: {colors_np[:5]}")

                    vtk_colors = numpy_support.numpy_to_vtk(
                        colors_np, 
                        deep=True, 
                        array_type=vtk.VTK_UNSIGNED_CHAR
                    )
                    vtk_colors.SetNumberOfComponents(3)
                    vtk_colors.SetName("Colors")
                    polydata.GetPointData().SetScalars(vtk_colors)

                    # Verify they were set
                    scalars = polydata.GetPointData().GetScalars()
                    if scalars:
                        print(f"    ✅ VTK scalars set: {scalars.GetNumberOfTuples()} values, {scalars.GetNumberOfComponents()} components")
                        print(f"    First few values: {[scalars.GetTuple(i) for i in range(min(5, scalars.GetNumberOfTuples()))]}")
                    else:
                        print(f"    ❌ VTK scalars NOT set!")

                else:
                    print(f"    ⚠ No vertex colors found in mesh")

                append_filter.AddInputData(polydata)
                mesh_count += 1
            
            if mesh_count == 0:
                print(f"    No valid meshes found in {mesh_path.name}")
                return None
            
            append_filter.Update()
            result = append_filter.GetOutput()
            
            # Generate normals for lighting
            normals_filter = vtk.vtkPolyDataNormals()
            normals_filter.SetInputData(result)
            normals_filter.ComputePointNormalsOn()
            normals_filter.ComputeCellNormalsOff()
            normals_filter.SplittingOff()
            normals_filter.Update()
            result = normals_filter.GetOutput()
            
            print(f"    ✓ Loaded {mesh_count} meshes from {mesh_path.name} "
                f"({result.GetNumberOfPoints()} points, {result.GetNumberOfCells()} cells)")
            return result
            
        except Exception as e:
            print(f"    ERROR loading {mesh_path.name} with trimesh: {e}")
            return None
    
    def _load_box_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a box primitive."""
        size = geom.get('size', [1, 1, 1])
        
        # Create box source
        box = vtk.vtkCubeSource()
        box.SetXLength(size[0])
        box.SetYLength(size[1])
        box.SetZLength(size[2])
        box.Update()
        
        self._create_primitive_actor(link_name, geom, box.GetOutput(), index)
    
    def _load_cylinder_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a cylinder primitive."""
        radius = geom.get('radius', 0.5)
        length = geom.get('length', 1.0)
        
        # Create cylinder source
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(length)
        cylinder.SetResolution(32)
        cylinder.Update()
        
        self._create_primitive_actor(link_name, geom, cylinder.GetOutput(), index)
    
    def _load_sphere_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a sphere primitive."""
        radius = geom.get('radius', 0.5)
        
        # Create sphere source
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetThetaResolution(32)
        sphere.SetPhiResolution(16)
        sphere.Update()
        
        self._create_primitive_actor(link_name, geom, sphere.GetOutput(), index)
    
    def _create_primitive_actor(self, link_name: str, geom: Dict, polydata: vtk.vtkPolyData, index: int):
        """Create actor for primitive geometry."""
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputData(polydata)
        
        # Create initial transform
        initial_transform = vtk.vtkTransform()
        
        # Apply visual origin transform
        origin_transform_matrix = geom.get('origin_transform', np.eye(4))
        origin_transform = self._matrix_to_vtk_transform(origin_transform_matrix)
        if origin_transform:
            initial_transform.Concatenate(origin_transform)
        
        transform_filter.SetTransform(initial_transform)
        transform_filter.Update()
        
        # Store base transform
        base_transform = vtk.vtkTransform()
        base_transform.DeepCopy(initial_transform)
        
        # Store transform filter
        filter_key = f"{link_name}_{index}"
        self.transform_filters[filter_key] = {
            'filter': transform_filter,
            'base_transform': base_transform,
            'link_name': link_name
        }
        
        # Create mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        
        # Create actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        
        # Apply color
        color = geom.get('color', [0.8, 0.2, 0.2])
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(geom.get('opacity', 1.0))
        
        # Store actor
        actor_key = f"{link_name}_{index}"
        self.link_actors[actor_key] = {
            'actor': actor,
            'link_name': link_name,
            'filter_key': filter_key
        }
        
        self.renderer.AddActor(actor)
        
        geom_type = geom.get('type', 'primitive')
        print(f"  ✓ {link_name}[{index}]: {geom_type}")
    
    def _matrix_to_vtk_transform(self, matrix: np.ndarray) -> Optional[vtk.vtkTransform]:
        """
        Convert a 4x4 numpy matrix to vtkTransform.
        
        Args:
            matrix: 4x4 homogeneous transform matrix.
            
        Returns:
            vtk.vtkTransform or None if conversion fails.
        """
        if matrix is None or matrix.shape != (4, 4):
            return None
        
        transform = vtk.vtkTransform()
        
        # VTK expects 16-element flat array in row-major order
        flat_matrix = matrix.flatten(order='C')
        transform.SetMatrix(flat_matrix)
        
        return transform

    def count_actors(self):
        """Count all actors in the renderer and in our internal storage."""
        if not self.renderer:
            print("No renderer attached")
            return
        
        # Count actors in VTK renderer
        actor_collection = self.renderer.GetActors()
        actor_collection.InitTraversal()
        vtk_actor_count = 0
        for i in range(actor_collection.GetNumberOfItems()):
            actor = actor_collection.GetNextActor()
            vtk_actor_count += 1
        
        # Count our stored actors
        stored_count = len(self.link_actors)
        
        # Count transforms
        transform_count = len(self.link_transforms)
        filter_count = len(self.transform_filters)
        
        print(f"\n=== Actor Count ===")
        print(f"VTK Renderer actors: {vtk_actor_count}")
        print(f"Stored in link_actors: {stored_count}")
        print(f"Stored transforms: {transform_count}")
        print(f"Stored filters: {filter_count}")
        print(f"==================\n")
        
        # Check for mismatch (potential leak)
        if vtk_actor_count > stored_count:
            print(f"⚠️ WARNING: Renderer has {vtk_actor_count - stored_count} more actors than stored!")
        elif vtk_actor_count < stored_count:
            print(f"⚠️ WARNING: Missing {stored_count - vtk_actor_count} actors in renderer!")

    def _create_placeholder_geometry(self, link_name: str, geom: Dict, index: int, reason: str):
        """
        Create a simple placeholder geometry when mesh loading fails.
        
        Args:
            link_name: Name of the link.
            geom: Geometry dictionary.
            index: Index for multiple geometries.
            reason: Reason for placeholder.
        """
        print(f"  ⚠ {link_name}[{index}]: Creating placeholder ({reason})")
        
        # Create a simple box as placeholder
        placeholder_geom = {
            'type': 'box',
            'size': [0.1, 0.1, 0.1],
            'color': [0.8, 0.2, 0.2],
            'opacity': 0.5,  # Semi-transparent to indicate placeholder
            'origin_transform': geom.get('origin_transform', np.eye(4))
        }
        self._load_box_geometry(link_name, placeholder_geom, index)

    def set_visible(self, visible: bool):
        """
        Toggles visibility of all robot mesh actors.
        
        Args:
            visible: If True, show the robot; if False, hide it.
        """
        self._is_visible = visible
        
        for actor_info in self.link_actors.values():
            actor_info['actor'].SetVisibility(visible)
        
        if self.renderer is not None:
            render_window = self.renderer.GetRenderWindow()
            if render_window:
                render_window.Render()
    
    def get_actor(self, link_name: str) -> List[vtk.vtkActor]:
        """
        Get VTK actors for a specific link.
        
        Args:
            link_name: Name of the link.
            
        Returns:
            List of vtk.vtkActor objects for the link.
        """
        actors = []
        for actor_info in self.link_actors.values():
            if actor_info['link_name'] == link_name:
                actors.append(actor_info['actor'])
        return actors
    
    def detach(self):
        """
        Clean up resources when display is removed from renderer.
        """
        if not self.is_attached:
            return
        
        # Remove callback from registry
        self.registry.remove_callback(self._on_transform_updated)
        
        # Remove actors from renderer
        for actor_info in self.link_actors.values():
            self.renderer.RemoveActor(actor_info['actor'])
        
        # Clear references
        self.link_actors.clear()
        self.transform_filters.clear()
        self.mesh_readers.clear()
        self.renderer = None
        self.is_attached = False
        
        print("KinematicDisplay: Detached from renderer")