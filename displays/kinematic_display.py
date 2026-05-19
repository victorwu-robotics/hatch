"""
Kinematic Display - VTK-based visualization of a KinematicModel.

Renders robot links as VTK actors, updated by transform changes.
Uses MeshLoader for mesh file loading (does not load files directly).
Subscribes to TransformRegistry callbacks for efficient updates.

Principle #3: Visualizer as Mind-Prying Tool. Reads state, doesn't control.
Principle #9: UI Separate from Services. Pure presentation.
"""

import vtk
import numpy as np
import logging
from pathlib import Path
from typing import Dict, List, Optional

from core.world_state.transform_registry import TransformRegistry

logger = logging.getLogger(__name__)


class KinematicDisplay:
    """
    Visual display that renders a KinematicModel in a 3D VTK scene.

    Creates VTK actors for each robot link and updates their transforms
    when the TransformRegistry notifies of changes. Uses MeshLoader for
    mesh file loading — no direct file I/O.

    Performance: transforms are created once and modified in place.
    Rendering is triggered via the _needs_render flag, polled by
    VisualizerEngine's 60Hz timer.
    """

    def __init__(self,
                 kinematic_model,
                 registry: TransformRegistry,
                 mesh_loader=None,
                 asset_id: str = None):
        """
        Initialize the kinematic display.

        Args:
            kinematic_model: KinematicModel providing link transforms and mesh info.
            registry: TransformRegistry for transform queries and change callbacks.
            mesh_loader: MeshLoader service for loading mesh files.
            asset_id: Asset identifier for frame name matching.
        """
        self.kinematic_model = kinematic_model
        self.registry = registry
        self.mesh_loader = mesh_loader
        self.asset_id = asset_id
        self.renderer = None

        # VTK actors and transforms
        self.link_actors: Dict[str, dict] = {}
        self.transform_filters: Dict[str, vtk.vtkTransformPolyDataFilter] = {}
        self.link_transforms: Dict[str, vtk.vtkTransform] = {}
        self.base_transforms: Dict[str, vtk.vtkTransform] = {}

        # State
        self.is_attached = False
        self._is_visible = True
        self._needs_render = False

        # Get visual geometries from model
        self.visual_geometries = self.kinematic_model.get_visual_geometries()

        # Subscribe to transform registry updates
        self.registry.register_callback(self._on_transform_updated)

        logger.info(f"KinematicDisplay created for asset: {asset_id}")

    # =================================================================
    # Attach / Detach
    # =================================================================

    def attach(self, renderer):
        """
        Attach display to VTK renderer and create all actors.

        Args:
            renderer: vtkRenderer to add actors to.
        """
        if self.is_attached:
            return

        self.renderer = renderer
        logger.info(f"Attaching to VTK renderer...")

        # Load all visual geometries
        for link_name, geometries in self.visual_geometries.items():
            if not geometries:
                # Virtual link — no visual representation
                continue

            for i, geom in enumerate(geometries):
                self._load_geometry(link_name, geom, i)

        self.is_attached = True
        loaded_count = len(self.link_actors)
        logger.info(f"Loaded {loaded_count} visual geometries")

        # Force initial transform update

        '''
        if self.asset_id:
            print(f"\n=== Transform Debug for {self.asset_id} ===")
            for link_name in self.kinematic_model.link_transforms.keys():
                frame_name = f"{self.asset_id}_{link_name}"
                try:
                    T_world = self.registry.get_transform(frame_name, "world")
                    pos = T_world[:3, 3]
                    print(f"  {link_name}: world pos = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                except ValueError:
                    print(f"  {link_name}: NOT IN REGISTRY")
            print("=" * 40)
        '''

        if self.asset_id:
            registered_frames = set(self.registry.list_frames())
            for link_name in self.kinematic_model.link_transforms.keys():
                frame_name = f"{self.asset_id}_{link_name}"
                if frame_name not in registered_frames:
                    continue
                try:
                    T_world = self.registry.get_transform(frame_name, "world")
                    self._update_link_transforms(link_name, T_world)
                except ValueError:
                    pass


    def detach(self):
        """Clean up resources when display is removed from renderer."""
        if not self.is_attached:
            return

        self.registry.remove_callback(self._on_transform_updated)

        for actor_info in self.link_actors.values():
            self.renderer.RemoveActor(actor_info['actor'])

        self.link_actors.clear()
        self.transform_filters.clear()
        self.link_transforms.clear()
        self.base_transforms.clear()
        self.renderer = None
        self.is_attached = False

        logger.info("KinematicDisplay detached from renderer")

    # =================================================================
    # Geometry Loading (delegates to MeshLoader)
    # =================================================================

    def _load_geometry(self, link_name: str, geom: Dict, index: int):
        """
        Load a single geometry and create its VTK actor.

        Args:
            link_name: Name of the link this geometry belongs to.
            geom: Geometry dictionary from KinematicModel.
            index: Index for multiple geometries on the same link.
        """
        geom_type = geom.get('type', 'mesh')

        if geom_type == 'mesh':
            self._load_mesh_geometry(link_name, geom, index)
        elif geom_type == 'box':
            self._load_box_geometry(link_name, geom, index)
        elif geom_type == 'cylinder':
            self._load_cylinder_geometry(link_name, geom, index)
        elif geom_type == 'sphere':
            self._load_sphere_geometry(link_name, geom, index)
        else:
            logger.warning(f"Unknown geometry type '{geom_type}' "
                          f"for {link_name}[{index}]")

    def _load_mesh_geometry(self, link_name: str, geom: Dict, index: int):
        """
        Load mesh geometry using MeshLoader service.

        Falls back to placeholder if mesh file is missing or loading fails.
        """
        mesh_path = geom.get('mesh_path')

        if mesh_path is None or not mesh_path.exists():
            logger.warning(f"Mesh not found for {link_name}: {mesh_path}")
            self._create_placeholder(link_name, geom, index,
                                     "missing" if mesh_path else "no_path")
            return

        polydata = None

        # Use MeshLoader if available
        if self.mesh_loader:
            try:
                handle = self.mesh_loader.load_mesh(mesh_path)
                polydata = self.mesh_loader.get_mesh_data(handle)
            except Exception as e:
                logger.warning(f"MeshLoader failed for {mesh_path}: {e}")

        # Fallback: load directly if MeshLoader unavailable or failed
        if polydata is None:
            try:
                polydata = self._load_mesh_direct(mesh_path)
            except Exception as e:
                logger.warning(f"Direct load failed for {mesh_path}: {e}")
                self._create_placeholder(link_name, geom, index, "error")
                return

        if polydata is None or polydata.GetNumberOfPoints() == 0:
            self._create_placeholder(link_name, geom, index, "empty")
            return

        self._create_actor_from_polydata(link_name, geom, polydata, index)
        logger.debug(f"Loaded {link_name}[{index}]: {mesh_path.name} "
                    f"({polydata.GetNumberOfPoints()} points)")

    def _load_mesh_direct(self, mesh_path: Path) -> Optional[vtk.vtkPolyData]:
        """Direct mesh loading fallback when MeshLoader unavailable."""
        ext = mesh_path.suffix.lower()

        if ext == '.stl':
            reader = vtk.vtkSTLReader()
        elif ext == '.obj':
            reader = vtk.vtkOBJReader()
        elif ext == '.ply':
            reader = vtk.vtkPLYReader()
        elif ext == '.vtp':
            reader = vtk.vtkXMLPolyDataReader()
        elif ext == '.dae':
            # return self._load_collada_direct(mesh_path)
            return self._load_collada_with_trimesh(mesh_path)
        else:
            logger.warning(f"Unsupported format: {ext}")
            return None

        reader.SetFileName(str(mesh_path))
        reader.Update()
        return reader.GetOutput()

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

    # =================================================================
    # Primitive Geometry
    # =================================================================

    def _load_box_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a box primitive actor."""
        size = geom.get('size', [1, 1, 1])
        box = vtk.vtkCubeSource()
        box.SetXLength(size[0])
        box.SetYLength(size[1])
        box.SetZLength(size[2])
        box.Update()
        self._create_actor_from_polydata(link_name, geom, box.GetOutput(), index)

    def _load_cylinder_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a cylinder primitive actor."""
        radius = geom.get('radius', 0.5)
        length = geom.get('length', 1.0)
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(length)
        cylinder.SetResolution(32)
        cylinder.Update()
        self._create_actor_from_polydata(link_name, geom, cylinder.GetOutput(), index)

    def _load_sphere_geometry(self, link_name: str, geom: Dict, index: int):
        """Create a sphere primitive actor."""
        radius = geom.get('radius', 0.5)
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetThetaResolution(32)
        sphere.SetPhiResolution(16)
        sphere.Update()
        self._create_actor_from_polydata(link_name, geom, sphere.GetOutput(), index)

    # =================================================================
    # Actor Creation
    # =================================================================

    def _create_actor_from_polydata(self,
                                    link_name: str,
                                    geom: Dict,
                                    polydata: vtk.vtkPolyData,
                                    index: int):
        """Create VTK actor from PolyData with transform pipeline."""
        
        # STEP 1: Apply scale to mesh vertices directly (in local frame)
        scale = geom.get('scale', [1, 1, 1])
        if scale != [1, 1, 1]:
            scale_transform = vtk.vtkTransform()
            scale_transform.Scale(scale)
            scale_filter = vtk.vtkTransformPolyDataFilter()
            scale_filter.SetInputData(polydata)
            scale_filter.SetTransform(scale_transform)
            scale_filter.Update()
            polydata = scale_filter.GetOutput()
        
        # STEP 2: Create transform filter for kinematic updates
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputData(polydata)

        # STEP 3: Base transform — visual origin only (no scale, already applied)
        base_transform = vtk.vtkTransform()
        origin_matrix = geom.get('origin_transform', np.eye(4))
        origin_transform = self._numpy_to_vtk_transform(origin_matrix)
        if origin_transform:
            base_transform.Concatenate(origin_transform)

        # STEP 4: Chained transform — world kinematic + base origin
        transform_key = f"{link_name}_{index}"
        chained_transform = vtk.vtkTransform()
        
        # Use model FK for initial position (authoritative) - same as old working code
        kinematic_transform = self.kinematic_model.get_vtk_transform(link_name)
        if kinematic_transform:
            chained_transform.DeepCopy(kinematic_transform)
            chained_transform.Concatenate(base_transform)

        transform_filter.SetTransform(chained_transform)
        transform_filter.Update()

        # Store references
        filter_key = f"{link_name}_{index}"
        self.transform_filters[filter_key] = transform_filter
        self.link_transforms[transform_key] = chained_transform
        self.base_transforms[transform_key] = base_transform

        # Mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())

        # Actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Color: use embedded vertex colors if available, otherwise URDF color
        has_vertex_colors = polydata.GetPointData().GetScalars() is not None

        if has_vertex_colors:
            mapper.ScalarVisibilityOn()
            mapper.SetScalarModeToUsePointData()
            actor.GetProperty().SetOpacity(geom.get('opacity', 1.0))
            actor.GetProperty().LightingOn()
        else:
            color = geom.get('color', [0.7, 0.7, 0.7])
            actor.GetProperty().SetColor(color)
            actor.GetProperty().SetOpacity(geom.get('opacity', 1.0))
            actor.GetProperty().LightingOn()

        # Store actor
        actor_key = f"{link_name}_{index}"
        self.link_actors[actor_key] = {
            'actor': actor,
            'link_name': link_name,
            'filter_key': filter_key,
            'transform_key': transform_key
        }

        self.renderer.AddActor(actor)

    def _create_placeholder(self,
                            link_name: str,
                            geom: Dict,
                            index: int,
                            reason: str):
        """Create a placeholder box when mesh loading fails."""
        logger.debug(f"Creating placeholder for {link_name}[{index}] ({reason})")
        placeholder = {
            'type': 'box',
            'size': [0.1, 0.1, 0.1],
            'color': [0.8, 0.2, 0.2],
            'opacity': 0.5,
            'origin_transform': geom.get('origin_transform', np.eye(4))
        }
        self._load_box_geometry(link_name, placeholder, index)

    # =================================================================
    # Transform Updates
    # =================================================================

    def _on_transform_updated(self, frame_name: str, transform: np.ndarray):
        """
        Callback from TransformRegistry when a frame's transform changes.

        Updates all VTK actors for the affected link.
        Sets _needs_render flag — VisualizerEngine's timer polls this.
        """
        if not self.is_attached or not self._is_visible or not self.asset_id:
            return

        if not frame_name.startswith(f"{self.asset_id}_"):
            return

        link_name = frame_name[len(self.asset_id) + 1:]

        # Get world transform from registry
        try:
            # T_world = self.registry.get_transform(frame_name, "world")
            T_world = self.registry.get_transform("world", frame_name)
        except Exception as e:
            logger.warning(f"Could not get world transform for {frame_name}: {e}")
            return

        self._update_link_transforms(link_name, T_world)

        # Also update all descendant links
        for child_link in self.kinematic_model.link_children.get(link_name, []):
            child_frame = f"{self.asset_id}_{child_link}"
            try:
                T_world = self.registry.get_transform("world", child_frame)
                self._update_link_transforms(child_link, T_world)
            except ValueError:
                pass

        self._needs_render = True

    def _update_link_transforms(self, link_name: str, T_world: np.ndarray):
        """Update all VTK transforms for a given link."""
        kinematic_transform = self._numpy_to_vtk_transform(T_world)

        for actor_key, actor_info in self.link_actors.items():
            if actor_info['link_name'] != link_name:
                continue

            transform_key = actor_info['transform_key']
            if transform_key not in self.link_transforms:
                continue

            chained_transform = self.link_transforms[transform_key]
            base_transform = self.base_transforms[transform_key]

            # Modify in place (no allocation)
            chained_transform.Identity()
            chained_transform.Concatenate(kinematic_transform)
            chained_transform.Concatenate(base_transform)

    # =================================================================
    # Visibility
    # =================================================================

    def set_visible(self, visible: bool):
        """Show or hide all robot actors."""
        self._is_visible = visible
        for actor_info in self.link_actors.values():
            actor_info['actor'].SetVisibility(visible)

        if self.renderer:
            self._needs_render = True

    def get_actor(self, link_name: str) -> List[vtk.vtkActor]:
        """Get VTK actors for a specific link."""
        return [
            info['actor'] for info in self.link_actors.values()
            if info['link_name'] == link_name
        ]

    # =================================================================
    # Helpers
    # =================================================================

    @staticmethod
    def _numpy_to_vtk_transform(matrix: np.ndarray) -> vtk.vtkTransform:
        """Convert 4x4 numpy array to vtkTransform."""
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(matrix.flatten(order='C'))
        return vtk_transform