"""
Mesh Loader - Pure mesh loading service.

Loads 3D mesh files (STL, OBJ, PLY, DAE) and returns vtkPolyData.
Manages caching and retrieval. No UI, no transforms, no events.
Returns raw vtkPolyData only — no VTK actors.

Principle: Visualizer as Mind-Prying Tool. Loading is a service.
Principle: UI Separate from Services. Pure data loading, no presentation.
"""

import hashlib
import logging
from pathlib import Path
from typing import List, Dict, Optional, Union
import numpy as np

import vtk
from vtk.util import numpy_support

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False

logger = logging.getLogger(__name__)


class MeshHandle:
    """
    Opaque handle for loaded meshes.

    Users should treat this as an opaque identifier.
    Do not access internal attributes directly.
    """

    def __init__(self, identifier: str):
        self._id = identifier

    def __str__(self) -> str:
        return self._id

    def __repr__(self) -> str:
        return f"MeshHandle('{self._id}')"

    def __eq__(self, other) -> bool:
        if isinstance(other, MeshHandle):
            return self._id == other._id
        return False

    def __hash__(self) -> int:
        return hash(self._id)

    @property
    def id(self) -> str:
        """Get the internal identifier string (for debugging only)."""
        return self._id


class MeshLoader:
    """
    Pure mesh loading service.

    Manages loading, caching, and retrieval of 3D mesh data.
    Returns raw VTK PolyData for visualization pipelines.
    Supports STL, OBJ, PLY, and DAE (via trimesh conversion).

    Usage:
        loader = MeshLoader()
        handle = loader.load_mesh("robot_arm.stl")
        polydata = loader.get_mesh_data(handle)
        # Use polydata in VTK pipeline
        loader.unload(handle)
    """

    _SUPPORTED_EXTENSIONS = {'.stl', '.obj', '.ply', '.dae'}

    def __init__(self, enable_color_extraction: bool = True):
        """
        Initialize MeshLoader with empty cache.

        Args:
            enable_color_extraction: If True, extract vertex colors from DAE files.
        """
        self._cache: Dict[str, vtk.vtkPolyData] = {}
        self._handle_to_path: Dict[MeshHandle, str] = {}
        self._path_to_handle: Dict[str, MeshHandle] = {}
        self._enable_color_extraction = enable_color_extraction

        if not TRIMESH_AVAILABLE:
            logger.info("trimesh not installed — DAE files not supported")

    # =================================================================
    # Public API
    # =================================================================

    @classmethod
    def get_supported_formats(cls) -> List[str]:
        """Get list of supported file extensions."""
        return list(cls._SUPPORTED_EXTENSIONS)

    def load_mesh(self, filepath: Union[str, Path]) -> MeshHandle:
        """
        Load a mesh from file and return a handle.

        Args:
            filepath: Path to mesh file (STL, OBJ, PLY, DAE).

        Returns:
            MeshHandle for accessing the loaded mesh data.

        Raises:
            FileNotFoundError: If file doesn't exist.
            ValueError: If file extension is not supported.
            RuntimeError: If loading fails.
        """
        filepath = Path(filepath).resolve()

        if not filepath.exists():
            raise FileNotFoundError(f"Mesh file not found: {filepath}")

        ext = filepath.suffix.lower()
        if ext not in self._SUPPORTED_EXTENSIONS:
            raise ValueError(
                f"Unsupported format: {ext}. "
                f"Supported: {', '.join(self._SUPPORTED_EXTENSIONS)}"
            )

        # Return cached if already loaded
        cache_key = str(filepath)
        if cache_key in self._cache:
            return self._path_to_handle[cache_key]

        # Load the mesh
        try:
            polydata = self._load_mesh_file(filepath, ext)

            if polydata is None or polydata.GetNumberOfPoints() == 0:
                raise RuntimeError(f"Loaded mesh has no points: {filepath}")

            self._cache[cache_key] = polydata

            handle = MeshHandle(cache_key)
            self._handle_to_path[handle] = cache_key
            self._path_to_handle[cache_key] = handle

            logger.debug(f"Loaded: {filepath.name} "
                        f"({polydata.GetNumberOfPoints()} points)")
            return handle

        except Exception as e:
            raise RuntimeError(f"Failed to load mesh {filepath}: {e}") from e

    def get_mesh_data(self, handle: MeshHandle) -> vtk.vtkPolyData:
        """
        Get VTK PolyData for a loaded mesh (shallow copy, read-only).

        Args:
            handle: MeshHandle from load_mesh().

        Returns:
            vtkPolyData — do not modify.

        Raises:
            KeyError: If handle is invalid or mesh was unloaded.
        """
        if handle not in self._handle_to_path:
            raise KeyError(f"Invalid or unloaded mesh handle: {handle}")

        filepath = self._handle_to_path[handle]
        polydata = self._cache.get(filepath)

        if polydata is None:
            raise KeyError(f"Mesh data missing from cache: {handle}")

        return polydata

    def get_mesh_data_copy(self, handle: MeshHandle) -> vtk.vtkPolyData:
        """
        Get a deep copy of VTK PolyData (safe to modify).

        Use this if you need to modify the mesh data.
        For read-only visualization, use get_mesh_data().

        Args:
            handle: MeshHandle from load_mesh().

        Returns:
            Deep copy of vtkPolyData.

        Raises:
            KeyError: If handle is invalid or mesh was unloaded.
        """
        original = self.get_mesh_data(handle)
        copy = vtk.vtkPolyData()
        copy.DeepCopy(original)
        return copy

    def unload(self, handle: MeshHandle) -> None:
        """
        Unload a mesh and free resources.

        Args:
            handle: MeshHandle to unload.

        Raises:
            KeyError: If handle is invalid.
        """
        if handle not in self._handle_to_path:
            raise KeyError(f"Cannot unload invalid handle: {handle}")

        filepath = self._handle_to_path[handle]

        if filepath in self._cache:
            del self._cache[filepath]

        del self._handle_to_path[handle]
        del self._path_to_handle[filepath]

    def list_loaded(self) -> List[MeshHandle]:
        """List all currently loaded mesh handles."""
        return list(self._handle_to_path.keys())

    def is_loaded(self, filepath: Union[str, Path]) -> bool:
        """Check if a mesh file is currently loaded."""
        filepath = str(Path(filepath).resolve())
        return filepath in self._cache

    def get_handle_for_file(self, filepath: Union[str, Path]) -> Optional[MeshHandle]:
        """Get the handle for a loaded mesh file, or None."""
        filepath = str(Path(filepath).resolve())
        return self._path_to_handle.get(filepath)

    def clear_all(self) -> None:
        """Clear all loaded meshes from cache."""
        self._cache.clear()
        self._handle_to_path.clear()
        self._path_to_handle.clear()

    def get_cache_info(self) -> dict:
        """Get cache statistics."""
        return {
            'num_loaded': len(self._cache),
            'handles': [str(h) for h in self._handle_to_path.keys()],
            'filepaths': list(self._cache.keys())
        }

    # =================================================================
    # Internal Loading Methods
    # =================================================================

    def _load_mesh_file(self, filepath: Path, ext: str) -> vtk.vtkPolyData:
        """Route to the appropriate loader based on extension."""
        if ext == '.stl':
            return self._load_with_reader(filepath, vtk.vtkSTLReader())
        elif ext == '.obj':
            return self._load_with_reader(filepath, vtk.vtkOBJReader())
        elif ext == '.ply':
            return self._load_with_reader(filepath, vtk.vtkPLYReader())
        elif ext == '.dae':
            return self._load_dae(filepath)
        else:
            raise ValueError(f"Unsupported extension: {ext}")

    @staticmethod
    def _load_with_reader(filepath: Path, reader) -> vtk.vtkPolyData:
        """Load a mesh using a VTK reader."""
        reader.SetFileName(str(filepath))
        reader.Update()
        polydata = reader.GetOutput()
        if polydata is None:
            raise RuntimeError(f"VTK reader returned None for {filepath}")
        return polydata

    def _load_dae(self, filepath: Path) -> vtk.vtkPolyData:
        if not TRIMESH_AVAILABLE:
            raise ImportError(
                "DAE files require trimesh. Install with: pip install trimesh"
            )

        scene = trimesh.load(str(filepath))

        meshes = []
        if isinstance(scene, trimesh.Scene):
            meshes = scene.dump()  # ← Changed: dump() applies scene transforms
        elif isinstance(scene, trimesh.Trimesh):
            meshes = [scene]
        else:
            raise RuntimeError(f"Unsupported DAE content: {type(scene)}")

        if not meshes:
            raise RuntimeError("No mesh geometry found in DAE file")

        combined = meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)
        return self._trimesh_to_vtk(combined)

    def _trimesh_to_vtk(self, mesh) -> vtk.vtkPolyData:
        """Convert trimesh object to vtkPolyData."""
        if len(mesh.vertices) == 0:
            raise RuntimeError("Trimesh has no vertices")

        # Vertices
        points = vtk.vtkPoints()
        vtk_vertices = numpy_support.numpy_to_vtk(
            mesh.vertices.astype(np.float32),
            deep=True,
            array_type=vtk.VTK_FLOAT
        )
        points.SetData(vtk_vertices)

        # Faces
        cell_data = np.column_stack([
            np.full(len(mesh.faces), 3, dtype=np.int64),
            mesh.faces
        ]).flatten()

        cells = vtk.vtkCellArray()
        vtk_cells = numpy_support.numpy_to_vtk(
            cell_data,
            deep=True,
            array_type=vtk.VTK_ID_TYPE
        )
        cells.SetCells(len(mesh.faces), vtk_cells)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(cells)

        # Vertex colors
        if self._enable_color_extraction and hasattr(mesh, 'visual'):
            colors = None

            if hasattr(mesh.visual, 'vertex_colors') and mesh.visual.vertex_colors is not None:
                colors = mesh.visual.vertex_colors
                if colors.shape[1] >= 3:
                    colors = colors[:, :3] if colors.shape[1] == 4 else colors[:, :3]
                    colors = colors.astype(np.uint8)

            if colors is not None:
                vtk_colors = numpy_support.numpy_to_vtk(
                    colors,
                    deep=True,
                    array_type=vtk.VTK_UNSIGNED_CHAR
                )
                vtk_colors.SetNumberOfComponents(3)
                vtk_colors.SetName("Colors")
                polydata.GetPointData().SetScalars(vtk_colors)

        # Normals (improves lighting)
        if hasattr(mesh, 'vertex_normals') and mesh.vertex_normals is not None:
            normals = mesh.vertex_normals.astype(np.float32)
            vtk_normals = numpy_support.numpy_to_vtk(
                normals,
                deep=True,
                array_type=vtk.VTK_FLOAT
            )
            vtk_normals.SetNumberOfComponents(3)
            vtk_normals.SetName("Normals")
            polydata.GetPointData().SetNormals(vtk_normals)

        return polydata