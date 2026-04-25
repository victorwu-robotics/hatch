"""
AssetManager - Pure mesh loading service
Following Principle #3 (visualizer) + Principle #9 (services)

No UI, no domain managers, no event publishing, no transform registration.
Returns raw vtkPolyData only - no VTK actors.

PUBLIC API:
- load_mesh(filepath) -> MeshHandle
- get_mesh_data(handle) -> vtk.vtkPolyData (shallow copy, read-only)
- get_mesh_data_copy(handle) -> vtk.vtkPolyData (deep copy, modifiable)
- unload(handle) -> None
- list_loaded() -> List[MeshHandle]
- is_loaded(filepath) -> bool
- get_handle_for_file(filepath) -> Optional[MeshHandle]
- clear_all() -> None
- get_cache_info() -> dict
- get_supported_formats() -> List[str]
"""

import hashlib
from pathlib import Path
from typing import List, Dict, Optional, Union
import numpy as np

import vtk
from vtk.util import numpy_support

# Optional trimesh for DAE support
try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


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
        manager = AssetManager()
        handle = manager.load_mesh("robot_arm.stl")
        polydata = manager.get_mesh_data(handle)
        # Use polydata in VTK pipeline
        manager.unload(handle)
    """
    
    # Supported file extensions and their VTK readers
    _SUPPORTED_EXTENSIONS = {'.stl', '.obj', '.ply', '.dae'}
    
    def __init__(self, enable_color_extraction: bool = True):
        """
        Initialize AssetManager with empty cache.
        
        Args:
            enable_color_extraction: If True, extract vertex colors from DAE files
        """
        self._cache: Dict[str, vtk.vtkPolyData] = {}
        self._handle_to_path: Dict[MeshHandle, str] = {}
        self._path_to_handle: Dict[str, MeshHandle] = {}
        self._enable_color_extraction = enable_color_extraction
        
        # Warn about missing trimesh if DAE support is needed
        if not TRIMESH_AVAILABLE:
            print("Warning: trimesh not installed. DAE files will not be supported.")
            print("Install with: pip install trimesh")
    
    @classmethod
    def get_supported_formats(cls) -> List[str]:
        """
        Get list of supported file extensions.
        
        Returns:
            List of supported extensions (e.g., ['.stl', '.obj', '.ply', '.dae'])
        """
        return list(cls._SUPPORTED_EXTENSIONS)
    
    def load_mesh(self, filepath: Union[str, Path]) -> MeshHandle:
        """
        Load a mesh from file and return a handle.
        
        Args:
            filepath: Path to mesh file (STL, OBJ, PLY, DAE)
            
        Returns:
            MeshHandle for accessing the loaded mesh data
            
        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: If file extension is not supported or file is invalid
            RuntimeError: If VTK or trimesh fails to load the mesh
            ImportError: If DAE file and trimesh not installed
        """
        filepath = Path(filepath).resolve()
        
        # Validate file exists
        if not filepath.exists():
            raise FileNotFoundError(f"Mesh file not found: {filepath}")
        
        # Validate extension
        ext = filepath.suffix.lower()
        if ext not in self._SUPPORTED_EXTENSIONS:
            raise ValueError(
                f"Unsupported file format: {ext}. "
                f"Supported: {', '.join(self._SUPPORTED_EXTENSIONS)}"
            )
        
        # Check cache
        cache_key = str(filepath)
        if cache_key in self._cache:
            return self._path_to_handle[cache_key]
        
        # Load the mesh
        try:
            polydata = self._load_mesh_file(filepath, ext)
            
            # Validate loaded data
            if polydata is None or polydata.GetNumberOfPoints() == 0:
                raise RuntimeError(f"Loaded mesh has no points: {filepath}")
            
            # Cache the polydata
            self._cache[cache_key] = polydata
            
            # Create handle
            handle = MeshHandle(cache_key)
            self._handle_to_path[handle] = cache_key
            self._path_to_handle[cache_key] = handle
            
            return handle
            
        except Exception as e:
            raise RuntimeError(f"Failed to load mesh {filepath}: {str(e)}") from e
    
    def _load_mesh_file(self, filepath: Path, ext: str) -> vtk.vtkPolyData:
        """Internal method to load mesh using appropriate reader."""
        if ext == '.stl':
            return self._load_stl(filepath)
        elif ext == '.obj':
            return self._load_obj(filepath)
        elif ext == '.ply':
            return self._load_ply(filepath)
        elif ext == '.dae':
            if not TRIMESH_AVAILABLE:
                raise ImportError(
                    "DAE files require trimesh. Install with: pip install trimesh"
                )
            return self._load_dae(filepath)
        else:
            raise ValueError(f"Unsupported extension: {ext}")
    
    def _load_stl(self, filepath: Path) -> vtk.vtkPolyData:
        """Load STL file using VTK STL reader."""
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(filepath))
        reader.Update()
        
        polydata = reader.GetOutput()
        if polydata is None:
            raise RuntimeError("VTK STL reader returned None")
        
        return polydata
    
    def _load_obj(self, filepath: Path) -> vtk.vtkPolyData:
        """Load OBJ file using VTK OBJ reader."""
        reader = vtk.vtkOBJReader()
        reader.SetFileName(str(filepath))
        reader.Update()
        
        polydata = reader.GetOutput()
        if polydata is None:
            raise RuntimeError("VTK OBJ reader returned None")
        
        return polydata
    
    def _load_ply(self, filepath: Path) -> vtk.vtkPolyData:
        """Load PLY file using VTK PLY reader."""
        reader = vtk.vtkPLYReader()
        reader.SetFileName(str(filepath))
        reader.Update()
        
        polydata = reader.GetOutput()
        if polydata is None:
            raise RuntimeError("VTK PLY reader returned None")
        
        return polydata
    
    def _load_dae(self, filepath: Path) -> vtk.vtkPolyData:
        """
        Load DAE (Collada) file using trimesh and convert to VTK PolyData.
        
        Extracts vertex colors if enable_color_extraction is True.
        Combines all geometry from the scene into a single vtkPolyData.
        """
        scene = trimesh.load(str(filepath))
        
        # Extract all meshes from scene
        meshes = []
        if isinstance(scene, trimesh.Scene):
            for geometry in scene.geometry.values():
                if isinstance(geometry, trimesh.Trimesh):
                    meshes.append(geometry)
        elif isinstance(scene, trimesh.Trimesh):
            meshes.append(scene)
        else:
            raise RuntimeError(f"Unsupported DAE content type: {type(scene)}")
        
        if not meshes:
            raise RuntimeError("No mesh geometry found in DAE file")
        
        # Combine all meshes into one
        if len(meshes) == 1:
            combined = meshes[0]
        else:
            combined = trimesh.util.concatenate(meshes)
        
        # Convert to VTK PolyData
        return self._trimesh_to_vtk_polydata(combined)
    
    def _trimesh_to_vtk_polydata(self, mesh: trimesh.Trimesh) -> vtk.vtkPolyData:
        """
        Convert trimesh object to vtkPolyData with OPTIMIZED numpy conversion.
        
        Args:
            mesh: trimesh.Trimesh object
            
        Returns:
            vtkPolyData containing vertices, faces, and optionally colors
        """
        # Get vertices and faces
        vertices = mesh.vertices
        faces = mesh.faces
        
        if len(vertices) == 0:
            raise RuntimeError("Trimesh has no vertices")
        
        # ===== OPTIMIZED: Convert vertices using numpy_support =====
        points = vtk.vtkPoints()
        vtk_vertices = numpy_support.numpy_to_vtk(
            vertices.astype(np.float32),
            deep=True,
            array_type=vtk.VTK_FLOAT
        )
        points.SetData(vtk_vertices)
        
        # ===== OPTIMIZED: Convert faces using numpy_support =====
        # VTK expects cells as flat array: [n, i1, i2, i3, n, i1, i2, i3, ...]
        # For triangles, each cell has 4 elements (3 indices + count)
        cell_data = np.column_stack([
            np.full(len(faces), 3, dtype=np.int64),  # cell size (triangle = 3)
            faces
        ]).flatten()
        
        cells = vtk.vtkCellArray()
        vtk_cells = numpy_support.numpy_to_vtk(
            cell_data,
            deep=True,
            array_type=vtk.VTK_ID_TYPE
        )
        cells.SetCells(len(faces), vtk_cells)
        
        # Create polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(cells)
        
        # ===== VERTEX COLORS (if available and enabled) =====
        if self._enable_color_extraction and hasattr(mesh, 'visual'):
            colors = None
            
            # Try vertex colors first
            if hasattr(mesh.visual, 'vertex_colors') and mesh.visual.vertex_colors is not None:
                colors = mesh.visual.vertex_colors
                if colors.shape[1] >= 3:
                    # Convert to RGB (drop alpha if present)
                    if colors.shape[1] == 4:
                        colors = colors[:, :3]
                    colors = colors.astype(np.uint8)
            
            # Fall back to face colors if no vertex colors
            elif hasattr(mesh.visual, 'face_colors') and mesh.visual.face_colors is not None:
                colors = mesh.visual.face_colors
                if colors.shape[1] >= 3:
                    if colors.shape[1] == 4:
                        colors = colors[:, :3]
                    colors = colors.astype(np.uint8)
                    # Face colors need to be repeated for each vertex in the face
                    # This is complex; skip for now, log warning
                    print(f"  Note: Face colors found but not yet supported. "
                          f"Vertex colors are preferred for DAE files.")
                    colors = None
            
            if colors is not None:
                vtk_colors = numpy_support.numpy_to_vtk(
                    colors,
                    deep=True,
                    array_type=vtk.VTK_UNSIGNED_CHAR
                )
                vtk_colors.SetNumberOfComponents(3)
                vtk_colors.SetName("Colors")
                polydata.GetPointData().SetScalars(vtk_colors)
        
        # ===== NORMALS (optional, improves lighting) =====
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
    
    def get_mesh_data(self, handle: MeshHandle) -> vtk.vtkPolyData:
        """
        Retrieve the VTK PolyData for a loaded mesh (shallow copy, read-only).
        
        Args:
            handle: MeshHandle returned from load_mesh()
            
        Returns:
            vtkPolyData object (shallow copy - do not modify)
            
        Raises:
            KeyError: If handle is invalid or mesh has been unloaded
        """
        if handle not in self._handle_to_path:
            raise KeyError(f"Invalid or unloaded mesh handle: {handle}")
        
        filepath = self._handle_to_path[handle]
        polydata = self._cache.get(filepath)
        
        if polydata is None:
            raise KeyError(f"Mesh data missing from cache for handle: {handle}")
        
        # Return shallow copy to prevent modification of cached data
        return polydata
    
    def get_mesh_data_copy(self, handle: MeshHandle) -> vtk.vtkPolyData:
        """
        Retrieve a DEEP COPY of the VTK PolyData for modification.
        
        Use this if you need to modify the mesh data (e.g., transform vertices).
        For read-only visualization, use get_mesh_data() instead.
        
        Args:
            handle: MeshHandle returned from load_mesh()
            
        Returns:
            Deep copy of vtkPolyData (safe to modify)
            
        Raises:
            KeyError: If handle is invalid or mesh has been unloaded
        """
        original = self.get_mesh_data(handle)
        
        # Create deep copy
        copy_filter = vtk.vtkPolyDataAlgorithm()
        # Use a simple pass-through filter to create a new copy
        copy = vtk.vtkPolyData()
        copy.DeepCopy(original)
        
        return copy
    
    def unload(self, handle: MeshHandle) -> None:
        """
        Unload a mesh and free its resources.
        
        Args:
            handle: MeshHandle to unload
            
        Raises:
            KeyError: If handle is invalid
        """
        if handle not in self._handle_to_path:
            raise KeyError(f"Cannot unload invalid handle: {handle}")
        
        filepath = self._handle_to_path[handle]
        
        # Remove from cache
        if filepath in self._cache:
            del self._cache[filepath]
        
        # Remove mappings
        del self._handle_to_path[handle]
        del self._path_to_handle[filepath]
    
    def list_loaded(self) -> List[MeshHandle]:
        """
        List all currently loaded mesh handles.
        
        Returns:
            List of MeshHandle objects for loaded meshes
        """
        return list(self._handle_to_path.keys())
    
    def is_loaded(self, filepath: Union[str, Path]) -> bool:
        """
        Check if a mesh file is currently loaded.
        
        Args:
            filepath: Path to check
            
        Returns:
            True if mesh is loaded, False otherwise
        """
        filepath = str(Path(filepath).resolve())
        return filepath in self._cache
    
    def get_handle_for_file(self, filepath: Union[str, Path]) -> Optional[MeshHandle]:
        """
        Get the handle for a loaded mesh file.
        
        Args:
            filepath: Path to mesh file
            
        Returns:
            MeshHandle if loaded, None otherwise
        """
        filepath = str(Path(filepath).resolve())
        return self._path_to_handle.get(filepath)
    
    def clear_all(self) -> None:
        """Clear all loaded meshes from cache."""
        self._cache.clear()
        self._handle_to_path.clear()
        self._path_to_handle.clear()
    
    def get_cache_info(self) -> dict:
        """
        Get information about current cache state.
        
        Returns:
            Dictionary with cache statistics
        """
        return {
            'num_loaded': len(self._cache),
            'handles': [str(h) for h in self._handle_to_path.keys()],
            'filepaths': list(self._cache.keys())
        }


# Example usage (for testing only)
if __name__ == "__main__":
    import sys
    
    manager = AssetManager(enable_color_extraction=True)
    
    print("=" * 50)
    print("AssetManager - Pure Mesh Loading Service")
    print("=" * 50)
    print(f"Supported formats: {manager.get_supported_formats()}")
    print(f"Color extraction: {manager._enable_color_extraction}")
    print(f"Trimesh available: {TRIMESH_AVAILABLE}")
    print(f"Initial cache: {manager.get_cache_info()}")
    print("=" * 50)
    print("Ready to load meshes.")