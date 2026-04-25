"""
Transform Registry - Lazy evaluation with proper frame transforms.
"""

import numpy as np
from typing import Dict, Callable, List, Optional, Tuple, Set
from dataclasses import dataclass, field
from enum import Enum
import logging

# Optional: Set up logging
logger = logging.getLogger(__name__)


class FrameStatus(Enum):
    """Status of a frame in the registry."""
    STATIC = "static"
    DYNAMIC = "dynamic"
    UNKNOWN = "unknown"


@dataclass
class FrameInfo:
    """Metadata about a frame."""
    name: str
    status: FrameStatus
    parent: Optional[str] = None
    description: str = ""
    transform_parent: Optional[np.ndarray] = None  # Transform relative to parent


class TransformRegistry:
    """
    Lazy-evaluation transform registry for all frames.
    
    Frames are registered with transform relative to parent.
    Transforms are computed only when requested via get_transform().
    
    Features:
    - Lazy evaluation with caching
    - Automatic cache invalidation for descendants
    - Callback system for transform updates
    - Transform validation
    - Optimized child lookups
    - Frame chain resolution
    """
    
    def __init__(self, debug: bool = False):
        """
        Initialize the transform registry.
        
        Args:
            debug: Enable debug logging and verbose output
        """
        self._frames: Dict[str, FrameInfo] = {}
        self._world_cache: Dict[str, np.ndarray] = {}
        self._callbacks: List[Callable[[str, np.ndarray], None]] = []
        self._children_cache: Dict[str, List[str]] = {}
        self._debug = debug
        
        # Add world frame
        world_transform = np.eye(4)
        self._frames["world"] = FrameInfo(
            name="world",
            status=FrameStatus.STATIC,
            parent=None,
            description="World origin",
            transform_parent=world_transform
        )
        self._world_cache["world"] = world_transform.copy()
        self._update_children_cache("world", add=True)

        self._asset_bases: Dict[str, str] = {}  # asset_id -> base_frame_name
    
    def _validate_transform(self, transform: np.ndarray, name: str = "") -> None:
        """
        Validate that a transform is a proper 4x4 homogeneous transformation matrix.
        
        Args:
            transform: 4x4 numpy array to validate
            name: Optional frame name for error messages
        
        Raises:
            ValueError: If transform is invalid
        """
        if transform.shape != (4, 4):
            raise ValueError(
                f"Transform for '{name}' must be 4x4, got {transform.shape}"
            )
        
        # Check last row is [0, 0, 0, 1]
        if not np.allclose(transform[3, :], [0, 0, 0, 1]):
            raise ValueError(
                f"Transform for '{name}' must have [0, 0, 0, 1] as last row, "
                f"got {transform[3, :]}"
            )
        
        # Check rotation part is orthonormal (optional, but good practice)
        rotation = transform[:3, :3]
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
            raise ValueError(
                f"Transform for '{name}' has invalid rotation matrix "
                f"(not orthonormal)"
            )
    
    def _update_children_cache(self, parent: str, add: bool = True) -> None:
        """
        Update the children cache for a parent frame.
        
        Args:
            parent: Parent frame name
            add: True if adding a child, False if removing
        """
        if add:
            # Rebuild children cache for this parent on addition
            # (simpler than tracking which child was added)
            self._children_cache.pop(parent, None)
        else:
            # Invalidate cache for removal
            self._children_cache.pop(parent, None)
    
    def _invalidate_cache(self, frame_name: str) -> None:
        """
        Invalidate cache for this frame and all descendants.
        
        Args:
            frame_name: Frame to invalidate
        """
        self._world_cache.pop(frame_name, None)
        for child in self._get_children(frame_name):
            self._invalidate_cache(child)
    
    def _get_children(self, parent: str) -> List[str]:
        """
        Get all frames that have this frame as parent.
        Uses caching for performance.
        
        Args:
            parent: Parent frame name
        
        Returns:
            List of child frame names
        """
        if parent not in self._children_cache:
            self._children_cache[parent] = [
                name for name, info in self._frames.items() 
                if info.parent == parent
            ]
        return self._children_cache[parent]
    
    def set(self, frame_name: str, transform: np.ndarray,
            status: FrameStatus = FrameStatus.DYNAMIC,
            parent: str = "world",
            description: str = "") -> None:
        """
        Register a new frame or update an existing one.
        
        Args:
            frame_name: Name of the frame
            transform: 4x4 transform from parent to this frame
            status: STATIC or DYNAMIC
            parent: Parent frame name
            description: Optional description
        """
        # Validate transform
        self._validate_transform(transform, frame_name)
        
        # Check if parent exists
        if parent not in self._frames:
            raise ValueError(f"Parent frame '{parent}' not found")
        
        # Check for circular dependencies
        if self._would_create_cycle(frame_name, parent):
            raise ValueError(
                f"Setting parent of '{frame_name}' to '{parent}' would create "
                f"a circular dependency"
            )
        
        # Update or add frame
        old_parent = None
        if frame_name in self._frames:
            old_parent = self._frames[frame_name].parent
        
        self._frames[frame_name] = FrameInfo(
            name=frame_name,
            status=status,
            parent=parent,
            description=description,
            transform_parent=transform.copy()
        )
        
        # Update children cache if parent changed
        if old_parent != parent:
            if old_parent:
                self._update_children_cache(old_parent, add=False)
            self._update_children_cache(parent, add=True)
        
        # Invalidate cache for this frame and its descendants
        self._invalidate_cache(frame_name)
        
        # Notify callbacks
        for cb in self._callbacks:
            try:
                cb(frame_name, transform)
            except Exception as e:
                logger.error(f"Callback error for {frame_name}: {e}")
        
        if self._debug:
            logger.debug(f"Set frame '{frame_name}' with parent '{parent}'")
    
    def _would_create_cycle(self, frame_name: str, new_parent: str) -> bool:
        """
        Check if setting parent would create a cycle in the frame tree.
        
        Args:
            frame_name: Frame to check
            new_parent: Proposed parent
        
        Returns:
            True if cycle would be created
        """
        # Check if new_parent is a descendant of frame_name
        current = new_parent
        while current and current != frame_name:
            info = self._frames.get(current)
            if info is None:
                break
            current = info.parent
        
        return current == frame_name
    
    def update(self, frame_name: str, transform: np.ndarray) -> None:
        """
        Update an existing frame's transform.
        
        Args:
            frame_name: Name of the frame
            transform: New 4x4 transform from parent to this frame
        """
        if frame_name not in self._frames:
            raise ValueError(f"Frame '{frame_name}' not registered")
        
        if frame_name == "world":
            raise ValueError("Cannot update world frame")
        
        # Validate transform
        self._validate_transform(transform, frame_name)
        
        self._frames[frame_name].transform_parent = transform.copy()
        self._invalidate_cache(frame_name)
        
        for cb in self._callbacks:
            try:
                cb(frame_name, transform)
            except Exception as e:
                logger.error(f"Callback error for {frame_name}: {e}")
        
        if self._debug:
            logger.debug(f"Updated frame '{frame_name}'")
    
    def remove(self, frame_name: str, force: bool = False) -> None:
        """
        Remove a frame from the registry.
        
        Args:
            frame_name: Name of the frame to remove
            force: If True, also remove all descendants
        """
        if frame_name == "world":
            raise ValueError("Cannot remove world frame")
        
        if frame_name not in self._frames:
            raise ValueError(f"Frame '{frame_name}' not found")
        
        # Check for children
        children = self._get_children(frame_name)
        if children and not force:
            raise ValueError(
                f"Cannot remove '{frame_name}' because it has children: {children}. "
                f"Use force=True to remove all descendants."
            )
        
        # Remove all descendants if force
        if force:
            for child in children:
                self.remove(child, force=True)
        
        # Get parent for cache update
        parent = self._frames[frame_name].parent
        
        # Remove frame
        del self._frames[frame_name]
        self._world_cache.pop(frame_name, None)
        
        # Update children cache
        self._update_children_cache(parent, add=False)
        
        if self._debug:
            logger.debug(f"Removed frame '{frame_name}'")
    
    def get_world_transform(self, frame_name: str) -> np.ndarray:
        """
        Get transform from frame to world.
        
        Args:
            frame_name: Name of the frame
        
        Returns:
            4x4 homogeneous transform matrix
        """
        if frame_name == "world":
            return np.eye(4)
        
        if frame_name in self._world_cache:
            return self._world_cache[frame_name].copy()
        
        info = self._frames.get(frame_name)
        if not info:
            raise ValueError(f"Frame '{frame_name}' not found")
        
        if info.parent is None:
            raise ValueError(f"Frame '{frame_name}' has no parent")
        
        T_parent_world = self.get_world_transform(info.parent)
        T = T_parent_world @ info.transform_parent
        
        # Debug output if enabled
        if self._debug:
            logger.debug(f"get_world_transform({frame_name}):")
            logger.debug(f"  parent={info.parent}")
            logger.debug(f"  Result position: ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})")
        
        self._world_cache[frame_name] = T.copy()
        return T
    
    def get_transform(self, target_frame: str, source_frame: str = "world") -> np.ndarray:
        """
        Get transform from source to target.
        
        Transform that converts points in source frame to points in target frame.
        
        Args:
            target_frame: Target frame name
            source_frame: Source frame name (default: "world")
        
        Returns:
            4x4 homogeneous transform matrix: points_in_target = T @ points_in_source
        """
        if target_frame == source_frame:
            return np.eye(4)
        
        T_target_world = self.get_world_transform(target_frame)
        T_source_world = self.get_world_transform(source_frame)
        
        # We want: points_in_target = T @ points_in_source
        # We know: points_in_world = T_target_world @ points_in_target
        # And: points_in_world = T_source_world @ points_in_source
        # So: T_target_world @ points_in_target = T_source_world @ points_in_source
        # Thus: points_in_target = inv(T_target_world) @ T_source_world @ points_in_source
        return np.linalg.inv(T_target_world) @ T_source_world
    
    def transform_pose(self, pose: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Transform a pose from one frame to another.
        
        Args:
            pose: 4x1 homogeneous point or 4x4 homogeneous transform
            from_frame: Source frame
            to_frame: Target frame
        
        Returns:
            Transformed pose (same shape as input)
        """
        T = self.get_transform(to_frame, from_frame)
        
        if pose.shape == (4, 1) or (len(pose.shape) == 1 and pose.shape[0] == 4):
            # Point transformation
            if len(pose.shape) == 1:
                pose = pose.reshape(4, 1)
            return T @ pose
        elif pose.shape == (4, 4):
            # Transform transformation
            return T @ pose
        else:
            raise ValueError(f"Invalid pose shape {pose.shape}. Expected (4,), (4,1), or (4,4)")
    
    def get(self, frame_name: str, default: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Get frame in world coordinates (backward compatibility).
        
        Args:
            frame_name: Name of the frame
            default: Default transform if frame not found
        
        Returns:
            4x4 homogeneous transform from frame to world
        """
        try:
            return self.get_world_transform(frame_name)
        except ValueError:
            return default if default is not None else np.eye(4)
    
    def lookup(self, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Get transform from_frame → to_frame.
        
        Args:
            from_frame: Source frame
            to_frame: Target frame
        
        Returns:
            4x4 homogeneous transform
        """
        return self.get_transform(to_frame, from_frame)
    
    def get_info(self, frame_name: str) -> Optional[FrameInfo]:
        """Get metadata about a frame."""
        return self._frames.get(frame_name)
    
    def list_frames(self) -> List[str]:
        """List all registered frame names."""
        return list(self._frames.keys())
    
    def register_callback(self, callback: Callable[[str, np.ndarray], None]) -> None:
        """
        Register to be notified when any transform changes.
        
        Args:
            callback: Function called with (frame_name, transform)
        """
        self._callbacks.append(callback)
    
    def remove_callback(self, callback: Callable) -> None:
        """Remove a previously registered callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)
    
    def get_chain(self, from_frame: str, to_frame: str) -> List[str]:
        """
        Get the chain of frames from from_frame to to_frame.
        
        Args:
            from_frame: Starting frame
            to_frame: Ending frame
        
        Returns:
            List of frame names from from_frame to to_frame
        """
        # Check if frames exist
        if from_frame not in self._frames:
            raise ValueError(f"Frame '{from_frame}' not found")
        if to_frame not in self._frames:
            raise ValueError(f"Frame '{to_frame}' not found")
        
        # Get path from from_frame to root
        path_to_root = []
        current = from_frame
        while current is not None:
            path_to_root.append(current)
            info = self._frames.get(current)
            current = info.parent if info else None
        
        # Get path from to_frame to root
        target_path = []
        current = to_frame
        while current is not None:
            target_path.append(current)
            info = self._frames.get(current)
            current = info.parent if info else None
        
        # Find common ancestor
        common_ancestor = None
        for frame in path_to_root:
            if frame in target_path:
                common_ancestor = frame
                break
        
        if common_ancestor is None:
            return []  # No common ancestor (shouldn't happen with world root)
        
        # Build chain: from_frame up to common_ancestor, then down to to_frame
        chain = []
        
        # Add frames from from_frame to common_ancestor (excluding common_ancestor)
        for frame in path_to_root:
            if frame == common_ancestor:
                break
            chain.append(frame)
        
        # Add common ancestor
        chain.append(common_ancestor)
        
        # Add frames from common_ancestor down to to_frame (excluding common_ancestor)
        for frame in reversed(target_path):
            if frame == common_ancestor:
                continue
            chain.append(frame)
        
        return chain
    
    def clear_cache(self) -> None:
        """Clear all cached world transforms."""
        self._world_cache.clear()
        # Re-cache world frame
        self._world_cache["world"] = np.eye(4)
        
        if self._debug:
            logger.debug("Cache cleared")
    
    def get_statistics(self) -> Dict[str, int]:
        """
        Get statistics about the registry.
        
        Returns:
            Dictionary with statistics
        """
        static_frames = sum(1 for info in self._frames.values() 
                           if info.status == FrameStatus.STATIC)
        dynamic_frames = sum(1 for info in self._frames.values() 
                            if info.status == FrameStatus.DYNAMIC)
        
        return {
            "total_frames": len(self._frames),
            "static_frames": static_frames,
            "dynamic_frames": dynamic_frames,
            "cached_transforms": len(self._world_cache),
            "registered_callbacks": len(self._callbacks)
        }
    
    def verify_tree_integrity(self) -> Tuple[bool, List[str]]:
        """
        Verify the integrity of the frame tree.
        
        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []
        
        # Check all frames exist
        for name, info in self._frames.items():
            if info.parent is not None and info.parent not in self._frames:
                errors.append(f"Frame '{name}' has missing parent '{info.parent}'")
        
        # Check for cycles
        visited: Set[str] = set()
        rec_stack: Set[str] = set()
        
        def has_cycle(frame: str) -> bool:
            visited.add(frame)
            rec_stack.add(frame)
            
            info = self._frames.get(frame)
            if info and info.parent:
                if info.parent not in visited:
                    if has_cycle(info.parent):
                        return True
                elif info.parent in rec_stack:
                    return True
            
            rec_stack.remove(frame)
            return False
        
        for frame in self._frames:
            if frame not in visited:
                if has_cycle(frame):
                    errors.append(f"Cycle detected involving frame '{frame}'")
        
        # Check transform validity
        for name, info in self._frames.items():
            if info.transform_parent is not None:
                try:
                    self._validate_transform(info.transform_parent, name)
                except ValueError as e:
                    errors.append(f"Invalid transform for '{name}': {e}")
        
        return len(errors) == 0, errors

    def set_asset_base(self, asset_id: str, base_frame: str) -> None:
        """
        Register which frame should be used as the base for Cartesian control
        for a specific asset.
        
        Args:
            asset_id: Unique identifier for the asset
            base_frame: Full frame name (including asset prefix) to use as base
        """
        self._asset_bases[asset_id] = base_frame
        if self._debug:
            logger.debug(f"Set asset base: {asset_id} -> {base_frame}")
    
    def get_asset_base_frame(self, asset_id: str) -> str:
        """
        Get the base frame for Cartesian control for an asset.
        
        Args:
            asset_id: Unique identifier for the asset
        
        Returns:
            Frame name to use as Cartesian base
        """
        return self._asset_bases.get(asset_id, "world")
    
    def get_asset_base_transform(self, asset_id: str) -> np.ndarray:
        """
        Get the transform from world to the asset's Cartesian base.
        
        Args:
            asset_id: Unique identifier for the asset
        
        Returns:
            4x4 homogeneous transform matrix
        """
        base_frame = self.get_asset_base_frame(asset_id)
        return self.get_world_transform(base_frame)
    
    def transform_to_asset_base(self, asset_id: str, pose_world: np.ndarray) -> np.ndarray:
        """
        Transform a pose from world coordinates to asset's base coordinates.
        
        Args:
            asset_id: Asset identifier
            pose_world: Pose in world coordinates (4x4)
        
        Returns:
            Pose in asset base coordinates
        """
        T_base_to_world = self.get_asset_base_transform(asset_id)
        T_world_to_base = np.linalg.inv(T_base_to_world)
        return T_world_to_base @ pose_world
    
    def transform_from_asset_base(self, asset_id: str, pose_base: np.ndarray) -> np.ndarray:
        """
        Transform a pose from asset base coordinates to world coordinates.
        
        Args:
            asset_id: Asset identifier
            pose_base: Pose in asset base coordinates (4x4)
        
        Returns:
            Pose in world coordinates
        """
        T_base_to_world = self.get_asset_base_transform(asset_id)
        return T_base_to_world @ pose_base
