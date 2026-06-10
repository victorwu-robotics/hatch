"""
Transform Registry - Lazy evaluation with cache invalidation.

Principle: Space = TransformRegistry.
All relative poses in one place. Lazy evaluation — transforms computed
only when requested. Cache invalidation on change. No polling.

Supports STATIC and DYNAMIC frames. The callback system notifies
subscribers when transforms change (used by KinematicDisplay for
efficient re-rendering and future collision monitors).
"""

import numpy as np
from typing import Dict, Callable, List, Optional, Tuple, Set
from dataclasses import dataclass
from enum import Enum
import logging

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

    Not thread-safe. Designed for single-threaded use per Hatch architecture.

    Frames are registered with a transform relative to their parent.
    World transforms are computed only when requested via get_transform().

    Features:
    - Lazy evaluation with automatic caching
    - Cache invalidation for frame and all descendants on update
    - Callback system for transform change notifications
    - Transform validation (4x4, orthonormal rotation, [0,0,0,1] bottom row)
    - Circular dependency detection
    - Frame chain resolution
    - Tree integrity verification

    Usage:
        registry = TransformRegistry()

        # Register a frame
        registry.register_frame(
            "robot_base",
            transform=np.eye(4),
            parent="world",
            status=FrameStatus.STATIC
        )

        # Update a dynamic frame
        registry.update_frame("robot_tcp", new_transform)

        # Query transforms
        T = registry.get_transform("robot_tcp", "world")
        point_in_tcp = registry.transform_point([1, 0, 0], "world", "robot_tcp")
    """

    def __init__(self, debug: bool = False):
        """
        Initialize the transform registry.

        Args:
            debug: Enable debug logging
        """
        self._frames: Dict[str, FrameInfo] = {}
        self._world_cache: Dict[str, np.ndarray] = {}
        self._callbacks: List[Callable[[str, np.ndarray], None]] = []
        self._children_cache: Dict[str, List[str]] = {}
        self._debug = debug

        # Add world frame (the root of all transform trees)
        world_transform = np.eye(4)
        self._frames["world"] = FrameInfo(
            name="world",
            status=FrameStatus.STATIC,
            parent=None,
            description="World origin",
            transform_parent=world_transform
        )
        self._world_cache["world"] = world_transform.copy()

    # =================================================================
    # Public API — Frame Registration
    # =================================================================

    def register_frame(self,
                       name: str,
                       transform: np.ndarray,
                       parent: str = "world",
                       status: FrameStatus = FrameStatus.DYNAMIC,
                       description: str = "") -> None:
        """
        Register a new frame or replace an existing one.

        Args:
            name: Unique frame name
            transform: 4x4 homogeneous transform from parent to this frame
            parent: Parent frame name (default: "world")
            status: STATIC or DYNAMIC
            description: Human-readable description

        Raises:
            ValueError: If transform is invalid, parent doesn't exist,
                        or registration would create a cycle
        """
        self._validate_transform(transform, name)

        if parent not in self._frames:
            raise ValueError(f"Parent frame '{parent}' not found")

        if self._would_create_cycle(name, parent):
            raise ValueError(
                f"Setting parent of '{name}' to '{parent}' "
                f"would create a circular dependency"
            )

        # Track parent change for cache updates
        old_parent = self._frames[name].parent if name in self._frames else None

        self._frames[name] = FrameInfo(
            name=name,
            status=status,
            parent=parent,
            description=description,
            transform_parent=transform.copy()
        )

        # Update children cache
        if old_parent is not None and old_parent != parent:
            self._children_cache.pop(old_parent, None)
        self._children_cache.pop(parent, None)

        # Invalidate cache for this frame and all descendants
        self._invalidate_cache(name)

        # Notify callbacks
        self._notify_callbacks(name, transform)

        if self._debug:
            logger.debug(f"Registered frame '{name}' (parent: '{parent}', "
                        f"status: {status.value})")

    def update_frame(self, name: str, transform: np.ndarray) -> None:
        """
        Update an existing frame's parent-relative transform.

        Args:
            name: Frame name (must already be registered)
            transform: New 4x4 transform from parent to this frame

        Raises:
            ValueError: If frame doesn't exist or is the world frame
        """
        if name not in self._frames:
            raise ValueError(f"Frame '{name}' not registered. "
                           f"Use register_frame() first.")

        if name == "world":
            raise ValueError("Cannot update world frame")

        self._validate_transform(transform, name)

        self._frames[name].transform_parent = transform.copy()
        self._invalidate_cache(name)
        self._notify_callbacks(name, transform)

        if self._debug:
            logger.debug(f"Updated frame '{name}'")

    def remove_frame(self, name: str, force: bool = False) -> None:
        """
        Remove a frame from the registry.

        Args:
            name: Frame to remove
            force: If True, also remove all descendants

        Raises:
            ValueError: If frame doesn't exist, is 'world', or has children
        """
        if name == "world":
            raise ValueError("Cannot remove world frame")

        if name not in self._frames:
            raise ValueError(f"Frame '{name}' not found")

        children = self._get_children(name)
        if children and not force:
            raise ValueError(
                f"Cannot remove '{name}': it has children {children}. "
                f"Use force=True to remove all descendants."
            )

        # Remove descendants first if forcing
        if force:
            for child in list(children):
                self.remove_frame(child, force=True)

        parent = self._frames[name].parent
        del self._frames[name]
        self._world_cache.pop(name, None)
        self._children_cache.pop(parent, None)

        if self._debug:
            logger.debug(f"Removed frame '{name}'")

    # =================================================================
    # Public API — Transform Queries
    # =================================================================

    def get_transform(self, target: str, source: str = "world") -> np.ndarray:
        """
        Get the transform from source frame to target frame.

        Returns the 4x4 matrix T such that: point_in_target = T @ point_in_source

        Args:
            target: Target frame name
            source: Source frame name (default: "world")

        Returns:
            4x4 homogeneous transform matrix
        """
        if target == source:
            return np.eye(4)

        T_target_world = self._get_world_transform(target)
        T_source_world = self._get_world_transform(source)

        return np.linalg.inv(T_target_world) @ T_source_world

    def transform_point(self,
                        point: np.ndarray,
                        from_frame: str,
                        to_frame: str) -> np.ndarray:
        """
        Transform a 3D point between frames.

        Args:
            point: 3-element array [x, y, z] or 4-element [x, y, z, 1]
            from_frame: Frame the point is currently in
            to_frame: Target frame

        Returns:
            Transformed point (same dimension as input)
        """
        T = self.get_transform(to_frame, from_frame)

        if len(point) == 3:
            point_h = np.append(point, 1.0)
            result = T @ point_h
            return result[:3]
        elif len(point) == 4:
            return T @ point
        else:
            raise ValueError(f"Point must have 3 or 4 elements, got {len(point)}")

    def transform_frame_pose(self,
                             pose: np.ndarray,
                             from_frame: str,
                             to_frame: str) -> np.ndarray:
        """
        Transform a 4x4 pose matrix between frames.

        Args:
            pose: 4x4 homogeneous transform in from_frame
            from_frame: Frame the pose is currently in
            to_frame: Target frame

        Returns:
            4x4 homogeneous transform in to_frame
        """
        if pose.shape != (4, 4):
            raise ValueError(f"Pose must be 4x4, got {pose.shape}")

        T = self.get_transform(to_frame, from_frame)
        return T @ pose

    # =================================================================
    # Public API — Frame Queries
    # =================================================================

    def get_frame_info(self, name: str) -> Optional[FrameInfo]:
        """Get metadata about a frame."""
        return self._frames.get(name)

    def has_frame(self, name: str) -> bool:
        """Check if a frame exists in the registry."""
        return name in self._frames

    def list_frames(self) -> List[str]:
        """List all registered frame names."""
        return list(self._frames.keys())

    def get_chain(self, from_frame: str, to_frame: str) -> List[str]:
        """
        Get the chain of frames from from_frame to to_frame.

        Returns:
            List of frame names forming the path between the two frames.
            Empty list if no path exists.
        """
        if from_frame not in self._frames:
            raise ValueError(f"Frame '{from_frame}' not found")
        if to_frame not in self._frames:
            raise ValueError(f"Frame '{to_frame}' not found")

        # Build path from each frame to root
        path_from = self._path_to_root(from_frame)
        path_to = self._path_to_root(to_frame)

        # Find common ancestor
        common = None
        for frame in path_from:
            if frame in path_to:
                common = frame
                break

        if common is None:
            return []

        # Build chain: from_frame → ... → common → ... → to_frame
        chain = []
        for frame in path_from:
            chain.append(frame)
            if frame == common:
                break

        # Reverse the to-path from common down to to_frame
        for frame in reversed(path_to):
            if frame == common:
                continue
            chain.append(frame)

        return chain

    def _path_to_root(self, frame: str) -> List[str]:
        """Get path from a frame to the world root."""
        path = []
        current = frame
        visited = set()
        while current is not None and current not in visited:
            path.append(current)
            visited.add(current)
            info = self._frames.get(current)
            current = info.parent if info else None
        return path

    # =================================================================
    # Public API — Callbacks
    # =================================================================

    def register_callback(self, callback: Callable[[str, np.ndarray], None]) -> None:
        """
        Register to be notified when any frame's transform changes.

        Callback signature: callback(frame_name: str, transform: np.ndarray) -> None

        Args:
            callback: Function called with (frame_name, parent_relative_transform)
        """
        self._callbacks.append(callback)

    def remove_callback(self, callback: Callable[[str, np.ndarray], None]) -> None:
        """Remove a previously registered callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    # =================================================================
    # Public API — Maintenance
    # =================================================================

    def clear_cache(self) -> None:
        """Clear all cached world transforms (force recomputation)."""
        self._world_cache.clear()
        self._world_cache["world"] = np.eye(4)

        if self._debug:
            logger.debug("Cache cleared")

    def get_statistics(self) -> Dict[str, int]:
        """Get statistics about the registry."""
        static_count = sum(
            1 for info in self._frames.values()
            if info.status == FrameStatus.STATIC
        )
        dynamic_count = sum(
            1 for info in self._frames.values()
            if info.status == FrameStatus.DYNAMIC
        )

        return {
            "total_frames": len(self._frames),
            "static_frames": static_count,
            "dynamic_frames": dynamic_count,
            "cached_transforms": len(self._world_cache),
            "registered_callbacks": len(self._callbacks)
        }

    def verify_tree_integrity(self) -> Tuple[bool, List[str]]:
        """
        Verify the integrity of the frame tree.

        Checks:
        - All parents exist
        - No circular dependencies
        - All transforms are valid 4x4 matrices

        Returns:
            (is_valid, list_of_errors)
        """
        errors = []

        # Check parent references
        for name, info in self._frames.items():
            if info.parent is not None and info.parent not in self._frames:
                errors.append(f"Frame '{name}' references missing parent '{info.parent}'")

        # Check for cycles using DFS
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

            rec_stack.discard(frame)
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

    # =================================================================
    # Private — Internal Methods
    # =================================================================

    def _get_world_transform(self, name: str) -> np.ndarray:
        """
        Get the transform from a frame to world (lazy, cached).

        Args:
            name: Frame name

        Returns:
            4x4 homogeneous transform matrix
        """
        if name == "world":
            return np.eye(4)

        # Return cached if available
        if name in self._world_cache:
            return self._world_cache[name].copy()

        info = self._frames.get(name)
        if info is None:
            raise ValueError(f"Frame '{name}' not found")

        if info.parent is None:
            raise ValueError(f"Frame '{name}' has no parent")

        # Compute: T_world = T_parent_world @ T_frame_parent
        T_parent_world = self._get_world_transform(info.parent)
        T_world = T_parent_world @ info.transform_parent

        self._world_cache[name] = T_world.copy()
        return T_world

    def _get_children(self, parent: str) -> List[str]:
        """
        Get all direct children of a frame (cached).

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

    def _invalidate_cache(self, name: str) -> None:
        """
        Invalidate cached world transforms for this frame and all descendants.

        Args:
            name: Frame to invalidate
        """
        self._world_cache.pop(name, None)
        for child in self._get_children(name):
            self._invalidate_cache(child)

    def _notify_callbacks(self, name: str, transform: np.ndarray) -> None:
        """Notify all registered callbacks of a transform change."""
        for cb in self._callbacks:
            try:
                cb(name, transform)
            except Exception as e:
                logger.error(f"Callback error for frame '{name}': {e}")

    def _validate_transform(self, transform: np.ndarray, name: str = "") -> None:
        """
        Validate that a transform is a proper 4x4 homogeneous matrix.

        Checks:
        - Shape is (4, 4)
        - Bottom row is [0, 0, 0, 1]
        - Rotation part is orthonormal

        Args:
            transform: Matrix to validate
            name: Frame name for error messages

        Raises:
            ValueError: If transform is invalid
        """
        if transform.shape != (4, 4):
            raise ValueError(
                f"Transform for '{name}' must be 4x4, got {transform.shape}"
            )

        if not np.allclose(transform[3, :], [0, 0, 0, 1]):
            raise ValueError(
                f"Transform for '{name}' must have [0, 0, 0, 1] as last row, "
                f"got {transform[3, :]}"
            )

        rotation = transform[:3, :3]
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
            raise ValueError(
                f"Transform for '{name}' has non-orthonormal rotation matrix"
            )

    def _would_create_cycle(self, name: str, new_parent: str) -> bool:
        """
        Check if setting parent would create a cycle.

        Args:
            name: Frame being reparented
            new_parent: Proposed parent frame

        Returns:
            True if a cycle would be created
        """
        current = new_parent
        while current is not None and current != name:
            info = self._frames.get(current)
            current = info.parent if info else None
        return current == name
