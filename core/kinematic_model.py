"""
PHASE 2 - Pure Data Source: Kinematic Model

Contract: Loads a URDF model, provides kinematic data, with NO visualization code.
Now using pure URDF parsing with ElementTree - NO PINOCCHIO DEPENDENCY.
"""

import numpy as np
import xml.etree.ElementTree as ET
import pdb
import logging
logging.basicConfig(level=logging.DEBUG)

from pathlib import Path
from typing import Dict, List, Optional, Tuple
import vtk


class KinematicModel:
    """
    A pure data module for URDF model loading and state management.
    Handles URDF parsing, mesh path resolution, and provides kinematic information.
    NO PINOCCHIO - pure Python with NumPy for transformations.
    """

    def __init__(self, urdf_path, package_dirs=None, transform_registry=None, asset_id=None,
                 update_registry_on_state_change=True):
        """
        Initialize the kinematic model data source.

        Args:
            urdf_path (str or Path): Path to the URDF file.
            package_dirs (list, optional): Directories for resolving
                `package://` style mesh paths. Defaults to None.
        """
        self._update_registry_on_state_change = update_registry_on_state_change
        self.transform_registry = transform_registry
        self.asset_id = asset_id
        
        # Use Path from pathlib
        self.urdf_path = Path(urdf_path).absolute()
        self.package_dirs = [Path(d).absolute() for d in (package_dirs or [])]

        # Parse URDF XML
        self.urdf_tree = ET.parse(str(self.urdf_path))
        self.urdf_root = self.urdf_tree.getroot()

        # Core data structures
        self.links: Dict[str, Dict] = {}      # Link name -> link properties
        self.joints: Dict[str, Dict] = {}     # Joint name -> joint properties
        self.link_parents: Dict[str, str] = {}  # Link name -> parent link name
        self.link_children: Dict[str, List[str]] = {}  # Link name -> list of child links

        # Transform tree - current state
        self.link_transforms: Dict[str, np.ndarray] = {}  # Link name -> 4x4 transform matrix

        # Visual geometry info
        self.visual_geometries: Dict[str, List[Dict]] = {}  # Link name -> list of visual geometry dicts

        # Joint state cache
        self._current_joint_positions: Dict[str, float] = {}  # Joint name -> current position
        self._current_q = None  # Keep for backward compatibility

        # ===== NEW: True root detection =====
        self.first_moving_joint = None
        self.true_root = None
        # ================================

        # Parse the URDF
        self._parse_urdf()

        # Build kinematic tree
        self._build_kinematic_tree()

        # ===== NEW: Find true root after parsing =====
        self._find_true_root()
        # ============================================

        # Initialize transforms to neutral position
        self._neutral_state()

        # ===== PHASE 1 ADDITIONS START =====
        # Tool transform (default = identity, meaning TCP = wrist_3_link)
        self._tool_transform = np.eye(4)
        
        # Name of the link that tools attach to (standard for UR robots)
        self.tool_mount_link = "wrist_3_link"
        
        # Optional: Verify this link exists in your model
        if self.tool_mount_link not in self.links:
            print(f"Note: {self.tool_mount_link} not found in URDF. Tools will attach to root?")
        # ===== PHASE 1 ADDITIONS END =====

    def _find_first_moving_joint(self):
        """
        Find the first joint that actually moves.
        Returns joint name or None if no moving joints found.
        """
        for joint_name, joint in self.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                return joint_name
        return None

    def _find_true_root(self):
        """
        Find the true root of the robot: the parent of the first moving joint.
        This is the reference frame from which all kinematics should be calculated.
        """
        self.first_moving_joint = self._find_first_moving_joint()
        
        if self.first_moving_joint:
            self.true_root = self.joints[self.first_moving_joint]['parent']
            print(f"\n  True root detected: {self.true_root}")
            print(f"  First moving joint: {self.first_moving_joint}")
        else:
            # No moving joints found - use first root link as fallback
            if self.root_links:
                self.true_root = self.root_links[0]
            else:
                self.true_root = "base_link"
            print(f"\n  No moving joints found. Using fallback root: {self.true_root}")
    
    def get_true_root(self) -> str:
        """
        Get the true root of the robot.
        This is the parent of the first moving joint.
        
        Returns:
            Name of the true root link
        """
        return self.true_root
    
    def get_first_moving_joint(self) -> Optional[str]:
        """
        Get the first moving joint of the robot.
        
        Returns:
            Name of the first moving joint, or None if no moving joints
        """
        return self.first_moving_joint

    def _parse_urdf(self):
        """Parse URDF XML and extract links, joints, and geometries."""

        # Parse links
        for link in self.urdf_root.findall('link'):
            link_name = link.get('name')
            self.links[link_name] = {'name': link_name}
            self.visual_geometries[link_name] = []

            # Parse visual geometries
            for visual in link.findall('visual'):
                geom_info = self._parse_visual_element(visual, link_name)
                if geom_info:
                    self.visual_geometries[link_name].append(geom_info)

        # Parse joints
        for joint in self.urdf_root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')

            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')

            # Parse origin (transform from parent to joint)
            origin_elem = joint.find('origin')
            if origin_elem is not None:
                xyz = origin_elem.get('xyz', '0 0 0')
                rpy = origin_elem.get('rpy', '0 0 0')
                origin_xyz = [float(x) for x in xyz.split()]
                origin_rpy = [float(r) for r in rpy.split()]
            else:
                origin_xyz = [0, 0, 0]
                origin_rpy = [0, 0, 0]

            # Parse axis (for revolute/prismatic joints)
            axis_elem = joint.find('axis')
            if axis_elem is not None:
                axis_xyz = [float(a) for a in axis_elem.get('xyz', '1 0 0').split()]
            else:
                axis_xyz = [1, 0, 0]

            # Parse limits
            limit_elem = joint.find('limit')
            if limit_elem is not None:
                lower = float(limit_elem.get('lower', '-3.14'))
                upper = float(limit_elem.get('upper', '3.14'))
                effort = float(limit_elem.get('effort', '0'))
                velocity = float(limit_elem.get('velocity', '0'))
            else:
                lower = -np.inf
                upper = np.inf
                effort = 0
                velocity = 0

            # Store joint info
            self.joints[joint_name] = {
                'name': joint_name,
                'type': joint_type,
                'parent': parent,
                'child': child,
                'origin_xyz': origin_xyz,
                'origin_rpy': origin_rpy,
                'axis': axis_xyz,
                'limit': {'lower': lower, 'upper': upper},
                'effort': effort,
                'velocity': velocity,
                'value': 0.0  # Current position
            }

            # Update parent-child relationships
            self.link_parents[child] = parent
            if parent not in self.link_children:
                self.link_children[parent] = []
            self.link_children[parent].append(child)

    def _parse_visual_element(self, visual_elem, link_name) -> Optional[Dict]:
        """
        Parse a visual element from URDF.

        Returns:
            Dict with geometry information, or None if parsing fails.
        """
        geometry = visual_elem.find('geometry')
        if geometry is None:
            return None

        # Handle different geometry types
        mesh = geometry.find('mesh')
        if mesh is not None:
            filename = mesh.get('filename')
            scale = mesh.get('scale', '1 1 1')
            scale_vals = [float(s) for s in scale.split()]

            # Resolve mesh path
            mesh_path = self._resolve_mesh_path(filename)

            # Parse origin transform
            origin = visual_elem.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                origin_xyz = [float(x) for x in xyz.split()]
                origin_rpy = [float(r) for r in rpy.split()]
            else:
                origin_xyz = [0, 0, 0]
                origin_rpy = [0, 0, 0]

            # Parse material
            material = visual_elem.find('material')
            if material is not None:
                color = material.find('color')
                if color is not None:
                    rgba = [float(c) for c in color.get('rgba', '0.7 0.7 0.7 1').split()]
                else:
                    rgba = [0.7, 0.7, 0.7, 1.0]
            else:
                rgba = [0.7, 0.7, 0.7, 1.0]

            return {
                'type': 'mesh',
                'link_name': link_name,
                'filename': filename,
                'mesh_path': mesh_path,
                'scale': scale_vals,
                'origin_xyz': origin_xyz,
                'origin_rpy': origin_rpy,
                'color': rgba[:3],
                'opacity': rgba[3],
                'origin_transform': self._compose_transform(origin_xyz, origin_rpy)
            }

        # TODO: Add support for box, cylinder, sphere geometries
        return None

    def _resolve_mesh_path(self, filename: str) -> Optional[Path]:
        """
        Resolve a mesh filename to absolute path using package directories.
        Handles package://, file://, and relative paths.

        Args:
            filename: Mesh filename from URDF (may contain package:// or file://).

        Returns:
            Path: Resolved mesh path, or None if not found.
        """
        # Handle file:// URIs
        if filename.startswith('file://'):
            file_path = filename[7:]  # Remove 'file://'
            path = Path(file_path)
            if path.exists():
                return path
            # Try relative to URDF
            path = self.urdf_path.parent / file_path
            if path.exists():
                return path
            return None

        # Handle package:// URIs
        if filename.startswith('package://'):
            package_path = filename[10:]  # Remove 'package://'
            parts = package_path.split('/')

            # Try each package directory
            for pkg_dir in self.package_dirs:
                candidate = pkg_dir / '/'.join(parts)
                if candidate.exists():
                    return candidate

                # Try with the first part as package name subdirectory
                if len(parts) > 0:
                    candidate = pkg_dir / parts[0] / '/'.join(parts[1:])
                    if candidate.exists():
                        return candidate

            # Fallback: try cache directory
            cache_dir = Path.home() / ".cache" / "robot_descriptions"
            candidate = cache_dir / '/'.join(parts)
            if candidate.exists():
                return candidate

            return None
        else:
            # Handle relative paths
            mesh_path = self.urdf_path.parent / filename
            if mesh_path.exists():
                return mesh_path

            # Try package directories with relative path
            for pkg_dir in self.package_dirs:
                candidate = pkg_dir / filename
                if candidate.exists():
                    return candidate

            return None

    def _build_kinematic_tree(self):
        """Build the kinematic tree structure."""
        # Find root link (has no parent)
        all_links = set(self.links.keys())
        children_links = set(self.link_parents.keys())
        root_links = all_links - children_links

        if not root_links:
            # If no root found, assume first link is root
            if all_links:
                root_links = {list(all_links)[0]}

        self.root_links = list(root_links)

    def _neutral_state(self):
        """Initialize transforms to neutral position."""
        # Initialize all transforms to identity
        for link_name in self.links:
            self.link_transforms[link_name] = np.eye(4)

        # Set root link transforms to identity
        for root_link in self.root_links:
            self.link_transforms[root_link] = np.eye(4)

        # Initialize joint positions to 0
        for joint_name in self.joints:
            self.joints[joint_name]['value'] = 0.0
            self._current_joint_positions[joint_name] = 0.0

        # Compute forward kinematics
        self._forward_kinematics()

        # Create configuration vector for backward compatibility
        self._update_config_vector()

    def _compose_transform(self, xyz: List[float], rpy: List[float]) -> np.ndarray:
        """
        Compose a 4x4 transform matrix from translation and Euler angles (RPY).

        Args:
            xyz: Translation [x, y, z]
            rpy: Euler angles [roll, pitch, yaw] in radians

        Returns:
            4x4 homogeneous transform matrix
        """
        transform = np.eye(4)

        # Translation
        transform[:3, 3] = xyz

        # Rotation (ZYX order - yaw, pitch, roll)
        roll, pitch, yaw = rpy

        # Rotation around Z (yaw)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Rotation around Y (pitch)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        # Rotation around X (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Combined rotation: R = Rz @ Ry @ Rx
        R = Rz @ Ry @ Rx
        transform[:3, :3] = R

        return transform

    def _transform_axis(self, axis: List[float], transform: np.ndarray) -> np.ndarray:
        """
        Transform an axis vector by a 4x4 transform matrix.

        Args:
            axis: 3D vector
            transform: 4x4 homogeneous transform

        Returns:
            Transformed 3D vector
        """
        axis_h = np.array([*axis, 0])
        transformed = transform @ axis_h
        return transformed[:3]

    def _forward_kinematics(self):
        """
        Compute forward kinematics for all links.
        Updates link_transforms dictionary.
        """
        # Start from root links
        for root_link in self.root_links:
            self.link_transforms[root_link] = np.eye(4)
            self._update_children_transforms(root_link)

    def _update_children_transforms(self, parent_link: str):
        """
        Recursively update transforms for all children of a link.

        Args:
            parent_link: Name of the parent link
        """
        if parent_link not in self.link_children:
            return

        for child_link in self.link_children[parent_link]:
            # Find the joint connecting parent to child
            joint = None
            for j in self.joints.values():
                if j['parent'] == parent_link and j['child'] == child_link:
                    joint = j
                    break

            if joint is None:
                continue

            # Get parent transform
            parent_transform = self.link_transforms[parent_link]

            # Compute joint transform
            joint_transform = self._compute_joint_transform(joint)

            # Child transform = parent_transform * joint_transform
            self.link_transforms[child_link] = parent_transform @ joint_transform

            # Recursively update grandchildren
            self._update_children_transforms(child_link)

    def _compute_joint_transform(self, joint: Dict) -> np.ndarray:
        """
        Compute the 4x4 transform for a joint at its current position.

        Args:
            joint: Joint dictionary

        Returns:
            4x4 transform matrix from parent to child
        """
        # Start with fixed origin transform
        transform = self._compose_transform(joint['origin_xyz'], joint['origin_rpy'])

        # Apply joint movement based on type
        joint_type = joint['type']
        value = joint['value']
        axis = np.array(joint['axis'])

        if joint_type == 'revolute' or joint_type == 'continuous':
            # Rotation around axis
            angle = value
            # Create rotation matrix around axis
            axis = axis / np.linalg.norm(axis)  # Normalize

            # Rodrigues rotation formula
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

            # Apply rotation to transform
            rotation_transform = np.eye(4)
            rotation_transform[:3, :3] = R
            transform = transform @ rotation_transform

        elif joint_type == 'prismatic':
            # Translation along axis
            displacement = value
            translation = axis * displacement

            translation_transform = np.eye(4)
            translation_transform[:3, 3] = translation
            transform = transform @ translation_transform

        # For fixed joints, just return the origin transform

        return transform

    def load(self):
        """
        Load the model (compatibility method).
        With pure URDF parsing, loading happens in __init__.
        """
        # Already parsed in __init__
        self._validate_mesh_paths()
        return self

    def _validate_mesh_paths(self):
        """Verify all visual mesh files exist."""
        missing_meshes = []

        for link_name, geometries in self.visual_geometries.items():
            for geom in geometries:
                if geom['type'] == 'mesh' and geom['mesh_path'] is None:
                    missing_meshes.append(f"{link_name}: {geom['filename']}")

        if missing_meshes:
            pass  # Silently handle missing meshes

    def get_joint_info(self):
        """
        Returns structured information about the robot's joints.
        """
        joint_names = []
        joint_limits = {}
        current_positions = {}

        for joint_name, joint in self.joints.items():
            # Skip fixed joints
            if joint['type'] == 'fixed':
                continue

            joint_names.append(joint_name)
            joint_limits[joint_name] = (
                joint['limit']['lower'],
                joint['limit']['upper']
            )
            current_positions[joint_name] = joint['value']

        return {
            'names': joint_names,
            'limits': joint_limits,
            'current_positions': current_positions
        }

    def get_current_joint_positions(self) -> np.ndarray:
        """
        Get current joint positions as an array in the correct order.
        
        Returns:
            numpy array of joint positions (6,) in the order of non-fixed joints
        """
        # Get all non-fixed joints in order
        joint_names = [name for name, j in self.joints.items() if j['type'] != 'fixed']
        
        # Build array of current positions
        positions = np.array([self.joints[name]['value'] for name in joint_names])
        
        return positions

    # ===== PHASE 1 ADDITIONS START =====
    def get_tool_transform(self):
        """
        Get the current tool transform (flange to TCP).
        
        Returns:
            4x4 numpy array representing transform from mounting link to tool tip
        """
        return self._tool_transform.copy()

    def set_tool_transform(self, transform):
        """
        Set the tool transform (flange to TCP).
        
        Args:
            transform: 4x4 numpy array representing transform from mounting link to tool tip
        """
        self._tool_transform = transform.copy()
        print(f"Tool transform updated")
        # Optional: print the transform for debugging
        # print(transform)

    def get_tcp_pose(self):
        """
        Get the current TCP (Tool Center Point) pose in base frame.
        
        TCP pose = mounting_link_pose @ tool_transform
        
        Returns:
            4x4 numpy array representing TCP pose in base frame
        """
        # Get the mounting link pose (default to identity if not found)
        mount_pose = self.link_transforms.get(self.tool_mount_link, np.eye(4))
        
        # Combine with tool transform to get TCP pose
        tcp_pose = mount_pose @ self._tool_transform
        
        return tcp_pose

    def get_tcp_vtk_transform(self):
        """
        Get VTK transform for TCP visualization.
        Useful for adding a visual marker at the TCP.
        """
        tcp_pose = self.get_tcp_pose()
        
        # Convert to VTK transform (using your existing helper)
        return self._matrix_to_vtk_transform(tcp_pose)

    # ===== PHASE 1 ADDITIONS END =====

    def update_state(self, q, update_registry=None):
        """
        Update the internal joint state.

        Args:
            q: Either a numpy array of joint positions (backward compatibility)
               or a dictionary mapping joint names to positions.
        """
        logging.debug(f"[KM] update_state called with type={type(q)}")
        logging.debug(f"[KM] q values: {q}")
        
        if isinstance(q, dict):
            logging.debug("[KM] Processing as dict")
            for name, pos in q.items():
                if name in self.joints:
                    old = self.joints[name]['value']
                    if old != pos:
                        logging.debug(f"[KM] {name}: {old:.4f} → {pos:.4f}")
                    self.joints[name]['value'] = pos
        elif isinstance(q, (list, np.ndarray)):
            logging.debug("[KM] Processing as list/array")
            joint_names = [name for name, j in self.joints.items() if j['type'] != 'fixed']
            logging.debug(f"[KM] Joint order: {joint_names}")
            for i, name in enumerate(joint_names):
                if i < len(q):
                    old = self.joints[name]['value']
                    new = q[i]
                    if old != new:
                        logging.debug(f"[KM] {name}: {old:.4f} → {new:.4f}")
                    self.joints[name]['value'] = new
        else:
            logging.error(f"[KM] Unknown type: {type(q)}")
            return
        
        logging.debug("[KM] Calling _forward_kinematics()")
        self._forward_kinematics()

        # Update transform registry if needed
        if self.transform_registry and self.asset_id:
            self._update_registry()

        # Verify results
        for link in ['shoulder_link', 'upper_arm_link']:
            if link in self.link_transforms:
                T = self.link_transforms[link]
                logging.debug(f"[KM] After FK, {link} at ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})")

        # Update config vector for backward compatibility
        self._update_config_vector()

    def _update_registry(self):
        """Update transform registry with current link transforms."""
        if not hasattr(self, 'transform_registry') or not self.transform_registry:
            return
        
        # print(f"\n[DEBUG] _update_registry() called")
        # import traceback
        # traceback.print_stack()
        
        # First, get the current world transforms from the model
        T_world = self.link_transforms.copy()
        
        for link_name, T_world_link in T_world.items():
            frame_name = f"{self.asset_id}_{link_name}"
            
            # Determine parent frame
            if link_name in self.root_links:
                parent_frame = "world"
                T_rel = T_world_link
            else:
                parent_link = self.link_parents.get(link_name)
                if parent_link is None:
                    parent_frame = "world"
                    T_rel = T_world_link
                else:
                    parent_frame = f"{self.asset_id}_{parent_link}"
                    # FIX: Use model's own link_transforms, not registry
                    T_world_parent = self.link_transforms.get(parent_link, np.eye(4))
                    # Compute relative transform directly from model's world transforms
                    T_rel = np.linalg.inv(T_world_parent) @ T_world_link
            
            # Update registry (this will invalidate cache and notify callbacks)
            self.transform_registry.update(frame_name, T_rel)

    def _update_config_vector(self):
        """Update the configuration vector for backward compatibility."""
        joint_names = [name for name, j in self.joints.items() if j['type'] != 'fixed']
        self._current_q = np.array([self.joints[name]['value'] for name in joint_names])

    def neutral_state(self):
        """
        Get the neutral (default) joint configuration.

        Returns:
            np.ndarray: Neutral joint positions (zeros for all non-fixed joints).
        """
        joint_names = [name for name, j in self.joints.items() if j['type'] != 'fixed']
        return np.zeros(len(joint_names))

    def get_vtk_transform(self, link_name: str) -> Optional[vtk.vtkTransform]:
        """
        Get VTK transform for a specific link at current state.

        Args:
            link_name: Name of the link.

        Returns:
            vtk.vtkTransform: Transform from world to link, or None if not found.
        """
        if link_name not in self.link_transforms:
            return None

        transform_matrix = self.link_transforms[link_name]

        # Convert to VTK transform
        transform = vtk.vtkTransform()

        # Extract translation
        translation = transform_matrix[:3, 3]
        transform.Translate(translation)

        # Extract rotation matrix and convert to Euler angles
        rotation = transform_matrix[:3, :3]
        euler = self._rotation_matrix_to_euler(rotation)

        # Apply in ZYX order (VTK convention)
        transform.RotateZ(np.degrees(euler[2]))
        transform.RotateY(np.degrees(euler[1]))
        transform.RotateX(np.degrees(euler[0]))

        return transform

    def _rotation_matrix_to_euler(self, R: np.ndarray) -> np.ndarray:
        """
        Convert rotation matrix to ZYX Euler angles (VTK convention).

        Args:
            R: 3x3 rotation matrix.

        Returns:
            np.ndarray: Euler angles [roll, pitch, yaw] in radians.
        """
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def get_urdf_root(self) -> ET.Element:
        """
        Get URDF XML root element.

        Returns:
            ET.Element: URDF root element.
        """
        return self.urdf_root

    def get_urdf_path(self) -> Path:
        """
        Get URDF file path.

        Returns:
            Path: Path to URDF file.
        """
        return self.urdf_path

    def get_visual_geometries(self) -> Dict[str, List[Dict]]:
        """
        Get all visual geometries.

        Returns:
            Dict mapping link names to lists of visual geometry dictionaries.
        """
        return self.visual_geometries

    def get_arm_chain(self, base_link_name: str = "base_link") -> List[str]:
        """
        Get the kinematic chain of revolute joints starting from base_link.
        
        This method handles both URDF conventions:
        - Direct connection: base_link → revolute joint
        - Via fixed joints: base_link → (fixed) → ... → revolute joint
        
        It traverses through fixed joints to reach the first revolute joint,
        then collects all subsequent revolute joints in order.
        
        Args:
            base_link_name: Name of the robot's base link (default: "base_link")
        
        Returns:
            List of joint names in order from base to TCP
        
        Raises:
            ValueError: If base_link not found or no revolute joints found
        """
        # Verify base link exists
        if base_link_name not in self.link_transforms:
            raise ValueError(f"Base link '{base_link_name}' not found. "
                            f"Available links: {list(self.link_transforms.keys())[:10]}...")
        
        chain = []
        current_link = base_link_name
        visited = set()  # Prevent infinite loops
        
        print(f"Building arm chain from base: {base_link_name}")
        
        # First, traverse through fixed joints to find the first revolute joint
        first_revolute_found = False
        while current_link in self.link_children and current_link not in visited:
            visited.add(current_link)
            next_revolute = None
            next_fixed = None
            
            for child in self.link_children[current_link]:
                for j_name, j in self.joints.items():
                    if j['parent'] == current_link and j['child'] == child:
                        if j['type'] in ['revolute', 'continuous']:
                            next_revolute = (j_name, child)
                            break
                        elif j['type'] == 'fixed':
                            next_fixed = child
                            break
                if next_revolute or next_fixed:
                    break
            
            if next_revolute:
                # Found the first revolute joint
                j_name, child_link = next_revolute
                chain.append(j_name)
                current_link = child_link
                first_revolute_found = True
                print(f"  Found first revolute joint: {j_name}")
                break
            elif next_fixed:
                # Follow fixed joint, continue looking
                current_link = next_fixed
                print(f"  Following fixed joint to: {current_link}")
            else:
                break
        
        if not first_revolute_found:
            raise ValueError(f"No revolute joints found from base link '{base_link_name}'")
        
        # Now continue collecting subsequent revolute joints
        while current_link in self.link_children and current_link not in visited:
            visited.add(current_link)
            next_joint = None
            next_link = None
            
            for child in self.link_children[current_link]:
                for j_name, j in self.joints.items():
                    if j['parent'] == current_link and j['child'] == child:
                        if j['type'] in ['revolute', 'continuous']:
                            next_joint = j_name
                            next_link = child
                            break
                if next_joint:
                    break
            
            if next_joint:
                chain.append(next_joint)
                current_link = next_link
                print(f"  Adding revolute joint: {next_joint}")
            else:
                break
        
        print(f"Arm chain complete: {chain}")
        return chain

    def get_joint_child_transforms(self, joint_names: List[str], base_link_name: str = "base_link") -> List[np.ndarray]:
        """
        Get the transform of each joint's child link relative to the robot base.
        
        Args:
            joint_names: List of joint names in order
            base_link_name: Name of the robot's base link
        
        Returns:
            List of 4x4 transform matrices for each joint's child link
        """
        # Get base transform
        T_base = self.link_transforms.get(base_link_name, np.eye(4))
        T_base_inv = np.linalg.inv(T_base)
        
        transforms = []
        for joint_name in joint_names:
            joint = self.joints.get(joint_name)
            if not joint:
                raise ValueError(f"Joint '{joint_name}' not found")
            
            child_link = joint['child']
            T_world = self.link_transforms.get(child_link, np.eye(4))
            T_rel = T_base_inv @ T_world
            transforms.append(T_rel)
        
        return transforms

    # In kinematic_model.py, add to KinematicModel class

    def set_ik_solver(self, ik_solver):
        """Set the IK solver for this kinematic model."""
        self._ik_solver = ik_solver
        print(f"IK solver attached to model")

    def solve_ik_for_tcp(self, target_pose: np.ndarray, q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose."""
        if not hasattr(self, '_ik_solver') or self._ik_solver is None:
            print("ERROR: IK solver not configured")
            raise RuntimeError("IK solver not configured. Call set_ik_solver() first.")
        return self._ik_solver.solve_ik_for_tcp(target_pose, q_guess)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using configured solver."""
        if hasattr(self, '_ik_solver') and self._ik_solver is not None:
            return self._ik_solver.forward_kinematics(q)
        
        # Fallback to URDF-based FK
        self.update_state(q)
        return self.get_tcp_pose()

