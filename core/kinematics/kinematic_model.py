"""
Kinematic Model - Pure data source for URDF kinematics.

Loads URDF, parses joint/links, computes forward kinematics.
No visualization code. No Pinocchio dependency.
Pure Python with NumPy for transformations.

Principle #4: Everything in URDF.
Principle #7: Movements as Models.
"""

import numpy as np
import xml.etree.ElementTree as ET
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import vtk

logger = logging.getLogger(__name__)


class KinematicModel:
    """
    A pure data module for URDF model loading and state management.

    Handles URDF parsing, mesh path resolution, and forward kinematics.
    No external robotics library required — pure Python with NumPy.

    The model detects the true kinematic root (parent of first moving joint)
    to handle URDFs where base_link is not the kinematic root (e.g., UR robots
    with a base_inertia fixed joint).

    Registry updates are controlled by update_registry_on_state_change.
    Set to False when StateHandler is managing registry updates externally.
    """

    def __init__(self,
                 urdf_path,
                 package_dirs=None,
                 transform_registry=None,
                 asset_id=None,
                 update_registry_on_state_change=True):
        """
        Initialize the kinematic model.

        Args:
            urdf_path: Path to URDF file.
            package_dirs: Directories for resolving package:// mesh paths.
            transform_registry: Optional TransformRegistry for frame registration.
            asset_id: Unique asset identifier for frame namespacing.
            update_registry_on_state_change: If True, update registry on each
                state change. Set to False when StateHandler manages this.
        """
        self._update_registry_on_state_change = update_registry_on_state_change
        self.transform_registry = transform_registry
        self.asset_id = asset_id

        self.urdf_path = Path(urdf_path).absolute()
        self.package_dirs = [Path(d).absolute() for d in (package_dirs or [])]

        # Parse URDF XML
        self.urdf_tree = ET.parse(str(self.urdf_path))
        self.urdf_root = self.urdf_tree.getroot()

        # Core data structures
        self.links: Dict[str, Dict] = {}
        self.joints: Dict[str, Dict] = {}
        self.link_parents: Dict[str, str] = {}
        self.link_children: Dict[str, List[str]] = {}

        # Transform state
        self.link_transforms: Dict[str, np.ndarray] = {}

        # Visual geometry info
        self.visual_geometries: Dict[str, List[Dict]] = {}

        # Joint state
        self._current_joint_positions: Dict[str, float] = {}
        self._current_q = None

        # True root detection
        self.first_moving_joint: Optional[str] = None
        self.true_root: Optional[str] = None

        # Tool configuration
        self._tool_transform = np.eye(4)
        self.tool_mount_link = None     # will be set in _find_true_root()

        # Parse and build
        self._parse_urdf()
        self._build_kinematic_tree()
        self._find_true_root()
        self._neutral_state()

    # =================================================================
    # URDF Parsing
    # =================================================================

    def _parse_urdf(self):
        """Parse URDF XML and extract links, joints, and geometries."""
        # Parse links
        for link in self.urdf_root.findall('link'):
            link_name = link.get('name')
            self.links[link_name] = {'name': link_name}
            self.visual_geometries[link_name] = []

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

            # Parse origin
            origin_elem = joint.find('origin')
            if origin_elem is not None:
                xyz = origin_elem.get('xyz', '0 0 0')
                rpy = origin_elem.get('rpy', '0 0 0')
                origin_xyz = [float(x) for x in xyz.split()]
                origin_rpy = [float(r) for r in rpy.split()]
            else:
                origin_xyz = [0, 0, 0]
                origin_rpy = [0, 0, 0]

            # Parse axis
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
                'value': 0.0
            }

            self.link_parents[child] = parent
            if parent not in self.link_children:
                self.link_children[parent] = []
            self.link_children[parent].append(child)

    def _parse_visual_element(self, visual_elem, link_name) -> Optional[Dict]:
        """Parse a visual element from URDF."""
        geometry = visual_elem.find('geometry')
        if geometry is None:
            return None

        mesh = geometry.find('mesh')
        if mesh is not None:
            filename = mesh.get('filename')
            scale = mesh.get('scale', '1 1 1')
            scale_vals = [float(s) for s in scale.split()]

            mesh_path = self._resolve_mesh_path(filename)

            origin = visual_elem.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                origin_xyz = [float(x) for x in xyz.split()]
                origin_rpy = [float(r) for r in rpy.split()]
            else:
                origin_xyz = [0, 0, 0]
                origin_rpy = [0, 0, 0]

            material = visual_elem.find('material')
            color = material.find('color') if material is not None else None
            if color is not None:
                rgba = [float(c) for c in color.get('rgba', '0.7 0.7 0.7 1').split()]
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

        return None

    def _resolve_mesh_path(self, filename: str) -> Optional[Path]:
        """Resolve mesh filename to absolute path using package directories."""
        if filename.startswith('file://'):
            file_path = filename[7:]
            path = Path(file_path)
            if path.exists():
                return path
            path = self.urdf_path.parent / file_path
            return path if path.exists() else None

        if filename.startswith('package://'):
            package_path = filename[10:]  # e.g., "ur10/meshes/base_link.stl"
            parts = package_path.split('/', 1)  # ["ur10", "meshes/base_link.stl"]

            if len(parts) == 2:
                package_name, relative_path = parts

                for pkg_dir in self.package_dirs:
                    # Look for pkg_dir/package_name/relative_path
                    candidate = Path(pkg_dir) / package_name / relative_path
                    if candidate.exists():
                        return candidate

            return None

        # Relative path
        mesh_path = self.urdf_path.parent / filename
        if mesh_path.exists():
            return mesh_path

        for pkg_dir in self.package_dirs:
            candidate = pkg_dir / filename
            if candidate.exists():
                return candidate

        return None

    # =================================================================
    # Kinematic Tree
    # =================================================================

    def _build_kinematic_tree(self):
        """Build the kinematic tree structure, finding root links."""
        all_links = set(self.links.keys())
        children_links = set(self.link_parents.keys())
        root_links = all_links - children_links

        if not root_links and all_links:
            root_links = {list(all_links)[0]}

        self.root_links = list(root_links)

    def _find_true_root(self):
        """
        Find the true kinematic root: parent of the first moving joint.

        Handles URDFs where base_link is not the kinematic root
        (e.g., UR robots with base_inertia fixed joint).
        """
        self.first_moving_joint = self._find_first_moving_joint()

        if self.first_moving_joint:
            self.true_root = self.joints[self.first_moving_joint]['parent']
            logger.info(f"True root: {self.true_root} "
                       f"(first moving joint: {self.first_moving_joint})")

            # Detect tool mount link (last link in the kinematic chain)
            try:
                arm_chain = self.get_arm_chain(base_link_name=self.true_root)
                if arm_chain:
                    last_joint_name = arm_chain[-1]
                    last_joint = self.joints.get(last_joint_name)
                    if last_joint:
                        self.tool_mount_link = last_joint['child']

                        # Follow any fixed joints to reach the actual tool mounting point
                        current_link = self.tool_mount_link
                        while current_link in self.link_children:
                            children = self.link_children[current_link]
                            if len(children) != 1:
                                break
                            child = children[0]
                            # Check that the child is connected by a fixed joint
                            is_fixed = False
                            for j in self.joints.values():
                                if (j['parent'] == current_link and
                                    j['child'] == child and
                                    j['type'] == 'fixed'):
                                    is_fixed = True
                                    break
                            if is_fixed:
                                current_link = child
                            else:
                                break
                        self.tool_mount_link = current_link

                        logger.info(f"Tool mount link: {self.tool_mount_link}")
            except Exception as e:
                self.tool_mount_link = "wrist_3_link"  # fallback
                logger.info(f"Tool mount link (fallback): {self.tool_mount_link} ({e})")

        else:
            if self.root_links:
                self.true_root = self.root_links[0]
            else:
                self.true_root = "base_link"
            logger.info(f"No moving joints found. Using fallback root: {self.true_root}")

    def _find_first_moving_joint(self) -> Optional[str]:
        """Find the first joint that is revolute, continuous, or prismatic."""
        for joint_name, joint in self.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                return joint_name
        return None

    def get_true_root(self) -> str:
        """Get the true kinematic root of the robot."""
        return self.true_root

    def get_first_moving_joint(self) -> Optional[str]:
        """Get the first moving joint of the robot."""
        return self.first_moving_joint

    # =================================================================
    # Forward Kinematics
    # =================================================================

    def _neutral_state(self):
        """Initialize all transforms and joint positions to zero."""
        for link_name in self.links:
            self.link_transforms[link_name] = np.eye(4)

        for root_link in self.root_links:
            self.link_transforms[root_link] = np.eye(4)

        for joint_name in self.joints:
            self.joints[joint_name]['value'] = 0.0
            self._current_joint_positions[joint_name] = 0.0

        self._forward_kinematics()
        self._update_config_vector()

    def _forward_kinematics(self):
        """Compute forward kinematics for all links."""
        for root_link in self.root_links:
            self.link_transforms[root_link] = np.eye(4)
            self._update_children_transforms(root_link)

    def _update_children_transforms(self, parent_link: str):
        """Recursively update transforms for all children of a link."""
        if parent_link not in self.link_children:
            return

        for child_link in self.link_children[parent_link]:
            joint = None
            for j in self.joints.values():
                if j['parent'] == parent_link and j['child'] == child_link:
                    joint = j
                    break

            if joint is None:
                continue

            parent_transform = self.link_transforms[parent_link]
            joint_transform = self._compute_joint_transform(joint)
            self.link_transforms[child_link] = parent_transform @ joint_transform

            self._update_children_transforms(child_link)

    def _compute_joint_transform(self, joint: Dict) -> np.ndarray:
        """Compute the 4x4 transform for a joint at its current position."""
        transform = self._compose_transform(joint['origin_xyz'], joint['origin_rpy'])

        joint_type = joint['type']
        value = joint['value']
        axis = np.array(joint['axis'])

        if joint_type in ('revolute', 'continuous'):
            axis = axis / np.linalg.norm(axis)
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R = np.eye(3) + np.sin(value) * K + (1 - np.cos(value)) * K @ K
            rotation_transform = np.eye(4)
            rotation_transform[:3, :3] = R
            transform = transform @ rotation_transform

        elif joint_type == 'prismatic':
            translation = axis * value
            translation_transform = np.eye(4)
            translation_transform[:3, 3] = translation
            transform = transform @ translation_transform

        return transform

    @staticmethod
    def _compose_transform(xyz: List[float], rpy: List[float]) -> np.ndarray:
        """Compose 4x4 transform from translation and RPY Euler angles."""
        transform = np.eye(4)
        transform[:3, 3] = xyz

        roll, pitch, yaw = rpy

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        transform[:3, :3] = Rz @ Ry @ Rx
        return transform

    # =================================================================
    # State Management
    # =================================================================

    def update_state(self, q, update_registry=None):
        """
        Update joint state and recompute forward kinematics.

        Args:
            q: Either a numpy array of joint positions (in joint order)
               or a dictionary mapping joint names to positions.
            update_registry: Override for update_registry_on_state_change.
        """
        if isinstance(q, dict):
            for name, pos in q.items():
                if name in self.joints:
                    self.joints[name]['value'] = pos
        elif isinstance(q, (list, np.ndarray)):
            joint_names = [name for name, j in self.joints.items()
                          if j['type'] != 'fixed']
            for i, name in enumerate(joint_names):
                if i < len(q):
                    self.joints[name]['value'] = q[i]
        else:
            logger.error(f"Unknown type for update_state: {type(q)}")
            return

        self._forward_kinematics()

        # Update registry if configured (StateHandler sets this to False)
        should_update = (update_registry if update_registry is not None
                        else self._update_registry_on_state_change)
        if should_update and self.transform_registry and self.asset_id:
            self._update_registry()

        self._update_config_vector()

    def update_registry(self):
        """Update TransformRegistry with current link transforms."""
        if not self.transform_registry:
            return

        for link_name, T_world_link in self.link_transforms.items():
            frame_name = f"{self.asset_id}_{link_name}"

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
                    T_world_parent = self.link_transforms.get(parent_link, np.eye(4))
                    T_rel = np.linalg.inv(T_world_parent) @ T_world_link

            self.transform_registry.update_frame(frame_name, T_rel)

    def _update_registry(self):
        """Update TransformRegistry with current link transforms (topological order)."""
        if not self.transform_registry:
            return

        for name in sorted(self.link_transforms.keys()):
            parent = self.link_parents.get(name, 'NONE')
            is_root = name in self.root_links
            print(f"  {'[ROOT]' if is_root else '      '} {name} -> parent: {parent}")

        # Build a dependency graph: child -> parent
        # Root links have no parent and should be registered first
        registered = set()
        pending = set(self.link_transforms.keys())

        # Register all links in topological order (parents before children)
        while pending:
            progress = False
            for link_name in list(pending):
                frame_name = f"{self.asset_id}_{link_name}"

                if link_name in self.root_links:
                    # Root link: attach to world
                    T_world_link = self.link_transforms[link_name]
                    self.transform_registry.update_frame(frame_name, T_world_link)
                    registered.add(link_name)
                    pending.discard(link_name)
                    progress = True
                else:
                    parent_link = self.link_parents.get(link_name)
                    if parent_link is None:
                        # Orphaned link: attach to world
                        T_world_link = self.link_transforms[link_name]
                        self.transform_registry.update_frame(frame_name, T_world_link)
                        registered.add(link_name)
                        pending.discard(link_name)
                        progress = True
                    else:
                        parent_frame = f"{self.asset_id}_{parent_link}"
                        # Only register if parent is already registered
                        if parent_link in registered or parent_frame in self.transform_registry.list_frames():
                            T_world_link = self.link_transforms[link_name]
                            T_world_parent = self.link_transforms.get(parent_link, np.eye(4))
                            T_rel = np.linalg.inv(T_world_parent) @ T_world_link
                            self.transform_registry.update_frame(frame_name, T_rel)
                            registered.add(link_name)
                            pending.discard(link_name)
                            progress = True

            if not progress:
                # Circular dependency or missing parents — register remaining to world
                for link_name in pending:
                    frame_name = f"{self.asset_id}_{link_name}"
                    T_world_link = self.link_transforms[link_name]
                    self.transform_registry.update_frame(frame_name, T_world_link)
                break

    def _update_config_vector(self):
        """Update the configuration vector for backward compatibility."""
        joint_names = [name for name, j in self.joints.items()
                      if j['type'] != 'fixed']
        self._current_q = np.array([self.joints[name]['value']
                                    for name in joint_names])

    # =================================================================
    # Public API
    # =================================================================

    def load(self):
        """Load the model (compatibility method — parsing happens in __init__)."""
        return self

    def neutral_state(self) -> np.ndarray:
        """Get the neutral (zero) joint configuration."""
        joint_names = [name for name, j in self.joints.items()
                      if j['type'] != 'fixed']
        return np.zeros(len(joint_names))

    def get_joint_info(self) -> Dict:
        """Get structured joint information."""
        joint_names = []
        joint_limits = {}
        current_positions = {}

        for joint_name, joint in self.joints.items():
            if joint['type'] == 'fixed':
                continue
            joint_names.append(joint_name)
            joint_limits[joint_name] = (joint['limit']['lower'], joint['limit']['upper'])
            current_positions[joint_name] = joint['value']

        return {
            'names': joint_names,
            'limits': joint_limits,
            'current_positions': current_positions
        }

    def get_current_joint_positions(self) -> np.ndarray:
        """Get current joint positions as array in joint order."""
        joint_names = [name for name, j in self.joints.items()
                      if j['type'] != 'fixed']
        return np.array([self.joints[name]['value'] for name in joint_names])

    def get_tcp_pose(self) -> np.ndarray:
        """Get current TCP pose in world frame."""
        mount_pose = self.link_transforms.get(self.tool_mount_link, np.eye(4))
        return mount_pose @ self._tool_transform

    def get_tool_transform(self) -> np.ndarray:
        """Get the tool transform (mount link to TCP)."""
        return self._tool_transform.copy()

    def set_tool_transform(self, transform: np.ndarray):
        """Set the tool transform."""
        self._tool_transform = transform.copy()

    def set_ik_solver(self, ik_solver):
        """Attach an IK solver to this model."""
        self._ik_solver = ik_solver
        logger.info("IK solver attached to model")

    def solve_ik_for_tcp(self,
                         target_pose: np.ndarray,
                         q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose using attached solver."""
        print(f"[KINEMATIC] going to call ik_solver")
        if not hasattr(self, '_ik_solver') or self._ik_solver is None:
            print(f"[KINEMATIC] Call set_ik_solver() first.")
            raise RuntimeError("IK solver not configured. Call set_ik_solver() first.")
        return self._ik_solver.solve_ik_for_tcp(target_pose, q_guess)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Compute forward kinematics for given joint positions."""
        if hasattr(self, '_ik_solver') and self._ik_solver is not None:
            return self._ik_solver.forward_kinematics(q)
        self.update_state(q)
        return self.get_tcp_pose()

    def get_arm_chain(self, base_link_name: str = "base_link") -> List[str]:
        """
        Get the kinematic chain of revolute joints from base_link.

        Traverses through fixed joints to reach the first revolute joint,
        then collects all subsequent revolute joints in order.
        """
        if base_link_name not in self.links:
            raise ValueError(
                f"Base link '{base_link_name}' not found. "
                f"Available: {list(self.link_transforms.keys())[:10]}..."
            )

        chain = []
        current_link = base_link_name
        visited = set()
        first_revolute_found = False

        # Traverse through fixed joints to first revolute
        while current_link in self.link_children and current_link not in visited:
            visited.add(current_link)
            next_revolute = None
            next_fixed = None

            for child in self.link_children[current_link]:
                for j_name, j in self.joints.items():
                    if j['parent'] == current_link and j['child'] == child:
                        if j['type'] in ('revolute', 'continuous'):
                            next_revolute = (j_name, child)
                            break
                        elif j['type'] == 'fixed':
                            next_fixed = child
                            break
                if next_revolute or next_fixed:
                    break

            if next_revolute:
                j_name, child_link = next_revolute
                chain.append(j_name)
                current_link = child_link
                first_revolute_found = True
                break
            elif next_fixed:
                current_link = next_fixed
            else:
                break

        if not first_revolute_found:
            raise ValueError(f"No revolute joints found from '{base_link_name}'")

        # Collect remaining revolute joints
        while current_link in self.link_children and current_link not in visited:
            visited.add(current_link)
            next_joint = None
            next_link = None

            for child in self.link_children[current_link]:
                for j_name, j in self.joints.items():
                    if j['parent'] == current_link and j['child'] == child:
                        if j['type'] in ('revolute', 'continuous'):
                            next_joint = j_name
                            next_link = child
                            break
                if next_joint:
                    break

            if next_joint:
                chain.append(next_joint)
                current_link = next_link
            else:
                break

        return chain

    def get_joint_child_transforms(self,
                                   joint_names: List[str],
                                   base_link_name: str = "base_link") -> List[np.ndarray]:
        """Get transform of each joint's child link relative to robot base."""
        T_base = self.link_transforms.get(base_link_name, np.eye(4))
        T_base_inv = np.linalg.inv(T_base)

        transforms = []
        for joint_name in joint_names:
            joint = self.joints.get(joint_name)
            if not joint:
                raise ValueError(f"Joint '{joint_name}' not found")
            child_link = joint['child']
            T_world = self.link_transforms.get(child_link, np.eye(4))
            transforms.append(T_base_inv @ T_world)

        return transforms

    def get_urdf_path(self) -> Path:
        """Get the URDF file path."""
        return self.urdf_path

    def get_visual_geometries(self) -> Dict[str, List[Dict]]:
        """Get all visual geometries."""
        return self.visual_geometries

    def get_vtk_transform(self, link_name: str) -> Optional[vtk.vtkTransform]:
        """Get VTK transform for a link at current state."""
        if link_name not in self.link_transforms:
            return None

        transform_matrix = self.link_transforms[link_name]
        vtk_transform = vtk.vtkTransform()
        vtk_transform.Translate(transform_matrix[:3, 3])

        rotation = transform_matrix[:3, :3]
        euler = self._rotation_matrix_to_euler(rotation)
        vtk_transform.RotateZ(np.degrees(euler[2]))
        vtk_transform.RotateY(np.degrees(euler[1]))
        vtk_transform.RotateX(np.degrees(euler[0]))

        return vtk_transform

    @staticmethod
    def _rotation_matrix_to_euler(R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to ZYX Euler angles (radians)."""
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
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

