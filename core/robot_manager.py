"""
Robot Manager - Pure Python service for robot lifecycle management.

Owns the RealRobot and SimulatedRobot instances.
Handles robot loading, connection management, and mode switching.
Publishes all state changes via StateChannel events.
Does NOT emit Qt signals. Does NOT inherit from QObject.

Principle #4: Everything in URDF.
Principle #8: Pure Python (no Qt in core services).
Principle #9: UI Separate from Services.
Principle #10: One Robot Per Session.
"""

from typing import Optional, Dict, Any
from pathlib import Path
import numpy as np
import logging

from core.world_state.state_channel import StateChannel
from core.world_state.transform_registry import TransformRegistry
from core.world_state.event_types import EventType
from core.mode import Mode
from displays.joint_frame_display import JointFrameDisplay

logger = logging.getLogger(__name__)


class RobotManager:
    """
    Manages robot lifecycle: loading, connection, and mode switching.

    Owns the RealRobot and SimulatedRobot instances.
    Publishes events to StateChannel for all components to consume.
    UI panels subscribe to StateChannel directly — no Qt signals needed.

    Responsibilities:
    - Load robots from URDF files
    - Manage robot instances (simulated and real)
    - Handle connection/disconnection to hardware
    - Manage operating mode (simulate / real)
    - Know which frame is the Cartesian base for each asset

    Does NOT:
    - Handle commands (that's CommandHandler)
    - Update kinematic models on state changes (that's StateHandler)
    - Emit Qt signals (UI subscribes to StateChannel directly)
    - Hold business logic for joint/Cartesian control
    """

    def __init__(self,
                 transform_registry: TransformRegistry,
                 state_channel: StateChannel,
                 engine):  # VisualizerEngine for display registration
        """
        Initialize RobotManager.

        Args:
            transform_registry: Central transform registry
            state_channel: Application event bus
            engine: VisualizerEngine for registering kinematic displays
        """
        self.transform_registry = transform_registry
        self.state_channel = state_channel
        self.engine = engine

        # Robot instances (injected by MainWindow)
        self._simulated_robot = None
        self._real_robot = None

        self.mesh_loader = None # Set by MainWindow if available

        # Current robot state
        self.current_asset_id: Optional[str] = None
        self.current_kinematic_model = None
        self.current_mode = Mode.SIMULATE_LOCAL
        self.is_connected = False
        self.robot_ip: Optional[str] = None

        # Asset base frame mapping: asset_id -> base_frame_name
        # The base frame is the true kinematic root for Cartesian control.
        self._asset_bases: Dict[str, str] = {}

        # Registry of loaded robots (one at a time per Principle #10)
        self._loaded_robots: Dict[str, Dict[str, Any]] = {}

        # Subscribe to events that RobotManager handles
        self._setup_subscriptions()

        logger.info("RobotManager initialized")

    # =================================================================
    # Robot Instance Injection (called by MainWindow)
    # =================================================================

    def set_simulated_robot(self, simulated_robot):
        """Inject the SimulatedRobot instance."""
        self._simulated_robot = simulated_robot

    def set_real_robot(self, real_robot):
        """Inject the RealRobot instance."""
        self._real_robot = real_robot

    # =================================================================
    # Event Subscriptions
    # =================================================================

    def _setup_subscriptions(self):
        """Subscribe to StateChannel events."""
        self.state_channel.subscribe(EventType.ROBOT_LOAD_REQUEST, self._on_load_request)

    def _on_load_request(self, event):
        """Handle ROBOT_LOAD_REQUEST from UI (File → Load URDF)."""
        urdf_path = event.data.get('urdf_path')
        robot_id = event.data.get('robot_id')

        if not urdf_path:
            logger.warning("Load request missing urdf_path")
            return

        logger.info(f"Loading robot: {robot_id} from {urdf_path}")
        self.load_robot(urdf_path, robot_id)

    # =================================================================
    # Robot Loading
    # =================================================================

    def load_robot(self, urdf_path: str, asset_id: str = None) -> Optional[str]:
        """
        Load a robot from URDF or xacro file.

        Parses the file (preprocessing xacro if needed), creates kinematic
        model and display, registers transforms, publishes ROBOT_LOADED.

        Args:
            urdf_path: Path to .urdf or .xacro file.
            asset_id: Optional ID (auto-generated from filename).

        Returns:
            asset_id if successful, None otherwise.
        """
        from pathlib import Path
        import tempfile
        from core.kinematics.kinematic_model import KinematicModel
        from displays.kinematic_display import KinematicDisplay

        urdf_path = Path(urdf_path).expanduser().resolve()

        try:
            if asset_id is None:
                asset_id = urdf_path.stem

            # Handle duplicate IDs
            if asset_id in self._loaded_robots:
                original = asset_id
                asset_id = f"{asset_id}_{len(self._loaded_robots)}"
                logger.info(f"Asset ID '{original}' exists, using '{asset_id}'")

            # Package directories for resolving package:// paths
            package_dirs = [
                str(urdf_path.parent),                          # URDF's directory
                str(urdf_path.parent.parent),                   # Package root
                str(urdf_path.parent.parent.parent),            # Category
                str(Path.home() / "hatch" / "assets"),           # Top-level assets
                str(Path.home() / "hatch" / "assets" / "scenes"),
                str(Path.home() / "hatch" / "assets" / "robots"),
                str(Path.home() / "hatch" / "assets" / "sensors"),
                str(Path.home() / "hatch" / "assets" / "ugv"),
                str(Path.home() / "hatch" / "assets" / "tools"),
                str(Path.home() / ".cache" / "robot_descriptions"),
            ]

            logger.info(f"Loading: {urdf_path}")

            # Preprocess xacro files, or load URDF directly
            if urdf_path.suffix == '.xacro':
                from core.urdf_preprocessor import URDFPreprocessor
                preprocessor = URDFPreprocessor(package_dirs)
                urdf_xml = preprocessor.process(str(urdf_path))

                # Write preprocessed URDF to temp file for KinematicModel
                with tempfile.NamedTemporaryFile(
                    mode='w', suffix='.urdf', delete=False
                ) as f:
                    f.write(urdf_xml)
                    temp_path = f.name

                model = KinematicModel(
                    urdf_path=temp_path,
                    package_dirs=package_dirs,
                    transform_registry=self.transform_registry,
                    asset_id=asset_id,
                    update_registry_on_state_change=False
                )
            else:
                model = KinematicModel(
                    urdf_path=str(urdf_path),
                    package_dirs=package_dirs,
                    transform_registry=self.transform_registry,
                    asset_id=asset_id,
                    update_registry_on_state_change=False
                )

            model.load()
            logger.info(f"Kinematic model loaded: {asset_id}")

            # Attach IK solver
            self._attach_ik_solver(model)

            # Register initial transforms
            self._register_initial_transforms(asset_id, model)

            # Store asset base frame (true kinematic root for Cartesian control)
            true_root = model.get_true_root()
            self._asset_bases[asset_id] = f"{asset_id}_{true_root}"
            logger.info(f"Asset base frame: {self._asset_bases[asset_id]}")

            # Create visual display
            display = KinematicDisplay(
                model,
                self.transform_registry,
                mesh_loader=self.mesh_loader,  # Pass mesh_loader if available
                asset_id=asset_id
            )
            display.attach(self.engine.get_renderer())
            self.engine.register_display(display)

            # Add joint frame display
            joint_display = JointFrameDisplay(model, self.transform_registry, 
                                            asset_id=asset_id, scale=0.3)
            joint_display.attach(self.engine.get_renderer())

            # Store reference for cleanup
            self._joint_display = joint_display

            # Store in registry
            self._loaded_robots[asset_id] = {
                'model': model,
                'display': display,
                'urdf_path': str(urdf_path)
            }

            # Set as current
            self.current_asset_id = asset_id
            self.current_kinematic_model = model

            # Update simulated robot
            if self._simulated_robot:
                self._simulated_robot.set_kinematic_model(model)

            # Publish ROBOT_LOADED
            self.state_channel.publish(
                EventType.ROBOT_LOADED,
                data={
                    'asset_id': asset_id,
                    'urdf_path': str(urdf_path),
                    'kinematic_model': model
                },
                source="robot_manager",
                description=f"Robot {asset_id} loaded"
            )

            logger.info(f"Robot loaded successfully: {asset_id}")
            return asset_id

        except Exception as e:
            logger.error(f"Failed to load robot: {e}", exc_info=True)

            self.state_channel.publish(
                EventType.ERROR_OCCURRED,
                data={'error': f"Failed to load robot: {e}"},
                source="robot_manager"
            )
            return None

    def _attach_ik_solver(self, model):
        """Attach IK solver to the kinematic model."""
        try:
            from core.kinematics.ik_solver import IKSolver
            ik_solver = IKSolver(model)
            model.set_ik_solver(ik_solver)
            if ik_solver is not None:
                print("[ROB MANAGER]ik_solver is not None.")
                logger.info("IK solver attached")
            else:
                print("[ROB MANAGER]ik_solver is None.")
        except ImportError:
            logger.info("IK solver not available (missing dependencies)")
        except Exception as e:
            logger.warning(f"Could not attach IK solver: {e}")

    def _register_initial_transforms(self, asset_id: str, model):
        """
        Register all robot frames in TransformRegistry on initial load.

        Computes parent-relative transforms for each link.
        Uses the true kinematic root (parent of first moving joint)
        as the reference. Fixed joints between world and true root
        are preserved in the transform chain for visualization.
        """
        from core.world_state.transform_registry import FrameStatus

        if not hasattr(model, 'link_transforms'):
            return

        true_root = model.get_true_root()
        logger.info(f"Registering transforms for {asset_id} "
                   f"(true root: {true_root})")

        # Collect frames with parent relationships
        frames_info = []
        for link_name in model.link_transforms.keys():
            frame_name = f"{asset_id}_{link_name}"

            if link_name == true_root:
                parent = "world"
            elif link_name in model.link_parents:
                parent_link = model.link_parents.get(link_name)
                parent = f"{asset_id}_{parent_link}" if parent_link else "world"
            else:
                parent = "world"

            frames_info.append({
                'name': frame_name,
                'original_name': link_name,
                'parent': parent,
                'is_true_root': (link_name == true_root)
            })

        # Sort by depth: parents before children
        frames_info.sort(key=lambda f: self._frame_depth(f, frames_info))

        # Register each frame
        registered = 0
        for frame in frames_info:
            frame_name = frame['name']
            parent = frame['parent']
            link_name = frame['original_name']
            T_world = model.link_transforms[link_name]

            if parent == "world":
                T_rel = T_world
            else:
                parent_link = parent.replace(f"{asset_id}_", "")
                if parent_link in model.link_transforms:
                    T_parent_world = model.link_transforms[parent_link]
                    T_rel = np.linalg.inv(T_parent_world) @ T_world
                else:
                    logger.warning(f"Parent {parent} not found, using world")
                    T_rel = T_world

            try:
                self.transform_registry.register_frame(
                    frame_name,
                    T_rel,
                    parent=parent,
                    status=FrameStatus.DYNAMIC,
                    description=f"Link: {link_name}"
                )
                registered += 1
            except ValueError as e:
                logger.warning(f"Failed to register {frame_name}: {e}")

        # Register TCP frame
        tcp_frame = f"{asset_id}_tcp"
        mount_link = model.tool_mount_link or "wrist_3_link"
        parent_frame = f"{asset_id}_{mount_link}"

        try:
            self.transform_registry.register_frame(
                tcp_frame,
                np.eye(4),  # TCP at mount point by default
                parent=parent_frame,
                status=FrameStatus.DYNAMIC,
                description="Tool Center Point"
            )
            registered += 1
        except ValueError as e:
            logger.warning(f"Could not register TCP frame: {e}")

        logger.info(f"Registered {registered} frames for {asset_id}")

    def _frame_depth(self, frame, frames_info):
        """Calculate depth of a frame in the tree (world = depth 0)."""
        depth = 0
        current = frame
        visited = set()
        while current['parent'] != 'world' and depth < 100:
            if current['name'] in visited:
                break
            visited.add(current['name'])
            parent_name = current['parent']
            parent = next((f for f in frames_info if f['name'] == parent_name), None)
            if parent is None:
                break
            current = parent
            depth += 1
        return depth

    # =================================================================
    # Connection Management
    # =================================================================

    def connect_robot(self, ip: str, **kwargs) -> bool:
        """Connect to real robot."""
        if not self._real_robot:
            logger.warning("No RealRobot instance available")
            return False

        success = self._real_robot.connect(ip, **kwargs)
        if success:
            self.is_connected = True
            self.robot_ip = ip
            logger.info(f"Connected to robot at {ip}")
        return success

    def disconnect_robot(self):
        """Disconnect from real robot."""
        if self._real_robot:
            self._real_robot.disconnect()
        self.is_connected = False
        self.robot_ip = None
        logger.info("Disconnected from robot")

    # =================================================================
    # Mode Management
    # =================================================================

    def set_mode(self, mode: str):
        """
        Request a mode switch.

        Valid modes: "simulate_local", "simulate_real_ik", "real"

        Publishes MODE_SWITCH_REQUEST. CommandHandler processes it
        and publishes MODE_SWITCHED on completion.
        """
        valid_modes = ["simulate_local", "simulate_real_ik", "real"]
        if mode not in valid_modes:
            logger.warning(f"Invalid mode: {mode}")
            return

        if mode == "real" and not self.is_connected:
            self.state_channel.publish(
                EventType.ERROR_OCCURRED,
                data={'error': "Cannot switch to real mode: Not connected"},
                source="robot_manager"
            )
            return

        self.state_channel.publish(
            EventType.MODE_SWITCH_REQUEST,
            data={'mode': mode},
            source="robot_manager"
        )

    # =================================================================
    # Asset Base Frame Queries (for Cartesian control)
    # =================================================================

    def get_asset_base_frame(self, asset_id: str) -> str:
        """
        Get the base frame for Cartesian control.

        Returns the true kinematic root frame name for the asset.
        Defaults to "world" if asset not found.

        Args:
            asset_id: Unique asset identifier

        Returns:
            Frame name for Cartesian base
        """
        return self._asset_bases.get(asset_id, "world")

    def get_asset_base_transform(self, asset_id: str) -> np.ndarray:
        """
        Get the transform from world to the asset's Cartesian base.

        Args:
            asset_id: Unique asset identifier

        Returns:
            4x4 homogeneous transform matrix
        """
        base_frame = self.get_asset_base_frame(asset_id)
        return self.transform_registry.get_transform(base_frame, "world")

    # =================================================================
    # Public Query Methods
    # =================================================================

    def get_current_joint_positions(self) -> Optional[Dict[str, float]]:
        """Get current joint positions from simulated robot."""
        if not self._simulated_robot:
            return None

        state = self._simulated_robot.get_state()
        if state and 'joint_positions' in state and self.current_kinematic_model:
            positions = state['joint_positions']
            joint_names = self.current_kinematic_model.get_joint_info()['names']
            return dict(zip(joint_names, positions))

        return None

    def stop_robot(self):
        """Emergency stop."""
        if self._real_robot:
            self._real_robot.stop()

    def cleanup(self):
        """Clean up resources on shutdown."""
        if self.is_connected:
            self.disconnect_robot()
        logger.info("RobotManager cleanup complete")