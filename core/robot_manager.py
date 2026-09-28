"""
Robot Manager - Pure Python service for robot lifecycle management.

Owns the RealRobot and SimulatedRobot instances.
Handles robot loading, connection management, and mode switching.
Publishes all state changes via StateChannel events.
Does NOT emit Qt signals. Does NOT inherit from QObject.

Principle: Everything in URDF.
Principle: Pure Python (no Qt in core services).
Principle: UI Separate from Services.
Principle: One Robot Per Session.
"""

from typing import Optional, List, Dict, Any
from pathlib import Path
import numpy as np
import logging
import tempfile
import os
import shutil

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

        self.mesh_loader = None  # Set by MainWindow if available

        # Package resolver (created fresh, reads HATCH_PACKAGE_PATH env var)
        from utils.package_resolver import PackageResolver
        self.package_resolver = PackageResolver()

        # Current robot state
        self.current_asset_id: Optional[str] = None
        self.current_kinematic_model = None
        self.current_mode = Mode.SIMULATE_LOCAL
        self.is_connected = False
        self.robot_ip: Optional[str] = None
        self.current_urdf_path: Optional[Path] = None

        # Asset base frame mapping: asset_id -> base_frame_name
        self._asset_bases: Dict[str, str] = {}

        # Registry of loaded robots (one at a time per Principle #10)
        self._loaded_robots: Dict[str, Any] = {}
        self._displays: Dict[str, Any] = {}

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
        """Load a robot from URDF or xacro file."""
        from core.kinematics.kinematic_model import KinematicModel
        from displays.kinematic_display import KinematicDisplay
        from core.urdf_preprocessor import URDFPreprocessor

        urdf_path = Path(urdf_path).expanduser().resolve()

        try:
            if asset_id is None:
                asset_id = urdf_path.stem

            # Handle duplicate IDs
            if asset_id in self._loaded_robots:
                original = asset_id
                asset_id = f"{asset_id}_{len(self._loaded_robots)}"
                logger.info(f"Asset ID '{original}' exists, using '{asset_id}'")

            # Step 1: Preprocess URDF/xacro to plain URDF XML
            logger.info(f"Loading: {urdf_path}")
            preprocessor = URDFPreprocessor(self.package_resolver)
            urdf_xml = preprocessor.process(str(urdf_path))

            # Step 2: Write preprocessed URDF to temp file
            with tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.urdf',
                encoding='utf-8',
                delete=False
            ) as tmp_file:
                tmp_file.write(urdf_xml)
                temp_urdf_path = tmp_file.name

            # Step 3: Parse URDF with KinematicModel
            model = KinematicModel(
                urdf_path=temp_urdf_path,
                package_resolver=self.package_resolver,
                transform_registry=self.transform_registry,
                asset_id=asset_id
            )
            model.load()
            logger.info(f"Kinematic model loaded: {asset_id}")

            # Clean up temp file
            Path(temp_urdf_path).unlink()

            # Step 4: Attach IK solver
            self._attach_ik_solver(model)

            # Step 5: Inject model into simulated robot
            if self._simulated_robot is not None:
                self._simulated_robot.set_kinematic_model(model)

            # Step 6: Store the model
            self.current_kinematic_model = model
            self.current_asset_id = asset_id
            self.current_urdf_path = urdf_path
            self._loaded_robots[asset_id] = model

            # Step 7: Register transforms
            self._register_initial_transforms(asset_id, model)

            # Step 8: Create and attach visual display
            display = KinematicDisplay(model, 
                                       self.transform_registry,
                                       mesh_loader=self.mesh_loader,
                                       asset_id=asset_id)
            display.attach(self.engine.get_renderer())
            self.engine.register_display(display)
            self._displays[asset_id] = display

            # Step 9: Publish event
            self.state_channel.publish(
                EventType.ROBOT_LOADED,
                {
                    "asset_id": asset_id,
                    "urdf_path": str(urdf_path),
                    "model": model,
                }
            )

            logger.info(f"Robot loaded successfully: {asset_id}")
            return asset_id

        except Exception as e:
            logger.error(f"Failed to load robot from {urdf_path}: {e}", exc_info=True)
            return None

    def _attach_ik_solver(self, model):
        """Attach IK solver to the kinematic model."""
        try:
            from core.kinematics.ik_solver import IKSolver
            ik_solver = IKSolver(model)
            model.set_ik_solver(ik_solver)
            if ik_solver is not None:
                logger.debug("[ROB MANAGER] ik_solver is not None.")
                logger.info("IK solver attached")
            else:
                logger.debug("[ROB MANAGER] ik_solver is None.")
        except ImportError:
            logger.info("IK solver not available (missing dependencies)")
        except Exception as e:
            logger.warning(f"Could not attach IK solver: {e}")

    def _register_initial_transforms(self, asset_id: str, model):
        """
        Register all robot frames in TransformRegistry on initial load.
        Uses multi-pass registration to handle arbitrary parent-child ordering.
        """
        from core.world_state.transform_registry import FrameStatus

        if not hasattr(model, 'link_transforms'):
            return

        true_root = model.get_true_root()
        logger.info(f"Registering transforms for {asset_id} "
                    f"(true root: {true_root})")

        # Collect all frames that need to be registered
        frames_to_register = {}

        for link_name in model.link_transforms.keys():
            frame_name = f"{asset_id}_{link_name}"

            if link_name == true_root:
                parent_frame = "world"
            elif link_name in model.link_parents:
                parent_link = model.link_parents[link_name]
                parent_frame = f"{asset_id}_{parent_link}" if parent_link else "world"
            else:
                parent_frame = "world"

            frames_to_register[frame_name] = {
                'link_name': link_name,
                'parent_frame': parent_frame,
                'is_root': (link_name == true_root)
            }

        # Multi-pass registration
        registered_frames = set()
        remaining_frames = set(frames_to_register.keys())
        max_passes = len(remaining_frames) + 1

        for pass_num in range(max_passes):
            if not remaining_frames:
                break

            frames_registered_this_pass = []

            for frame_name in list(remaining_frames):
                info = frames_to_register[frame_name]
                parent_frame = info['parent_frame']

                if parent_frame == "world" or parent_frame in registered_frames:
                    link_name = info['link_name']
                    T_world = model.link_transforms[link_name]

                    if parent_frame == "world":
                        T_rel = T_world
                    else:
                        parent_link = parent_frame.replace(f"{asset_id}_", "")
                        if parent_link in model.link_transforms:
                            T_parent_world = model.link_transforms[parent_link]
                            T_rel = np.linalg.inv(T_parent_world) @ T_world
                        else:
                            logger.warning(f"Parent transform not found for {parent_link}")
                            T_rel = T_world

                    try:
                        self.transform_registry.register_frame(
                            frame_name,
                            T_rel,
                            parent=parent_frame,
                            status=FrameStatus.DYNAMIC,
                            description=f"Link: {link_name}"
                        )
                        registered_frames.add(frame_name)
                        frames_registered_this_pass.append(frame_name)
                    except ValueError as e:
                        logger.error(f"Failed to register {frame_name}: {e}")

            for frame_name in frames_registered_this_pass:
                remaining_frames.discard(frame_name)

            if not frames_registered_this_pass and remaining_frames:
                logger.warning(f"Pass {pass_num}: Cannot register {len(remaining_frames)} frames. "
                             f"Attaching remaining to world: {remaining_frames}")

                for frame_name in remaining_frames:
                    info = frames_to_register[frame_name]
                    link_name = info['link_name']
                    T_world = model.link_transforms.get(link_name, np.eye(4))

                    try:
                        self.transform_registry.register_frame(
                            frame_name,
                            T_world,
                            parent="world",
                            status=FrameStatus.DYNAMIC,
                            description=f"Link (fallback): {link_name}"
                        )
                        registered_frames.add(frame_name)
                    except ValueError as e:
                        logger.error(f"Failed fallback registration for {frame_name}: {e}")

                remaining_frames.clear()
                break

        # Register TCP frame
        tcp_frame = f"{asset_id}_tcp"
        mount_link = model.tool_mount_link or "wrist_3_link"
        parent_frame = f"{asset_id}_{mount_link}"

        try:
            self.transform_registry.register_frame(
                tcp_frame,
                np.eye(4),
                parent=parent_frame,
                status=FrameStatus.DYNAMIC,
                description="Tool Center Point"
            )
        except ValueError as e:
            logger.warning(f"Could not register TCP frame: {e}")

        logger.info(f"Registered {len(registered_frames)} frames for {asset_id}")

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
        """Set operating mode."""
        valid_modes = ["simulate", "real"]
        if mode not in valid_modes:
            logger.warning(f"Invalid mode: {mode}")
            return

        if mode == "real" and not self.is_connected:
            logger.warning("Cannot switch to real mode: Not connected")
            return

        self.state_channel.publish(
            EventType.MODE_SWITCH_REQUEST,
            data={'mode': mode},
            source="robot_manager"
        )

    # =================================================================
    # Asset Base Frame Queries
    # =================================================================

    def get_asset_base_frame(self, asset_id: str) -> str:
        """Get the base frame for Cartesian control."""
        return self._asset_bases.get(asset_id, "world")

    def get_asset_base_transform(self, asset_id: str) -> np.ndarray:
        """Get the transform from world to the asset's Cartesian base."""
        base_frame = self.get_asset_base_frame(asset_id)
        return self.transform_registry.get_transform(base_frame, "world")

    # =================================================================
    # Public Query Methods
    # =================================================================

    def get_current_joint_positions(self) -> Optional[List[float]]:
        """Get current joint positions from the active robot."""
        if self.current_mode == Mode.REAL and self._real_robot and self._real_robot.is_connected():
            state = self._real_robot.get_state()
            return state.get('joint_positions')
        elif self.current_kinematic_model:
            return self.current_kinematic_model.get_current_joint_positions()
        return None

    def get_model(self):
        """Get the current kinematic model."""
        return self.current_kinematic_model

    @property
    def model(self):
        """Property accessor for kinematic model."""
        return self.current_kinematic_model

    def stop_robot(self):
        """Emergency stop."""
        if self._real_robot:
            self._real_robot.stop()

    def cleanup(self):
        """Clean up resources on shutdown."""
        if self.is_connected:
            self.disconnect_robot()
        logger.info("RobotManager cleanup complete")