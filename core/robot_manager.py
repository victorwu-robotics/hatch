"""
Robot Arm Manager - Facade for robot control.
Now delegates to CommandHandler and StateHandler.
"""

from typing import Optional, Dict, Any

from PyQt5.QtCore import QObject, pyqtSignal

from core.world_state.state_channel import StateChannel
from core.world_state.transform_registry import TransformRegistry
from core.world_state.event_types import EventType
from core.mode import Mode
from core.command_handler import CommandHandler
from core.state_handler import StateHandler
from drivers.robot_arm.ur_rtde_bridge import URRobotDriver
from drivers.robot_arm.real_robot import RealRobot
from drivers.robot_arm.simulated_robot import SimulatedRobot


class RobotManager(QObject):
    """
    Manages robot arm connection and mode switching.
    Now delegates to CommandHandler and StateHandler internally.
    
    Signals are kept for backward compatibility with existing UI.
    """
    
    # Signals for UI updates (kept for compatibility)
    state_received = pyqtSignal(dict)           # New robot state received
    connection_changed = pyqtSignal(bool, str)  # Connected status, message
    error_occurred = pyqtSignal(str)            # Error message
    mode_changed = pyqtSignal(str)              # "simulate" or "real"
    
    def __init__(self, 
                 transform_registry: TransformRegistry,
                 state_channel: StateChannel,
                 engine,  # VisualizerEngine (for status updates)
                 parent: Optional[QObject] = None):
        super().__init__(parent)
        
        self.transform_registry = transform_registry
        self.state_channel = state_channel
        self.engine = engine
        
        # Will be set when asset is loaded
        self.current_asset_id: Optional[str] = None
        self.current_kinematic_model = None
        
        # Internal components (created when asset is set)
        self._real_robot: Optional[RealRobot] = None
        self._simulated_robot: Optional[SimulatedRobot] = None
        self._state_handler: Optional[StateHandler] = None
        
        # Connection status
        self.is_connected = False
        self.robot_ip = None
        
        # Subscribe to state channel events (for UI signals)
        self._setup_subscriptions()
    
        # For tracking loaded robots
        self._loaded_robots = {}
        self.current_asset_id = None
        self.current_kinematic_model = None

        self.current_mode = Mode.SIMULATE_LOCAL
        self.is_connected = False

    def _setup_subscriptions(self):
        """Subscribe to state channel events to emit Qt signals."""
        self.state_channel.subscribe(EventType.ROBOT_STATE, self._on_state_for_ui)
        self.state_channel.subscribe(EventType.ROBOT_CONNECTED, self._on_connected_for_ui)
        self.state_channel.subscribe(EventType.ROBOT_DISCONNECTED, self._on_disconnected_for_ui)
        self.state_channel.subscribe(EventType.ROBOT_ERROR, self._on_error_for_ui)
        self.state_channel.subscribe(EventType.ROBOT_MODE_CHANGED, self._on_mode_changed_for_ui)
        self.state_channel.subscribe(EventType.ROBOT_LOAD_REQUEST, self._on_load_request)

        self.state_channel.subscribe(EventType.JOINT_COMMAND, self._on_joint_command)

    def connect_robot(self, ip: str, **kwargs) -> bool:
        """Connect to real robot."""
        if not self._real_robot:
            return False
        
        success = self._real_robot.connect(ip, **kwargs)
        if success:
            self.is_connected = True
            self.robot_ip = ip
        return success
    
    def disconnect_robot(self):
        """Disconnect from real robot."""
        if self._real_robot:
            self._real_robot.disconnect()
        self.is_connected = False
        self.robot_ip = None
        
    def set_mode(self, mode: str):
        """
        Switch between modes.
        
        Args:
            mode: "simulate_local", "simulate_real_ik", or "real"
        """
        if mode not in ["simulate_local", "simulate_real_ik", "real"]:
            return
        
        # For real mode, ensure we're connected
        if mode == "real" and not self.is_connected:
            self.error_occurred.emit(
                "Cannot switch to real mode: Not connected to robot."
            )
            return
        
        # Publish mode switch event (CommandHandler will handle it)
        self.state_channel.publish(
            EventType.MODE_SWITCH,
            data={'mode': mode},
            source="robot_manager"
        )
    
    # ===== Signal forwarders (for backward compatibility) =====
    
    def _on_state_for_ui(self, event):
        """Forward ROBOT_STATE to Qt signal."""
        self.state_received.emit(event.data)
    
    def _on_connected_for_ui(self, event):
        """Forward ROBOT_CONNECTED to Qt signal."""
        self.connection_changed.emit(True, event.data.get('message', 'Connected'))
    
    def _on_disconnected_for_ui(self, event):
        """Forward ROBOT_DISCONNECTED to Qt signal."""
        self.connection_changed.emit(False, event.data.get('message', 'Disconnected'))
    
    def _on_error_for_ui(self, event):
        """Forward ROBOT_ERROR to Qt signal."""
        self.error_occurred.emit(event.data.get('error', 'Unknown error'))
    
    def _on_mode_changed_for_ui(self, event):
        """Forward ROBOT_MODE_CHANGED to Qt signal."""
        mode = event.data.get('mode', 'simulate_local')
        # Convert to simple "simulate" or "real" for backward compatibility
        simple_mode = "real" if mode == "real" else "simulate"
        self.mode_changed.emit(simple_mode)

    def _register_robot_transforms(self, asset_id: str, model):
        """
        Register robot transforms with correct parent-relative transforms.
        TCP is set directly to wrist_3_link (no extra ROS-Industrial frames).
        
        This method is adapted from the original AssetManager.
        """
        from core.world_state.transform_registry import FrameStatus
        import numpy as np
        
        if not hasattr(model, 'link_transforms'):
            return
        
        # Get the true root from the model
        true_root = model.get_true_root()
        print(f"\n--- Registering transforms for {asset_id} ---")
        print(f"  True kinematic base: {true_root}")
        
        # First, collect all frames with their parent relationships
        frames_info = []
        
        for link_name in model.link_transforms.keys():
            frame_name = f"{asset_id}_{link_name}"
            
            # Determine parent frame
            if link_name == true_root:
                parent = "world"
            elif link_name in model.link_parents:
                parent_link = model.link_parents.get(link_name)
                if parent_link:
                    parent = f"{asset_id}_{parent_link}"
                else:
                    parent = "world"
            else:
                parent = "world"
            
            frames_info.append({
                'name': frame_name,
                'original_name': link_name,
                'parent': parent,
                'is_true_root': (link_name == true_root)
            })
        
        # Sort frames by depth (parents before children)
        def get_depth(frame_name, frames_dict, visited=None):
            if visited is None:
                visited = set()
            
            if frame_name in visited:
                return 0
            
            visited.add(frame_name)
            
            frame = frames_dict.get(frame_name)
            if not frame or frame['parent'] == 'world':
                return 0
            
            return get_depth(frame['parent'], frames_dict, visited) + 1
        
        # Create lookup dict
        frames_dict = {f['name']: f for f in frames_info}
        
        # Calculate depths
        depth_cache = {}
        for frame in frames_info:
            depth_cache[frame['name']] = get_depth(frame['name'], frames_dict)
        
        # Sort by depth (parents first)
        frames_info.sort(key=lambda f: depth_cache[f['name']])
        
        # Register frames in order, computing parent-relative transforms
        registered_count = 0
        
        for frame in frames_info:
            frame_name = frame['name']
            parent = frame['parent']
            link_name = frame['original_name']
            
            # Get world transform from model
            T_world = model.link_transforms[link_name]
            
            # Compute transform RELATIVE to parent
            if parent == "world":
                # Parent is world, so transform is world → frame
                T_rel = T_world
            else:
                # Parent is another frame, compute relative transform
                parent_link = parent.replace(f"{asset_id}_", "")
                if parent_link in model.link_transforms:
                    T_parent_world = model.link_transforms[parent_link]
                    # T_rel = inv(T_parent_world) @ T_world
                    T_rel = np.linalg.inv(T_parent_world) @ T_world
                else:
                    print(f"  Warning: Parent {parent} not found in model transforms")
                    T_rel = T_world
            
            try:
                if frame_name in self.transform_registry.list_frames():
                    self.transform_registry.update(frame_name, T_rel)
                    print(f"  Updated: {frame_name} (parent: {parent})")
                else:
                    self.transform_registry.set(
                        frame_name,
                        T_rel,
                        status=FrameStatus.DYNAMIC,
                        parent=parent,
                        description=f"Link: {link_name}"
                    )
                    print(f"  Created: {frame_name} (parent: {parent})")
                registered_count += 1
            except ValueError as e:
                print(f"  Warning: Failed to register {frame_name}: {e}")
                # Fallback: register with world parent
                if parent != 'world':
                    print(f"    Retrying with world parent...")
                    try:
                        self.transform_registry.set(
                            frame_name,
                            T_world,
                            status=FrameStatus.DYNAMIC,
                            parent='world',
                            description=f"Link: {link_name} (fallback)"
                        )
                        registered_count += 1
                        print(f"    Created with world parent")
                    except Exception as e2:
                        print(f"    Still failed: {e2}")
        
        # ===== TCP REGISTRATION =====
        # TCP is directly at wrist_3_link (standard for UR robots)
        tcp_frame = f"{asset_id}_tcp"
        mount_link = "wrist_3_link"  # Direct mount point
        parent_frame = f"{asset_id}_{mount_link}"
        
        # TCP is exactly at the mount link, so transform is identity
        T_tcp_parent = np.eye(4)
        
        try:
            if tcp_frame in self.transform_registry.list_frames():
                self.transform_registry.update(tcp_frame, T_tcp_parent)
                print(f"  Updated TCP frame: {tcp_frame} (parent: {parent_frame})")
            else:
                self.transform_registry.set(
                    tcp_frame,
                    T_tcp_parent,
                    status=FrameStatus.DYNAMIC,
                    parent=parent_frame,
                    description="Tool Center Point (wrist_3_link)"
                )
                print(f"  Created TCP frame: {tcp_frame} (parent: {parent_frame})")
        except ValueError as e:
            print(f"  Warning: Could not create TCP frame: {e}")
        
        # Set the asset base in TransformRegistry
        true_root_frame_name = f"{asset_id}_{true_root}"
        self.transform_registry.set_asset_base(asset_id, true_root_frame_name)
        print(f"  Set asset base: {asset_id} -> {true_root_frame_name}")
        
        # Debug: Verify transforms
        try:
            tcp_frame = f"{asset_id}_tcp"
            T = self.transform_registry.get_transform(tcp_frame, true_root_frame_name)
            print(f"\n  Verification: TCP in true_root coordinates:")
            print(f"    Position: ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})")
            
            T = self.transform_registry.get_transform(true_root_frame_name, "world")
            print(f"  World → true_root position: ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})")
        except Exception as e:
            print(f"  Verification failed: {e}")
        
        print(f"\nFinished registering {registered_count} frames for {asset_id}\n")

    def _on_load_request(self, event):
        """Handle ROBOT_LOAD_REQUEST event."""
        urdf_path = event.data.get('urdf_path')
        robot_id = event.data.get('robot_id')
        
        if not urdf_path:
            print("[RobotManager] Load request missing urdf_path")
            return

        print(f"[RobotManager] Received load request for: {robot_id} from {urdf_path}")

        # Load the new robot
        self.load_robot(urdf_path, robot_id)

    def _on_joint_command(self, event):
        """Handle JOINT_COMMAND event."""
        positions = event.data.get('positions')
        if positions and self.current_kinematic_model:
            print(f"[RobotManager] Received JOINT_COMMAND: {positions}")
            # Update kinematic model
            self.current_kinematic_model.update_state(positions)
            # Also send to real robot if connected and in real mode

    # ===== Public methods for UI (kept for compatibility) =====

    def load_robot(self, urdf_path: str, asset_id: str = None) -> Optional[str]:
        """
        Load a robot from URDF file.
        
        Args:
            urdf_path: Path to URDF file
            asset_id: Optional asset ID (auto-generated from filename if not provided)
        
        Returns:
            asset_id (str) if successful, None otherwise
        """
        from pathlib import Path
        from core.kinematics.kinematic_model import KinematicModel
        from displays.kinematic_display import KinematicDisplay
        
        try:
            # Generate asset ID from filename if not provided
            if asset_id is None:
                asset_id = Path(urdf_path).stem
            
            # Handle duplicate asset IDs
            if asset_id in self._loaded_robots:
                original_id = asset_id
                asset_id = f"{asset_id}_{len(self._loaded_robots)}"
                print(f"Note: Asset ID '{original_id}' already exists. Using '{asset_id}'")
            
            # Package directories for mesh resolution
            package_dirs = [
                str(Path(urdf_path).parent),
                str(Path(urdf_path).parent / "meshes"),
                str(Path(urdf_path).parent / "visual"),
                str(Path(urdf_path).parent / "meshes/visual"),
                str(Path.home() / ".cache" / "robot_descriptions"),
            ]
            
            print(f"Loading robot from: {urdf_path}")
            
            # Create kinematic model
            model = KinematicModel(
                urdf_path=str(urdf_path),
                package_dirs=package_dirs,
                transform_registry=self.transform_registry,
                asset_id=asset_id,
                update_registry_on_state_change=False
            )
            model.load()
            print(f"✅ Kinematic model loaded: {asset_id}")
            
            # Attach IK solver
            try:
                from core.kinematics.ik_solver import IKSolver
                ik_solver = IKSolver(model)
                model.set_ik_solver(ik_solver)
                print(f"✅ IK solver attached")
            except ImportError as e:
                print(f"Note: IK solver not available: {e}")
            except Exception as e:
                print(f"Note: Could not attach IK solver: {e}")
            
            # Register transforms
            self._register_robot_transforms(asset_id, model)
            
            # Create visual display
            display = KinematicDisplay(
                model, 
                self.transform_registry, 
                asset_id=asset_id
            )
            display.attach(self.engine.get_renderer())
            self.engine.register_display(display)
            print(f"✅ Visual display created")
            
            # Store in asset registry
            self._loaded_robots[asset_id] = {
                'model': model,
                'display': display,
                'urdf_path': str(urdf_path)
            }
            
            # Set as current asset
            self.current_asset_id = asset_id
            self.current_kinematic_model = model
            
            # Publish robot loaded event
            print(f"[DEBUG] Publishing ROBOT_LOADED with robot_id={asset_id}, urdf_path={urdf_path}")
            self.state_channel.publish(
                EventType.ROBOT_LOADED,
                data={'asset_id': asset_id,
                      'urdf_path': str(urdf_path),
                      'kinematic_model': model},
                source="robot_manager",
                description=f"Robot {asset_id} loaded"
            )
            
            print(f"✅ Robot loaded successfully: {asset_id}")
            return asset_id
            
        except Exception as e:
            print(f"❌ Failed to load robot: {e}")
            import traceback
            traceback.print_exc()
            return None

    def get_current_joint_positions(self) -> Optional[Dict[str, float]]:
        """Get current joint positions as dict (for UI display)."""
        if not self._simulated_robot:
            return None
        
        state = self._simulated_robot.get_state()
        if state and 'joint_positions' in state:
            positions = state['joint_positions']
            joint_names = self.current_kinematic_model.get_joint_info()['names']
            return dict(zip(joint_names, positions))
        
        return None
    
    def stop_robot(self):
        """Emergency stop."""
        if self._real_robot:
            self._real_robot.stop()
    
    def cleanup(self):
        """Clean up resources."""
        if self.is_connected:
            self.disconnect_robot()