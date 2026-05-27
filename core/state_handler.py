"""
State Handler - Updates kinematic model and transform registry from robot state.
"""

import numpy as np
import logging
from typing import Optional, Set

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.world_state.transform_registry import TransformRegistry, FrameStatus
from core.kinematics.kinematic_model import KinematicModel

logger = logging.getLogger(__name__)

class StateHandler:
    """
    Subscribes to ROBOT_STATE events.
    Updates KinematicModel and TransformRegistry.
    This is the ONLY place that modifies these systems.
    """
    
    def __init__(self,
                 state_channel: StateChannel,
                 kinematic_model: KinematicModel,
                 transform_registry: TransformRegistry,
                 asset_id: str):
        """
        Initialize state handler.
        
        Args:
            state_channel: Application event bus
            kinematic_model: Kinematic data model to update
            transform_registry: Transform registry to update
            asset_id: Unique ID for this robot (for frame naming)
        """
        self._channel = state_channel
        self._model = kinematic_model
        self._registry = transform_registry
        self._asset_id = asset_id
        
        # Build set of links that are in the main kinematic chain
        self._arm_chain_links = self._build_arm_chain_links()
        
        # Subscribe to robot state events
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)
        
        print(f"[StateHandler] Initialized for asset: {asset_id}")
        print(f"[StateHandler] Arm chain has {len(self._arm_chain_links)} links")
    
    def _build_arm_chain_links(self) -> Set[str]:
        """
        Build the set of link names in the main kinematic chain.

        Includes all moving links and any fixed links attached to them.
        Fixed links after the last moving joint (tools, sensors, flanges)
        move rigidly with the wrist and must be updated.

        Handles multiple fixed children (branching) at any link.
        """
        arm_links = set()

        try:
            arm_joints = self._model.get_arm_chain()

            # Add parents and children of all moving joints
            for joint_name in arm_joints:
                joint = self._model.joints.get(joint_name)
                if joint:
                    arm_links.add(joint['parent'])
                    arm_links.add(joint['child'])

            # Add the true root
            true_root = self._model.get_true_root()
            arm_links.add(true_root)

            # Add ALL fixed descendants of any arm link (handles branching)
            to_process = list(arm_links)
            while to_process:
                link_name = to_process.pop()
                if link_name in self._model.link_children:
                    for child in self._model.link_children[link_name]:
                        if child not in arm_links:
                            # Check if connected by a fixed joint
                            for j in self._model.joints.values():
                                if (j['parent'] == link_name and
                                    j['child'] == child and
                                    j['type'] == 'fixed'):
                                    arm_links.add(child)
                                    to_process.append(child)
                                    break

        except Exception as e:
            logger.warning(f"Could not build arm chain: {e}. Using all links.")
            arm_links = set(self._model.link_transforms.keys())

        return arm_links
    
    def _on_robot_state(self, event):
        """
        Handle ROBOT_STATE event.
        
        Event data format:
            {'joint_positions': List[float], 'tcp_pose': ..., 'timestamp': float}
        """
        joint_positions = event.data.get('joint_positions')
        source = event.data.get('source', 'unknown')
        # print(f"[StateHandler] Received ROBOT_STATE from {source}: {joint_positions}")

        # print(f"[SH] joint_positions from event: {joint_positions}")
        # print(f"[SH] Type: {type(joint_positions)}, length: {len(joint_positions) if joint_positions else 0}")
        
        if joint_positions is None:
            return
        
        # 1. Update kinematic model (recomputes all link transforms)
        # print(f"[SH] Updating model with {joint_positions[:3]}...")
        self._model.update_state(joint_positions)
        # print(f"[StateHandler] Model updated. TCP: {self._model.get_tcp_pose()[:3,3]}")
        
        # 2. Update transform registry (for display and queries)
        self._update_transform_registry()
        # print(f"[StateHandler] Registry updated")
    
    def _update_transform_registry(self):
        """
        Update TransformRegistry with current link transforms from the model.

        Computes parent-relative transforms for each link in the arm chain
        and updates the registry. Links are updated in depth order (parents
        before children) so that when a child's transform change triggers
        callbacks, the parent's world transform is already up to date in
        the registry cache.

        Also updates the TCP frame at the tool mount link.

        This is called by _on_robot_state every time a ROBOT_STATE event
        arrives. It is the single owner of runtime registry updates.
        """

        if not hasattr(self._model, 'link_transforms'):
            return

        true_root = self._model.get_true_root()

        # Collect frames with their parent info
        updates = []
        for link_name, T_world in self._model.link_transforms.items():
            if link_name not in self._arm_chain_links:
                continue

            if link_name == true_root:
                parent_frame = "world"
                T_rel = T_world
                depth = 0
            else:
                parent_link = self._model.link_parents.get(link_name)
                if parent_link:
                    parent_frame = f"{self._asset_id}_{parent_link}"
                    T_parent_world = self._model.link_transforms.get(parent_link, np.eye(4))
                    T_rel = np.linalg.inv(T_parent_world) @ T_world
                else:
                    parent_frame = "world"
                    T_rel = T_world

            # Compute depth by walking up to root
            depth = 0
            current = link_name
            while current != true_root and depth < 100:
                current = self._model.link_parents.get(current)
                if current is None:
                    break
                depth += 1

            updates.append((depth, link_name, T_rel, parent_frame))

        # Sort by depth: parents before children
        updates.sort(key=lambda x: x[0])

        for depth, link_name, T_rel, parent_frame in updates:
            frame_name = f"{self._asset_id}_{link_name}"

            try:
                self._registry.update_frame(frame_name, T_rel)
            except ValueError:
                try:
                    self._registry.register_frame(
                        frame_name, T_rel,
                        status=FrameStatus.DYNAMIC,
                        parent=parent_frame,
                        description=f"Link: {link_name}"
                    )
                except ValueError:
                    continue

        # Update TCP frame
        mount_link = self._model.tool_mount_link or "wrist_3_link"
        if mount_link in self._arm_chain_links:
            tcp_frame = f"{self._asset_id}_tcp"
            parent_frame = f"{self._asset_id}_{mount_link}"
            try:
                self._registry.update_frame(tcp_frame, np.eye(4))
            except ValueError:
                try:
                    self._registry.register_frame(
                        tcp_frame, np.eye(4),
                        status=FrameStatus.DYNAMIC,
                        parent=parent_frame,
                        description="Tool Center Point"
                    )
                except ValueError:
                    pass