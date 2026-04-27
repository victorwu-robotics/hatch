"""
State Handler - Updates kinematic model and transform registry from robot state.

Subscribes to ROBOT_STATE events.
This is the ONLY place that modifies the kinematic model and transform registry
in response to state changes.

Principle #7: Movements as Models. State updates flow through here.
Principle #5: Space = TransformRegistry. Single owner of registry updates.
"""

import numpy as np
import logging
from typing import Set

from core.world_state.state_channel import StateChannel
from core.world_state.event_types import EventType
from core.world_state.transform_registry import TransformRegistry, FrameStatus
from core.kinematics.kinematic_model import KinematicModel

logger = logging.getLogger(__name__)


class StateHandler:
    """
    Subscribes to ROBOT_STATE events.
    Updates KinematicModel and TransformRegistry.

    This is the SINGLE owner of model and registry updates during operation.
    Initial registration is done by RobotManager on load.
    Runtime updates are done here on each ROBOT_STATE event.
    """

    def __init__(self,
                 state_channel: StateChannel,
                 kinematic_model: KinematicModel,
                 transform_registry: TransformRegistry,
                 asset_id: str):
        """
        Initialize state handler.

        Args:
            state_channel: Application event bus.
            kinematic_model: Model to update with joint positions.
            transform_registry: Registry to update with new transforms.
            asset_id: Unique ID for frame namespacing.
        """
        self._channel = state_channel
        self._model = kinematic_model
        self._registry = transform_registry
        self._asset_id = asset_id

        # Build set of links in the main kinematic chain
        # (excludes camera frames, tool attachments not in the chain)
        self._arm_chain_links = self._build_arm_chain_links()

        # Subscribe to robot state events
        self._channel.subscribe(EventType.ROBOT_STATE, self._on_robot_state)

        logger.info(f"StateHandler initialized for asset: {asset_id} "
                   f"({len(self._arm_chain_links)} links in arm chain)")

    # =================================================================
    # Arm Chain Detection
    # =================================================================

    def _build_arm_chain_links(self) -> Set[str]:
        """
        Build the set of link names in the main kinematic chain.

        Excludes fixed-joint intermediaries, camera frames,
        laser scanners, and tool attachments that are not part
        of the moving chain.
        """
        arm_links = set()

        try:
            arm_joints = self._model.get_arm_chain()

            for joint_name in arm_joints:
                joint = self._model.joints.get(joint_name)
                if joint:
                    arm_links.add(joint['parent'])
                    arm_links.add(joint['child'])

            true_root = self._model.get_true_root()
            arm_links.add(true_root)

        except Exception as e:
            logger.warning(f"Could not build arm chain: {e}. "
                          "Using all links as fallback.")
            arm_links = set(self._model.link_transforms.keys())

        return arm_links

    # =================================================================
    # Event Handling
    # =================================================================

    def _on_robot_state(self, event):
        """
        Handle ROBOT_STATE event.

        This is the ONLY place that updates the kinematic model and
        transform registry during operation.

        Flow:
            1. Update kinematic model with new joint positions
            2. Model recomputes forward kinematics internally
            3. Update transform registry with new link transforms
            4. Registry notifies callbacks (KinematicDisplay, etc.)
        """
        joint_positions = event.data.get('joint_positions')
        if joint_positions is None:
            return

        # 1. Update kinematic model (recomputes FK internally)
        self._model.update_state(joint_positions)

        # 2. Update transform registry from model's new transforms
        self._update_transform_registry()

    def _update_transform_registry(self):
        """
        Update all robot frames in TransformRegistry.

        Only updates links in the main kinematic chain.
        Computes parent-relative transforms from world transforms.
        """
        if not hasattr(self._model, 'link_transforms'):
            return

        true_root = self._model.get_true_root()

        for link_name, T_world in self._model.link_transforms.items():
            # Skip links not in the main kinematic chain
            if link_name not in self._arm_chain_links:
                continue

            frame_name = f"{self._asset_id}_{link_name}"

            # Compute parent-relative transform
            if link_name == true_root:
                parent_frame = "world"
                T_rel = T_world
            else:
                parent_link = self._model.link_parents.get(link_name)
                if parent_link:
                    parent_frame = f"{self._asset_id}_{parent_link}"
                    T_parent_world = self._model.link_transforms.get(
                        parent_link, np.eye(4)
                    )
                    T_rel = np.linalg.inv(T_parent_world) @ T_world
                else:
                    parent_frame = "world"
                    T_rel = T_world

            # Update existing frame or register if new
            try:
                self._registry.update_frame(frame_name, T_rel)
            except ValueError:
                # Frame doesn't exist yet — register it
                try:
                    self._registry.register_frame(
                        frame_name,
                        T_rel,
                        status=FrameStatus.DYNAMIC,
                        parent=parent_frame,
                        description=f"Link: {link_name}"
                    )
                except ValueError:
                    # Parent frame not found — skip
                    continue

        # Update TCP frame
        tcp_frame = f"{self._asset_id}_tcp"
        mount_link = "wrist_3_link"

        if mount_link in self._arm_chain_links:
            parent_frame = f"{self._asset_id}_{mount_link}"
            T_tcp_parent = np.eye(4)  # TCP at mount point by default

            try:
                self._registry.update_frame(tcp_frame, T_tcp_parent)
            except ValueError:
                try:
                    self._registry.register_frame(
                        tcp_frame,
                        T_tcp_parent,
                        status=FrameStatus.DYNAMIC,
                        parent=parent_frame,
                        description="Tool Center Point"
                    )
                except ValueError:
                    pass
