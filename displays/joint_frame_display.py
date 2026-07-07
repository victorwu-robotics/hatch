"""
Joint Frame Display - Visualizes coordinate frames for all links.

Shows RGB axis markers at each link frame. Supports configurable
axis length and thickness. Shows all frames by default, including
sensor optical frames and fixed links.
"""

import vtk
import numpy as np
import logging

logger = logging.getLogger(__name__)


class JointFrameDisplay:
    """Displays coordinate frames for all links in the model."""

    def __init__(self, kinematic_model, registry, asset_id, scale=0.1, thickness=0.008):
        self.kinematic_model = kinematic_model
        self.registry = registry
        self.asset_id = asset_id
        self.scale = scale
        self.thickness = thickness
        self.renderer = None
        
        self.joint_axes = {}
        self.physical_axes = {}
        self.frame_info_list = []
        
        # Build frame list from ALL links (not just arm chain)
        self._build_frame_list()
        
        self.registry.register_callback(self._on_transform_updated)
        logger.info(f"JointFrameDisplay created for {asset_id} "
                   f"({len(self.frame_info_list)} frames)")

    def _build_frame_list(self):
        """Build list of all frames: arm chain + sensor/optical frames."""
        # First, try to get the arm chain
        try:
            arm_chain = self.kinematic_model.get_arm_chain(
                self.kinematic_model.get_true_root()
            )
            for joint_name in arm_chain:
                joint = self.kinematic_model.joints[joint_name]
                self.frame_info_list.append(("joint", joint_name, joint['child']))
        except (ValueError, KeyError):
            pass  # No moving joints
        
        # Add remaining links that aren't in the arm chain
        arm_links = {child for _, _, child in self.frame_info_list}
        for link_name in self.kinematic_model.link_transforms:
            if link_name not in arm_links and link_name != "world":
                self.frame_info_list.append(("link", link_name, link_name))

    def get_joint_names(self):
        """Return list of display names for the panel."""
        return [f"{'J' if kind=='joint' else 'L'}: {name}" 
                for kind, name, _ in self.frame_info_list]

    def attach(self, renderer):
        """Create axis actors for all frames. Hidden initially."""
        self.renderer = renderer
        
        for kind, name, child_link in self.frame_info_list:
            frame_name = f"{self.asset_id}_{child_link}"
            
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(self.scale, self.scale, self.scale)
            axes.SetShaftTypeToCylinder()
            axes.SetCylinderRadius(self.thickness)
            # axes.SetConeRadius(self.thickness * 3)
            axes.SetConeRadius(0.0)
            axes.SetAxisLabels(False)
            axes.SetVisibility(False)
            
            renderer.AddActor(axes)
            
            self.joint_axes[name] = {
                'actor': axes,
                'frame_name': frame_name,
                'visible': False,
                'kind': kind,
            }
            
            try:
                T_world = self.registry.get_transform("world", frame_name)
                self._update_axes_position(axes, T_world)
            except ValueError:
                pass
        
        # Add TCP frame
        self._add_tcp_frame(renderer)
        
        logger.info(f"JointFrameDisplay attached with "
                   f"{len(self.joint_axes)} frames + TCP")

    def _add_tcp_frame(self, renderer):
        """Create a distinct TCP frame at the tool mount link."""
        tcp_link = self.kinematic_model.tool_mount_link
        if tcp_link is None:
            return
        
        frame_name = f"{self.asset_id}_{tcp_link}"
        
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(self.scale * 1.5, self.scale * 1.5, self.scale * 1.5)
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(self.thickness * 1.5)
        axes.SetConeRadius(self.thickness * 4)
        axes.SetAxisLabels(False)
        axes.SetVisibility(False)
        
        magenta = (1.0, 0.0, 1.0)
        axes.GetXAxisShaftProperty().SetColor(*magenta)
        axes.GetYAxisShaftProperty().SetColor(*magenta)
        axes.GetZAxisShaftProperty().SetColor(*magenta)
        axes.GetXAxisTipProperty().SetColor(*magenta)
        axes.GetYAxisTipProperty().SetColor(*magenta)
        axes.GetZAxisTipProperty().SetColor(*magenta)
        
        renderer.AddActor(axes)
        
        self.tcp_actor = axes
        self.tcp_frame_name = frame_name
        
        try:
            T = self.registry.get_transform("world", frame_name)
            self._update_axes_position(axes, T)
        except ValueError:
            pass

    def set_joint_visible(self, name, visible):
        """Show or hide a specific frame."""
        if name in self.joint_axes:
            self.joint_axes[name]['actor'].SetVisibility(visible)
            self.joint_axes[name]['visible'] = visible
            if self.renderer:
                self.renderer.GetRenderWindow().Render()

    def set_tcp_visible(self, visible):
        """Show or hide the TCP frame."""
        if hasattr(self, 'tcp_actor'):
            self.tcp_actor.SetVisibility(visible)
        if self.renderer:
            self.renderer.GetRenderWindow().Render()

    def set_scale(self, scale):
        """Change axis length for all frames."""
        self.scale = scale
        for info in self.joint_axes.values():
            info['actor'].SetTotalLength(scale, scale, scale)
        if hasattr(self, 'tcp_actor'):
            self.tcp_actor.SetTotalLength(scale * 1.5, scale * 1.5, scale * 1.5)
        if self.renderer:
            self.renderer.GetRenderWindow().Render()

    def set_thickness(self, thickness):
        """Change shaft thickness for all frames."""
        self.thickness = thickness
        for info in self.joint_axes.values():
            info['actor'].SetCylinderRadius(thickness)
            info['actor'].SetConeRadius(thickness * 3)
        if hasattr(self, 'tcp_actor'):
            self.tcp_actor.SetCylinderRadius(thickness * 1.5)
            self.tcp_actor.SetConeRadius(thickness * 4)
        if self.renderer:
            self.renderer.GetRenderWindow().Render()

    def get_frame_poses(self):
        """Return world transforms of all visible frames."""
        poses = {}
        for name, info in self.joint_axes.items():
            try:
                T = self.registry.get_transform("world", info['frame_name'])
                poses[name] = T
            except ValueError:
                pass
        return poses

    def get_tcp_pose(self):
        """Return world transform of the TCP frame."""
        if hasattr(self, 'tcp_frame_name'):
            try:
                return self.registry.get_transform("world", self.tcp_frame_name)
            except ValueError:
                pass
        return None

    def _on_transform_updated(self, frame_name, transform):
        """Update frame positions when transforms change."""
        for name, info in self.joint_axes.items():
            if info['frame_name'] == frame_name and info['visible']:
                try:
                    T_world = self.registry.get_transform("world", frame_name)
                    self._update_axes_position(info['actor'], T_world)
                except ValueError:
                    pass
        
        if hasattr(self, 'tcp_frame_name') and frame_name == self.tcp_frame_name:
            if hasattr(self, 'tcp_actor') and self.tcp_actor.GetVisibility():
                try:
                    T_world = self.registry.get_transform("world", self.tcp_frame_name)
                    self._update_axes_position(self.tcp_actor, T_world)
                except ValueError:
                    pass

    def _update_axes_position(self, axes, T_world):
        """Position an axes actor at the given world transform."""
        transform = vtk.vtkTransform()
        transform.SetMatrix(T_world.flatten())
        axes.SetUserTransform(transform)

    def detach(self):
        """Remove all actors."""
        if self.renderer:
            for info in self.joint_axes.values():
                self.renderer.RemoveActor(info['actor'])
            if hasattr(self, 'tcp_actor'):
                self.renderer.RemoveActor(self.tcp_actor)
        self.joint_axes.clear()
        self.registry.remove_callback(self._on_transform_updated)
        logger.info("JointFrameDisplay detached")