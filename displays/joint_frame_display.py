"""
Joint Frame Display - Visualizes coordinate frames at each joint.

Axes are hidden by default. Toggle visibility via JointFramePanel.
"""

import vtk
import numpy as np
import logging

logger = logging.getLogger(__name__)


class JointFrameDisplay:
    """Displays coordinate frames at joints. Hidden by default."""

    def __init__(self, kinematic_model, registry, asset_id, scale=0.1):
        self.kinematic_model = kinematic_model
        self.registry = registry
        self.asset_id = asset_id
        self.scale = scale
        self.renderer = None
        
        self.joint_axes = {}
        self.physical_axes = {}
        self.joint_info_list = []
        
        # Build joint list from arm chain
        arm_chain = self.kinematic_model.get_arm_chain(
            self.kinematic_model.get_true_root()
        )
        for joint_name in arm_chain:
            joint = self.kinematic_model.joints[joint_name]
            self.joint_info_list.append((joint_name, joint['child']))
        
        self.registry.register_callback(self._on_transform_updated)
        logger.info(f"JointFrameDisplay created for {asset_id}")

    def get_joint_names(self):
        """Return list of joint names for the panel."""
        return [name for name, _ in self.joint_info_list]

    def attach(self, renderer):
        """Create axis actors. All hidden initially."""
        self.renderer = renderer
        
        for joint_name, child_link in self.joint_info_list:
            frame_name = f"{self.asset_id}_{child_link}"
            
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(self.scale, self.scale, self.scale)
            axes.SetShaftTypeToCylinder()
            axes.SetCylinderRadius(0.008)
            axes.SetConeRadius(0.025)
            axes.SetAxisLabels(False)
            axes.SetVisibility(False)  # Hidden by default
            
            renderer.AddActor(axes)
            
            self.joint_axes[joint_name] = {
                'actor': axes,
                'frame_name': frame_name,
                'visible': False,
            }
            
            # Set initial position
            try:
                T_world = self.registry.get_transform("world", frame_name)
                self._update_axes_position(axes, T_world)
            except ValueError:
                pass

        # --- add TCP frame ---
        self._add_tcp_frame(renderer)

        logger.info(f"JointFrameDisplay attached with "
                   f"{len(self.joint_axes)} frames (hidden)")

    def _add_tcp_frame(self, renderer):
        """Create a distinct TCP frame at the tool mount link."""
        tcp_link = self.kinematic_model.tool_mount_link          # e.g. "elfin_link6"
        frame_name = f"{self.asset_id}_{tcp_link}"

        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(self.scale * 1.5, self.scale * 1.5, self.scale * 1.5)  # larger
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.012)
        axes.SetConeRadius(0.035)
        axes.SetAxisLabels(False)
        axes.SetVisibility(False)   # hidden by default

        # Magenta color for TCP
        magenta = (1.0, 0.0, 1.0)
        axes.GetXAxisShaftProperty().SetColor(*magenta)
        axes.GetYAxisShaftProperty().SetColor(*magenta)
        axes.GetZAxisShaftProperty().SetColor(*magenta)
        axes.GetXAxisTipProperty().SetColor(*magenta)
        axes.GetYAxisTipProperty().SetColor(*magenta)
        axes.GetZAxisTipProperty().SetColor(*magenta)

        renderer.AddActor(axes)

        # Label "TCP" at the end of the X axis
        T = self.registry.get_transform("world", frame_name)
        x_endpoint_local = np.array([self.scale * 1.8, 0, 0, 1])
        x_endpoint_world = T @ x_endpoint_local

        text = vtk.vtkVectorText()
        text.SetText("TCP")
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        text_actor = vtk.vtkFollower()
        text_actor.SetMapper(text_mapper)
        text_actor.SetScale(0.06, 0.06, 0.06)
        text_actor.SetPosition(x_endpoint_world[:3])
        text_actor.GetProperty().SetColor(*magenta)
        text_actor.SetCamera(self.renderer.GetActiveCamera())
        renderer.AddActor(text_actor)

        self.tcp_actor = axes
        self.tcp_label = text_actor
        self.tcp_frame_name = frame_name

    def set_tcp_visible(self, visible):
        """Show or hide the TCP frame."""
        if hasattr(self, 'tcp_actor'):
            self.tcp_actor.SetVisibility(visible)
            self.tcp_label.SetVisibility(visible)
        if self.renderer:
            self.renderer.GetRenderWindow().Render()

    def set_joint_visible(self, joint_name, visible):
        """Show or hide a specific joint's frame."""
        if joint_name in self.joint_axes:
            info = self.joint_axes[joint_name]
            info['actor'].SetVisibility(visible)
            info['visible'] = visible
            if self.renderer:
                self.renderer.GetRenderWindow().Render()

    def _on_transform_updated(self, frame_name, transform):
        """Update frame positions when transforms change."""
        for joint_name, info in self.joint_axes.items():
            if info['frame_name'] == frame_name and info['visible']:
                try:
                    T_world = self.registry.get_transform("world", frame_name)
                    self._update_axes_position(info['actor'], T_world)
                except ValueError:
                    pass

        # Update TCP frame if visible
        if hasattr(self, 'tcp_frame_name') and frame_name == self.tcp_frame_name:
            if hasattr(self, 'tcp_actor') and self.tcp_actor.GetVisibility():
                try:
                    T_world = self.registry.get_transform("world", self.tcp_frame_name)
                    self._update_axes_position(self.tcp_actor, T_world)
                    # Update label position
                    x_end = (T_world @ np.array([self.scale * 1.8, 0, 0, 1]))[:3]
                    self.tcp_label.SetPosition(x_end)
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
        self.joint_axes.clear()
        self.registry.remove_callback(self._on_transform_updated)
        logger.info("JointFrameDisplay detached")