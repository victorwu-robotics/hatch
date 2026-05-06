"""
Joint Frame Display - Visualizes coordinate frames at each joint.

Adds RGB axis markers at each joint position in the 3D scene.
Updates when transforms change, like KinematicDisplay.
"""

import vtk
import numpy as np
import logging

logger = logging.getLogger(__name__)


class JointFrameDisplay:
    """
    Displays coordinate frames at each joint in the kinematic chain.

    Each frame shows:
    - Red axis: X direction
    - Green axis: Y direction  
    - Blue axis: Z direction (joint rotation axis)
    
    Frames are positioned at joint origins and update with robot motion.
    """

    def __init__(self, kinematic_model, registry, asset_id, scale=0.1):
        self.kinematic_model = kinematic_model
        self.registry = registry
        self.asset_id = asset_id
        self.scale = scale
        self.renderer = None
        
        # One set of axes per joint
        self.joint_axes = {}     # joint_name -> vtkAxesActor
        self.joint_names = []
        
        # Get the arm chain
        arm_chain = self.kinematic_model.get_arm_chain(
            self.kinematic_model.get_true_root()
        )
        
        # For each joint, find its position frame (parent link)
        for joint_name in arm_chain:
            joint = self.kinematic_model.joints[joint_name]
            parent_link = joint['parent']
            self.joint_names.append((joint_name, parent_link))
        
        # Subscribe to transform updates
        self.registry.register_callback(self._on_transform_updated)
        
        logger.info(f"JointFrameDisplay created for {asset_id} "
                   f"with {len(self.joint_names)} joints")

    def attach(self, renderer):
        """Create and add axis actors for each joint."""
        self.renderer = renderer
        self.physical_axes = {}

        # Get visual geometries
        visual_geometries = self.kinematic_model.get_visual_geometries()

        # Add kinematic frames first (existing code)
        for i, (joint_name, parent_link) in enumerate(self.joint_names):
            joint = self.kinematic_model.joints[joint_name]
            child_link = joint['child']  # Child link moves with this joint
            frame_name = f"{self.asset_id}_{child_link}"  # CORRECT

            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(self.scale, self.scale, self.scale)
            axes.SetShaftTypeToCylinder()
            axes.SetCylinderRadius(0.01)
            axes.SetConeRadius(0.03)
            axes.SetAxisLabels(False)
            
            # Store the axes and its frame name
            self.joint_axes[joint_name] = {
                'actor': axes,
                'frame_name': frame_name,
            }
            
            renderer.AddActor(axes)
            
            # Set initial position
            try:
                T_world = self.registry.get_transform("world", frame_name)
                self._update_axes_position(axes, T_world)
                self._add_joint_label(joint_name, i, T_world)  # ← Add this
            except ValueError:
                pass  # Frame not registered yet
        
        # Now add physical frames for each link in the arm chain
        arm_chain = self.kinematic_model.get_arm_chain(
            self.kinematic_model.get_true_root()
        )

        # Add physical frame for the base
        true_root = self.kinematic_model.get_true_root()
        T_base = self.kinematic_model.link_transforms[true_root]
        self._add_physical_frame(true_root, "B", T_base, visual_geometries)
        
        # Add physical frames for each joint's child link
        for i, joint_name in enumerate(arm_chain):
            joint = self.kinematic_model.joints[joint_name]
            child_link = joint['child']
            T_child = self.kinematic_model.link_transforms[child_link]
            self._add_physical_frame(child_link, str(i+1), T_child, visual_geometries)

        logger.info(f"JointFrameDisplay attached with "
                   f"{len(self.joint_axes)} frames")

    def _on_transform_updated(self, frame_name, transform):
        """Called when any transform changes in the registry."""
        for joint_name, info in self.joint_axes.items():
            if info['frame_name'] == frame_name:
                try:
                    T_world = self.registry.get_transform("world", frame_name)
                    self._update_axes_position(info['actor'], T_world)
                except ValueError:
                    pass
        # Update physical frames
        for key, info in self.physical_axes.items():
            if info['frame_name'] == frame_name:
                try:
                    T_kinematic = self.registry.get_transform("world", frame_name)
                    
                    # Apply visual origin offset
                    link_name = info['link_name']
                    visual_geometries = self.kinematic_model.get_visual_geometries()
                    origin_transform = np.eye(4)
                    if link_name in visual_geometries and visual_geometries[link_name]:
                        origin_transform = visual_geometries[link_name][0].get('origin_transform', np.eye(4))
                    
                    T_physical = T_kinematic @ origin_transform
                    
                    # Update axis actor
                    transform_vtk = vtk.vtkTransform()
                    transform_vtk.SetMatrix(T_physical.flatten())
                    info['actor'].SetUserTransform(transform_vtk)
                    
                    # Update label position and text
                    pos = T_physical[:3, 3]
                    x_end = (T_physical @ np.array([self.scale * 0.8, 0, 0, 1]))[:3]
                    info['label'].SetPosition(x_end)
                    
                    # Rebuild label text
                    label_parts = key.split('_', 1)
                    label_id = label_parts[1] if len(label_parts) > 1 else "?"
                    label_text = f"P{label_id} ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                    
                    text_source = vtk.vtkVectorText()
                    text_source.SetText(label_text)
                    text_mapper = vtk.vtkPolyDataMapper()
                    text_mapper.SetInputConnection(text_source.GetOutputPort())
                    info['label'].SetMapper(text_mapper)
                    
                except ValueError:
                    pass

    def _update_axes_position(self, axes, T_world):
        transform = vtk.vtkTransform()
        transform.SetMatrix(T_world.flatten())
        axes.SetUserTransform(transform)
        
        pos = T_world[:3, 3]
        x_endpoint_local = np.array([self.scale * 1.2, 0, 0, 1])
        x_endpoint_world = T_world @ x_endpoint_local
        
        for joint_name, info in self.joint_axes.items():
            if info['actor'] == axes and 'label' in info:
                # Update position
                info['label'].SetPosition(x_endpoint_world[:3])
                
                # Rebuild text with current position
                joint_index = list(self.joint_axes.keys()).index(joint_name)
                label_text = f"J{joint_index+1} ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                
                text_source = vtk.vtkVectorText()
                text_source.SetText(label_text)
                
                text_mapper = vtk.vtkPolyDataMapper()
                text_mapper.SetInputConnection(text_source.GetOutputPort())
                
                info['label'].SetMapper(text_mapper)
                break

    def _add_physical_frame(self, link_name, label, T_kinematic, visual_geometries):
        """Add a frame at the physical link position (including visual offset)."""
        # Get the visual origin transform for this link
        origin_transform = np.eye(4)
        if link_name in visual_geometries and visual_geometries[link_name]:
            # Use the first geometry's origin transform
            origin_transform = visual_geometries[link_name][0].get('origin_transform', np.eye(4))
        
        # Combined transform: kinematic @ visual_origin
        T_physical = T_kinematic @ origin_transform
        
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(self.scale * 0.7, self.scale * 0.7, self.scale * 0.7)  # Slightly smaller
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.008)
        axes.SetConeRadius(0.025)
        axes.SetAxisLabels(False)
        
        # Make physical frames yellow to distinguish from kinematic (white)
        axes.GetXAxisShaftProperty().SetColor(1, 0.8, 0)      # Orange X
        axes.GetYAxisShaftProperty().SetColor(0, 0.8, 1)      # Cyan Y
        axes.GetZAxisShaftProperty().SetColor(1, 1, 0)         # Yellow Z
        axes.GetXAxisTipProperty().SetColor(1, 0.8, 0)
        axes.GetYAxisTipProperty().SetColor(0, 0.8, 1)
        axes.GetZAxisTipProperty().SetColor(1, 1, 0)
        
        transform = vtk.vtkTransform()
        transform.SetMatrix(T_physical.flatten())
        axes.SetUserTransform(transform)
        
        self.renderer.AddActor(axes)
        
        # Label
        pos = T_physical[:3, 3]
        x_end = (T_physical @ np.array([self.scale * 0.8, 0, 0, 1]))[:3]
        label_text = f"P{label} ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
        
        text = vtk.vtkVectorText()
        text.SetText(label_text)
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        
        text_actor = vtk.vtkFollower()
        text_actor.SetMapper(text_mapper)
        text_actor.SetScale(0.035, 0.035, 0.035)
        text_actor.SetPosition(x_end)
        text_actor.GetProperty().SetColor(1, 0.8, 0)
        text_actor.SetCamera(self.renderer.GetActiveCamera())
        self.renderer.AddActor(text_actor)
        
        # Store for updates
        key = f"physical_{link_name}"
        self.physical_axes[key] = {
            'actor': axes,
            'label': text_actor,
            'frame_name': f"{self.asset_id}_{link_name}",
            'link_name': link_name,
        }
        
        print(f"[PHYSICAL {label}] {link_name}: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

    def _add_joint_label(self, joint_name, joint_index, T_world):
        """Add a text label at the end of the X-axis for this joint."""
        # Calculate the endpoint of the X-axis
        x_endpoint_local = np.array([self.scale * 1.2, 0, 0, 1])
        x_endpoint_world = T_world @ x_endpoint_local
        
        # Create text actor
        text = vtk.vtkVectorText()
        text.SetText(f"J{joint_index+1}")
        
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        
        text_actor = vtk.vtkFollower()
        text_actor.SetMapper(text_mapper)
        text_actor.SetScale(0.05, 0.05, 0.05)
        text_actor.SetPosition(x_endpoint_world[:3])
        text_actor.GetProperty().SetColor(1, 0, 0)  # Red text
        
        # Make it always face the camera
        text_actor.SetCamera(self.renderer.GetActiveCamera())
        
        self.renderer.AddActor(text_actor)

        # Store for updates
        self.joint_axes[joint_name]['label'] = text_actor
        
        # Also print the position to console
        pos = T_world[:3, 3]
        print(f"[JOINT {joint_index+1}] {joint_name}: "
            f"origin=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}), "
            f"X-end=({x_endpoint_world[0]:.4f}, {x_endpoint_world[1]:.4f}, {x_endpoint_world[2]:.4f})")

    def set_visible(self, visible):
        """Show or hide all joint frames."""
        for info in self.joint_axes.values():
            info['actor'].SetVisibility(visible)

    def set_scale(self, scale):
        """Change the size of all axis markers."""
        for info in self.joint_axes.values():
            info['actor'].SetTotalLength(scale, scale, scale)

    def detach(self):
        """Remove all actors from the renderer."""
        if self.renderer:
            for info in self.joint_axes.values():
                self.renderer.RemoveActor(info['actor'])
        self.joint_axes.clear()
        self.registry.remove_callback(self._on_transform_updated)
        logger.info("JointFrameDisplay detached")