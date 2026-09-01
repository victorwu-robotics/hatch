# core/kinematics/ik_solver.py

import numpy as np
from typing import Optional, List
from math import pi

from core.kinematics.ur_ik_solver import URIKSolver

import logging
logger = logging.getLogger(__name__)

class OffsetWristIKSolver:
    """IK solver wrapper that creates a parameterized UR IK solver."""
    
    def __init__(self, kinematic_model=None):
        self.model = kinematic_model
        
        # Default UR10e parameters (fallback)
        self.d1 = 0.1273
        self.a2 = -0.612
        self.a3 = -0.5723
        self.d4 = 0.163941
        self.d5 = 0.1157
        self.d6 = 0.0922
        
        # Base frame compensation (will be set dynamically)
        self._base_compensation = None
        self._true_base_frame = None
        self._ik_base_frame = None
        
        # Try to extract from model if provided
        if kinematic_model:
            self._extract_from_model()
            self._setup_base_compensation()
        
        # Create the parameterized IK solver
        self.ik = URIKSolver(
            d1=self.d1,
            a2=self.a2,
            a3=self.a3,
            d4=self.d4,
            d5=self.d5,
            d6=self.d6
        )
        
        logger.debug(f"IKSolver: Created UR IK solver with parameters:")
        logger.debug(f"  d1={self.d1:.6f}, a2={self.a2:.6f}, a3={self.a3:.6f}")
        logger.debug(f"  d4={self.d4:.6f}, d5={self.d5:.6f}, d6={self.d6:.6f}")
        
        if self._base_compensation is not None:
            logger.debug(f"  Base compensation: {self._true_base_frame} → {self._ik_base_frame}")
    
    def _setup_base_compensation(self):
        """
        Determine base frame compensation between true_base and true_root.
        
        The IK solver works in the true_root frame (parent of first moving joint).
        The Cartesian control panel works in the true_base frame (mounting base).
        
        For UR robots: true_base = 'base_link', true_root = 'base_link_inertia'
                    (compensation is the fixed joint transform between them)
        For Farino:    true_base = 'base_link', true_root = 'base_link'
                    (compensation is identity)
        
        The compensation transforms poses from true_base to true_root for IK,
        and from true_root back to true_base for FK.
        """
        if not self.model:
            return
        
        # Get true base (mounting reference for Cartesian control)
        self._true_base_frame = self.model.get_true_base()
        logger.debug(f"True base (Cartesian reference): {self._true_base_frame}")
        
        # Get true root (IK reference, parent of first moving joint)
        self._true_root_frame = self.model.get_true_root()
        logger.debug(f"True root (IK reference): {self._true_root_frame}")
        
        # The IK base is the same as true root
        self._ik_base_frame = self._true_root_frame
        
        # Compute compensation: transform from true_base to true_root
        if self._true_base_frame != self._true_root_frame:
            if (self._true_base_frame in self.model.link_transforms and 
                self._true_root_frame in self.model.link_transforms):
                
                T_base_world = self.model.link_transforms[self._true_base_frame]
                T_root_world = self.model.link_transforms[self._true_root_frame]
                
                # Transform from true_base to true_root:
                # T_root_base = inv(T_root_world) @ T_base_world
                self._T_base_to_root = np.linalg.inv(T_root_world) @ T_base_world

                # Transform from true_root to true_base:
                self._T_root_to_base = np.linalg.inv(self._T_base_to_root)

                logger.debug(f"Base compensation computed:")
                logger.debug(f"  T_base_to_root:\n{self._T_base_to_root}")
                
        else:
            self._T_base_to_root = np.eye(4)
            self._T_root_to_base = np.eye(4)
    
    def _extract_from_model(self):
        """Extract DH parameters from the arm chain transforms."""
        if not self.model:
            return
        
        # Reset all joints to zero
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                joint['value'] = 0.0
        
        # Update transforms
        self.model._forward_kinematics()
        
        # Find the true base (parent of first moving joint)
        true_base = None
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                true_base = joint['parent']
                break
        
        if true_base is None:
            logger.debug("No moving joints found")
            return
        
        # Get the arm chain starting from true base
        try:
            true_root = self.model.get_true_root()
            logger.debug(f"\n  Extracting DH parameters using true root: {true_root}")
            chain = self.model.get_arm_chain(true_root)
        except ValueError as e:
            logger.debug(f"Error: {e}")
            return
        
        if len(chain) < 6:
            logger.debug(f"Warning: Expected 6 joints, found {len(chain)}")
            return
        
        logger.debug(f"\nExtracting DH parameters from arm chain (base: {true_base}):")
        
        # Get transforms relative to true base
        transforms = self.model.get_joint_child_transforms(chain, true_base)
        
        # Print transforms for debugging
        for i, (j_name, T) in enumerate(zip(chain, transforms)):
            pos = T[:3, 3]
            logger.debug(f"  {j_name} → {self.model.joints[j_name]['child']}: ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
        
        # Extract DH parameters
        dh_params = self._extract_dh_from_transforms(transforms)
        
        # Update instance variables
        self.d1 = dh_params['d1']
        self.a2 = dh_params['a2']
        self.a3 = dh_params['a3']
        self.d4 = dh_params['d4']
        self.d5 = dh_params['d5']
        self.d6 = dh_params['d6']
        
    def _extract_dh_from_transforms(self, transforms: List[np.ndarray]) -> dict:
        """Extract DH parameters from link transforms at zero position."""
        if len(transforms) < 6:
            raise ValueError(f"Need 6 transforms, got {len(transforms)}")
        
        # Extract positions
        p0 = transforms[0][:3, 3]
        p1 = transforms[1][:3, 3]
        p2 = transforms[2][:3, 3]
        p3 = transforms[3][:3, 3]
        p4 = transforms[4][:3, 3]
        p5 = transforms[5][:3, 3]
        
        # d1: Z offset of first link
        d1 = p0[2]
        
        # a2: X offset from joint1 to joint2
        a2 = p2[0] - p1[0]
        
        # a3: X offset from joint2 to joint3
        a3 = p3[0] - p2[0]
        
        # d4: Y offset from joint2 to joint3
        d4 = p3[1] - p2[1]
        
        # d5: Z offset from joint3 to joint4
        d5 = p3[2] - p4[2]
        
        # d6: Y offset from joint4 to joint5
        d6 = p5[1] - p4[1]
        
        # Apply sign conventions to match standard UR
        if a2 > 0:
            a2 = -a2
        if a3 > 0:
            a3 = -a3
        if d5 < 0:
            d5 = -d5
        if d6 < 0:
            d6 = -d6

        # IMPORTANT: Take absolute values for lengths
        d4 = abs(d4)
        d5 = abs(d5)
        d6 = abs(d6)


        return {
            'd1': d1,
            'a2': a2,
            'a3': a3,
            'd4': d4,
            'd5': d5,
            'd6': d6,
        }
    
    def solve_ik_for_tcp(self, target_pose: np.ndarray, q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """Solve IK for target TCP pose in true_base frame."""
        # Transform from true_base to true_root
        if hasattr(self, '_T_base_to_root'):
            target_pose_ik = self._T_base_to_root @ target_pose
        else:
            target_pose_ik = target_pose
        
        return self.ik.inverse(target_pose_ik, q_guess)
    
    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """FK returning TCP pose in true_base frame."""
        tcp_ik = self.ik.forward(q)
        
        if hasattr(self, '_T_root_to_base'):
            return self._T_root_to_base @ tcp_ik
        
        return tcp_ik
