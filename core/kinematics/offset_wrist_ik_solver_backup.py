# core/kinematics/ik_solver.py

"""
Offset Wrist IK Solver - Parameterized for UR-style and FR-style robots.

Extracts DH parameters from the URDF joint origins directly.
Handles both UR (with Y offsets at wrist) and FR (with Z offsets at wrist).
"""

import numpy as np
from typing import Optional, List
from math import pi

from core.kinematics.ur_ik_solver import URIKSolver

import logging
logger = logging.getLogger(__name__)


class OffsetWristIKSolver:
    """IK solver wrapper that creates a parameterized IK solver."""
    
    def __init__(self, kinematic_model=None):
        self.model = kinematic_model
        
        # DH parameters (will be extracted from model)
        self.d1 = 0.1273
        self.a2 = -0.612
        self.a3 = -0.5723
        self.d4 = 0.163941
        self.d5 = 0.1157
        self.d6 = 0.0922
        
        # Base frame compensation
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
        
        logger.debug(f"OffsetWristIKSolver created with DH parameters:")
        logger.debug(f"  d1={self.d1:.6f}, a2={self.a2:.6f}, a3={self.a3:.6f}")
        logger.debug(f"  d4={self.d4:.6f}, d5={self.d5:.6f}, d6={self.d6:.6f}")
    
    def _extract_from_model(self):
        """
        Extract DH parameters from URDF joint origins directly.
        
        Works for both UR-style (with Y wrist offsets) and FR-style
        (with Z wrist offsets) robots.
        """
        if not self.model:
            return
        
        # Reset all joints to zero
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                joint['value'] = 0.0
        
        # Update transforms
        self.model._forward_kinematics()
        
        # Find the true root (parent of first moving joint)
        true_root = self.model.get_true_root()
        
        # Get the arm chain
        try:
            chain = self.model.get_arm_chain(true_root)
            logger.debug(f"Arm chain: {chain}")
        except ValueError as e:
            logger.error(f"Cannot get arm chain: {e}")
            return
        
        if len(chain) < 6:
            logger.warning(f"Expected 6 joints, found {len(chain)}")
            return
        
        # Get joint origins directly from URDF
        origins = []
        for joint_name in chain:
            joint = self.model.joints[joint_name]
            xyz = joint['origin_xyz']
            rpy = joint['origin_rpy']
            origins.append({
                'xyz': np.array(xyz),
                'rpy': np.array(rpy),
                'name': joint_name
            })
            logger.debug(f"  {joint_name}: xyz={xyz}, rpy={rpy}")
        
        # ===== Extract DH parameters =====
        # d1: Vertical offset from base to first joint axis
        # For UR: j1 has xyz[2]=0.1273
        # For FR: j1 has xyz[2]=0.0, but j2 has xyz[2]=0.18
        if abs(origins[0]['xyz'][2]) > 0.001:
            d1 = abs(origins[0]['xyz'][2])
            logger.debug(f"  d1 from j1 Z offset: {d1}")
        else:
            d1 = abs(origins[1]['xyz'][2])
            logger.debug(f"  d1 from j2 Z offset: {d1}")
        
        # a2: Upper arm length (distance from shoulder to elbow)
        # This is the X component of joint 3's origin (elbow)
        a2 = abs(origins[2]['xyz'][0])
        logger.debug(f"  a2 from j3 X offset: {a2}")
        
        # a3: Forearm length (distance from elbow to wrist)
        # This is the X component of joint 4's origin
        a3 = abs(origins[3]['xyz'][0])
        logger.debug(f"  a3 from j4 X offset: {a3}")
        
        # d4: Wrist 1 offset
        # For UR: the Y offset in the URDF is actually in Z component
        # UR10: xyz="-0.5723 0 0.163941" → d4 = 0.163941 (Z component)
        # FR10: xyz="-0.586 0 0" → d4 = 0.0
        if abs(origins[3]['xyz'][1]) > 0.001:
            d4 = abs(origins[3]['xyz'][1])  # Y offset (if present)
        elif abs(origins[3]['xyz'][2]) > 0.001:
            d4 = abs(origins[3]['xyz'][2])  # Z offset (UR-style)
        else:
            d4 = 0.0

        # d5: Wrist 2 offset
        # For UR: Y offset of j5 (0.1157)
        # For FR: Z offset of j5 (0.159)
        if abs(origins[4]['xyz'][1]) > 0.001:
            d5 = abs(origins[4]['xyz'][1])
            logger.debug(f"  d5 from j5 Y offset: {d5}")
        else:
            d5 = abs(origins[4]['xyz'][2])
            logger.debug(f"  d5 from j5 Z offset: {d5}")
        
        # d6: Wrist 3 offset (to tool flange)
        # For UR: Y offset of j6 (0.0922)
        # For FR: Z offset of j6 (0.114)
        if abs(origins[5]['xyz'][1]) > 0.001:
            d6 = abs(origins[5]['xyz'][1])
            logger.debug(f"  d6 from j6 Y offset: {d6}")
        else:
            d6 = abs(origins[5]['xyz'][2])
            logger.debug(f"  d6 from j6 Z offset: {d6}")
        
        # Update instance variables
        self.d1 = d1
        self.a2 = -a2  # UR IK uses negative a2
        self.a3 = -a3  # UR IK uses negative a3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6
        
        logger.info(f"Extracted DH parameters:")
        logger.info(f"  d1 = {self.d1:.6f}")
        logger.info(f"  a2 = {self.a2:.6f}")
        logger.info(f"  a3 = {self.a3:.6f}")
        logger.info(f"  d4 = {self.d4:.6f}")
        logger.info(f"  d5 = {self.d5:.6f}")
        logger.info(f"  d6 = {self.d6:.6f}")
    
    def _setup_base_compensation(self):
        """
        Determine base frame compensation between true_base and true_root.
        
        For UR: true_base = 'base_link', true_root = 'base_link_inertia'
                (compensation is the fixed joint transform between them)
        For FR: true_base = 'base_link', true_root = 'base_link'
                (compensation is identity)
        """
        if not self.model:
            return
        
        # Get true base (mounting reference for Cartesian control)
        self._true_base_frame = self.model.get_true_base()
        logger.debug(f"True base (Cartesian reference): {self._true_base_frame}")
        
        # Get true root (IK reference, parent of first moving joint)
        self._ik_base_frame = self.model.get_true_root()
        logger.debug(f"True root (IK reference): {self._ik_base_frame}")
        
        # Compute compensation: transform from true_base to true_root
        if self._true_base_frame != self._ik_base_frame:
            if (self._true_base_frame in self.model.link_transforms and 
                self._ik_base_frame in self.model.link_transforms):
                
                T_base_world = self.model.link_transforms[self._true_base_frame]
                T_root_world = self.model.link_transforms[self._ik_base_frame]
                
                # Transform from true_base to true_root
                self._base_compensation = np.linalg.inv(T_root_world) @ T_base_world
                
                logger.debug(f"Base compensation (true_base → true_root):")
                logger.debug(f"  T_base_world:\n{T_base_world}")
                logger.debug(f"  T_root_world:\n{T_root_world}")
                logger.debug(f"  T_base_to_root:\n{self._base_compensation}")
            else:
                logger.warning(f"Cannot compute compensation: missing transforms")
                self._base_compensation = None
        else:
            # Same frame — no compensation needed
            self._base_compensation = None
            logger.debug(f"No base compensation needed (true_base == true_root)")
    
    def solve_ik_for_tcp(self, target_pose: np.ndarray, q_guess: np.ndarray = None) -> Optional[np.ndarray]:
        """
        Solve IK for target TCP pose.
        
        Args:
            target_pose: Target pose in TRUE BASE coordinates (base_link)
            q_guess: Initial joint guess
        
        Returns:
            Joint angles in robot's joint space
        """
        # Transform from true_base to true_root for IK
        if self._base_compensation is not None:
            # WRONG: target_pose_ik = np.linalg.inv(self._base_compensation) @ target_pose
            # CORRECT: Apply compensation from true_base to true_root
            target_pose_ik = self._base_compensation @ target_pose
        else:
            target_pose_ik = target_pose
        
        return self.ik.inverse(target_pose_ik, q_guess)
    
    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics returning TCP in TRUE BASE coordinates."""
        tcp_ik = self.ik.forward(q)
        
        if self._base_compensation is not None:
            # Transform from true_root back to true_base
            return np.linalg.inv(self._base_compensation) @ tcp_ik
        
        return tcp_ik
