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
        Dynamically determine base frame compensation.
        
        Strategy:
        1. Find the first moving joint's parent (true kinematic base)
        2. Find the frame that the DH parameters expect (first link in arm chain)
        3. Compute compensation between them
        """
        if not self.model:
            return
        
        # Step 1: Find true kinematic base (parent of first moving joint)
        true_base = None
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                true_base = joint['parent']
                break
        
        if true_base is None:
            logger.debug("  No moving joints found - robot may be static")
            return
        
        self._true_base_frame = true_base
        logger.debug(f"\n  True kinematic base: {true_base}")
        
        # Step 2: Find the frame that DH parameters expect
        # This is typically the first link in the arm chain
        try:
            arm_chain = self.model.get_arm_chain(true_base)
            if arm_chain:
                # The parent of the first joint in the chain is the IK base
                first_joint = arm_chain[0]
                ik_base = self.model.joints[first_joint]['parent']
                self._ik_base_frame = ik_base
                logger.debug(f"  IK expects base: {ik_base}")
        except Exception as e:
            logger.debug(f"  Could not determine IK base: {e}")
            return
        
        # Step 3: If true base and IK base are different, compute compensation
        if true_base != self._ik_base_frame:
            # Get transforms in world coordinates at zero position
            if true_base in self.model.link_transforms and self._ik_base_frame in self.model.link_transforms:
                T_true_world = self.model.link_transforms[true_base]
                T_ik_world = self.model.link_transforms[self._ik_base_frame]
                
                # Compute transform from true base to IK base
                self._base_compensation = np.linalg.inv(T_true_world) @ T_ik_world
                
                logger.debug(f"  ✅ Base compensation enabled: {true_base} → {self._ik_base_frame}")
                
                # Check if it's a 180° rotation (for debugging)
                R = self._base_compensation[:3, :3]
                if np.allclose(R, np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), atol=1e-6):
                    logger.debug(f"      (This is a 180° rotation around Z)")
            else:
                logger.debug(f"  ⚠️  Could not compute compensation - missing transforms")
        else:
            logger.debug(f"  ✅ No compensation needed - true base matches IK base")
    
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
        """
        Solve IK for target TCP pose.
        
        Args:
            target_pose: Target pose in TRUE robot base coordinates
            q_guess: Initial joint guess
        
        Returns:
            Joint angles in robot's joint space
        """
        # Apply base compensation if needed
        if self._base_compensation is not None:
            # Transform target from true base to IK base coordinates
            target_pose_compensated = self._base_compensation @ target_pose
        else:
            target_pose_compensated = target_pose
        
        # Call the underlying IK solver
        return self.ik.inverse(target_pose_compensated, q_guess)
    
    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics returning TCP in TRUE base coordinates."""
        # Get TCP in IK base coordinates
        tcp_ik = self.ik.forward(q)
        
        # Transform to true base coordinates if needed
        if self._base_compensation is not None:
            # T_true_to_ik = self._base_compensation
            # So T_ik_to_true = inv(T_true_to_ik)
            T_ik_to_true = np.linalg.inv(self._base_compensation)
            tcp_true = T_ik_to_true @ tcp_ik
            return tcp_true
        
        return tcp_ik
