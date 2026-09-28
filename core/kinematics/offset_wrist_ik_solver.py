"""
Offset Wrist IK Solver - Unified analytical IK for UR-style and FR-style robots.
"""

import numpy as np
from typing import Optional, List
from math import pi

from core.kinematics.ur_ik_solver import URIKSolver

import logging
logger = logging.getLogger(__name__)


class OffsetWristIKSolver:
    """
    Unified analytical IK solver for 6-DOF offset-wrist robots.
    
    The IK solver works in the 'true_root' frame (parent of first revolute joint).
    Targets are specified in the 'true_TCP' frame (child of last fixed joint after arm).
    The solver transforms targets from true_TCP to true_root before solving IK.
    """
    
    def __init__(self, kinematic_model=None):
        self.model = kinematic_model
        
        # DH parameters (extracted from URDF)
        self.d1 = 0.0
        self.a2 = 0.0
        self.a3 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0
        
        # Non-standard offsets
        self.base_offset = 0.0
        self.tool_offset = 0.0
        
        # Frame transforms
        self._true_base_to_true_root = None  # Transform from true_base to true_root
        self._true_root_to_true_tcp = None   # Transform from true_root to true_TCP
        
        # Try to extract from model if provided
        if kinematic_model:
            self._extract_from_model()
            self._setup_frame_transforms()
        
        # Create the parameterized IK solver
        self.ik = URIKSolver(
            d1=self.d1,
            a2=self.a2,
            a3=self.a3,
            d4=self.d4,
            d5=self.d5,
            d6=self.d6 + self.tool_offset,  # Include tool offset in d6
            base_offset=self.base_offset
        )
        
        logger.info(f"OffsetWristIKSolver created with DH parameters:")
        logger.info(f"  d1={self.d1:.6f}, a2={self.a2:.6f}, a3={self.a3:.6f}")
        logger.info(f"  d4={self.d4:.6f}, d5={self.d5:.6f}, d6={self.d6:.6f}")
        if self.base_offset > 1e-6:
            logger.info(f"  base_offset={self.base_offset:.6f} (non-standard)")
        if self.tool_offset > 1e-6:
            logger.info(f"  tool_offset={self.tool_offset:.6f}")
    
    def _extract_from_model(self):
        """
        Extract DH parameters from the URDF joint origins.
        """
        if not self.model:
            return
        
        # Reset all joints to zero for consistent FK
        for joint_name, joint in self.model.joints.items():
            if joint['type'] in ['revolute', 'continuous', 'prismatic']:
                joint['value'] = 0.0
        
        # Update transforms
        self.model._forward_kinematics()
        
        # Get the true root (parent of first moving joint)
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
        
        # Extract joint origins from URDF
        origins = []
        for joint_name in chain:
            joint = self.model.joints[joint_name]
            origins.append({
                'name': joint_name,
                'xyz': np.array(joint['origin_xyz']),
                'rpy': np.array(joint['origin_rpy']),
            })
            logger.debug(f"  {joint_name}: xyz={joint['origin_xyz']}, rpy={joint['origin_rpy']}")
        
        # ===== Determine wrist offset convention =====
        has_y_offset = False
        has_z_offset = False
        
        for i in range(3, 6):
            xyz = origins[i]['xyz']
            if abs(xyz[1]) > 1e-6:
                has_y_offset = True
            if abs(xyz[2]) > 1e-6:
                has_z_offset = True
        
        if has_y_offset:
            logger.debug("  Detected UR-style wrist (Y-offsets)")
        elif has_z_offset:
            logger.debug("  Detected FR-style wrist (Z-offsets)")
        else:
            logger.debug("  Detected spherical wrist (no offsets)")
        
        # ===== Extract d1 (base to shoulder) =====
        if abs(origins[0]['xyz'][2]) > 1e-6:
            # UR-style: d1 is directly the Z offset of joint 1
            self.d1 = abs(origins[0]['xyz'][2])
            logger.debug(f"  d1 from j1 Z offset: {self.d1}")
        elif abs(origins[1]['xyz'][2]) > 1e-6:
            # FR-style: the Z offset at joint 2 is the base offset
            self.d1 = 0.0
            self.base_offset = origins[1]['xyz'][2]
            logger.debug(f"  base_offset from j2 Z offset: {self.base_offset}")
        else:
            self.d1 = 0.0
            logger.debug(f"  d1 = 0 (no base offset)")
        
        # ===== Extract a2 (shoulder to elbow) =====
        self.a2 = -abs(origins[2]['xyz'][0])
        logger.debug(f"  a2 from j3 X offset: {self.a2}")
        
        # ===== Extract a3 (elbow to wrist_1) =====
        self.a3 = -abs(origins[3]['xyz'][0])
        logger.debug(f"  a3 from j4 X offset: {self.a3}")
        
        # ===== Extract d4 (wrist_1 offset) =====
        j4_xyz = origins[3]['xyz']
        self.d4 = max(abs(j4_xyz[1]), abs(j4_xyz[2]))
        logger.debug(f"  d4 from j4 (max of Y,Z): {self.d4}")
        
        # ===== Extract d5 (wrist_2 offset) =====
        j5_xyz = origins[4]['xyz']
        self.d5 = max(abs(j5_xyz[1]), abs(j5_xyz[2]))
        logger.debug(f"  d5 from j5 (max of Y,Z): {self.d5}")
        
        # ===== Extract d6 (wrist_3 offset) =====
        j6_xyz = origins[5]['xyz']
        self.d6 = max(abs(j6_xyz[1]), abs(j6_xyz[2]))
        logger.debug(f"  d6 from j6 (max of Y,Z): {self.d6}")
        
        # ===== Extract tool offset =====
        self.tool_offset = self._extract_tool_offset(chain[-1])
        if self.tool_offset > 1e-6:
            logger.debug(f"  tool_offset: {self.tool_offset}")
        
        logger.info(f"Extracted DH parameters:")
        logger.info(f"  d1={self.d1:.6f}, a2={self.a2:.6f}, a3={self.a3:.6f}")
        logger.info(f"  d4={self.d4:.6f}, d5={self.d5:.6f}, d6={self.d6:.6f}")
    
    def _extract_tool_offset(self, last_joint_name):
        """
        Extract tool offset from fixed joints after the last revolute joint.
        """
        if not self.model:
            return 0.0
        
        last_joint = self.model.joints.get(last_joint_name)
        if not last_joint:
            return 0.0
        
        current_link = last_joint['child']
        total_offset = 0.0
        
        # Walk through fixed joints
        while current_link in self.model.link_children:
            children = self.model.link_children[current_link]
            if len(children) != 1:
                break
            
            child = children[0]
            
            # Check if connected by a fixed joint
            is_fixed = False
            for j in self.model.joints.values():
                if (j['parent'] == current_link and
                    j['child'] == child and
                    j['type'] == 'fixed'):
                    total_offset += abs(j['origin_xyz'][2])
                    is_fixed = True
                    break
            
            if is_fixed:
                current_link = child
            else:
                break
        
        return total_offset
    
    def _setup_frame_transforms(self):
        """
        Setup transforms between true_base, true_root, and true_TCP frames.
        """
        if not self.model:
            return
        
        # Get frame names
        true_base = self.model.get_true_base()
        true_root = self.model.get_true_root()
        
        logger.debug(f"True base: {true_base}")
        logger.debug(f"True root: {true_root}")
        
        # Compute true_base to true_root transform
        if true_base != true_root:
            if (true_base in self.model.link_transforms and 
                true_root in self.model.link_transforms):
                
                T_base_world = self.model.link_transforms[true_base]
                T_root_world = self.model.link_transforms[true_root]
                
                # T_true_base_to_true_root = T_root_world^-1 @ T_base_world
                self._true_base_to_true_root = np.linalg.inv(T_root_world) @ T_base_world
                
                logger.debug(f"Transform true_base → true_root:")
                logger.debug(f"  {self._true_base_to_true_root}")
            else:
                logger.warning(f"Cannot compute transform: missing link transforms")
                self._true_base_to_true_root = None
        else:
            self._true_base_to_true_root = np.eye(4)
            logger.debug(f"No transform needed (true_base == true_root)")
        
        # Find true_TCP (child of last fixed joint after arm)
        # The tool_offset is already included in d6, so the IK solver
        # returns the true_TCP position in true_root frame.
        # We need the transform from true_root to true_TCP to verify FK.
        
        # For now, we don't need _true_root_to_true_tcp because
        # the tool_offset is included in d6.
    
    def solve_ik_for_tcp(self, target_pose, q_guess=None):
        """Solve IK for target TCP pose with detailed logging."""
        import logging
        logger = logging.getLogger(__name__)
        
        logger.info(f"\n{'='*60}")
        logger.info(f"Solving IK for TCP")
        logger.info(f"{'='*60}")
        logger.info(f"Target pose (in true_base frame):\n{target_pose}")
        
        # Transform target from true_base to true_root frame
        if self._true_base_to_true_root is not None:
            target_in_true_root = self._true_base_to_true_root @ target_pose
            logger.info(f"Target pose (in true_root frame):\n{target_in_true_root}")
        else:
            target_in_true_root = target_pose
            logger.info(f"Target pose (no transform, already in true_root frame):\n{target_in_true_root}")
        
        # Solve IK in true_root frame
        solutions = self.ik.inverse_all(target_in_true_root)
        
        if solutions is None or solutions.shape[1] == 0:
            logger.warning(f"NO SOLUTIONS FOUND!")
            logger.warning(f"Target in true_root frame:\n{target_in_true_root}")
            
            # Debug: check P_05
            P_05 = target_in_true_root @ np.array([0, 0, -self.ik.d6, 1])
            logger.warning(f"P_05: {P_05}")
            
            # Check r
            r = np.linalg.norm(P_05[0:2])
            logger.warning(f"r = {r}, d4 = {self.ik.d4}")
            
            if r < self.ik.d4:
                logger.warning(f"Target UNREACHABLE: r = {r} < d4 = {self.ik.d4}")
            
            return None
        
        logger.info(f"Found {solutions.shape[1]} solutions")
        
        # Select best solution based on q_guess
        if q_guess is not None:
            best_q = self._select_best_solution(solutions, q_guess)
            logger.info(f"Selected solution: {best_q}")
        else:
            best_q = solutions[:, 0]
            logger.info(f"Using first solution: {best_q}")
        
        return best_q
    
    def solve_ik(self, target_pose, q_guess=None):
        """Alias for solve_ik_for_tcp."""
        return self.solve_ik_for_tcp(target_pose, q_guess)
    
    def _select_best_solution(self, solutions, q_current):
        """
        Select best solution based on:
        1. Proximity to current joint state
        2. Configuration branch continuity
        """
        if q_current is None:
            q_current = np.zeros(6)
        
        q_current = np.array(q_current)
        
        # Determine current configuration
        current_config = self._get_configuration(q_current)
        
        best_q = None
        best_score = float('inf')
        
        for i in range(solutions.shape[1]):
            q_sol = solutions[:, i]
            
            # Compute normalized diff
            diff = q_sol - q_current
            diff = (diff + pi) % (2 * pi) - pi
            
            # Base score: weighted squared differences
            weights = np.array([2.0, 3.0, 3.0, 2.0, 1.0, 1.0])
            score = np.sum(weights * diff**2)
            
            # Configuration continuity check
            sol_config = self._get_configuration(q_sol)
            if sol_config != current_config:
                score += 10000  # Heavy penalty for configuration change
            
            if score < best_score:
                best_score = score
                best_q = q_sol
        
        return best_q
    
    def _get_configuration(self, q):
        """Determine the configuration branch of a joint solution."""
        if q[0] > -pi/2 and q[0] < pi/2:
            shoulder = 'left'
        else:
            shoulder = 'right'
        
        if q[2] > 0:
            elbow = 'up'
        else:
            elbow = 'down'
        
        if q[4] > 0:
            wrist = 'flip'
        else:
            wrist = 'no_flip'
        
        return (shoulder, elbow, wrist)
    
    def forward_kinematics(self, q):
        """Forward kinematics returning TCP in true_TCP frame."""
        # The IK solver's forward() returns TCP in true_root frame
        tcp_in_true_root = self.ik.forward(q)
        
        # Transform from true_root to true_base (inverse of base transform)
        if self._true_base_to_true_root is not None:
            tcp_in_true_base = np.linalg.inv(self._true_base_to_true_root) @ tcp_in_true_root
            return tcp_in_true_base
        
        return tcp_in_true_root
    
    def forward(self, q):
        """Alias for forward_kinematics."""
        return self.forward_kinematics(q)
