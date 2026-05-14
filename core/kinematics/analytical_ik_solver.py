"""
Pure Analytical IK Solver for 6-DOF spherical wrist robot.

Uses geometric decoupling: arm (position) + wrist (orientation).
No numerical optimization — purely closed-form analytical solutions.
"""

import numpy as np
from typing import Optional, List, Tuple
from scipy.spatial.transform import Rotation as R
import logging

logger = logging.getLogger(__name__)


class AnalyticalIKSolver:
    """
    Pure analytical IK solver for 6-DOF robot with spherical wrist.
    
    Decomposes into arm (θ1,θ2,θ3) positioning the wrist center,
    and wrist (θ4,θ5,θ6) achieving the desired orientation.
    Provides all 8 solutions in closed form.
    """
    
    def __init__(self, kinematic_mdel, d1: float, a2: float, a3: float, d6: float):
        """
        Args:
            d1: Shoulder height offset (base to J2 along Z, at zero)
            a2: Upper arm length (J2 to J3)
            a3: Forearm length (J3 to wrist center = J5/J6 origin)
            d6: Tool offset from wrist center to TCP (along tool Z-axis)
        """
        self.model = kinematic_mdel
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.d6 = d6
    
    def solve_ik(self, target_pose: np.ndarray) -> List[np.ndarray]:
        """
        Solve IK analytically. Returns all valid solutions.
        
        Args:
            target_pose: 4x4 homogeneous transformation matrix
            
        Returns:
            List of 6-element joint angle arrays (up to 8 solutions)
        """
        pos_target = target_pose[:3, 3]
        rot_target = target_pose[:3, :3]
        
        return self._solve_all(pos_target, rot_target)
    
    def solve_ik_closest(self, target_pose: np.ndarray, 
                         q_current: np.ndarray) -> Optional[np.ndarray]:
        """
        Solve IK and return solution closest to current joint positions.
        
        Args:
            target_pose: 4x4 homogeneous transformation matrix
            q_current: Current joint positions (for selecting closest solution)
            
        Returns:
            6-element joint angle array, or None if no solution
        """
        solutions = self.solve_ik(target_pose)
        if not solutions:
            return None
        return self._select_closest(solutions, q_current)
    
    def _solve_all(self, pos_target: np.ndarray, 
                   rot_target: np.ndarray) -> List[np.ndarray]:
        """Generate all analytical IK solutions."""
        # Wrist center position
        a_vec = rot_target[:, 2]
        P_wrist = pos_target - self.d6 * a_vec
        
        # Solve arm (up to 4 solutions)
        arm_solutions = self._solve_arm(P_wrist)
        
        # Solve wrist for each arm solution
        all_solutions = []
        for θ1, θ2, θ3 in arm_solutions:
            wrist_sols = self._solve_wrist(θ1, θ2, θ3, rot_target)
            for θ4, θ5, θ6 in wrist_sols:
                q = np.array([θ1, θ2, θ3, θ4, θ5, θ6])
                all_solutions.append(q)
        
        return all_solutions
    
    def _solve_arm(self, P_wrist: np.ndarray) -> List[Tuple[float, float, float]]:
        """Solve arm joints analytically. Returns up to 4 solutions."""
        x, y, z = P_wrist
        
        r_xy = np.sqrt(x**2 + y**2)
        
        # Shoulder: two solutions
        if r_xy > 1e-10:
            θ1_options = [np.arctan2(y, x), np.arctan2(-y, -x)]
        else:
            θ1_options = [0.0]
        
        solutions = []
        
        for θ1 in θ1_options:
            # Project wrist into arm plane
            r_proj = x * np.cos(θ1) + y * np.sin(θ1)
            z_rel = z - self.d1
            
            D_sq = r_proj**2 + z_rel**2
            
            # Law of cosines for θ3
            cos_θ3 = (D_sq - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
            
            # Check reachability
            if cos_θ3 > 1.0 + 1e-9 or cos_θ3 < -1.0 - 1e-9:
                continue
            
            cos_θ3 = np.clip(cos_θ3, -1.0, 1.0)
            
            # Two elbow solutions
            for sign in [+1, -1]:
                sin_θ3 = sign * np.sqrt(1.0 - cos_θ3**2)
                θ3 = np.arctan2(sin_θ3, cos_θ3)
                
                # θ2 from linear equations
                K1 = self.a2 + self.a3 * cos_θ3
                K2 = self.a3 * sin_θ3
                
                sin_θ2 = (-K1 * r_proj + K2 * z_rel) / D_sq
                cos_θ2 = ( K2 * r_proj + K1 * z_rel) / D_sq
                θ2 = np.arctan2(sin_θ2, cos_θ2)
                
                solutions.append((θ1, θ2, θ3))
        
        return self._deduplicate(solutions)
    
    def _compute_R03(self, θ1: float, θ2: float, θ3: float) -> np.ndarray:
        """
        Compute R₀₃: orientation of frame 3.
        
        J1: Rz(θ1), J2: Ry(-θ2), J3: Ry(θ3)
        """
        R01 = R.from_rotvec([0, 0, θ1]).as_matrix()
        R12 = R.from_rotvec([0, -θ2, 0]).as_matrix()
        R23 = R.from_rotvec([0, θ3, 0]).as_matrix()
        return R01 @ R12 @ R23
    
    def _solve_wrist(self, θ1: float, θ2: float, θ3: float,
                     R_des: np.ndarray) -> List[Tuple[float, float, float]]:
        """
        Solve wrist joints analytically.
        
        R_des = R₀₃ @ Rz(θ4) @ Ry(θ5) @ Rz(θ6)
        => R_target = R₀₃ᵀ @ R_des = Rz(θ4) @ Ry(θ5) @ Rz(θ6)
        """
        R03 = self._compute_R03(θ1, θ2, θ3)
        R_target = R03.T @ R_des
        
        # return self._extract_zyz_euler(R_target)
        return self._extract_zyz_euler(R_target)
    
    def _extract_zyz_euler(self, R_mat: np.ndarray) -> List[Tuple[float, float, float]]:
        """Extract Z-Y-Z Euler angles from rotation matrix."""
        r13 = R_mat[0, 2]
        r23 = R_mat[1, 2]
        r33 = R_mat[2, 2]
        r31 = R_mat[2, 0]
        r32 = R_mat[2, 1]
        
        solutions = []
        
        for sign in [+1, -1]:
            sin_θ5 = sign * np.sqrt(max(0.0, 1.0 - r33**2))
            θ5 = np.arctan2(sin_θ5, r33)
            
            if abs(sin_θ5) > 1e-6:
                θ4 = np.arctan2(r23 / sin_θ5, r13 / sin_θ5)
                θ6 = np.arctan2(r32 / sin_θ5, -r31 / sin_θ5)
            else:
                # Singularity: θ5 = 0 or π
                θ4 = 0.0
                if r33 > 0:
                    # θ5 = 0 → R_target = Rz(θ4 + θ6)
                    θ6 = np.arctan2(R_mat[1, 0], R_mat[0, 0])
                else:
                    # θ5 = π
                    θ6 = np.arctan2(-R_mat[1, 0], R_mat[0, 0])
            
            solutions.append((θ4, θ5, θ6))
        
        return solutions

    def _extract_xzx_euler(self, R_mat):
        """Extract X-Z-X Euler angles from rotation matrix."""
        r11 = R_mat[0, 0]
        r12 = R_mat[0, 1]
        r13 = R_mat[0, 2]
        r21 = R_mat[1, 0]
        r31 = R_mat[2, 0]
        
        solutions = []
        
        for sign in [+1, -1]:
            sin_θ5 = sign * np.sqrt(max(0.0, 1.0 - r11**2))
            θ5 = np.arctan2(sin_θ5, r11)
            
            if abs(sin_θ5) > 1e-6:
                θ4 = np.arctan2(r31 / sin_θ5, r21 / sin_θ5)
                θ6 = np.arctan2(r13 / sin_θ5, -r12 / sin_θ5)
            else:
                # Singularity: θ5 = 0 or π
                θ4 = 0.0
                if r11 > 0:  # θ5 = 0
                    # R = Rx(θ4 + θ6)
                    θ6 = np.arctan2(R_mat[2, 1], R_mat[1, 1])
                else:  # θ5 = π
                    θ6 = np.arctan2(-R_mat[2, 1], R_mat[1, 1])
            
            solutions.append((θ4, θ5, θ6))
        
        return solutions

    def _deduplicate(self, solutions: List[Tuple[float, ...]], 
                     tol: float = 1e-3) -> List[Tuple[float, ...]]:
        """Remove duplicate solutions."""
        unique = []
        for sol in solutions:
            is_dup = False
            for existing in unique:
                diff = np.abs(np.array(sol) - np.array(existing))
                diff = np.minimum(diff, 2*np.pi - diff)
                if np.all(diff < tol):
                    is_dup = True
                    break
            if not is_dup:
                unique.append(sol)
        return unique
    
    def _select_closest(self, solutions: List[np.ndarray], 
                        q_current: np.ndarray) -> np.ndarray:
        """Select solution closest to current joint positions."""
        best_q = None
        best_dist = float('inf')
        
        for q in solutions:
            diff = q - q_current
            # Account for circular angle wrap
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            dist = np.linalg.norm(diff)
            
            if dist < best_dist:
                best_dist = dist
                best_q = q
        
        return best_q

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics using the model's native FK."""
        self.model.update_state(q)
        return self.model.get_tcp_pose()



