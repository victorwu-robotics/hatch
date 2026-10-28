import numpy as np
from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry
from core.kinematics.unified_ik_solver import UnifiedIKSolver

np.set_printoptions(precision=6, suppress=True)

def debug_one(name, urdf_path, q_true):
    print("=" * 70)
    print(name)
    print("=" * 70)

    model = KinematicModel(urdf_path=urdf_path).load()
    geom = extract_arm_geometry(model)
    solver = UnifiedIKSolver(geom)

    print(f"Parameters:")
    print(f"  base_height      = {solver.base_height:.4f}")
    print(f"  upper_arm        = {solver.upper_arm:.4f}")
    print(f"  forearm_to_root  = {solver.forearm_to_root:.4f}")
    print(f"  AB_length        = {solver.AB_length:.4f}")
    print(f"  BC_length        = {solver.BC_length:.4f}")
    print(f"  flange_offset    = {solver.flange_offset:.4f}")
    print()

    # 1. Get the flange pose in the model's frame (which is world).
    T_flange_world = model.get_flange_pose(q_true)
    
    # 2. Construct the True Root pose from the geom object.
    # The true root is the parent of the first moving joint (J1).
    # We need its origin and orientation.
    # The geom object should have this information.
    # Let's assume the true root is at the origin of J1, but with the correct orientation.
    # For UR10, the true root is 'base_link_inertia', which is rotated by pi about Z.
    # The J1 origin is at (0, 0, 0.1273) relative to the true root.
    # So the true root pose relative to world is:
    # - translation: (0, 0, 0) (since base_link_inertia is at origin)
    # - rotation: pi about Z.
    
    # We can construct this from the model's world tree.
    # Since we don't have get_link_pose, we'll use a workaround:
    # The model's get_flange_pose(q=0) gives us the flange in world at zero pose.
    # We can use that to infer the true root orientation.
    
    # Actually, the simplest way: use the geom object's axis directions.
    # The geom.axes[0] is J1. Its direction in the true root frame is (0, 0, 1).
    # In the world frame, it's (0, 0, 1) (for UR10, since J1 is along Z).
    # The true root orientation can be deduced from the axis twists.
    
    # Let's just print the geom's axes and origins for now.
    print("Geom axes (in true root frame):")
    for i, ax in enumerate(geom.axes):
        print(f"  J{i+1}: origin={ax.point}, direction={ax.direction}")
    print()

    # 3. Transform the flange pose into the true root frame.
    # The true root frame is defined by the geom.axes[0] (J1).
    # The origin of the true root is the origin of J1.
    # The orientation of the true root is such that J1 points along Z.
    # For UR10, the true root is rotated by pi about Z from the world.
    # We can find this rotation by comparing the world J1 axis to the true root J1 axis.
    
    # For now, let's just use the geom to find the true root pose.
    # If the geom is correctly extracted, its axes are in the true root frame.
    # But we don't know the true root's world pose from geom alone.
    
    # The key is: the solver expects T_flange in the true root frame.
    # We need to compute T_flange_true_root = T_true_root_world^{-1} @ T_flange_world.
    
    # For UR10, T_true_root_world is a 180 deg rotation about Z.
    # For FR5 and Elfin, it may be identity or something else.
    
    # Let's assume the true root is at the world origin, but with an orientation
    # that aligns J1 with Z.
    # We can compute the orientation from the geom.axes[0].direction.
    # In the world frame, J1 direction is (0, 0, 1) for UR10.
    # In the true root frame, J1 direction is (0, 0, 1) for UR10.
    # So the rotation is about Z. The angle can be found from the origin of J2.
    # J2 origin in world is (0, 0, 0.1273) rotated by pi about Z -> (0, 0, 0.1273).
    # J2 origin in true root is (0, 0, 0.1273).
    # So the rotation is pi about Z.
    
    # For a general solution, we need the true root world pose.
    # Let's compute it from the model's world tree.
    # The model has a method to get the world pose of any link.
    # We can use model.get_link_pose("base_link_inertia") if it exists.
    # If not, we can use the kinematic model's forward kinematics to get it.
    
    # Let's try to use the model's forward kinematics to get the true root pose.
    # We can set q=0 and get the pose of the true root link.
    # But we need to know the link name.
    # For UR10, it's "base_link_inertia".
    # For others, it might be the parent of the first joint.
    
    true_root_link_name = "base_link_inertia"  # For UR10
    if "FR5" in name:
        true_root_link_name = "base_link"  # Or whatever FR5 uses
    elif "Elfin" in name:
        true_root_link_name = "base_link"  # Or whatever Elfin uses
    
    try:
        T_true_root_world = model.get_link_pose(true_root_link_name)
        if T_true_root_world is None:
            raise AttributeError
    except AttributeError:
        # Fallback: use the J1 origin and J1 axis to construct the true root pose.
        # The true root frame has J1 as its Z axis, and origin at J1 origin.
        # So T_true_root_world is a translation to J1 origin, rotated so Z = J1 axis.
        j1_origin = model.get_joint_pose("shoulder_pan_joint")[:3, 3]
        j1_axis = model.get_joint_pose("shoulder_pan_joint")[:3, 2]  # Assuming Z is the axis
        # Construct rotation matrix that maps (0,0,1) to j1_axis.
        # This is not unique, but we can use the standard method.
        # For UR10, j1_axis = (0,0,1), so rotation is identity.
        # For others, it's different.
        # Let's just use identity for now and see.
        T_true_root_world = np.eye(4)
        T_true_root_world[:3, 3] = j1_origin
        # We'll need a better way to get the rotation.
        # For now, let's assume it's identity.
    
    T_true_root_world_inv = np.linalg.inv(T_true_root_world)
    
    T_flange_true_root = T_true_root_world_inv @ T_flange_world
    
    print(f"T_flange_true_root translation: {T_flange_true_root[:3, 3]}")
    print()

    # 4. Solve in the true root frame.
    sols = solver.solve(T_flange_true_root)
    print(f"Solutions returned: {len(sols)}")
    
    best_pos_err = float('inf')
    best_rot_err = float('inf')
    
    for s in sols:
        # Compute FK in true root frame.
        # We need a way to get the flange pose in the true root frame.
        # We can use the model's forward kinematics and then transform.
        T_check_world = model.get_flange_pose(s.q)
        T_check_true_root = T_true_root_world_inv @ T_check_world
        
        pos_err = np.linalg.norm(T_check_true_root[:3, 3] - T_flange_true_root[:3, 3])
        rot_err = np.linalg.norm(T_check_true_root[:3, :3] - T_flange_true_root[:3, :3], ord='fro')
        
        best_pos_err = min(best_pos_err, pos_err)
        best_rot_err = min(best_rot_err, rot_err)
        
        print(f"  q={s.q.round(4)} pos_err={pos_err:.4f} rot_err={rot_err:.4f}")
    print(f"\n  BEST: pos_err={best_pos_err:.6f}, rot_err={best_rot_err:.6f}")
    print()

    T_rt = model.get_flange_pose(q_true)
    print("FK(q_true) == T_true?", np.allclose(T_rt, T_flange_world))
    print()


debug_one(
    "UR10",
    "assets/robots/ur_description/urdf/ur10_nominal.urdf",
    np.array([1.549, -2.6037, -2.5957, -5.4679, 3.543, 4.6682]),
)

debug_one(
    "FR5",
    "assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf",
    np.array([0.753, -2.8365, -2.3361, -4.2288, 1.7223, 2.2692]),
)

debug_one(
    "Elfin",
    "assets/robots/E15_Pro/urdf/E15_Pro.urdf",
    np.array([0.7741, -0.9738, -2.1565, -2.7326, 1.4435, 2.3329]),
)