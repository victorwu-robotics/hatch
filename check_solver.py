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
    print("Axis directions (true root frame):")
    for i, d in enumerate(solver.axis_dirs):
        print(f"  J{i+1}: {d}")
    print()

    # get_flange_pose returns the pose in the TRUE ROOT frame.
    T_true = model.get_flange_pose(q_true)

    P_flange = T_true[:3, 3]
    a_flange = T_true[:3, 2]
    P5 = P_flange - solver.flange_offset * a_flange
    print(f"P5 (true root frame): {P5}")
    print()

    sols = solver.solve(T_true)
    print(f"Solutions returned: {len(sols)}")

    best_pos_err = float('inf')
    best_rot_err = float('inf')
    best_q = None

    for s in sols:
        T_check = model.get_flange_pose(s.q)
        pos_err = np.linalg.norm(T_check[:3, 3] - T_true[:3, 3])
        rot_err = np.linalg.norm(T_check[:3, :3] - T_true[:3, :3], ord='fro')

        if pos_err < best_pos_err:
            best_pos_err = pos_err
            best_rot_err = rot_err
            best_q = s.q

        print(f"  q={s.q.round(4)} tags={s.tags} pos_err={pos_err:.4f} rot_err={rot_err:.4f}")

    print(f"\n  BEST: pos_err={best_pos_err:.6f}, rot_err={best_rot_err:.6f}")
    if best_q is not None:
        print(f"  BEST q: {best_q.round(4)}")
        print(f"  TRUE q: {q_true.round(4)}")
    print()

    T_rt = model.get_flange_pose(q_true)
    print("FK(q_true) == T_true?", np.allclose(T_rt, T_true))
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