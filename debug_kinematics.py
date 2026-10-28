import numpy as np
from core.kinematics.kinematic_model import KinematicModel

model = KinematicModel(urdf_path="assets/robots/ur_description/urdf/ur10_nominal.urdf").load()
q_true = np.array([1.549, -2.6037, -2.5957, -5.4679, 3.543, 4.6682])

# Compute FK at q_true; this returns link transforms in the *world* frame.
transforms = model.compute_fk(q_true)
T_root = transforms[model.get_true_root()]
T_root_inv = np.linalg.inv(T_root)

def world_to_root(link):
    T = transforms[link]
    p = T_root_inv @ np.append(T[:3, 3], 1.0)
    return p[:3]

arm_chain = model.get_arm_chain(base_link_name=model.get_true_root())
print("Arm chain:", arm_chain)

# For each joint in the chain, get the joint's own frame origin.
# The child link of the joint is at the joint's origin (for revolute joints,
# the frame is placed at the joint's origin, rotated by q_i).
def joint_origin_in_root(joint_name):
    joint = model.joints[joint_name]
    parent_link = joint['parent']
    T_parent = transforms[parent_link]
    T_joint_local = model._compose_transform(joint['origin_xyz'], joint['origin_rpy'])
    T_joint_world = T_parent @ T_joint_local
    p_world = T_joint_world[:3, 3]
    p_root = (T_root_inv @ np.append(p_world, 1.0))[:3]
    return p_root

J1 = joint_origin_in_root('shoulder_pan_joint')
J2 = joint_origin_in_root('shoulder_lift_joint')
J3 = joint_origin_in_root('elbow_joint')
J4 = joint_origin_in_root('wrist_1_joint')
J5 = joint_origin_in_root('wrist_2_joint')
J6 = joint_origin_in_root('wrist_3_joint')

print(f"J1 = {J1}")
print(f"J2 = {J2}")
print(f"J3 = {J3}")
print(f"J4 = {J4}")
print(f"J5 = {J5}")
print(f"J6 = {J6}")
print()
print(f"|J3 - J2| = {np.linalg.norm(J3 - J2):.4f}")
print(f"|J5 - J3| = {np.linalg.norm(J5 - J3):.4f}")
print(f"|J5 - J2| = {np.linalg.norm(J5 - J2):.4f}")
print(f"q_true[2] = {q_true[2]:.4f}")

# Interior angle at J3.
v1 = J2 - J3
v2 = J5 - J3
cos_int = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
interior = np.arccos(cos_int)
print(f"interior = {interior:.4f}")
print(f"interior - q_true[2] = {interior - q_true[2]:.4f}")
print(f"interior + q_true[2] = {interior + q_true[2]:.4f}")

# Now check at q=0.
transforms0 = model.compute_fk(np.zeros(6))
T_root0 = transforms0[model.get_true_root()]
T_root0_inv = np.linalg.inv(T_root0)

def joint_origin_at_zero(joint_name):
    joint = model.joints[joint_name]
    T_parent = transforms0[joint['parent']]
    T_joint_local = model._compose_transform(joint['origin_xyz'], joint['origin_rpy'])
    T_joint_world = T_parent @ T_joint_local
    p_world = T_joint_world[:3, 3]
    return (T_root0_inv @ np.append(p_world, 1.0))[:3]

J2_0 = joint_origin_at_zero('shoulder_lift_joint')
J3_0 = joint_origin_at_zero('elbow_joint')
J5_0 = joint_origin_at_zero('wrist_2_joint')

v1_0 = J2_0 - J3_0
v2_0 = J5_0 - J3_0
interior_0 = np.arccos(np.dot(v1_0, v2_0) / (np.linalg.norm(v1_0) * np.linalg.norm(v2_0)))
print()
print(f"At q=0: J2={J2_0}, J3={J3_0}, J5={J5_0}")
print(f"|J3-J2| = {np.linalg.norm(J3_0 - J2_0):.4f}")
print(f"|J5-J3| = {np.linalg.norm(J5_0 - J3_0):.4f}")
print(f"interior at q=0 = {interior_0:.4f}")

# For each joint angle, compute |J5 - J2| and compare against the law-of-cosines prediction.
print()
print("Testing relation theta3 -> interior across q3 values:")
for q3 in [-3.0, -2.596, -2.0, -1.0, 0.0, 1.0, 2.0, 2.596, 3.0]:
    q_test = np.array([0.0, 0.0, q3, 0.0, 0.0, 0.0])
    T_test = model.compute_fk(q_test)
    T_root_t = T_test[model.get_true_root()]
    T_root_t_inv = np.linalg.inv(T_root_t)
    def j_at_test(name):
        j = model.joints[name]
        T_p = T_test[j['parent']]
        T_ll = model._compose_transform(j['origin_xyz'], j['origin_rpy'])
        p = (T_p @ T_ll)[:3, 3]
        return (T_root_t_inv @ np.append(p, 1.0))[:3]
    J2_t = j_at_test('shoulder_lift_joint')
    J3_t = j_at_test('elbow_joint')
    J5_t = j_at_test('wrist_2_joint')
    v1_t = J2_t - J3_t
    v2_t = J5_t - J3_t
    interior_t = np.arccos(np.dot(v1_t, v2_t) / (np.linalg.norm(v1_t) * np.linalg.norm(v2_t)))
    D = np.linalg.norm(J5_t - J2_t)
    # Solve for theta3 that the law-of-cosines would compute.
    a2, a3 = 0.612, 0.606457
    c3 = (D**2 - a2**2 - a3**2) / (2 * a2 * a3)
    theta3_lc = np.arccos(np.clip(c3, -1, 1))
    print(f"  q3={q3:+.4f}  D={D:.4f}  interior={interior_t:.4f}  theta3_lc={theta3_lc:.4f}")