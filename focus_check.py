from core.kinematics.kinematic_model import KinematicModel
import numpy as np

model = KinematicModel(
    urdf_path="assets/robots/ur_description/urdf/ur10_nominal.urdf"
).load()

print("true root:   ", model.get_true_root())
print("tool mount:  ", model.tool_mount_link)
print("flange:      ", model.get_flange())
print("match:       ", model.get_flange() == model.tool_mount_link)

# Flange pose at zero should be the pose of tool_mount_link in true-root frame
q0 = np.zeros(6)
T_flange = model.get_flange_pose(q0)
print("flange pose:\n", T_flange.round(6))