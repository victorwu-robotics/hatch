from core.kinematics.kinematic_model import KinematicModel
from core.kinematics.geometric_extraction import extract_arm_geometry

for name, path in [
    ("UR10",   "assets/robots/ur_description/urdf/ur10_nominal.urdf"),
    ("FR5",    "assets/robots/Farino/frcobot_description/urdf/fr5v6.urdf"),
    ("Elfin",  "assets/robots/E15_Pro/urdf/E15_Pro.urdf"),
]:
    model = KinematicModel(path)
    geom = extract_arm_geometry(model)
    print(f"\n=== {name} ===")
    print(geom.describe())