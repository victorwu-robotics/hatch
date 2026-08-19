"""
Standalone script to print the DH geometry report for the Farino FR5.
Run from the Hatch project root directory.
"""
import sys
from pathlib import Path

# Add project root to path so imports work
sys.path.insert(0, str(Path(__file__).parent))

from kinematic_model import KinematicModel
from dh_geometry import DHGeometry

# ===== 1. Point to your FR5 URDF file =====
urdf_path = Path("/home/victor/hatch/assets/robots/frcobot_description/urdf/fr5_robot.urdf")  # <-- EDIT THIS

# ===== 2. Package directories for resolving package:// mesh paths =====
# The FR5 URDF references: package://frcobot_description/meshes/fr5/...
# So package_dirs must contain the folder that holds "frcobot_description/"
package_dirs = [
    str(urdf_path.parent),
    str(urdf_path.parent.parent),
    str(urdf_path.parent.parent.parent),
    # Add the directory that contains the "frcobot_description" folder:
    str(Path.home() / "hatch" / "assets" / "robots"),
    # str(Path.home() / "your_frcobot_workspace" / "src"),
]

# ===== 3. Construct the model (same as RobotManager.load_robot) =====
model = KinematicModel(
    urdf_path=str(urdf_path),
    package_dirs=package_dirs,
    transform_registry=None,   # Not needed for geometry analysis
    asset_id="fr5"
)
model.load()

# ===== 4. Run the DH geometry analysis =====
geo = DHGeometry(model)
chain = model.get_arm_chain(model.get_true_root())

print(f"True root: {model.get_true_root()}")
print(f"Arm chain: {chain}")
print(f"Number of joints: {len(chain)}")
print()
print(geo.report(chain))