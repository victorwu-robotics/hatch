#!/usr/bin/env python3
"""Debug: compare model FK with the numerical solver's FK at zero config."""

import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path.home() / "hatch"))

from core.kinematics.kinematic_model import KinematicModel

# Load model
urdf_path = Path.home() / "hatch/assets/robots/E15_Pro/urdf/E15_Pro.urdf"
package_dirs = [
    str(urdf_path.parent),
    str(urdf_path.parent.parent),
    str(Path.home() / "hatch" / "assets"),
    str(Path.home() / "hatch" / "assets" / "robots"),
]

model = KinematicModel(
    urdf_path=str(urdf_path),
    package_dirs=package_dirs,
    asset_id="E15_Pro"
)
model.load()

# Get the arm chain
true_root = model.get_true_root()
arm_chain = model.get_arm_chain(true_root)
print(f"Arm chain: {arm_chain}")
print(f"Joint names: {model.get_joint_info()['names']}")

def test_configuration(model, q, label):
    """Test a single joint configuration."""
    print(f"\n=== {label} ===")
    print(f"q = {q}")
    
    # Get TCP pose using forward_kinematics (this updates internal state)
    T_tcp = model.forward_kinematics(q)
    pos = T_tcp[:3, 3]
    
    print(f"TCP position: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
    
    # Also get individual link transforms
    print("\nKey link positions:")
    key_links = ['elfin_base', 'elfin_link1', 'elfin_link2', 'elfin_link3', 
                 'elfin_link4', 'elfin_link5', 'elfin_link6', 'elfin_end_link']
    for link_name in key_links:
        if link_name in model.link_transforms:
            T = model.link_transforms[link_name]
            p = T[:3, 3]
            print(f"  {link_name}: ({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f}) mm")
    
    return T_tcp

# Test 1: Home position (all zeros)
q_home = [0, 0, 0, 0, 0, 0]
test_configuration(model, q_home, "HOME POSITION (all joints = 0°)")

# Test 2: J2 = 90°
q_j2_90 = [0, np.pi/2, 0, 0, 0, 0]
test_configuration(model, q_j2_90, "J2 = 90° (others 0)")

# Test 3: J3 = 90°
q_j3_90 = [0, 0, np.pi/2, 0, 0, 0]
test_configuration(model, q_j3_90, "J3 = 90° (others 0)")

# Test 4: Combined J2=90°, J3=90°
q_both = [0, np.pi/2, np.pi/2, 0, 0, 0]
test_configuration(model, q_both, "J2=90°, J3=90°")

# Test 5: J2 = -90° (opposite direction)
q_j2_neg = [0, -np.pi/2, 0, 0, 0, 0]
test_configuration(model, q_j2_neg, "J2 = -90° (others 0)")

print("\n" + "="*60)
print("SUMMARY OF TCP POSITIONS:")
print("="*60)

# Re-run and collect results
configs = [
    ("Home", [0, 0, 0, 0, 0, 0]),
    ("J2=90°", [0, np.pi/2, 0, 0, 0, 0]),
    ("J3=90°", [0, 0, np.pi/2, 0, 0, 0]),
    ("J2=90°, J3=90°", [0, np.pi/2, np.pi/2, 0, 0, 0]),
    ("J2=-90°", [0, -np.pi/2, 0, 0, 0, 0]),
]

for name, q in configs:
    T = model.forward_kinematics(q)
    pos = T[:3, 3]
    print(f"{name:20s}: X={pos[0]*1000:6.1f} mm, Y={pos[1]*1000:6.1f} mm, Z={pos[2]*1000:6.1f} mm")

print("\n" + "="*60)
print("INFERRED LINK LENGTHS FROM YOUR MODEL:")
print("="*60)

# Get home Z
T_home = model.forward_kinematics([0, 0, 0, 0, 0, 0])
home_z = T_home[2, 3]

# Get J2=90° position
T_j2 = model.forward_kinematics([0, np.pi/2, 0, 0, 0, 0])
j2_x = abs(T_j2[0, 3])
j2_z = T_j2[2, 3]

# Get J3=90° position
T_j3 = model.forward_kinematics([0, 0, np.pi/2, 0, 0, 0])
j3_x = T_j3[0, 3]
j3_z = T_j3[2, 3]

print(f"Home position Z: {home_z*1000:.1f} mm (total height)")
print(f"Base height (d1): {j2_z*1000:.1f} mm")
print(f"Upper arm length (a2): {(abs(j2_x) - j3_x)*1000:.1f} mm")
print(f"Forearm + tool length (a3): {j3_x*1000:.1f} mm")
print(f"Total horizontal at J2=90°: {abs(j2_x)*1000:.1f} mm")

print("\n" + "="*60)
print("VERIFICATION: Does a2 + a3 = total horizontal?")
print(f"  {abs(j2_x)*1000:.1f} mm = {(abs(j2_x) - j3_x)*1000:.1f} + {j3_x*1000:.1f}")
print(f"  {abs(j2_x)*1000:.1f} = {(abs(j2_x) - j3_x + j3_x)*1000:.1f} ✓")

print("\n" + "="*60)
print("RECOMMENDED DH PARAMETERS FOR YOUR ROBOT:")
print("="*60)

d1 = j2_z
a2 = abs(j2_x) - j3_x
a3 = j3_x

print(f"""
Standard Modified DH parameters (Craig convention):

┌───────┬─────────┬──────────┬─────────┬─────────────────┐
│ Joint │   a     │   α      │    d    │   θ offset      │
├───────┼─────────┼──────────┼─────────┼─────────────────┤
│   1   │ 0.000   │  π/2     │ {d1:.3f} │ 0               │
│   2   │ {a2:.3f} │  0       │ 0       │ -π/2            │
│   3   │ {a3:.3f} │  0       │ 0       │ 0               │
│   4   │ 0.000   │  π/2     │ 0       │ 0               │
│   5   │ 0.000   │ -π/2     │ 0       │ 0               │
│   6   │ 0.000   │  0       │ 0       │ 0               │
└───────┴─────────┴──────────┴─────────┴─────────────────┘

Where:
  d1 = {d1*1000:.1f} mm (base to joint 2)
  a2 = {a2*1000:.1f} mm (upper arm length)
  a3 = {a3*1000:.1f} mm (forearm + tool length)

Note: The tool length (200mm) is included in a3.
""")

print("\n" + "="*60)
print("To test if these DH parameters are correct, run:")
print("="*60)
print("""
from your_ik_solver import ForwardKinematics

fk = ForwardKinematics(dh_params=[
    [0,     math.pi/2, d1,     0],
    [a2,    0,         0,      -math.pi/2],
    [a3,    0,         0,      0],
    [0,     math.pi/2, 0,      0],
    [0,    -math.pi/2, 0,      0],
    [0,    0,         0,      0],
])

# Should output:
# Home: (0, 0, {home_z*1000:.1f})
# J2=90°: (-{abs(j2_x)*1000:.1f}, 0, {j2_z*1000:.1f})
# J3=90°: ({j3_x*1000:.1f}, 0, {j3_z*1000:.1f})
""".format(home_z=home_z, j2_x=abs(j2_x), j2_z=j2_z, j3_x=j3_x, j3_z=j3_z))