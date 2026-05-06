#!/usr/bin/env python3
"""
Verification of standard D‑H parameters for Han's E15‑Pro (spherical wrist).

Uses the standard Denavit‑Hartenberg convention:
    T = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)

This matches the new spherical wrist solver (spherical_wrist_ik.py).
"""

import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt


# =====================================================================
# Standard D‑H parameters for Han's E15‑Pro (TCP at wrist centre, d6=0)
# =====================================================================
dh_params = [
    {'a': 0.0,    'd': 0.262, 'alpha':  pi/2},   # Joint 1
    {'a': -0.73,  'd': 0.0,   'alpha':  pi},     # Joint 2 (a2 = -d_se)
    {'a': 0.0,    'd': 0.0,   'alpha':  pi/2},   # Joint 3
    {'a': 0.0,    'd': 0.57,  'alpha': -pi/2},   # Joint 4
    {'a': 0.0,    'd': 0.0,   'alpha':  pi/2},   # Joint 5
    {'a': 0.0,    'd': 0.0,   'alpha':  0.0},     # Joint 6 (TCP at wrist)
]

# Theta offsets (zero configuration angles)
theta_offset = [0.0, -pi/2, -pi/2, 0.0, 0.0, 0.0]


def dh_transform_standard(a, d, alpha, theta):
    """Standard D‑H transformation matrix."""
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Compute FK using standard D‑H parameters."""
    T = np.eye(4)
    for i, theta in enumerate(q):
        T = T @ dh_transform_standard(
            dh_params[i]['a'],
            dh_params[i]['d'],
            dh_params[i]['alpha'],
            theta
        )
    return T


# =====================================================================
# Test FK at zero configuration (all joints at theta_offset)
# =====================================================================
q_zero = np.array(theta_offset)
T_zero = forward_kinematics(q_zero)

print("FK at zero configuration:")
print(T_zero)
print(f"TCP position: ({T_zero[0,3]:.4f}, {T_zero[1,3]:.4f}, {T_zero[2,3]:.4f})")
print(f"TCP Z-axis:  ({T_zero[0,2]:.4f}, {T_zero[1,2]:.4f}, {T_zero[2,2]:.4f})")
print("Expected:     (0.0000, 0.0000, 1.5620), Z-axis pointing up\n")

assert np.allclose(T_zero[:3, 3], [0.0, 0.0, 1.562], atol=1e-3), \
    "FK at zero does not match expected TCP position!"
print("✅ FK at zero matches expected (0, 0, 1.562)\n")


# =====================================================================
# IK test with verbose debug
# =====================================================================
def inverse_kinematics_debug(T_target):
    """IK with detailed debug output."""
    d1 = dh_params[0]['d']
    a2 = dh_params[1]['a']
    d4 = dh_params[3]['d']
    d6 = dh_params[5]['d']

    print(f"[IK DEBUG] Parameters: d1={d1:.4f}, a2={a2:.4f}, d4={d4:.4f}, d6={d6:.4f}")

    # Wrist centre
    wrist_centre = T_target @ np.array([0, 0, -d6, 1])
    x_wc, y_wc, z_wc = wrist_centre[0], wrist_centre[1], wrist_centre[2]
    print(f"[IK DEBUG] Wrist centre: ({x_wc:.4f}, {y_wc:.4f}, {z_wc:.4f})")

    solutions = []
    r_xy = sqrt(x_wc**2 + y_wc**2)
    print(f"[IK DEBUG] r_xy={r_xy:.6f}")

    # Theta1 candidates
    if r_xy < 1e-4:
        t1_candidates = [theta_offset[0]]
        print(f"[IK DEBUG] r_xy near zero → t1 candidates: {t1_candidates}")
    elif r_xy < abs(d1) - 1e-6:
        print(f"[IK DEBUG] r_xy < d1 → UNREACHABLE")
        return []
    else:
        phi = atan2(y_wc, x_wc)
        offset = acos(np.clip(d1 / r_xy, -1.0, 1.0))
        t1_candidates = [
            phi + offset - pi/2 + theta_offset[0],
            phi - offset - pi/2 + theta_offset[0]
        ]
        print(f"[IK DEBUG] phi={phi:.4f}, offset={offset:.4f} → t1 candidates: {t1_candidates}")

    for t1 in t1_candidates:
        print(f"\n[IK DEBUG] --- Trying t1={t1:.4f} rad ({np.degrees(t1):.1f}°) ---")
        
        T_01 = dh_transform_standard(dh_params[0]['a'], dh_params[0]['d'],
                                     dh_params[0]['alpha'], t1)
        wc_in_1 = np.linalg.inv(T_01) @ wrist_centre
        x1, y1, z1 = wc_in_1[0], wc_in_1[1], wc_in_1[2]
        print(f"[IK DEBUG] wc in frame 1: ({x1:.4f}, {y1:.4f}, {z1:.4f})")

        r = sqrt(x1**2 + y1**2 + z1**2)
        print(f"[IK DEBUG] r={r:.4f}, a2+d4={abs(a2)+d4:.4f}, |a2-d4|={abs(abs(a2)-d4):.4f}")

        if r > abs(a2) + d4 or r < abs(abs(a2) - d4):
            print(f"[IK DEBUG] r out of range → SKIP")
            continue

        cos_t3 = (r**2 - a2**2 - d4**2) / (2 * a2 * d4)
        cos_t3 = np.clip(cos_t3, -1.0, 1.0)
        print(f"[IK DEBUG] cos_t3={cos_t3:.4f}")

        for t3_raw in [acos(cos_t3), -acos(cos_t3)]:
            t3 = t3_raw + theta_offset[2]
            print(f"\n[IK DEBUG]   --- t3_raw={t3_raw:.4f}, t3={t3:.4f} rad ({np.degrees(t3):.1f}°) ---")

            beta = atan2(z1, x1)
            gamma = atan2(d4 * sin(t3_raw), abs(a2) + d4 * cos(t3_raw))
            t2 = beta - gamma + theta_offset[1]
            print(f"[IK DEBUG]   beta={beta:.4f}, gamma={gamma:.4f} → t2={t2:.4f} rad ({np.degrees(t2):.1f}°)")

            q123 = [t1, t2, t3]
            T_03 = np.eye(4)
            for i in range(3):
                T_03 = T_03 @ dh_transform_standard(
                    dh_params[i]['a'], dh_params[i]['d'],
                    dh_params[i]['alpha'], q123[i]
                )
            T_36 = np.linalg.inv(T_03) @ T_target
            print(f"[IK DEBUG]   T_36 translation: ({T_36[0,3]:.4f}, {T_36[1,3]:.4f}, {T_36[2,3]:.4f})")

            # Wrist solutions
            R = T_36[:3, :3]
            print(f"[IK DEBUG]   T_36 rotation:\n{R}")
            
            cos_t5 = np.clip(R[1, 2], -1.0, 1.0)
            print(f"[IK DEBUG]   cos_t5={cos_t5:.4f}")

            for t5_raw in [acos(cos_t5), -acos(cos_t5)]:
                t5 = t5_raw + theta_offset[4]
                print(f"\n[IK DEBUG]     --- t5_raw={t5_raw:.4f}, t5={t5:.4f} rad ({np.degrees(t5):.1f}°) ---")

                if abs(sin(t5_raw)) < 1e-6:
                    t4 = 0.0 + theta_offset[3]
                    t6 = atan2(R[2, 0], R[0, 0]) + theta_offset[5]
                    print(f"[IK DEBUG]     singularity: t4={t4:.4f}, t6={t6:.4f}")
                else:
                    t4 = atan2(R[2, 2]/sin(t5_raw), -R[0, 2]/sin(t5_raw)) + theta_offset[3]
                    t6 = atan2(-R[1, 0]/sin(t5_raw), R[1, 1]/sin(t5_raw)) + theta_offset[5]
                    print(f"[IK DEBUG]     t4_raw={atan2(R[2, 2]/sin(t5_raw), -R[0, 2]/sin(t5_raw)):.4f}, t4={t4:.4f}")
                    print(f"[IK DEBUG]     t6_raw={atan2(-R[1, 0]/sin(t5_raw), R[1, 1]/sin(t5_raw)):.4f}, t6={t6:.4f}")

                q = np.array([t1, t2, t3, t4, t5, t6])
                T_check = forward_kinematics(q)
                pos_err = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
                print(f"[IK DEBUG]     Full q: [{', '.join(f'{a:.4f}' for a in q)}]")
                print(f"[IK DEBUG]     FK check pos: ({T_check[0,3]:.4f}, {T_check[1,3]:.4f}, {T_check[2,3]:.4f})")
                print(f"[IK DEBUG]     Position error: {pos_err:.6f} m")

                if pos_err < 1e-3:
                    solutions.append(q)
                    print(f"[IK DEBUG]     ✅ SOLUTION ACCEPTED")
                else:
                    print(f"[IK DEBUG]     ❌ REJECTED (error too large)")

    return solutions


print("Testing IK for the zero pose...")
sol = inverse_kinematics_debug(T_zero)
if sol:
    print(f"\n✅ Found {len(sol)} solution(s).")
else:
    print(f"\n❌ No IK solution found for zero pose")