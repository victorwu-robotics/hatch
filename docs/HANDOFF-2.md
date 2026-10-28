Here is a comprehensive Handoff document summarizing our journey, the corrections made, the breakthroughs achieved, and the exact state of the unconventional unified IK solver.

You can save this as `docs/HANDOFF.md` to bring a fresh AI up to speed in a new session.

***

# Hatch Handoff — Unified IK Solver (Unconventional Derivation)

**Purpose:** This file is the entry point for a fresh session. It explains where the project is, what was decided, and exactly where the Unified IK derivation got stuck. Read this carefully before touching any code.

---

## 1. The Core Philosophy: AI Bias vs. Unconventional Geometry

This session was a battle against "conventional bias." The AI repeatedly tried to steer the derivation back toward standard DH (Denavit-Hartenberg) parameters and UR textbook formulas. The user successfully forced the derivation to remain purely geometric, treating spherical and offset wrists as the same thing (a wrist circle tangency).

**The Geometric Truth (The User's Intuition):**
1. The arm plane (J1, J2, J3) must be **tangent** to the wrist-offset circle.
2. The tangency radius is the perpendicular distance from the wrist center **P5** to the arm plane, called **`d_plane`**.
3. **`d_plane` is NOT the same as `wrist_offset`** for UR/FR arms. It is `d4` for UR10 (0.1639), `wrist_offset` for FR5 (0.1021), and `0` for Elfin.
4. J4 is found by moving from P5 along the J5 axis (offset `d5`).
5. The 2-link chain is `J2 → J3 → J4`, using `a2` and `a3 = axis_gap(2)`.

## 2. Current Code State (`unified_ik_solver.py`)

We have written a solver that uses this geometry. It works **perfectly for the Elfin** on `theta1`, `theta2`, and `theta3`. It extracts the exact true joint angles for those three.

**What works:**
- The `_step2_shoulder` uses `d_plane` for the tangency radius.
- The `_step4_arm_chain` correctly handles `a3 = 0` (spherical) by using `forearm_length()`, and `a3 > 0` (offset) by using `axis_gap(2)`.
- The code uses `theta2_sign` to correctly handle the Elfin's antiparallel J2/J3 axes.

**Where it breaks:**
- **`theta5` (Wrist Angle):** The formula for `theta5` assumes `cos(theta5)` is the projection of the flange onto the arm plane normal. This is true for UR/FR, but false for Elfin because its J5 axis is Y (not Z).
- **`J4` Offset:** To solve the `J2 → J3 → J4` chain, we need the 3D coordinates of J4. We tried `J4 = P5 + d5 * Z4`, where `Z4` is the J5 axis. But calculating `Z4` requires `theta6`, which requires `theta4`, which creates a circular dependency.

## 3. The Unresolved Math (Where the next AI must focus)

To make the unconventional solver work without falling back to DH tables, we must solve these two specific geometric problems:

### Problem A: The Correct `theta5` Formula
For UR10 and FR5, `c = (P_flange · n_hat - d_plane) / d6` works. For Elfin, the normal `n_hat` is flipped, and the projection is a sine wave, not a cosine.
**The Fix:** We must project `P_flange` onto the **actual J5 axis direction**, not just the arm plane normal. The correct unified formula is:
`cos(theta5) = a_flange · J5_axis`.
Since `J5_axis` is `R04 @ (0,0,1)`, and `R04 = R_flange @ R56^T @ R45^T`, we can extract `theta5` directly from the rotation matrix without needing `theta2`, `theta3`, or `theta4`. (Hint: Look at the `ur_kinematics` standard formula for `theta5`, but adapt it to the URDF's raw axes).

### Problem B: The `J4` Position
We cannot use `J4 = P5 + d5 * Z4` because `Z4` (the J5 axis) depends on `theta6`, which we need to find `J4`.
**The Fix:** The standard UR IK uses a mathematical trick: it computes the in-plane coordinates `A` and `B` directly from `P5`, `d4`, `d5`, and `d6`. 
Specifically, `A = P05_x * cos(theta1) + P05_y * sin(theta1) - d4` and `B = P05_z - d1 - d5 * cos(theta5)`. 
If you can derive the exact geometric projection of J4 onto the arm plane using `theta1` and `theta5`, you bypass the circular dependency.

## 4. Test Results Status

*   **Elfin:** `theta2` and `theta3` match perfectly. The residual is `0.055` instead of `0.0`. The error is entirely in `theta5` and `theta6`.
*   **UR10 and FR5:** `theta2` and `theta3` are wrong. This is because the solver used `B = P5_z - d1` instead of the true projection of J4. The `d5` offset along the J5 axis shifted the `Z` coordinate, flipping the sign of `theta2`.

## 5. What the Next Session Must Do

1. **Do NOT use DH tables.** The user explicitly forbids standard UR IK. The goal is a pure geometric derivation.
2. **Fix `theta5`:** Derive the exact formula for `theta5` using the flange orientation and the URDF's J5 axis, bypassing the circular dependency.
3. **Fix `J4`:** Derive the exact in-plane coordinates `A` and `B` for the `J2 → J3 → J4` chain, accounting for the `d5` offset along the J5 axis. Once `theta5` is correct, `A` and `B` can be computed without needing `theta4`.
4. **Run `check_solver.py`:** The goal is `max residual < 1e-6` for UR10, FR5, and Elfin.

**The user is a Stubborn Student. They want to see the geometric truth, not just a working algorithm. Trust their intuition: the arm plane is tangent to the wrist circle.**