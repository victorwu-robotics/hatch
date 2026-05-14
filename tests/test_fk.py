import numpy as np
import sympy as sp

# Let's derive the analytical solution symbolically first
print("Deriving analytical solution...")

# Define symbols
θ1, θ2, θ3 = sp.symbols('θ1 θ2 θ3')
d1, a2, d4 = sp.symbols('d1 a2 d4')

# DH offsets
θ2_DH = θ2 + sp.pi/2
θ3_DH = θ3 + sp.pi/2

# T_01
T_01 = sp.Matrix([
    [sp.cos(θ1), 0, sp.sin(θ1), 0],
    [sp.sin(θ1), 0, -sp.cos(θ1), 0],
    [0, 1, 0, d1],
    [0, 0, 0, 1]
])

# T_12
T_12 = sp.Matrix([
    [sp.cos(θ2_DH), 0, sp.sin(θ2_DH), a2*sp.cos(θ2_DH)],
    [sp.sin(θ2_DH), 0, -sp.cos(θ2_DH), a2*sp.sin(θ2_DH)],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])

# T_23
T_23 = sp.Matrix([
    [sp.cos(θ3_DH), 0, sp.sin(θ3_DH), 0],
    [sp.sin(θ3_DH), 0, -sp.cos(θ3_DH), 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])

# Compute T_03
T_03 = T_01 * T_12 * T_23

# Wrist center in frame 3
wrist_in_3 = sp.Matrix([0, 0, d4, 1])

# Wrist center in base frame
wrist_in_base = T_03 * wrist_in_3

print("Wrist center equations:")
print(f"x = {sp.simplify(wrist_in_base[0])}")
print(f"y = {sp.simplify(wrist_in_base[1])}")
print(f"z = {sp.simplify(wrist_in_base[2])}")