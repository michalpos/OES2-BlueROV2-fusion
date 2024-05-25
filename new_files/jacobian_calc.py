import sympy as sp
from sympy import init_printing

# Define state variables
x = sp.symbols('self.ekf.x[0:12]')  # State vector x1 to x12
input = sp.symbols('self.input[0:6]')  # inputs i1 to i6
m, I_xx, I_yy, I_zz = sp.symbols('self.m self.I_xx self.I_yy self.I_zz')  # Mass and inertia
X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot = sp.symbols('self.X_udot self.Y_vdot self.Z_wdot self.K_pdot self.M_qdot self.N_rdot')  # Added mass
X_u, X_uu, Y_v, Y_vv, Z_w, Z_ww, K_p, K_pp, M_q, M_qq, N_r, N_rr = sp.symbols('self.X_u self.X_uu self.Y_v self.Y_vv self.Z_w self.Z_ww self.K_p self.K_pp self.M_q self.M_qq self.N_r self.N_rr')  # Damping coefficients

#matrices
M_rb = sp.diag(m, m, m, I_xx, I_yy, I_zz) # Rigid body mass matrix
M_a = sp.diag(X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot) # Added mass matrix
M = M_rb + M_a # Total mass matrix

C_rb = sp.Matrix([ # RB coriolis matrix
    [0, 0, 0, 0, m*x[8], -m*x[7]],
    [0, 0, 0, -m*x[8], 0, m*x[6]],
    [0, 0, 0, m*x[7], -m*x[6], 0],
    [0, m*x[8], -m*x[7], 0, -I_zz*x[11], -I_yy*x[10]],
    [-m*x[8], 0, m*x[6], I_zz*x[11], 0, I_xx*x[9]],
    [m*x[7], -m*x[6], 0, I_yy*x[10], -I_xx*x[9], 0]
])
C_a = sp.Matrix([ #Added mass coriolis matrix
    [0, 0, 0, 0, -Z_wdot*x[8], Y_vdot*x[7]],
    [0, 0, 0, Z_wdot*x[8], 0, -X_udot*x[6]],
    [0, 0, 0, -Y_vdot*x[7], X_udot*x[6], 0],
    [0, -Z_wdot*x[8], Y_vdot*x[7], 0, N_rdot*x[11], M_qdot*x[10]],
    [Z_wdot*x[8], 0, -X_udot*x[6], -N_rdot*x[11], 0, -K_pdot*x[9]],
    [-Y_vdot*x[7], X_udot*x[6], 0, -M_qdot*x[10], K_pdot*x[9], 0]
])
C = C_rb + C_a #total coriolis matrix

# D = sp.diag(X_u + X_uu * sp.Abs(x[6]), Y_v + Y_vv * sp.Abs(x[7]), Z_w + Z_ww * sp.Abs(x[8]),
#             K_p + K_pp * sp.Abs(x[9]), M_q + M_qq * sp.Abs(x[10]), N_r + N_rr * sp.Abs(x[11]))
D = sp.diag(X_u + X_uu * x[6], Y_v + Y_vv * x[7], Z_w + Z_ww * x[8], #let go of abs for simplicity, add it later!
            K_p + K_pp * x[9], M_q + M_qq * x[10], N_r + N_rr * x[11])

# Create a 12x12 zero matrix as the base
innovation = sp.zeros(12, 12)
# Top-right 6x6 with diagonal ones
for i in range(6):
    innovation[i, i + 6] = 1
bottom_right_block = M.inv() * (sp.Matrix(input) - (C+D) * sp.Matrix(x[6:12]))
sp.pprint(bottom_right_block)

jacobian = bottom_right_block.jacobian(sp.Matrix(x))
print(jacobian)
# # Place the bottom-right block into the innovation matrix
# for i in range(6):
#     for j in range(6):
#         innovation[i + 6, j + 6] = bottom_right_block[i, j]
#
# sp.pprint(innovation)
# # Compute the Jacobian of the Coriolis force vector
# innovation_jacobian = innovation.jacobian(sp.Matrix(x))  # Compute the Jacobian with respect to the full state vector
# print("Jacobian of the non linear system:")
# sp.pprint(innovation_jacobian)