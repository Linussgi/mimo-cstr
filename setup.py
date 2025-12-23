import numpy as np

# System parameters
HEAT_CAP = 4.0            # kJ/(kg K)
VOL_FR = 3e-3             # m^3/s
SYS_VOL = 1.0             # m^3
DENSITY = 1000            # kg/m^3
U = 1                     # kJ/(s m^2 K)
A = 5                     # m^2
REAC_ENTHALPY = -24000    # kJ/kmol
E_A = 40000               # kJ/kmol
K_ARR = 10000             # 1/s
R = 8.314                 # kJ/(kmol K)

# Steady state conditions
T0  = 350                 # K
CA0 = 1                   # kmol/m^3

tau = SYS_VOL / VOL_FR
sens_heat  = -REAC_ENTHALPY / (DENSITY * HEAT_CAP)
UA_rhoCpV = U * A / (DENSITY * HEAT_CAP * SYS_VOL)
rate_const = K_ARR * (np.exp(-E_A / (R * T0)))

a11 = - (1 / tau + rate_const)
a12 = - (E_A / (R * T0**2) * CA0 * rate_const)
a21 = - sens_heat * rate_const
a22 = - sens_heat * rate_const * CA0 * (E_A / (R * T0**2)) - 1/tau - UA_rhoCpV
b1  = 1 / tau
b2  = UA_rhoCpV

cv_mat = np.array([[a11, a12], [a21, a22]])    # Differential state space matrix
input_mat = np.array([[b1, 0], [0,  b2]])      # Input MVs matrix
disturb_mat = np.array([[1, 0], [0, 1]])       # Input disturbances matrix
state_mat = np.array([[1, 0], [0, 1]])         # Output CVs matrix

# Feedfoward matrix
feedforward_mat = np.zeros((2, 4))

# For instantaneous disturbances in CVs
feedforward_mat[0, 2] = 1.0
feedforward_mat[1, 3] = 1.0 

A = cv_mat
B = np.hstack((input_mat, disturb_mat))
C = state_mat
D = feedforward_mat