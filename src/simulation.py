import numpy as np
import matplotlib.pyplot as plt
import control as ct

from setup import A, B, C, D
from blocks import setpoint_block, add_actuators, add_pid_controllers, add_sensors, close_loop

CV_ARRAY = ["C_A", "T"]
MV_ARRAY = ["I_A", "T_c"]

plant = ct.ss(A, B, C, D, inputs=MV_ARRAY + [f"{cv}_disturb" for cv in CV_ARRAY], outputs=CV_ARRAY, name="plant")
set_points = setpoint_block([f"{cv}_sp" for cv in CV_ARRAY])

comp_pid_gains = [500, 50, 0]
temp_pid_gains = [500, 50, 0]

controllers = add_pid_controllers(CV_ARRAY, MV_ARRAY, [comp_pid_gains, temp_pid_gains])
actuators = add_actuators(MV_ARRAY, [1, 1])
sensors = add_sensors(CV_ARRAY, [0.25, 0.25])

blocks = [set_points, controllers, actuators, plant, sensors]

closed_loop = close_loop(blocks, CV_ARRAY, MV_ARRAY)

time = np.linspace(0, 300, 600)
u_input = np.zeros((4, time.size))

# Step CV setpoint changes
u_input[0, time >= 100] = 0
u_input[1, time >= 100] = 20

# Step CV disturbances
u_input[2, time >= 200] = 1.5
u_input[3, time >= 200] = 0

time, y_output = ct.forced_response(closed_loop, T=time, U=u_input)

# C_A plot
plt.figure()
plt.plot(time, y_output[0], label="C_A")
plt.plot(time, u_input[0], "r--", label="C_A setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Concentration Deviation (kmol/m3)")
plt.title("CSTR Concentration Response")
plt.grid()
plt.legend()

# T plot
plt.figure()
plt.plot(time, y_output[1], label="T", color="orange")
plt.plot(time, u_input[1], "g--", label="T setpoint")
plt.xlabel("Time (s)")
plt.ylabel("Temperature Deviation (K)")
plt.title("CSTR Temperature Response")
plt.grid()
plt.legend()

plt.show()