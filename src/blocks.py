import numpy as np
import control as ct
from control.statesp import StateSpace


def pid_controller(pid_gains: list[float]) -> StateSpace:
    Kp, Ki, Kd = pid_gains
    s = ct.tf("s")
    
    return ct.ss(Kp + Ki / s + Kd * s)


def hardware_delay(tau_s: float) -> StateSpace:
    if tau_s == 0:
        transfer_function = ct.tf([1], [1])
    else:
        transfer_function = ct.tf([1], [tau_s, 1])

    return ct.ss(transfer_function)


def setpoint_block(cvs: list[str], block_name="set_points") -> StateSpace:
    return ct.ss([], [], [], np.eye(len(cvs)), inputs=cvs, outputs=cvs, name=block_name)


def add_sensors(cv_array: list[str], time_const_array: list[float]):
    sensor_list = [hardware_delay(time_const) for time_const in time_const_array]

    sensors = ct.append(*sensor_list)
    sensors.input_labels  = cv_array
    sensors.output_labels = [f"sense_{cv}" for cv in cv_array]
    sensors.name = "sensors"

    return sensors


def add_actuators(mv_array: list[str], time_const_array: list[float]) -> StateSpace:
    actuator_list = [hardware_delay(time_const) for time_const in time_const_array]

    actuators = ct.append(*actuator_list)
    actuators.input_labels  = [f"command_{mv}" for mv in mv_array]
    actuators.output_labels = [f"effect_{mv}" for mv in mv_array]
    actuators.name = "actuators"

    return actuators


def add_pid_controllers(cv_array: list[str], mv_array: list[str], pid_array: list[list[float]]) -> StateSpace:
    pid_controller_list = [pid_controller(kp_ki_kd) for kp_ki_kd in pid_array]

    pid_controllers = ct.append(*pid_controller_list)
    pid_controllers.input_labels  = [f"e_{cv}" for cv in cv_array]
    pid_controllers.output_labels = [f"command_{mv}" for mv in mv_array]
    pid_controllers.name = "pid"

    return pid_controllers


def close_loop(blocks, cv_array, mv_array) -> StateSpace:
    connection_list = []
    input_list = []
    output_list = []
    for mv, cv in zip(mv_array, cv_array):
        pid_input = [f"pid.e_{cv}", f"set_points.{cv}_sp", ("sensors", f"sense_{cv}", -1)]
        actuator_input = [f"actuators.command_{mv}", f"pid.command_{mv}"]
        plant_input = [f"plant.{mv}", f"actuators.effect_{mv}"]
        sensor_input = [f"sensors.{cv}", f"plant.{cv}"]

        connection_list.append(pid_input)
        connection_list.append(actuator_input)
        connection_list.append(plant_input)
        connection_list.append(sensor_input)

        output_list.append(f"plant.{cv}")

    input_list = [f"set_points.{cv}_sp" for cv in cv_array] + [f"plant.{cv}_disturb" for cv in cv_array]

    closed_loop = ct.interconnect(
        blocks,
        connections=connection_list,
        inplist=input_list,
        outlist=output_list
    )

    return closed_loop