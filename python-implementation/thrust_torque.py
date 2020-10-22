import numpy as np 

from thrust_forces import thrust_forces

def thrust_torque(motor, aircraft, rpm):
    T_t = np.matmul(aircraft.matrix_FT, thrust_forces(motor, rpm)) * aircraft.lev_arm
    return T_t