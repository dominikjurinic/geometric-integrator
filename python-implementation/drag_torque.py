import numpy as np

def drag_torque(motor, rpm): 
    T_d = np.zeros((3,1))
    T_d[2,0] = np.sum(motor.k_d * np.multiply(rpm,np.multiply((np.absolute(rpm)),motor.rot_dir)))
    return T_d

