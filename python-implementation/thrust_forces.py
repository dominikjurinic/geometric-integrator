import numpy as np 

def thrust_forces(motor,rpm):
    F_t = -motor.k_t * np.power(rpm,2)
    return F_t