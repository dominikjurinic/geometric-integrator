import numpy as np 


def om2rpm(omega):
    rpm_out = (omega)/(2*np.pi) * 60
    return rpm_out

