import numpy as np 

from thrust_forces import thrust_forces
from skew import skew

def new_eul(aircraft,motor,F_t,T_t,T_d,omega,omegam,q):
    R = np.zeros((3,3,))
    R[0,0] = q[0]**2+q[1]**2-1/2
    R[0,1] = q[1]*q[2]-q[0]*q[3]
    R[0,2] = q[1]*q[3]+q[0]*q[2]
    R[1,0] = q[1]*q[2]+q[0]*q[3]
    R[1,1] = q[0]**2+q[2]**2-1/2
    R[1,2] = q[2]*q[3]-q[0]*q[1]
    R[2,0] = q[1]*q[3]-q[0]*q[2]
    R[2,1] = q[2]*q[3]+q[0]*q[1]
    R[2,2] = q[0]**2+q[3]**2-1/2
    R_new = 2 * R 
    vector = np.zeros((3,1))
    vector[2,0] = sum(F_t)
    dv = (np.add(np.matmul(R_new, vector), np.array([[0],[0],[aircraft.mass*9.80665]])))/aircraft.mass
    variable = T_t + T_d - np.matmul(np.matmul(skew(omega),aircraft.J),omega) - np.matmul(skew(omega),np.array([[0],[0],[motor.inertia * np.sum(np.multiply(omegam,motor.rot_dir))]]))
    domega = np.linalg.solve(aircraft.J,variable)
    return dv,domega






