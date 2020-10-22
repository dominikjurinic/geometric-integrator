import numpy as np 

from dexp import dexp 
from drag_torque import drag_torque
from exp_S3 import exp_S3
from lie_bracket import lie_bracket
from new_eul import new_eul
from om2rpm import om2rpm
from omegamfun import omegamfun
from quat_rot import quat_rot
from quat_vect import quat_vect 
from rk2cl import rk2cl
from rk2geom import rk2geom
from rk4cl import rk4cl
#from rk4geom import rk4geom
from skew import skew
from SO3_to_R3 import SO3_to_R3
from thrust_forces import thrust_forces
from thrust_torque import thrust_torque



class Motor:
    def __init__(self, k_d, k_t,rot_dir,inertia):
        self.k_d = k_d
        self.k_t = k_t 
        self.rot_dir = rot_dir
        self.inertia = inertia

class Aircraft:
    def __init__(self,mass,J,lev_arm,matrix_FT):
        self.mass = mass
        self.J = J          #np.diag([0.00365,0.00368,0.00703])
        self.lev_arm = lev_arm      
        self.matrix_FT = matrix_FT


aircraftMass = 0.5
aircraftJ = np.diag([0.00365,0.00368,0.00703])
aircraftLev_Arm = 0.17
aircraftMatrix_FT = (1/2) * np.array([[np.sqrt(2),np.sqrt(2),-np.sqrt(2),-np.sqrt(2)],[-np.sqrt(2),np.sqrt(2),np.sqrt(2),-np.sqrt(2)],[0,0,0,0]])

motor_k_t =6.11e-8      
motor_k_d = 1.5e-9
motor_rot_dir = np.array([[1],[-1],[1],[-1]])
motor_inertia = (1/2) * 0.011 * (8*2.54/100)**2



def rk4geom(t, h, aircraft, motor, omega, v, q, p, type):
    L = np.zeros((3,4))
    L[0,0] = -q[1]
    L[0,1] =  q[0]
    L[0,2] =  q[3]
    L[0,3] = -q[2]
    L[1,0] = -q[2]
    L[1,1] = -q[3]
    L[1,2] =  q[0]
    L[1,3] =  q[1]
    L[2,0] = -q[3]
    L[2,1] =  q[2]
    L[2,2] = -q[1]
    L[2,3] =  q[0]
    appendedMatrix = np.append(q,np.transpose(L), axis = 1)  
    omegam = omegamfun(t,type)
    (vk1, k1) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega, omegam, q)
    uk1 =  dexp(omega, np.zeros((3,1)))
    pk1 = v
    
    omegam = omegamfun(t+1/2*h,type)
    (vk2, k2) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)),thrust_torque(motor,aircraft,om2rpm(omegam)), drag_torque(motor,om2rpm(omegam)), omega + 1/2*h*k1, omegam, np.matmul(np.concatenate((q,np.transpose(L)),axis = 1),exp_S3(1/2*h * uk1)))
    uk2 = dexp(omega + 1/2 * h * k1, 1/2 * h* uk1)
    pk2 = v + 1/2 * h * vk1

    omegam = omegamfun(t+1/2*h, type)
    (vk3,k3) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)),thrust_torque(motor,aircraft,om2rpm(omegam)), drag_torque(motor,om2rpm(omegam)), omega + 1/2*h*k2, omegam, np.matmul(np.concatenate((q,np.transpose(L)),axis = 1),exp_S3(1/2*h * uk2)))
    uk3 = dexp(omega + 1/2 * h * k2, 1/2 * h * uk2)
    pk3 = v + 1/2 * h * vk2

    omegam = omegamfun(t + h, type)
    (vk4, k4) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)),thrust_torque(motor,aircraft,om2rpm(omegam)), drag_torque(motor,om2rpm(omegam)), omega + h*k3, omegam, np.matmul(np.concatenate((q,np.transpose(L)),axis = 1),exp_S3(h * uk3)))
    uk4 = dexp(omega + h*k3, h * uk3)
    pk4 = v + h *vk3

    omega_out = omega + 1/6 * h * (k1 + 2*k2 + 2*k3 + k4)
    q_out = np.matmul(appendedMatrix, exp_S3(1/6 *h * (uk1+ 2*uk2 + 2 * uk3+ uk4)))   

    v_out = v + 1/6 * h * (vk1 + 2*vk2 + 2*vk3 + vk4)
    p_out = p + 1/6 * h * (pk1 + 2*pk2 + 2*pk3 + pk4)



    return omega_out, v_out, q_out, p_out

