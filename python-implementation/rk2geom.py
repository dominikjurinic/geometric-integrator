import numpy as np 

from omegamfun import omegamfun
from new_eul import new_eul
from thrust_forces import thrust_forces
from thrust_torque import thrust_torque
from om2rpm import om2rpm
from drag_torque import drag_torque
from SO3_to_R3 import SO3_to_R3
from skew import skew
from exp_S3 import exp_S3


def rk2geom(t, h, aircraft, motor, omega, v, q, p, type):
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
    uk1 =  SO3_to_R3(skew(omega))
    pk1 = v
    
    omegam = omegamfun(t+h,type)
    (vk2, k2) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)),thrust_torque(motor,aircraft,om2rpm(omegam)), drag_torque(motor,om2rpm(omegam)), omega + h * k1, omegam, np.matmul(np.concatenate((q,np.transpose(L)),axis = 1),exp_S3(h * uk1)))
    uk2 = SO3_to_R3(skew(omega + h * k1))
    pk2 = v + h*vk1

    omega_out = omega + 1/2 * h * (k1 + k2)

    q_out = np.matmul(appendedMatrix, exp_S3(1/2 *h * (uk1+uk2)))

    v_out = v + 1/2 * h * (vk1 + vk2)
    p_out = p + 1/2 * h * (pk1 + pk2)


    return omega_out, v_out, q_out, p_out
