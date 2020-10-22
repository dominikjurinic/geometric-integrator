import numpy as np 

from omegamfun import omegamfun
from new_eul import new_eul
from thrust_forces import thrust_forces
from thrust_torque import thrust_torque
from om2rpm import om2rpm
from drag_torque import drag_torque
from SO3_to_R3 import SO3_to_R3
from skew import skew
from quat_vect import quat_vect

def rk2cl(t, h, aircraft, motor, omega, v, q, p, type):
    omegam = omegamfun(t,type)
    (vk1, k1) = new_eul(aircraft, motor, thrust_forces(motor,om2rpm(omegam)),thrust_torque(motor,aircraft,om2rpm(omegam)),drag_torque(motor,om2rpm(omegam)),omega,omegam,q)
    qk1 = 1/2 * quat_vect(q,omega)
    pk1 = v
    
    omegam = omegamfun(t+h,type)
    (vk2,k2) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega + h*k1, omegam, q+h*qk1)
    qk2 = 1/2 * quat_vect(q+h*qk1,omega + h * k1)
    pk2 = v + h * vk1

    omega_out = omega + 1/2 * h *(k1+k2)
    q_out = q + 1/2 * h * (qk1 + qk2)

    q_out = q_out / np.sqrt(np.matmul(np.transpose(q_out),q_out))

    v_out = v + 1/2 * h * (vk1 + vk2)
    p_out = p + 1/2 * h * (pk1 + pk2)

    return omega_out, v_out, q_out, p_out