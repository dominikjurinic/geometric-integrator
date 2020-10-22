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


def rk4cl(t, h, aircraft, motor, omega, v, q, p, type):
    omegam = omegamfun(t, type)
    (vk1,k1) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega, omegam, q)
    qk1 = 1/2 * quat_vect(q,omega)
    pk1 = v

    omegam = omegamfun((t+1)/(2*h),type)
    (vk2, k2) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega + 1/2*h*k1, omegam, q+1/2*h*qk1)
    qk2 = 1/2 * quat_vect(q+1/2*h*qk1,omega + 1/2*h*k1)
    pk2 = v + 1/2*h*vk1

    omegam = omegamfun(t+1/2*h, type)
    (vk3,k3) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega + 1/2*h*k2, omegam, q+1/2*h*qk2)
    qk3 = 1/2 * quat_vect(q+1/2*h*qk2, omega + 1/2*h * k2)
    pk3 = v + 1/2*h*vk2

    omegam = omegamfun(t+h, type)
    (vk4, k4) = new_eul(aircraft, motor, thrust_forces(motor, om2rpm(omegam)), thrust_torque(motor, aircraft, om2rpm(omegam)), drag_torque(motor, om2rpm(omegam)), omega + h*k3, omegam, q+h*qk3)
    qk4 = 1/2 * quat_vect(q+h*qk3, omega + h * k3)
    pk4 = v + h*vk3

    omega_out = omega + 1/6*h * (k1 + 2*k2 + 2*k3 + k4)
    q_out = q + 1/6*h * (qk1 + 2*qk2 + 2*qk3 + qk4)

    q_out = q_out / np.sqrt(np.matmul(np.transpose(q_out),q_out))

    v_out = v + 1/6*h * (vk1 + 2*vk2 + 2*vk3 + vk4)
    p_out = p + 1/6*h * (pk1 + 2*pk2 + 2*pk3 + pk4)

    return omega_out, v_out, q_out, p_out