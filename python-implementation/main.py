import numpy as np 
import time 

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
from rk4geom import rk4geom
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

startTime = time.time()

aircraftMass = 0.5
aircraftJ = np.diag([0.00365,0.00368,0.00703])
aircraftLev_Arm = 0.17
aircraftMatrix_FT = (1/2) * np.array([[np.sqrt(2),np.sqrt(2),-np.sqrt(2),-np.sqrt(2)],[-np.sqrt(2),np.sqrt(2),np.sqrt(2),-np.sqrt(2)],[0,0,0,0]])

motor_k_t =6.11e-8      
motor_k_d = 1.5e-9
motor_rot_dir = np.array([[1],[-1],[1],[-1]])
motor_inertia = (1/2) * 0.011 * (8*2.54/100)**2


specificAircraft = Aircraft(aircraftMass,aircraftJ,aircraftLev_Arm,aircraftMatrix_FT)
specificMotor = Motor(motor_k_d,motor_k_t,motor_rot_dir,motor_inertia)



#simulation parameters

type = 'roll'
h = 1e-04
t = np.arange(start = 0, stop = 1.0, step = h)    # time step vector 
N = len(t)


q = np.zeros((4,N))
q[:,[0]] = np.array([[1],[0],[0],[0]])          #initial rotational quaternion

omegaVector = np.zeros((3,N))
omegaVector[:,[0]] = np.array([[0],[0],[0]])    #initial angular speed

p = np.zeros((3, N))
p[:,[0]] = np.array([[0],[0],[-20]])

v = np.zeros((3, N))
v[:,[0]] = np.array([[0],[0],[0]])




for i in range(1,N):
    (omegaVector[:,[i]], v[:,[i]], q[:,[i]], p[:,[i]]) = rk4geom(t[i], h, specificAircraft, specificMotor, omegaVector[:,[i-1]], v[:,[i-1]], q[:,[i-1]], p[:, [i-1]], type)


