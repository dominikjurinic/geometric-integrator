import numpy as np 

def omegamfun(t, type):    
    if type == 'roll':
        omegam_out = (2*np.pi/60)*((0*np.sqrt(1/(6.11e-8)))*np.array([[1],[1],[0],[0]]) + np.sqrt(1.4516625/(6.11e-8))*np.array([[0],[0],[1],[1]]))
    elif type == 'yaw':
        omegam_out = (2*np.pi/60)*((np.sqrt(1.2/(6.11e-8)))*np.array([[1],[0],[1],[0]]) + np.sqrt(1.2516625/(6.11e-8))*np.array([[0],[1],[0],[1]]))
    elif type == 'yaw_climb': 
        omegam_out = (2*np.pi/60)*((np.sqrt(1.3/(6.11e-8)))*np.array([[1],[0],[1],[0]]) + np.sqrt(1.3516625/(6.11e-8))*np.array([[0],[1],[0],[1]]))
    return omegam_out





