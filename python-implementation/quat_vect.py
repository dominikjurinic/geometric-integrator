import numpy as np 

def quat_vect(q,v):    
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
    q_out = np.matmul(np.transpose(L),v)
    return q_out       


