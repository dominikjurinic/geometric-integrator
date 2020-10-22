import numpy as np 

def quat_rot(q,v): 
    R = np.zeros((3,3))
    R[0,0] = q[0]**2+q[1]**2-1/2
    R[0,1] = q[1]*q[2]-q[0]*q[3]
    R[0,2] = q[1]*q[3]+q[0]*q[2]
    R[1,0] = q[1]*q[2]+q[0]*q[3]
    R[1,1] = (q[0])**2+(q[2])**2-1/2
    R[1,2] = q[2]*q[3]-q[0]*q[1]
    R[2,0] = q[1]*q[3]-q[0]*q[2]
    R[2,1] = q[2]*q[3]+q[0]*q[1]
    R[2,2] = q[0]**2+q[3]**2-1/2
    v_out = np.matmul(2*R,v)
    return v_out    



