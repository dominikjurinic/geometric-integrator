import numpy as np 

def exp_S3(u):
    norm_u = np.linalg.norm(u)
    if norm_u < 10**(-12):
        u_S3 = np.array([[1],[0],[0],[0]])    
    else:
        u_S3 = np.cos((1/2) * norm_u) * np.array([[1],[0],[0],[0]]) + np.sin((1/2) * norm_u)/norm_u * np.concatenate((np.array([[0]]),u))
    return u_S3
