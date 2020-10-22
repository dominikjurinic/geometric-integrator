import numpy as np 

def skew(x):
    x_skew = np.zeros((3,3))
    x_skew[0,1] = (-1)*x[2]    
    x_skew[0,2] = x[1]         
    x_skew[1,2] = (-1)*x[0]    
    x_skew[1,0] = x[2]         
    x_skew[2,0] = (-1)*x[1]       
    x_skew[2,1] = x[0]        
    return x_skew


