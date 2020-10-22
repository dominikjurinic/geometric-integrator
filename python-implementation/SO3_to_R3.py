import numpy as np 

def SO3_to_R3(x_skew):  
    x = np.zeros((3,1))
    x[0,0] = -1*x_skew[1,2]   
    x[1,0] = x_skew[0,2]      
    x[2,0] = -1*x_skew[0,1]   
    return x            
