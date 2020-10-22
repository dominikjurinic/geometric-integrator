import numpy as np 

def lie_bracket(skew_x, skew_y):
    skew_x_out = np.matmul(skew_x,skew_y) - np.matmul(skew_y,skew_x)
    return skew_x_out
