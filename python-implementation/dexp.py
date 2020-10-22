import numpy as np

from skew import skew
from SO3_to_R3 import SO3_to_R3
from lie_bracket import lie_bracket


def dexp(omega,u):
    norm_u = np.linalg.norm(u)
    skew_u = skew(u)
    du = SO3_to_R3(skew(omega))
    

    return du


