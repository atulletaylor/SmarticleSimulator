import numpy as np
from smarticle import Smarticle

def time_to_steps(time_s):
    return int(time_s*240)


def load_smarticles(n,urdf_path, max_vel, dx,th,gait,dt,z):
    smarticles = []
    offset = -dx*(n//2)
    for ii in range(n):
        x = offset+ii*dx
        smarticles.append(Smarticle(urdf_path, maxvel, basePosition = [0,x,z],\
                                    baseOrientation = [0,0,th]))
        smarticles[-1].load_gait(np.array(gait),dt)
    return smarticles
