import numpy as np
from .smarticle import Smarticle

def time_to_steps(time_s):
    return int(time_s*240) #Default timestep in pybullet is 1/240


def load_smarticles(n,urdf_path,gait,dt,z,dx=0.032,th=np.pi/2):
    smarticles = []
    dz = 0.2
    offset = -dx*(n//2)
    for ii in range(n):
        x = offset+ii*dx
        z = offset+ii*dz
        smarticles.append(Smarticle(urdf_path, smartName = 'smart' + str(ii), basePosition = [0,0,z],\
                                    baseOrientation = [0,0,th]))
        # smarticles.append(Smarticle(urdf_path, smartName = 'smart' + str(ii), basePosition = [0,x,z],\
                                    # baseOrientation = [0,0,th]))
        smarticles[-1].load_gait(np.array(gait),dt)
    return smarticles
