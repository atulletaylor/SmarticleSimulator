# helper functions for geometry and transformations
import numpy as np

def rot(th):
    ''''
    creates coordinate frame rotation matrix given angle, th
    '''
    c,s = round(np.cos(th),8), round(np.sin(th),8)
    return np.array([[c,-s],[s,c]])

def transform_point(child_point, parent_global):
    '''
    transforms child_point, from parent frame to global frame given
    parent's state in global frame, parent_global
    '''
    r = rot(parent_global[2])
    return np.dot(r,child_point)+parent_global[0:2]

def ang_diff(th1,th2, abs=False):
    '''
    returns shortest difference in angle between target: th2 and source: th1
    '''
    d_theta = np.mod(np.pi+th2-th1,2*np.pi)-np.pi
    if not abs:
        return d_theta
    else:
        return np.abs(d_theta)

def bearing(p1, p2):
    '''
    calucaltes bearing from p1 to p2
    '''
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    return np.mod(np.arctan2(dy,dx),2*np.pi)
