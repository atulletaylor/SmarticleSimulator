import pybullet as p
import time
import pybullet_data
from simulation_smarticle import SimulationSmarticle as ss

from pdb import set_trace as bp

import numpy as np

#URDF paths
urdf_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'

def time_to_steps(time_s):
    return int(time_s*240)

def load_smarticles(n,urdf_path, max_vel, dx,th,gait,dt):
    smarticles = []
    offset = -dx*(n//2)
    for ii in range(n):
        x = offset+ii*dx
        smarticles.append(ss(p, urdf_path, maxvel, basePosition = [0,x,0],\
                             baseOrientation = [0,0,th]))
        smarticles[-1].load_gait(np.array(gait),dt)

    return smarticles




physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
dt = time_to_steps(0.45)
r = p.loadURDF(ring_path, basePosition = [0,0,0])
cid = p.createConstraint(r,-1,0,-1,p.JOINT_PRISMATIC,[0,0,0],[0,0,0],[0,0,0])
maxvel = 6.
dx = 0.032
th = np.pi/2
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]

s = load_smarticles(5,urdf_path, maxvel, dx,th,[L,R],dt)

for i in range (480):
    p.stepSimulation()

t_steps = time_to_steps(1200)
for i in range (t_steps):
    p.stepSimulation()
    time.sleep(1./240.)
    if i%dt==s[0].gait_phase:
        s[0].motor_step()
        s[0].update_position()
        # bp()
    if i%dt==s[1].gait_phase:
        s[1].motor_step()
    if i%dt==s[2].gait_phase:
        s[2].motor_step()
    if i%dt==s[3].gait_phase:
        s[3].motor_step()
    # if i%dt==s[4].gait_phase:
    #     s[4].motor_step()
p.disconnect()
