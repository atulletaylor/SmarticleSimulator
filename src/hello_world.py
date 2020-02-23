import pybullet as p
import time
import pybullet_data
from simulation_smarticle import SimulationSmarticle as ss
from flashlight import Flashlight

from pdb import set_trace as bp

import numpy as np

#URDF paths
smarticle_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'

def time_to_steps(time_s):
    return int(time_s*240)

def load_smarticles(n,urdf_path, max_vel, dx,th,gait,dt,z):
    smarticles = []
    offset = -dx*(n//2)
    for ii in range(n):
        x = offset+ii*dx
        smarticles.append(ss(urdf_path, maxvel, basePosition = [0,x,z],\
                             baseOrientation = [0,0,th]))
        smarticles[-1].load_gait(np.array(gait),dt)

    return smarticles



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("../urdf/table/table.urdf")
table_constraint_id = p.createConstraint(tableId,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])


n = 5
maxvel = 6.9
dx = 0.032
th = np.pi/2
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]
dt = time_to_steps(0.45)
z = 0.6

smarticles = load_smarticles(n,smarticle_path, maxvel, dx,th,[L,R],dt,z)
r = p.loadURDF(ring_path, basePosition = [0,0,z])

for i in range (2*480):
    p.stepSimulation()

t_steps = time_to_steps(30)
for i in range (t_steps):
    time.sleep(1/240.)
    p.stepSimulation()
    for s in smarticles:
        if i%dt==s.gait_phase:
            s.motor_step()
p.disconnect()
