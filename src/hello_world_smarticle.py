import pybullet as p
import time
import pybullet_data
from simulation_smarticle import SimulationSmarticle as ss
from flashlight import Flashlight

from pdb import set_trace as bp

import numpy as np

#URDF paths
urdf_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'
z = 0.6
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


def ray_check(s,fl):
    p.removeAllUserDebugItems()
    results = fl.draw_rays()
    smart_ids = [x.id for x in s]
    for smart in s:
        smart.set_plank(0)
        smart.update_position()
    for ray in results:
        if ray[0]>=smart_ids[0] and ray[1]==-1:
            index = smart_ids.index(ray[0])
            if s[index].light_plank(ray[3]):
                p.addUserDebugLine(fl.x, ray[3], fl.ray_hit_color)




fl = Flashlight([0,1,z+0.025], 3*np.pi/2)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("../urdf/table/table.urdf")
dt = time_to_steps(0.45)
r = p.loadURDF(ring_path, basePosition = [0,0,z])
# cid = p.createConstraint(r,-1,0,-1,p.JOINT_PRISMATIC,[0,0,0],[0,0,0],[0,0,0])
n = 5
maxvel = 6.9
dx = 0.032
th = np.pi/2
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]

smarticles = load_smarticles(n,urdf_path, maxvel, dx,th,[L,R],dt,z)

for i in range (480):
    p.stepSimulation()

t_steps = time_to_steps(1200)
for i in range (t_steps):
    p.stepSimulation()
    time.sleep(1./240.)
    if i%40==0:
        ray_check(smarticles,fl)
    for s in smarticles:
        if i%dt==s.gait_phase:
            s.motor_step()
p.disconnect()
