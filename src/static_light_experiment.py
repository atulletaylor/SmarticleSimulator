import pybullet as p
import time
import pybullet_data
import smarticlesimulation as sim
from smarticlesimulation.flashlight import Flashlight
from datetime import datetime

from pdb import set_trace as bp

import numpy as np

filename = datetime.now().strftime("../data/StaticLightSim_%m-%d_%H:%M:%S.csv")

#URDF paths
urdf_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'
urdf_80_20 = '../urdf/80_20.urdf'








runs = 20
z = 0.6
fl = Flashlight([0,1,z+0.025], 3*np.pi/2)
n = 5
maxvel = 6.9
dx = 0.032
th = np.pi/2
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]
time_s = 8*60
t_steps = time_to_steps(time_s)
t0 = time.time()
displacement = np.zeros(runs+1)
for iter in range(runs):

    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    tableId = p.loadURDF("../urdf/table/table.urdf")
    table_constraint_id = p.createConstraint(tableId,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
    plank1 = p.loadURDF("../urdf/80_20.urdf", basePosition=[-0.2,0.25,z])
    p1c_id = p.createConstraint(plank1,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
    plank2 = p.loadURDF("../urdf/80_20.urdf", basePosition=[0.2,0.25,z])
    p2c_id = p.createConstraint(plank2,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
    dt = time_to_steps(0.45)
    r = p.loadURDF(ring_path, basePosition = [0,0,z])


    smarticles = load_smarticles(n,urdf_path, maxvel, dx,th,[L,R],dt,z)
    for i in range (240*20):
        p.stepSimulation()
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.move_random_corners()

    start_pos, _ = p.getBasePositionAndOrientation(r)
    for i in range (t_steps):
        p.stepSimulation()
        # time.sleep(1./240.)
        if i%40==0:
            ray_check(smarticles,fl)
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.motor_step()
    pos, _ = p.getBasePositionAndOrientation(r)
    displacement[iter]= int(1000*(pos[1]-start_pos[1]))
    print('\n\ntime:{}, run:{}, displacement:{}\n\n'.format(time.time()-t0,iter,pos[1]))
    p.disconnect()
iteration = np.arange(0,runs+1)
iteration[-1]=0
displacement[-1] = sum(displacement)/(len(displacement-1))
data_out = np.vstack([iteration,displacement]).T
np.savetxt(filename,data_out,\
    header="iteration, displacement(m), last column is average",fmt="%d, %d")
print("Total Time:{}, average_displace:{}".format(time.time()-t0, displacement[-1]))
