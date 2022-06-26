import pybullet as p
import os
import time
import pybullet_data
from datetime import datetime
import numpy as np

import smarticlesimulation as sim
from smarticlesimulation.flashlight import Flashlight

import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt
mpl.style.use('seaborn')
GUI = False

#URDF paths
urdf_dir = '../urdf/'
smarticle_path = urdf_dir+'smarticle.urdf'
ring_path = urdf_dir+'ring.urdf' #CHANGE RING HERE
path_80_20 = urdf_dir+'80_20.urdf'
table_path = urdf_dir+'table/table.urdf'
flashlight_path = urdf_dir+'flashlight.urdf'



server = p.GUI if GUI else p.DIRECT
physicsClient = p.connect(server)#or p.DIRECT for non-graphical version
runs = 10
total_displacement = np.zeros(runs)
z = 0.6
fl = Flashlight(flashlight_path,[0,1,z+0.025], 3*np.pi/2) #CHANGE Flashlight BEAM WIDTH???
fl.set_polar_position([0,0],r=1,th=np.pi/2)
n = 5 #CHANGE NUMBER OF SMARTICLES HERE
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]
dt = sim.time_to_steps(0.45)
time_s = 8*60
t_steps = sim.time_to_steps(time_s)
t0 = time.time()
dir = datetime.now().strftime("../data/StaticLightSim_%m-%d_%H:%M")
filename = dir+"/Episode_{}.csv"
try:
    os.mkdir(dir)
except FileExistsError:
    print('folder already exisits')



p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF(table_path)
table_constraint_id = p.createConstraint(tableId,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
wall1 = p.loadURDF(path_80_20, basePosition=[-0.2,0.25,z])
p1c_id = p.createConstraint(wall1,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
wall2 = p.loadURDF(path_80_20, basePosition=[0.2,0.25,z])
p2c_id = p.createConstraint(wall2,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
r = p.loadURDF(ring_path, basePosition = [0,0,z])
smarticles = sim.load_smarticles(n,smarticle_path,[L,R],dt,z)
for iter in range(runs):
    displacement = []
    p.resetBasePositionAndOrientation(r,[0,0,z],p.getQuaternionFromEuler([0,0,0]))
    ring_constraint_id = p.createConstraint(r,-1,-1,-1,\
                                p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,z])
    for s in smarticles:
        s.reset_pose()
    for i in range (240*1):
        p.stepSimulation()
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.move_random_corners()
    p.removeConstraint(ring_constraint_id)
    plot_ii = 0
    for i in range (t_steps):
        p.stepSimulation()
        # time.sleep(1./240.)
        if i%40==0:
            pos,_ = p.getBasePositionAndOrientation(r)
            if plot_ii %18==0: #plot every 3 s
                displacement.append(pos[:2])
            plot_ii+=1
            fl.set_polar_position(pos[:2])
            fl.ray_check(smarticles)
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.motor_step()
    p.removeAllUserDebugItems()
    pos, _ = p.getBasePositionAndOrientation(r)
    displacement.append(pos[:2])
    displacement = 1000*np.array(displacement)
    step = np.expand_dims(np.arange(0,len(displacement)),1)
    data_out = np.hstack([step,displacement])
    plt.figure()
    plt.scatter(displacement[:,0], displacement[:,1],c=step[:,0],cmap='summer', zorder=200, linewidth = 0.5)
    plt.plot(displacement[:,0], displacement[:,1],color='k', zorder=20, linewidth = 1)
    plt.xlabel("x(mm)")
    plt.ylabel("y(mm)")
    plt.title("Run {}".format(1+iter))
    plt.show(block = False)
    plt.pause(0.01)
    path = filename.format(1+iter)
    total_displacement[iter]=displacement[-1,1]
    np.savetxt(path,data_out,\
        header="step, ring_center (x), ring_center (y)",fmt="%d, %d, %d")
    print('\n\ntime:{}, run:{}, displacement:{}'.format(time.time()-t0,iter,total_displacement[iter]))
p.disconnect()
print('Average Displacement: {}'.format(sum(total_displacement)/runs))
print("Done!")
