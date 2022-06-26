import sys
import pybullet as p
import time
import pybullet_data
import smarticlesimulation as sim
import numpy as np

#URDF paths
smarticle_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'
flashlight_path = '../urdf/flashlight.urdf'



z = 0.6

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("../urdf/table/table.urdf")
table_constraint_id = p.createConstraint(tableId,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])

if len(sys.argv)>1:
    n = int(sys.argv[1])
else:
    n = 3

maxvel = 6.9
dx = 0.032
th = np.pi/2
R = [-1.7,1.7,1.7,-1.7] #arm angles in radians
L = [1.7,1.7,-1.7,-1.7]
dt = sim.time_to_steps(0.45)

smarticles = sim.load_smarticles(n,smarticle_path, [L,R], dt, z, dx, th)

r = p.loadURDF(ring_path, basePosition = [0,0,z])

sim_time = 480.
for i in range (2*int(sim_time)):
    p.stepSimulation()

t_steps = sim.time_to_steps(300)
for i in range (t_steps):
    # print(i)
    time.sleep(1/(sim_time/2))
    p.stepSimulation()
    for s in smarticles:
        if i%dt==s.gait_phase: #1/240 s
            s.motor_step()
p.disconnect()
