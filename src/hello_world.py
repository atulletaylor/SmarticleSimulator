import pybullet as p
import time
import pybullet_data
import smarticlesimulation as sim
import numpy as np

#URDF paths
smarticle_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'


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
dt = sim.time_to_steps(0.45)
z = 0.6

smarticles = sim.load_smarticles(n,smarticle_path, maxvel, dx,th,[L,R],dt,z)
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
