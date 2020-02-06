import pybullet as p
import time
import pybullet_data
from pdb import set_trace as bp

import numpy as np

urdf_path = '../urdf/simulation_urdf2.urdf'

def time_to_steps(time_s):
    return int(time_s*240)



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
s1 = p.loadURDF(urdf_path, basePosition = [0.02,0.03,0])
s2 = p.loadURDF(urdf_path, basePosition = [0,0,0])

R = [-1.57,1.57,1.57,-1.57]
L = [1.57,1.57,-1.57,-1.57]
j1 = np.random.randint(0,4)
dt = time_to_steps(0.45)
n1 = int(dt*np.random.rand())
print(n1)
n2 = int(dt*np.random.rand())
print(n2)
j2 = np.random.randint(0,4)
t0 = time.time()
tlast = t0
control =  p.POSITION_CONTROL
n=4
maxvel = 5.95*np.ones(4)+0.2*np.random.rand(4)
joints = [0,1]
time.sleep(3)
for i in range (50000):
    p.stepSimulation()
    time.sleep(1./240.)
    if i%dt==n1:
        j1 = j1%len(R)
        p.setJointMotorControl2(s1,0, control,targetPosition=R[j1], maxVelocity = maxvel[0])
        p.setJointMotorControl2(s1,1, control,targetPosition=L[j1], maxVelocity = maxvel[1])
        j1+=1
    if i%dt==n2:
        j2 = j2%len(R)
        p.setJointMotorControl2(s2,0, control,targetPosition=R[j2], maxVelocity = maxvel[2])
        p.setJointMotorControl2(s2,1, control,targetPosition=L[j2], maxVelocity = maxvel[3])
        j2+=1

cubePos, cubeOrn = p.getBasePositionAndOrientation(s1)
print(cubePos,cubeOrn)
p.disconnect()
