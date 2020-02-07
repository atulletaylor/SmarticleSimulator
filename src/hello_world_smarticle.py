import pybullet as p
import time
import pybullet_data
from pdb import set_trace as bp

import numpy as np

#URDF paths
urdf_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'

def time_to_steps(time_s):
    return int(time_s*240)



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
x = 0.02
r = p.loadURDF(ring_path, basePosition = [0,0,0])
s1 = p.loadURDF(urdf_path, basePosition = [-0.027,x+0.03,0])
s2 = p.loadURDF(urdf_path, basePosition = [-0.027,x+0,0])
s3 = p.loadURDF(urdf_path, basePosition = [-0.027,x-0.03,0])
s4 = p.loadURDF(urdf_path, basePosition = [-0.027,x-0.06,0])
s5 = p.loadURDF(urdf_path, basePosition = [-0.027,x-0.09,0])

R = [-1.57,1.57,1.57,-1.57]
L = [1.57,1.57,-1.57,-1.57]
j1 = np.random.randint(0,4)
dt = time_to_steps(0.45)
n1 = int(dt*np.random.rand())
n2 = int(dt*np.random.rand())
n3 = int(dt*np.random.rand())
n4 = int(dt*np.random.rand())
n5 = int(dt*np.random.rand())

j1 = np.random.randint(0,4)
j2 = np.random.randint(0,4)
j3 = np.random.randint(0,4)
j4 = np.random.randint(0,4)
j5 = np.random.randint(0,4)

t0 = time.time()
tlast = t0
control =  p.POSITION_CONTROL
n=4
maxvel = 5.95*np.ones(10)+0.2*np.random.rand(10)
joints = [0,1]
time.sleep(3)
t_steps = time_to_steps(1200)
for i in range (t_steps):
    p.stepSimulation()
    time.sleep(1./4800.)
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
    if i%dt==n3:
        j3 = j3%len(R)
        p.setJointMotorControl2(s3,0, control,targetPosition=R[j3], maxVelocity = maxvel[4])
        p.setJointMotorControl2(s3,1, control,targetPosition=L[j3], maxVelocity = maxvel[5])
        j3+=1
    if i%dt==n4:
        j4 = j4%len(R)
        p.setJointMotorControl2(s4,0, control,targetPosition=R[j4], maxVelocity = maxvel[6])
        p.setJointMotorControl2(s4,1, control,targetPosition=L[j4], maxVelocity = maxvel[7])
        j4+=1
    if i%dt==n5:
        j5 = j5%len(R)
        p.setJointMotorControl2(s5,0, control,targetPosition=0, maxVelocity = maxvel[8])
        p.setJointMotorControl2(s5,1, control,targetPosition=0, maxVelocity = maxvel[9])
        j5+=1

cubePos, cubeOrn = p.getBasePositionAndOrientation(s1)
print(cubePos,cubeOrn)
p.disconnect()
