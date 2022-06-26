import sys
import pybullet as p
import time
import pybullet_data
import numpy as np


#Connect to the pybullet server with GUI
physicsClient = p.connect(p.GUI)


#URDF paths
smarticle_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'
flashlight_path = '../urdf/flashlight.urdf'


p.setGravity(0,0,-10)
smartId = p.loadURDF('../urdf/smarticle.urdf')
# planeId = p.loadURDF('plane.urdf')
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF('../urdf/smarticle.urdf',cubeStartPos, cubeStartOrientation)


for i in range (10000):
   p.stepSimulation()
   time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

p.disconnect()
