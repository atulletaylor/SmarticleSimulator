import pybullet as p
import time
import pybullet_data
import smarticlesimulation as sim
import numpy as np

#URDF paths
urdf_path = '../urdf/flashlight.urdf'
ring_path = '../urdf/ring.urdf'


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")


id = p.loadURDF(urdf_path)



while True:
    time.sleep(1/240.)
    p.stepSimulation()

p.disconnect()
