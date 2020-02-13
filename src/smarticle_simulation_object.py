# smarticle_simulation_object

import numpy as np
from pdb import set_trace as bp

class SmarticleSimulationObject(object):
    """docstring for SmarticleSimulationObject."""
    MAX_VEL_RANGE = 0.2
    MAX_ANGLE_OFFSET = 0.5

    def __init__(self, p, urdf_path, max_vel, basePosition):

        self.p = p
        self.id = self.p.loadURDF(urdf_path, basePosition = basePosition)
        self.maxvel = (max_vel-self.MAX_VEL_RANGE/2.)*np.ones(2)+\
            self.MAX_VEL_RANGE*np.random.rand(2)
        self.control = self.p.POSITION_CONTROL

    def load_gait(self, gait, gait_dt):
        '''DOC'''
        self.n = gait.shape[1]
        self.gait_index = np.random.randint(0,self.n)
        self.gait_period = gait_dt
        self.gait_phase = int(self.gait_period*np.random.rand())
        self.gaitL = self.MAX_ANGLE_OFFSET*np.random.rand(self.n)-self.MAX_ANGLE_OFFSET/2.+gait[0]
        self.gaitR = self.MAX_ANGLE_OFFSET*np.random.rand(self.n)-self.MAX_ANGLE_OFFSET/2.+gait[1]

    def motor_step(self):
        self.p.setJointMotorControl2(self.id,0, self.control,\
            targetPosition=self.gaitR[self.gait_index], maxVelocity = int(self.maxvel[0]))
        self.p.setJointMotorControl2(self.id,1, self.control,\
            targetPosition=self.gaitL[self.gait_index], maxVelocity = self.maxvel[1])
        self.gait_index= (self.gait_index+1)%self.n
