# smarticle_simulation_object

import numpy as np
from pdb import set_trace as bp
import transforms as tf

class SimulationSmarticle(object):
    MAX_VEL_RANGE = 0.2
    MAX_ANGLE_OFFSET = 0.5
    PR_LOC = 1e-3*np.array([0,14.5])
    RAY_BLOCK_POINTS = 1e-3*np.array([[0,-70.5],\
                                      [0,-41],\
                                      [0,-27.5],\
                                      [0,27.5],\
                                      [0,41],\
                                      [0,70.5]])

    def __init__(self, p, urdf_path, max_vel,\
                 basePosition=None, baseOrientation= None, max_r=30e-3):

        self.p = p
        self.x = np.zeros(3)
        if basePosition is None:
            basePosition = [0,0,0]
        elif basePosition == 'random':
            r = max_r*np.random.rand()
            th = 2*np.pi*np.random.rand()
            c,s = np.cos(th), np.sin(th)
            basePosition = [r*c,r*s,0]
        if baseOrientation is None:
            baseOrientation = [0,0,0]
        elif baseOrientation == 'random':
            th = 2*np.pi*np.random.rand()
            baseOrientation = [0,0,th]
        baseOrientation = self.p.getQuaternionFromEuler(baseOrientation)
        self.id = self.p.loadURDF(urdf_path, basePosition = basePosition,\
                                  baseOrientation = baseOrientation)
        self.maxvel = (max_vel-self.MAX_VEL_RANGE/2.)*np.ones(2)\
            + self.MAX_VEL_RANGE*np.random.rand(2)
        self.control = self.p.POSITION_CONTROL
        self.block_points = self.RAY_BLOCK_POINTS
        self.plank = 0
        self.arm_offset = self.MAX_ANGLE_OFFSET*np.random.rand(2)\
            - self.MAX_ANGLE_OFFSET/2.

    def load_gait(self, gait, gait_dt):
        '''DOC'''
        self.n = gait.shape[1]
        self.gait_index = np.random.randint(0,self.n)
        self.gait_period = gait_dt
        self.gait_phase = int(self.gait_period*np.random.rand())
        self.gaitL = self.arm_offset[0]+gait[0]
        self.gaitR = self.arm_offset[1]+gait[1]

    def move_arms(self,posL, posR):
        self.p.setJointMotorControl2(self.id,0, self.control,\
            targetPosition=posR, maxVelocity = int(self.maxvel[0]))
        self.p.setJointMotorControl2(self.id,1, self.control,\
            targetPosition=posL, maxVelocity = int(self.maxvel[0]))

    def set_plank(self, state):
        if state==1:
            self.plank = 1
            self.move_arms(self.arm_offset[0],self.arm_offset[1])

    def motor_step(self):
        if self.plank !=1:
            posL = self.gaitL[self.gait_index]
            posR = self.gaitR[self.gait_index]
        else:
            posL = self.arm_offset[0]
            posR = self.arm_offset[1]
        self.move_arms(posL, posR)
        self.gait_index = (self.gait_index+1)%self.n

    def update_position(self):
        pos, orient = self.p.getBasePositionAndOrientation(self.id)
        self.x[0:2]=np.array(pos[0:2])
        self.x[2]=np.mod(self.p.getEulerFromQuaternion(orient)[2],2*np.pi)
        for ii in range(len(self.block_points)):
            self.block_points[ii] = tf.transform_point(self.RAY_BLOCK_POINTS[ii],\
                                                       self.x)
        self.pr_loc = tf.transform_point(self.PR_LOC, self.x)
        print('state')
        print(self.x)
        print('block points: ')
        print(self.block_points)
        print('photoresistor')
        print(self.pr_loc)
