# smarticle_simulation_object

import numpy as np
from pdb import set_trace as bp

class SimulationSmarticle(object):
    MAX_VEL_RANGE = 0.2
    MAX_ANGLE_OFFSET = 0.5
    PR_LOC = 1e-3*np.array([[8,13,10],[-11,17.5,10]])

    def __init__(self, p, urdf_path, max_vel,\
                 basePosition=None, baseOrientation= None, max_r=30e-3):

        self.p = p
        self.x = np.zeros(3)
        self.pr_loc_global = np.zeros([2,3])
        self.plank = 0

        # load basePosition and baseOrientation
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

        # loadURDF
        self.id = self.p.loadURDF(urdf_path, basePosition = basePosition,\
                                  baseOrientation = baseOrientation)

      # add randomness to smarticle params
        self.maxvel = max_vel + self.MAX_VEL_RANGE*2.*(np.random.rand(2)-0.5)
        self.control = self.p.POSITION_CONTROL
        self.arm_offset = self.MAX_ANGLE_OFFSET*2.*(np.random.rand(2)-0.5)

        # set friction of arms to zero
        for joint in range(2):
            self.p.changeDynamics(self.id,joint,lateralFriction = 0)
        self.update_position()

    def load_gait(self, gait, gait_dt):
        '''DOC'''
        self.n = gait.shape[1]
        self.gait_index = np.random.randint(0,self.n)
        self.gait_period = gait_dt
        self.gait_phase = int(self.gait_period*np.random.rand())
        self.gaitL = gait[0]
        self.gaitR = gait[1]

    def move_arms(self,posL, posR):
        self.p.setJointMotorControl2(self.id,0, self.control,\
            targetPosition=posL+self.arm_offset[0],\
            maxVelocity = int(self.maxvel[0]))
        self.p.setJointMotorControl2(self.id,1, self.control,\
            targetPosition=posR+self.arm_offset[1],\
            maxVelocity = int(self.maxvel[0]))

    def set_plank(self, state):
        if state==1:
            self.plank = 1
            self.move_arms(0,0)

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
        for ii in range(len(self.pr_loc_global)):
            self.pr_loc_global[ii],_ = self.p.multiplyTransforms(pos, orient,\
                                                     self.PR_LOC[ii], [0,0,0,1])
