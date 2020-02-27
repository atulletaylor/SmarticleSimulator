import numpy as np
import pybullet as p

class Flashlight(object):
    """docstring for Flashlight."""

    def __init__(self,urdf_path, basePosition, yaw=0, beam_width=np.pi/12,\
                ray_count=100, ray_length=1.2):
        self.x = np.array(basePosition).astype(np.double)
        self.yaw = np.mod(yaw,2*np.pi)
        self.beam_width = beam_width
        self.ray_len = ray_length
        self.ray_count = ray_count
        self.ray_from = self.x*np.ones([self.ray_count,1])
        self.d_theta = beam_width/(ray_count-1)
        self.rays = -self.beam_width/2.\
                    +np.arange(0,self.beam_width+self.d_theta/2.,self.d_theta)
        self.ray_hit_color = [1,0.49,0]
        self.ray_miss_color = [1,0.75,0]

        self.fl_id= p.loadURDF(urdf_path,basePosition=basePosition)

    def update_position(self,x, pitch, yaw):
        self.x = np.array(x)
        self.ray_from = self.x*np.ones([self.ray_count,1])
        self.yaw = np.mod(pitch,2*np.pi)

    def draw_rays(self):
        ray_to = np.zeros([self.ray_count,3])
        ray_ids = np.zeros(self.ray_count)
        ray_angles = np.mod(self.yaw+self.rays,2*np.pi)
        for ii,th in enumerate(ray_angles):
            ray_to[ii] = self.x+ np.array([self.ray_len*np.cos(th),\
                                self.ray_len*np.sin(th),0])
            # p.addUserDebugLine(self.x, ray_to[ii], self.ray_miss_color)

    def ray_check(self, smarticles):
        p.removeAllUserDebugItems()
        results = self.draw_rays()
        smart_ids = [x.id for x in smarticles]
        for smart in smarticles:
            smart.update_position()
            smart.set_plank(0)
        for ray in results:
            if ray[0]>=smart_ids[0] and ray[1]==-1:
                index = smart_ids.index(ray[0])
                if s[index].light_plank(ray[3],self.yaw):
                    p.addUserDebugLine(self.x, ray[3], self.ray_hit_color)

        return p.rayTestBatch(self.ray_from,ray_to)
