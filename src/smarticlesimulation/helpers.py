
def time_to_steps(time_s):
    return int(time_s*240)

def load_smarticles(n,urdf_path, max_vel, dx,th,gait,dt,z):
    smarticles = []
    offset = -dx*(n//2)
    for ii in range(n):
        x = offset+ii*dx
        smarticles.append(ss(urdf_path, maxvel, basePosition = [0,x,z],\
                             baseOrientation = [0,0,th]))
        smarticles[-1].load_gait(np.array(gait),dt)

    return smarticles

def ray_check(s,fl):
    p.removeAllUserDebugItems()
    results = fl.draw_rays()
    smart_ids = [x.id for x in s]
    for smart in s:
        smart.update_position()
        smart.set_plank(0)
    for ray in results:
        if ray[0]>=smart_ids[0] and ray[1]==-1:
            index = smart_ids.index(ray[0])
            if s[index].light_plank(ray[3],fl.yaw):
                p.addUserDebugLine(fl.x, ray[3], fl.ray_hit_color)
