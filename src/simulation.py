import sys
import pybullet as p
import os
import time
import pybullet_data
import numpy as np
import matplotlib as mpl
from datetime import datetime

#for plotting
mpl.use('TkAgg')
import matplotlib.pyplot as plt
mpl.style.use('seaborn')
GUI = True
FLASHLIGHT = False

from smarticlesimulation.flashlight import Flashlight
import smarticlesimulation as sim

#Connect to the pybullet server
if GUI:
    physicsClient = p.connect(p.GUI) #with GUI
else:
    physicsClient = p.connect(p.DIRECT)


# sys.path.append("/home/darpa/Documents/Code/Smarticles/SmarticleSimulation2/src/")
#Allows you to use built-in URDF files from pybullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

####### SIMULATION PATHS AND SETUP #######
runs = 10
total_displacement = np.zeros(runs)
#URDF paths
smarticle_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'
flashlight_path = '../urdf/flashlight.urdf'

plane = p.loadURDF('plane.urdf')

#Define initial position and orientation of the smarticles relative to each other
dx = 0.032 #Spacing between smarticles when they initially load
th = np.pi/2

#Define gait in radians (arm angles)
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]

#Define how long it takes to get through one gait cycle for each smarticle
dt = sim.time_to_steps(0.45)
#affected by ray checking
num_smart_active = 3
active = sim.load_smarticles(num_smart_active,smarticle_path, [L,R], dt, 0.025, dx, th)
#inactive for the whole run
num_smart_inactive = 0
inactive = sim.load_smarticles(num_smart_inactive,smarticle_path, [L,R], dt, 0.025, dx, th) #need to change dx to be relative to the active smarticles
r = p.loadURDF(ring_path, basePosition = [0,0,0.025])
z = 0.6

#time allotted before smarticles start their gait
sim_time = 480.
for i in range (2*int(sim_time)):
    p.stepSimulation()

#for saving plots
dir = datetime.now().strftime("../data/3Smart_NoLight/Trial_%m-%d_%H:%M")
filename = dir+"/Episode_{}.csv"
try:
    os.mkdir(dir)
except FileExistsError:
    print('folder already exisits')

if FLASHLIGHT:
    #Load flashlight
    fl = Flashlight(flashlight_path,[0,1,0.025], 3*np.pi/2) #CHANGE Flashlight BEAM WIDTH???
    fl.set_polar_position([0,0],r=1,th=np.pi/2)

#Smarticles step through their gait
t_steps = sim.time_to_steps(300) #Convert simulation time (seconds) to pybullet timesteps

#### VARIABLES FOR PLOTTING ####
def create_pos_arr(thing):
    x = np.zeros((3))
    pos, orient = p.getBasePositionAndOrientation(thing)
    x[0:2]=np.array(pos[0:2])
    x[2]=np.mod(p.getEulerFromQuaternion(orient)[2],2*np.pi)
    return x

plot_ii = 0
displacement = []
sys_pos = np.zeros((1,12)) # smarticle and ring pos and orientation
#toggle frequency
togHz = 60 #not sure what units right now #i think this is every 4 sec
flag = 0
t0 = time.time()

for iter in range(runs):
    ####### RESET AFTER EVERY RUN #######
    displacement = []
    p.resetBasePositionAndOrientation(r,[0,0,z],p.getQuaternionFromEuler([0,0,0]))
    ring_constraint_id = p.createConstraint(r,-1,-1,-1,\
                                p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,z])

    for a in active:
        a.reset_pose()
    for i in range (240*1):
        p.stepSimulation()
        for a in active:
            if i%dt==a.gait_phase:
                a.move_random_corners()
    p.removeConstraint(ring_constraint_id)
    plot_ii = 0
    ####### SIMULATION MAIN LOOP #######
    for i in range (t_steps):
        time.sleep(1/(sim_time/2))

        #### COLLECTING DATA TO PLOT ####
        if i%40==0:
            pos,_ = p.getBasePositionAndOrientation(r)
            loop_list  = []
            for a in active:
                loop_list.append(a.return_position()) #(and orientation)
            ring_pos = create_pos_arr(r)
            get_pos = np.hstack((loop_list[0], loop_list[1], loop_list[2], ring_pos))
            if plot_ii %18==0: #plot every 3s
                if sys_pos.all() == 0:
                    sys_pos = get_pos
                else:
                    sys_pos = np.vstack((sys_pos, get_pos))
                displacement.append(pos[:2])
            plot_ii+=1

            #### TOGGLING THE FLASHLIGHT ####
            if FLASHLIGHT:
                if flag in range(0,togHz):
                    if flag==1:
                        fl.ray_check(active)
                        fl.set_z_position(0.025)
                #toggle light "off"
                elif flag in range(togHz,2*togHz+1):
                    if flag==togHz:
                        fl.clear_rays(active)
                        # fl.set_z_position(0.5)
                    if flag == 2*togHz:
                        flag = 0
                flag+=1
                # fl.ray_check(active)
                fl.set_polar_position(pos[:2])

        ####### UPDATE SIMULATION #######
        p.stepSimulation()
        for s in active:
            if i%dt==s.gait_phase: #1/240 s
                s.motor_step()
        for s in inactive:
            s.update_position()
            s.set_plank(0)


    ####### CLOSE SIMULATION AND PLOT #######
    p.removeAllUserDebugItems()
    pos, _ = p.getBasePositionAndOrientation(r)

    displacement.append(pos[:2])
    displacement = 1000*np.array(displacement)
    step = np.expand_dims(np.arange(0,len(displacement)),1)

    ####### APPEND FINAL ENTRY ON SYS_POS ########
    loop_list  = []
    for a in active:
        loop_list.append(a.return_position()) #(and orientation)
    ring_pos = create_pos_arr(r)
    get_pos = np.hstack((loop_list[0], loop_list[1], loop_list[2], ring_pos))
    sys_pos = np.vstack((sys_pos, get_pos))

    print('sys_pos.shape')
    print(sys_pos.shape)

    plt.figure()
    plt.scatter(sys_pos[:,:3*3:3], sys_pos[:,1:3*3:3], linewidth = 0.5)
    plt.plot(sys_pos[:,:3*3:3], sys_pos[:,1:3*3:3], zorder=20, linewidth = 1)

    plt.xlabel("x(mm)")
    plt.ylabel("y(mm)")
    # plt.title("Run {}".format(1+str(iter))
    plt.show(block = False)
    plt.pause(0.01)

    # data_out = np.hstack([step,displacement])
    # data_out = np.hstack([step,sys_pos])
    data_out = sys_pos
    # plt.figure()
    # plt.scatter(displacement[:,0], displacement[:,1],c=step[:,0],cmap='summer', zorder=200, linewidth = 0.5)
    # plt.plot(displacement[:,0], displacement[:,1],color='k', zorder=20, linewidth = 1)
    # plt.xlabel("x(mm)")
    # plt.ylabel("y(mm)")
    # # plt.title("Run {}".format(1+str(iter))
    # plt.show(block = False)
    # plt.pause(0.01)
    # iter = 0
    path = filename.format(1+iter)
    total_displacement[iter]=displacement[-1,1]
    # np.savetxt(path,data_out,\
        # header="step, ring_center (x), ring_center (y)",fmt="%d, %d, %d")
    np.savetxt(path,data_out)
    print("RUN NUMBER " + str(iter) + " SAVED")

print('\n\ntime:{}, run:{}, displacement:{}'.format(time.time()-t0,iter,total_displacement[iter]))
print('Average Displacement: {}'.format(sum(total_displacement)/runs))
print("Done!")


#Disconnect from GUI and pybullet server
p.disconnect()
plt.show()
