from ur3_setup import *
from sense import *
from motion import *
from math import cos, sin
import forward_kine as fk
import inverse_kine as ik


throw_angle1 = [0.538*np.pi, 0.555*np.pi]
throw_angle3 = [0.732*np.pi, 0.724*np.pi]

def pick_up_ball(p,sphere_handle):
    garbage = get_ball_coords()

    target_theta = ik.invk(p[0], p[1], p[2])
    SetJointPosition(target_theta)
    close_gripper()
    #Perform a pseudo grip
    vrep.simxSetObjectParent(clientID, sphere_handle, force_handle, True ,vrep.simx_opmode_oneshot)
    vrep.simxSetObjectPosition(clientID, sphere_handle, force_handle, (0,0,0), vrep.simx_opmode_blocking)
    
    SetJointPosition(throw_prep1)
    time.sleep(0.5)
    SetJointPosition(throw_prep2)
    time.sleep(3)

    move_single_joint(6, 0.5*np.pi)
    time.sleep(0.5)
    change_joint_velocity(2, 60)
    change_joint_velocity(3, 60)
    time.sleep(0.5)
    move_single_joint(1, throw_angle1[i])
    move_single_joint(3, throw_angle3[i])
    move_single_joint(2, -0.4*np.pi)

    time.sleep(0.5)
    move_single_joint(3, -0.1*np.pi)
    # time.sleep(0.5)
    vrep.simxSetObjectParent(clientID, sphere_handle, -1, False ,vrep.simx_opmode_oneshot)
    open_gripper()
    change_joint_velocity(2, 2.5)
    change_joint_velocity(3, 2.5)
    # SetJointPosition(home)
    time.sleep(2)


# ==================================================================================================== #
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

#Start Control
time.sleep(1)


# Go to initial
SetJointPosition(home)
time.sleep(1)

#Get coordinates
real_coord = get_ball_coords()
Ps = np.round(real_coord, 3).tolist()

for i in range(len(Ps)):
    sphere_handle = sphere_handles[i]
    p = Ps[i]
    p[2] += 0.15
    pick_up_ball(p, sphere_handle)

# **************************************************************************************************** #
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)

print("\n==================== ** Simulation Ended ** ====================\n")

# while 1:
# 	pass
# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
