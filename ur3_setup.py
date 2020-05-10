import vrep
import time
import numpy as np

# ************************************ Define Constants ********************************************** #
# Calibrate the camera
beta = 312.002
Or = 255
Oc = 255
tx = 0.325
ty = 0.0
# UR3 home location
home = [0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0]
throw_prep1 = [0.5*np.pi, -0.5*np.pi, 0*np.pi, 0*np.pi, 0.5*np.pi, 0*np.pi]
throw_prep2 = [0.5*np.pi, 0*np.pi, 0*np.pi, 0*np.pi, 0.5*np.pi, 0*np.pi]


# ************************************ Build Connections ********************************************** #
# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #
# Get "handle" to the spheres
result, sphere1_handle = vrep.simxGetObjectHandle(clientID, 'Sphere1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
result, sphere2_handle = vrep.simxGetObjectHandle(clientID, 'Sphere2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')

gripper_handler = 'JacoHand'
result, hand_handle = vrep.simxGetObjectHandle(clientID, gripper_handler, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for hand')

gripper_handler = 'JacoHand'
result, force_handle = vrep.simxGetObjectHandle(clientID, "hand_force_sensor", vrep.simx_opmode_blocking)

global_view_handler = 'global_view'
robot_view_handler = 'robot_view'
res, v0 = vrep.simxGetObjectHandle(clientID, global_view_handler, vrep.simx_opmode_oneshot_wait)

joint_handles = [joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle]
sphere_handles = [sphere1_handle, sphere2_handle]