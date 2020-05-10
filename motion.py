from ur3_setup import *

# Function that used to move joints
def SetJointPosition(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	# time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	# time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	# time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	# time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	# time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(1)

def close_gripper():
    #Pick the ball
    vrep.simxSetStringSignal(clientID,gripper_handler,'true',vrep.simx_opmode_blocking)
    time.sleep(1)

def open_gripper():
    vrep.simxSetStringSignal(clientID,gripper_handler,'false',vrep.simx_opmode_blocking)
    time.sleep(0.6)

def change_joint_velocity(joint, velocity):
    jt_handle = joint_handles[joint - 1]
    vrep.simxSetObjectFloatParameter(clientID, jt_handle, 2017, velocity, vrep.simx_opmode_oneshot)

def move_single_joint(joint, angle):
    jt_handle = joint_handles[joint - 1]
    vrep.simxSetJointTargetPosition(clientID, jt_handle, angle, vrep.simx_opmode_oneshot)