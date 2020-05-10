import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm
import matplotlib.pyplot as plt
import forward_kine as fk
import UR3CV as ur3cv
import inverse_kine as ik

from skimage.io import imread
import cv2

# ************************************ Define Constants ********************************************** #
# Calibrate the camera
beta = 312.002
Or = 255
Oc = 255
tx = 0.325
ty = 0.0

# Get distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint():
	X = []
	Y = []
	Z = []
	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Function that used to move joints
def SetJointPosition(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(0.5)

# Function that used to read joint angles
def GetJointAngle():
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return theta

def get_endor():
	_, vector=vrep.simxGetObjectPosition(clientID, hand_handle,base_handle,vrep.simx_opmode_blocking)
	# print("Hand trans:", vector)
	vector = np.round(vector, decimals=3)
	return vector

def get_xw_yw():
	result, sphere1_handle = vrep.simxGetObjectHandle(clientID, 'Sphere1', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for sixth joint')
	result, sphere2_handle = vrep.simxGetObjectHandle(clientID, 'Sphere2', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for sixth joint')
	_, vector1 = vrep.simxGetObjectPosition(clientID, sphere1_handle,base_handle,vrep.simx_opmode_blocking)
	_, vector2 = vrep.simxGetObjectPosition(clientID, sphere2_handle,base_handle,vrep.simx_opmode_blocking)
	return vector1, vector2

def get_cali_dist():
	vector1, vector2 = get_xw_yw()
	return np.linalg.norm(np.array(vector1) - np.array(vector2))

def get_beta(keypoints):
	exp_dist = get_cali_dist()
	img_dist = np.linalg.norm(np.array(keypoints[1]) - np.array(keypoints[0]))
	beta = img_dist / exp_dist
	return beta

def get_tx_ty(keypoints):
	R = np.array([[-1, 0], [0, 1]])
	vector, _ = get_xw_yw()
	xw = vector[0]
	yw = vector[1]
	r = keypoints[1][0] 
	c = keypoints[1][1]
	xc, yc = (np.array([r, c]) - np.array([Or,Oc])) / beta
	Pw = np.array([[xw],[yw]])
	Pc = np.array([[xc],[yc]])
	tx,ty = (Pc - np.dot(R,Pw))[:,0]
	return tx, ty

def get_real_coord(keypoints):
	real_coord = []
	for keypoint in keypoints:
		r = keypoint[0]
		c = keypoint[1]
		R = np.array([[-1, 0], [0, 1]])
		p_c = (np.array([[r],[c]]) - np.array([[Or],[Oc]])) / beta
		p_T = np.array([[tx],[ty]])
		p_w = np.matmul(np.linalg.inv(R), p_c - p_T)
		xw, yw = p_w[:,0]
		real_coord.insert(0, (xw, yw, 0))
	return real_coord

# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

'''
# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
print(stringData)
'''

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

# ==================================================================================================== #
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
time.sleep(1)
# UR3 home location
home = [0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0, 0*np.pi/180.0]
# Go to initial
SetJointPosition(home)

X,Y,Z = get_joint()
# print( [(X[i], Y[i], Z[i]) for i in range(len(X))])

M, S = fk.Get_MS(X,Y,Z,get_endor())

Goal_joint_angles = np.array([[0,0,-0.5*np.pi,0.5*np.pi,-0.5*np.pi,-0.5*np.pi], \
							[0.5*np.pi,0,-0.5*np.pi,0.5*np.pi,0.5*np.pi,-0.5*np.pi],\
							[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,-0.5*np.pi]])

global_view_handler = 'global_view'
robot_view_handler = 'robot_view'
res, v0 = vrep.simxGetObjectHandle(clientID, global_view_handler, vrep.simx_opmode_oneshot_wait)
err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_streaming)

detector = ur3cv.blob_search_init()

exp_dist = get_cali_dist()

res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_buffer)
img = np.array(image,dtype=np.uint8)
img.resize([resolution[1],resolution[0],3])
keypoints = ur3cv.blob_search(img, detector)
real_coord = get_real_coord(keypoints)

print("\n========================= Go to Sphere positions ===========================\n")
# print("Sphere position in Camera frame", keypoints)
# print("Sphere position in word frame: ", np.round(real_coord, 3).tolist())
print("Expected sphere position: ", np.round(get_xw_yw(), 3).tolist()[0])
# _, vector=vrep.simxGetObjectPosition(clientID, v0,base_handle,vrep.simx_opmode_blocking)
# print("Exp trans:", vector)
p = np.round(get_xw_yw(), 3).tolist()[0]
p[2] += 0.5
thetas, success = ik.get_ik_thetas(S, M, p, GetJointAngle())
print(thetas, success)
# p = [-0.25, 0.542,0.106]
# print(get_endor())
#print(p)
#SetJointPosition(target_theta)

# theta_e, success = ik.get_ik_thetas(S, M, p, 0, np.pi, gamma=None)
# print(theta_e, success)
# SetJointPosition(theta_e)
# print(get_endor())

# if success:
# 	theta_a = ik.get_traj_joint(home, theta_e)
	
# 	for thetas in theta_a:
# 		print("Passing Thetas:", thetas)
# 		SetJointPosition(thetas)
# else:
# 	print("Failed")
# print("Actual End Effector Location: ",get_endor())
# while True:
# 	pass
# init_T = fk.get_T(GetJointAngle(), M, S)
# lin_traj = ik.get_traj(init_T, T)
# #print(lin_traj)
# for X in lin_traj:
# 	print(X)
# 	time.sleep(1)
# 	thetas, success = ik.get_ik_thetas(S, M, X)
# 	print(thetas, success)
# 	if success:
# 		print(thetas)
# 		SetJointPosition(thetas)

# print("\n============================= Start Motion ===============================\n")
# for i in range(3):
# 	SetJointPosition(Goal_joint_angles[i])

# 	print("ORIGINAL END EFFECTOR:",get_endor())

# 	res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_buffer)
    
# 	T = fk.get_T(GetJointAngle(), M, S)
# 	#print("Trans calculated by Forward Kinematics: \n", T)
# 	exp_endor_trans = get_endor()
# 	#print("Expected Endor Trans:", exp_endor_trans, "\n")
# 	err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_buffer)

# 	# T = ik.get_expect_T(p)
# 	target_theta, success = ik.get_ik_thetas(S, M, T)
# 	if success:
# 		print("Target Theta get by IK:",target_theta)
# 		print("Going to IK returned thetas")
# 		time.sleep(1)
# 		# SetJointPosition(target_theta)
# 		print("RE_CAL END EFFECTOR:",get_endor())
		

# 	print("Expected Theta values:", Goal_joint_angles[i])

	# if err == vrep.simx_return_ok:
	# 	img = np.array(image,dtype=np.uint8)
	# 	img.resize([resolution[1],resolution[0],3])
	# 	ur3cv.blob_search(img, detector)
	# time.sleep(2)
	# print("Going to next set of values")

# Wait two seconds
time.sleep(2)
# Move the hand
gripper_handler = 'JacoHand'
for i in range(3):
	vrep.simxSetStringSignal(clientID,gripper_handler,'true',vrep.simx_opmode_oneshot)
	time.sleep(0.6)
	vrep.simxSetStringSignal(clientID,gripper_handler,'false',vrep.simx_opmode_oneshot)
	time.sleep(0.6)

# **************************************************************************************************** #

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)

print("\n==================== ** Simulation Ended ** ====================\n")

while 1:
	pass
# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #