from ur3_setup import *
from skimage.io import imread
import cv2
from scipy.linalg import expm,logm
import matplotlib.pyplot as plt
import UR3CV as ur3cv

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

def get_ball_coords():
    #Perceptron
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_streaming)

    detector = ur3cv.blob_search_init()

    exp_dist = get_cali_dist()

    res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_buffer)
    img = np.array(image,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    keypoints = ur3cv.blob_search(img, detector)
    real_coord = get_real_coord(keypoints)

    # print("Sphere position in Camera frame", keypoints)
    # print("Sphere position in word frame: ", np.round(real_coord, 3).tolist())
    # print("Expected sphere position: ", np.round(get_xw_yw(), 3).tolist()[1])
    # _, vector=vrep.simxGetObjectPosition(clientID, v0,base_handle,vrep.simx_opmode_blocking)
    # print("Exp trans:", vector)
    return real_coord