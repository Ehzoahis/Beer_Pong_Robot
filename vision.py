#!/usr/bin/env python

import vrep
import time

vrep.simxFinish(-1)

#clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

gripper_handler = 'JacoHand'
vrep.simxSetStringSignal(clientID,gripper_handler,'true',vrep.simx_opmode_oneshot)
time.sleep(0.6)
vrep.simxSetStringSignal(clientID,gripper_handler,'false',vrep.simx_opmode_oneshot)
time.sleep(0.6)
if clientID!=-1:
  print ('Connected to remote API server')

  res, v0 = vrep.simxGetObjectHandle(clientID, 'robot_view', vrep.simx_opmode_oneshot_wait)

  res, v1 = vrep.simxGetObjectHandle(clientID, 'passive_sensor', vrep.simx_opmode_oneshot_wait)

  err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_streaming_split)
  time.sleep(1)
  while vrep.simxGetConnectionId(clientID) != -1:
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_buffer)
    if err == vrep.simx_return_ok:
      vrep.simxSetVisionSensorImage(clientID, v1, image, 0, vrep.simx_opmode_oneshot)
    elif err == vrep.simx_return_novalue_flag:
      print("no image yet")
      pass
    else:
      print(err)
else:
  print("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)