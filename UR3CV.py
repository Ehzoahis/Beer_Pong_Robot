#!/usr/bin/env python

import cv2
import numpy as np 
import matplotlib.pyplot as plt

"""
To init blob search params, will be init (called) in the ImageConverter class
"""

def blob_search_init():

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    ################# Your Code Starts Here #################

    # Filter by Color 
    params.filterByColor = False


    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 280   

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.3
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False


    # Filter by Convexity
    params.filterByConvexity = False


    # Any other params to set???


    ################## Your Code Ends Here ##################

    # Create a detector with the parameters
    blob_detector = cv2.SimpleBlobDetector_create(params)

    return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

    # Convert the color image into the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    ############################ Your Code Starts Here ############################

    # Find lower & upper for orange
    
    lower = (90,100,100)      # yellow lower
    upper = (100,255,255)   # yellow upper

    ############################# Your Code Ends Here #############################


    # Define a mask using the lower and upper bounds of the orange color 
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

    blob_image_center = []

    # Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in 
    # crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

    # Make sure this blob center is in the full image pixels not the cropped image pixels
    blob_detector = detector
    keypoints = blob_detector.detect(mask_image)
    for i in range(len(keypoints)):
        new_coords = (keypoints[i].pt[0], keypoints[i].pt[1])
        blob_image_center.append([int(new_coords[0]), int(new_coords[1])])
    #print(keypoints)


    # Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    # Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
    # under that pixel location and see what the HSV values are for that color. 

    cv2.namedWindow("Masked Window")
    cv2.imshow("Masked Window", mask_image)

    cv2.namedWindow("Vision Window")
    cv2.imshow("Vision Window", im_with_keypoints)

    cv2.waitKey(2)

    return blob_image_center
