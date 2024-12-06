#!/usr/bin/env python
import os
import sys
# Get the root directory (one level up from src)
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
sys.path.append(root_dir)
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from src.vision.capture_data import Camera
from src.robot_model import GEN3_LITE
from scipy import optimize  
from mpl_toolkits.mplot3d import Axes3D  
from math import pi
from geometry_msgs.msg import Pose
from src.utils import *
import rospy

rospy.init_node("Calibrate")
# User options (change me)
# --------------- Setup options ---------------
rad2deg = 180/pi
workspace_limits = WORKSPACE_LIMITS # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.05
checkerboard_offset_from_tool = [-0.075,0,0.03]
tool_orientation = [90, 0, 90] # [0,-2.22,2.22] # [2.22,2.22,0]

# ---------------------------------------------


# Move robot to home pose
robot = GEN3_LITE()
camera = Camera()

# open the gripper
robot.move_gripper(0)
robot.move_trajectories(robot.calibration)


# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = camera.get_data()

def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, robot, click_point_pix
        click_point_pix = (x,y)

        # Get click point in camera coordinates
        click_z = camera_depth_img[y][x] * robot.cam_depth_scale
        click_x = np.multiply(x-camera.intrinsics[0][2],click_z/camera.intrinsics[0][0])
        click_y = np.multiply(y-camera.intrinsics[1][2],click_z/camera.intrinsics[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x,click_y,click_z])
        click_point.shape = (3,1)

        # Convert camera to robot coordinates
        # camera2robot = np.linalg.inv(robot.cam_pose)
        camera2robot = robot.cam_pose
        print(robot.cam_pose)
        target_position = np.dot(camera2robot[0:3,0:3],click_point) + camera2robot[0:3,3:]

        target_position = target_position[0:3,0]
        print(target_position)
        
    
        
        robot.move_pose(target_position, tool_orientation)


# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    camera_color_img, camera_depth_img = camera.get_data()
    bgr_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        bgr_data = cv2.circle(bgr_data, click_point_pix, 7, (0,0,255), 2)
    cv2.imshow('color', bgr_data)
    cv2.imshow('depth', camera_depth_img)
    
    if cv2.waitKey(1) == ord('c'):
        break

cv2.destroyAllWindows()
