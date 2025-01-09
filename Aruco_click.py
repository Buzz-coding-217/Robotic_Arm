#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import geometry_msgs.msg

# Global variables
clicked_points = []
marker_position = None
depth_scale = 0.001  # Depth scale for RealSense (adjust according to your camera's calibration)

# Camera intrinsic parameters (adjust these values based on your camera calibration)
fx = 615.0  # Focal length in x
fy = 615.0  # Focal length in y
cx = 320.0  # Optical center in x (image center)
cy = 240.0  # Optical center in y (image center)

# Marker size
marker_size = 0.2  # Size of the ArUco marker in meters (adjust as needed)

# Callback for mouse click event
def mouse_callback(event, x, y, flags, param):
    """
    This function is called whenever a mouse event occurs in the OpenCV window.
    It captures the clicked coordinates and computes the corresponding 3D position.
    """
    global clicked_points, marker_position
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Capture the clicked point (x, y) and map to (0, 0) being bottom-left
        height, width = param.shape[:2]
        
        # Map coordinates: bottom-left as (0, 0) and top-right as highest
        mapped_x = x
        mapped_y = height - y  # Flip y to make bottom-left (0, 0)
        
        # Save the clicked point
        clicked_points.append((mapped_x, mapped_y))
        
        # Log the 2D coordinates (rounded to 2 decimals)
        rospy.loginfo(f"Clicked at 2D: ({mapped_x:.2f}, {mapped_y:.2f})")
        
        if marker_position:
            # Project the 3D marker position to 2D (for visualization)
            marker_2d = project_3d_to_2d(marker_position.x, marker_position.y, marker_position.z)
            #rospy.loginfo(f"Marker 2D position in image: ({marker_2d[0]:.2f}, {marker_2d[1]:.2f})")
            
            # Compute corresponding 3D position from clicked point (if depth is available)
            depth = get_depth_from_image(mapped_x, mapped_y, param)
            if depth is not None:
                # Using depth and camera intrinsic parameters to compute the 3D position
                X, Y, Z = pixel_to_3d(mapped_x, mapped_y, depth)
                rospy.loginfo(f"3D Coordinates: X: {X:.2f}, Y: {Y:.2f}, Z: {Z:.2f}")
                rospy.loginfo(f"Marker Position -> x: {marker_position.x:.2f}, y: {marker_position.y:.2f}, z: {marker_position.z:.2f}")

# Function to get depth from depth image (you need depth image for this to work)
def get_depth_from_image(x, y, image):
    """
    Function to extract depth at the (x, y) pixel location from the depth image.
    In this example, we're assuming a static depth (replace with actual depth data).
    """
    # Placeholder for depth extraction, assume 3 meters for now
    return 3.0  # Replace with actual depth from a depth image

# Convert 2D pixel coordinates to 3D world coordinates
def pixel_to_3d(x, y, depth):
    """
    Converts 2D pixel coordinates to 3D world coordinates using the camera intrinsic parameters.
    """
    # Convert pixel coordinates to camera coordinates (in meters)
    X = (x - cx) * depth / fx
    Y = (y - cy) * depth / fy
    Z = depth  # Depth (in meters)

    return X, Y, Z

def project_3d_to_2d(x, y, z):
    """
    Project 3D world coordinates (x, y, z) onto a 2D image plane using the camera intrinsic parameters.
    """
    # Project 3D coordinates to 2D pixel coordinates
    u = fx * x / z + cx
    v = fy * y / z + cy
    return u, v

def marker_callback(msg):
    """
    This callback is triggered whenever a new message is received on the /aruco_single/marker topic.
    It stores the 3D position (x, y, z) of the marker.
    """
    global marker_position
    # Extract the position from the pose field
    marker_position = msg.pose.position
    #rospy.loginfo(f"Marker Position -> x: {marker_position.x:.2f}, y: {marker_position.y:.2f}, z: {marker_position.z:.2f}")

def image_callback(msg):
    """
    Callback function to receive and process the image data.
    Converts the ROS Image message to a format OpenCV can use and displays it.
    """
    bridge = CvBridge()
    
    # Convert ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Display the image using OpenCV
    cv2.imshow("RealSense Camera Feed", cv_image)
    
    # Draw a larger marker at the 2D projection of the marker position for visibility
    if marker_position:
        marker_2d = project_3d_to_2d(marker_position.x, marker_position.y, marker_position.z)
        cv2.circle(cv_image, (int(marker_2d[0]), int(marker_2d[1])), 15, (0, 0, 255), -1)  # Draw a red circle
        #rospy.loginfo(f"Marker projected at 2D position: ({marker_2d[0]:.2f}, {marker_2d[1]:.2f})")

    # Wait for the user to click on the image
    cv2.setMouseCallback("RealSense Camera Feed", mouse_callback, cv_image)
    
    # Refresh the image window
    cv2.waitKey(1)

def main():
    """
    Initializes the ROS node and subscribes to the /camera/color/image_raw and /aruco_single/marker topics.
    It listens for Image and Marker messages, and computes the 3D position from clicks.
    """
    # Initialize the ROS node
    rospy.init_node('marker_position_listener', anonymous=True)

    # Subscribe to the /aruco_single/marker topic, expecting visualization_msgs/Marker messages
    rospy.Subscriber('/aruco_single/marker', Marker, marker_callback)

    # Subscribe to the RealSense camera's color image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Create an OpenCV window
    cv2.namedWindow("RealSense Camera Feed")
    
    # Start listening for messages
    rospy.loginfo("Waiting for RealSense images and marker data...")
    rospy.spin()

    # Cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
