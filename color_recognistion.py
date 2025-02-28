#!/usr/bin/python3
import os
os.environ["ROS_NAMESPACE"] = "/my_gen3_lite"
import sys
import rospy
import moveit_commander
from typing import Tuple, Union
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotTrajectory
from geometry_msgs.msg import Pose, Point,Quaternion, PoseStamped, PoseArray
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
import random
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import numpy as np
# from tf2_geometry_msgs import PoseStamped
PlanTuple = Tuple[bool, RobotTrajectory, float, MoveItErrorCodes]
inside = False

# Function to detect brown color in the image
def detect_brown_color(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    
    # Adjusted brown range to capture only brown hues
    lower_brown = np.array([10, 50, 50])  # Adjust these values as needed
    upper_brown = np.array([20, 255, 255]) # Adjust these values as needed
    
    brown_mask = cv2.inRange(hsv_image, lower_brown, upper_brown)
    brown_detected = cv2.bitwise_and(image, image, mask=brown_mask)
    return brown_detected, brown_mask

# Function to detect yellow color in the image
def detect_yellow_color(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_yellow = np.array([0, 100, 100])  # Adjust these values as needed
    upper_yellow = np.array([10, 255, 255])  # Adjust these values as needed
    yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    yellow_detected = cv2.bitwise_and(image, image, mask=yellow_mask)
    return yellow_detected, yellow_mask

# Improved function to calculate lengths and percentages
def calculate_lengths_and_percentages(yellow_mask, brown_mask):
    contours_brown, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours_brown:  # Check if any brown contours were found
        return {"above": 0, "below": 0, "percentage_above": 0, "percentage_below": 0}

    ruler_contour = max(contours_brown, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(ruler_contour)

    # Create masks for above and below based on the bounding box of the brown ruler
    mask_above = np.zeros_like(yellow_mask, dtype=np.uint8)
    mask_below = np.zeros_like(yellow_mask, dtype=np.uint8)
    mask_above[:y, :] = yellow_mask[:y, :]
    mask_below[y+h:, :] = yellow_mask[y+h:, :]
    
    length_above = cv2.countNonZero(mask_above)
    length_below = cv2.countNonZero(mask_below)
    total_length = length_above + length_below
    # print(total_length)
    
    # Ensure we avoid division by zero if the total length is zero
    if total_length > 0:
        percentage_above = (length_above / total_length * 100)
        percentage_below = (length_below / total_length * 100)
    else:
        percentage_above = 0
        percentage_below = 0
    if percentage_above < 65:
        print("The cloth is in the hanger")
        robot.set_inside(True)
        print(inside)
    return {
        "above": length_above,
        "below": length_below,
        "percentage_above": percentage_above,
        "percentage_below": percentage_below
    }

# Callback function for the ROS subscriber
def callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Detect brown and yellow regions
        brown_image_rgb, brown_mask = detect_brown_color(cv_image_rgb)
        yellow_image_rgb, yellow_mask = detect_yellow_color(cv_image_rgb)

        # Calculate lengths and percentages
        results = calculate_lengths_and_percentages(yellow_mask, brown_mask)

        # Convert RGB back to BGR for OpenCV visualization
        brown_image_bgr = cv2.cvtColor(brown_image_rgb, cv2.COLOR_RGB2BGR)
        yellow_image_bgr = cv2.cvtColor(yellow_image_rgb, cv2.COLOR_RGB2BGR)
        cv_image_bgr = cv2.cvtColor(cv_image_rgb, cv2.COLOR_RGB2BGR)

        # Combine the images for display
        combined_display = np.hstack((cv_image_bgr, yellow_image_bgr, brown_image_bgr))

        # Add text annotations
        cv2.putText(combined_display, f"Above: {results['above']} px, Below: {results['below']} px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(combined_display, f"% Above: {results['percentage_above']:.2f}%, % Below: {results['percentage_below']:.2f}%",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Display the combined image
        cv2.imshow("Image Analysis", combined_display)

        # Close the window when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown")

    except Exception as e:
        rospy.logerr(f"Error: {e}")


class RobotInitialization:
    def __init__(self):
        
        # Initialize moveit! package
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Group names of the robot, found it by running the actual robot
        self.arm_group_name = 'arm'
        self.gripper_group_name = 'gripper'
        
        # Robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        # Interfaces for planning groups (group of joints) to plan an execute motions
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name, ns=rospy.get_namespace())

        # Ros publisher that is used to display trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace()+'/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        self.init_scene()
        self.inside = False
        # self.init_pose()
        
        self.andy =  np.array([0, 
                               -82 * pi/180, 
                               99 * pi/180, 
                               -90 * pi/180, 
                               92 * pi/180, 
                               90 * pi/180])

    def get_inside(self):
        return self.inside

    # Setter function for inside
    def set_inside(self, value: bool):
        self.inside = value

    def move_gripper(self, value: float):
        '''
        Moves the gripper to the relative positions.

        Args:
            value (float): a value from 0 to 1 to scale the gripper
        '''
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        if not gripper_joint_names:
            rospy.logerr("Gripper joint names not found in ROS parameters.")
            return

        # We only need one joint name since the rest will respond to one of them
        gripper_joint_name = gripper_joint_names[0]
        gripper_joint = self.robot.get_joint(gripper_joint_name)

        # Get the bounds for calculating the gripper's desired position
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()

        rospy.loginfo(f"Gripper bounds: Min: {gripper_min_absolute_pos}, Max: {gripper_max_absolute_pos}")

        # Calculate the target position
        target_position = value * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos
        rospy.loginfo(f"Moving gripper to target position: {target_position}")

        # Move the joint
        try:
            gripper_joint.move(target_position)
            rospy.loginfo("Gripper move command issued successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to move the gripper: {e}")

    
    def get_current_cartesian_pose(base_client):
        feedback = base_client.GetMeasuredCartesianPose()
        print(f"X: {feedback.x} m")
        print(f"Y: {feedback.y} m")
        print(f"Z: {feedback.z} m")
        print(f"Theta X: {feedback.theta_x} deg")
        print(f"Theta Y: {feedback.theta_y} deg")
        print(f"Theta Z: {feedback.theta_z} deg")
    
    # def init_scene(self):
    #     '''
    #     Adding a table under the robot to ensure it does not hit the table in reality
    #     '''
    #     table_size = [2, 2, 0.86]
        
    #     table_pose = geometry_msgs.msg.PoseStamped()
    #     table_pose.header.frame_id = "base_link"
    #     table_pose.pose.position.x = 0
    #     table_pose.pose.position.y = 0
    #     table_pose.pose.position.z = -table_size[2]/2-0.00001 
    #     table_name = "table"
    #     self.scene.add_box(table_name, table_pose, size=table_size)

    def init_scene(self):
        '''
        Adding a table under the robot to ensure it does not hit the table in reality.
        The table is rotated to the left (90 degrees around the Z-axis).
        The arm is positioned at the edge of the table:
        0.2m from the new left edge (longer side) and 0.03m from the new shorter side.
        The table will now be placed symmetrically on the other side with the same distance from the other end.
        '''
        import tf.transformations as tf

        # Define table dimensions: length, width, and height
        table_size = [1.52, 0.60, 0.95]  # Length, width, and height in meters

        # Define the table pose relative to the robot's base frame
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"  # Reference frame is the robot's base link

        # Position the table with respect to the robot after rotation
        # Since the longer side is now along the Y-axis:
        # Move the table to the other side symmetrically, keeping the same distance from the edge
        table_pose.pose.position.x = -(0.60 / 2) + 0.57  # Adjust for the shorter side
        table_pose.pose.position.y = (1.52 / 2) - 0.2  # Move to the other side symmetrically (same distance from the other edge)
        table_pose.pose.position.z = -table_size[2] / 2  # Half the height below the base frame

        # Rotate the table to the left (around the Z-axis)
        # Convert rotation angle to quaternion (90 degrees to the left)
        rotation_angle = 90 * (pi / 180)  # 90 degrees in radians
        quaternion = tf.quaternion_from_euler(0, 0, rotation_angle)  # Roll, pitch, yaw

        # Apply the quaternion to the pose
        table_pose.pose.orientation.x = quaternion[0]
        table_pose.pose.orientation.y = quaternion[1]
        table_pose.pose.orientation.z = quaternion[2]
        table_pose.pose.orientation.w = quaternion[3]

        # Name the table for the planning scene
        table_name = "table"

        # Add the table to the MoveIt planning scene
        self.scene.add_box(table_name, table_pose, size=table_size)






    # def init_pose(self):
    #     '''
    #     Hard-coded home pose (for easy_handeye package) of the robot
    #     '''
    #     # home pose of the robot
    #     joint_values = self.arm_group.get_current_joint_values()
    #     joint_values[0] = -48 * pi / 180
    #     joint_values[1] = 38 * pi / 180
    #     joint_values[2] = 137 * pi / 180
    #     joint_values[3] = 94 * pi / 180
    #     joint_values[4] = 74 * pi / 180
    #     joint_values[5] = 39 * pi / 180
    #     # joint_values[0] = 0
    #     # joint_values[1] = 0
    #     # joint_values[2] = 0
    #     # joint_values[3] = 0
    #     # joint_values[4] = 0
    #     # joint_values[5] = 0        
    #     self.arm_group.go(joint_values, wait=True)
    #     # self.move_gripper(1)

    def init_pose(self):
        """
        Set the robot's starting pose to the specified Cartesian pose.
        """
        rospy.loginfo("Setting robot to the starting position.")

        # Define the desired starting position and orientation
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.20049683525090756
        target_pose.position.y = -0.049368864525383416
        target_pose.position.z = 0.305482888579287
        target_pose.orientation.x = -0.0030826892238394335
        target_pose.orientation.y = 0.9996955936869681
        target_pose.orientation.z = -0.024171962986500286
        target_pose.orientation.w = 0.0038643492840737172

        # Set the target pose and plan movement
        self.arm_group.set_pose_target(target_pose)

        # Plan and execute the movement
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()  # Ensure no residual movement
        self.arm_group.clear_pose_targets()  # Clear targets after execution

        if success:
            rospy.loginfo("Successfully moved the robot to the starting position.")
        else:
            rospy.logerr("Failed to move the robot to the starting position.")

        
    def get_cartesion_pose(self):
        '''
        Get the current pose and display it
        '''
        pose: PoseStamped = self.arm_group.get_current_pose()
        
        print(f'The cartesian pose is:')
        print(pose.pose)
        quat = pose.pose.orientation
        print(f'The rotations angles:{euler_from_quaternion([quat.w, quat.x, quat.y, quat.z])}')
        return pose
    

    def get_arm_joint_values(self):
        '''
        Get the current joint and display it
        '''
        joints = self.arm_group.get_current_joint_values()
        
        for i in range(len(joints)):
            print(f"- joint_{i+1}: {joints[i]}")
            
        return joints
    
    
    def reach_joint_angles(self, values):
        '''
        vertical pose of the robot
        '''
        joint_values = self.arm_group.get_current_joint_values()
        joint_values = values
        self.arm_group.go(joint_values, wait=True)
        

    
    def camera_calibration_pose(self):
        '''
        Hard-coded calibration pose (for easy_handeye package) of the robot 
        '''

        # Gripper Pose
        # self.move_gripper(0.85)
        # Calibration pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[0] = 6 * pi/180
        joint_values[1] = -69 * pi / 180
        joint_values[2] = 94 * pi / 180
        joint_values[3] = -90 * pi / 180
        joint_values[4] = 129 * pi / 180
        joint_values[5] = 82 * pi / 180
        self.arm_group.go(joint_values, wait=True)
        
    def move(self, target):
        # self.arm_group.set_pose_target(target)
        # return True
        attempted = False

        self.arm_group.set_joint_value_target(target, True)
        plan_tuple: PlanTuple = self.arm_group.plan()
        plan = self.unpack_plan(plan_tuple)
        answer = input("Press Enter to proceed or q to replot")
        if answer != "q":
            attempted = self.arm_group.execute(plan, wait=True)

            
            
        return attempted
    
    
    def unpack_plan(self, plan_tuple: PlanTuple) -> Union[RobotTrajectory, None]:
        """Function used to unpack the tuple returned when planning with move_group.
        This seems to be different than is was in ros melodic, so this function
        is needed to adapt the old code to the changes.

        Args:
            plan_tuple: A plan tuple containing the plan and other success data.

        Returns:
            If the planning was successful, a trajectory that can be directly used for
            visualization and motion. If unsuccessful, None is returned.
        """

        # plan_tuple[0] is the success boolean flag
        if plan_tuple[0]:
            return plan_tuple[1]  # The RobotTrajectory
        else:
            # If needed, the exact error code can be parsed from plan_tuple[3]
            return None

    def move_to_cartesian_pose(self, position, orientation):
        """
        Moves the arm to the specified Cartesian pose and handles gripper actions.

        Args:
            position (dict): A dictionary with 'x', 'y', 'z' keys for the target position.
            orientation (dict): A dictionary with 'x', 'y', 'z', 'w' keys for the target orientation.
        """
        # Open the gripper before moving

        # Define the target pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = position['x']
        target_pose.position.y = position['y']
        target_pose.position.z = position['z']
        target_pose.orientation.x = orientation['x']
        target_pose.orientation.y = orientation['y']
        target_pose.orientation.z = orientation['z']
        target_pose.orientation.w = orientation['w']

        # Set the target pose
        self.arm_group.set_pose_target(target_pose)

        # Plan and execute the movement
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()  # Ensure no residual movement
        self.arm_group.clear_pose_targets()  # Clear targets after execution

def move_robot_in_reverse_order(robot):
    # Pose before the target location
        initial_position = {
            'x':  0.12648339905329067,
            'y': 0.16615180785148495,
            'z': 0.5119442787725982,
        }
        initial_orientation = {
            'x': -0.6185920284120413,
            'y': -0.7643867476633501,
            'z': -0.12567024620638037,
            'w': 0.13139174859994157,
        }

        position_1 = {
            'x': 0.19202992289366747,
            'y': 0.3043798452540504,
            'z': 0.6620082523496882,
        }
        orientation_1 = {
            'x': -0.5308801140067492,
            'y': -0.6013626773804739,
            'z': -0.5284313645556914,
            'w': 0.27800994183622324,
        }

        position_2 = {
            'x': 0.21784437509245486,
            'y': 0.34053995946068766,
            'z': 0.6191976480173325,
        }
        orientation_2 = {
            'x': -0.5599299408344248,
            'y': -0.6154649119444201,
            'z': -0.4828871988761347,
            'w': 0.27294936652054347,
        }

        final_position = {
            'x': 0.2747719335258573,
            'y': 0.4202574247639189,
            'z': 0.47268055505315854,
        }
        final_orientation = {
            'x': -0.6330581796646602,
            'y': -0.6449789775064566,
            'z': -0.3447203495127915,
            'w': 0.25378601294431513,
        }

        # Move the robot to the final position
        rospy.loginfo("Moving the robot to the Final Position...")
        robot.move_to_cartesian_pose(final_position, final_orientation)
        rospy.loginfo("Robot at the Final Position.")

        # Move the robot to Position 2
        rospy.loginfo("Moving the robot to Position 2...")
        robot.move_to_cartesian_pose(position_2, orientation_2)
        rospy.loginfo("Robot at Position 2.")

        # Move the robot to Position 1
        rospy.loginfo("Moving the robot to Position 1...")
        robot.move_to_cartesian_pose(position_1, orientation_1)
        rospy.loginfo("Robot at Position 1.")

        # Move the robot to the initial position
        rospy.loginfo("Moving the robot to the initial pose...")
        robot.move_to_cartesian_pose(initial_position, initial_orientation)
        rospy.loginfo("Robot moved to the initial pose.")



if __name__ == "__main__":

    rospy.init_node("test")
    robot = RobotInitialization()
    # robot.camera_calibration_pose()
    robot.init_pose()
    
    # # robot.move(robot.andy)
    # rospy.logwarn(robot.get_cartesion_pose())

     # Pose before the target location
    initial_position = {
        'x':  0.12648339905329067,
        'y': 0.16615180785148495,
        'z': 0.5119442787725982,
    }
    initial_orientation = {
        'x': -0.6185920284120413,
        'y': -0.7643867476633501,
        'z': -0.12567024620638037,
        'w': 0.13139174859994157,
    }

    position_1 = {
        'x': 0.19202992289366747,
        'y': 0.3043798452540504,
        'z': 0.6620082523496882,
    }
    orientation_1 = {
        'x': -0.5308801140067492,
        'y': -0.6013626773804739,
        'z': -0.5284313645556914,
        'w': 0.27800994183622324,
    }

    position_2 = {
        'x': 0.21784437509245486,
        'y': 0.34053995946068766,
        'z': 0.6191976480173325,
    }
    orientation_2 = {
        'x': -0.5599299408344248,
        'y': -0.6154649119444201,
        'z': -0.4828871988761347,
        'w': 0.27294936652054347,
    }

    final_position = {
        'x': 0.2747719335258573,
        'y': 0.4202574247639189,
        'z': 0.47268055505315854,
    }
    final_orientation = {
        'x': -0.6330581796646602,
        'y': -0.6449789775064566,
        'z': -0.3447203495127915,
        'w': 0.25378601294431513,
    }


    rospy.loginfo("Initial position: {}".format(initial_position))
    rospy.loginfo("Initial orientation: {}".format(initial_orientation))

    # Move the robot to the initial pose
    # rospy.loginfo("Moving the robot to the initial pose...")
    # robot.move_to_cartesian_pose(initial_position, initial_orientation)
    # rospy.loginfo("Robot moved to the initial pose.")

    # rospy.loginfo("Moving the robot to Position 1...")
    # robot.move_to_cartesian_pose(position_1, orientation_1)
    # rospy.loginfo("Robot at Position 1.")
    i = 0
    
    while i < 3 and not robot.get_inside():
        rospy.loginfo("Moving the robot to Position 2...")
        robot.move_to_cartesian_pose(position_2, orientation_2)
        rospy.loginfo("Robot at Position 2.")

        rospy.loginfo("Moving the robot to the Final Position...")
        robot.move_to_cartesian_pose(final_position, final_orientation)
        rospy.loginfo("Robot at the Final Position.")
        i += 1
        rospy.Subscriber('/camera/color/image_raw', Image, callback)
        print(robot.get_inside())

        # Cleanup (This part is reached after rospy.spin() is stopped)
        cv2.destroyAllWindows()


    # move_robot_in_reverse_order(robot)
   