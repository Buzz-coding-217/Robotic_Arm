import rospy
import moveit_commander
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.synchronization import RobotSynchronization
from moveit_commander.planning_interface import MoveGroupCommander
from moveit_commander.robot_trajectory import RobotTrajectory

def initialize_moveit():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_moveit', anonymous=True)
    
    # Initialize the MoveGroupCommander for both arms
    left_arm_group = MoveGroupCommander("left_arm")
    right_arm_group = MoveGroupCommander("right_arm")
    
    # Set reference frames
    left_arm_group.set_pose_reference_frame("base_link")
    right_arm_group.set_pose_reference_frame("base_link")
    
    # Optionally, set some basic parameters (you can tune these)
    left_arm_group.set_planning_time(10)
    right_arm_group.set_planning_time(10)
    
    # Return the MoveGroupCommander objects
    return left_arm_group, right_arm_group

def move_arm_to_home_position(arm_group):
    # Define the home position for the arm (all joints to 0)
    home_position = [0, 0, 0, 0, 0, 0]
    arm_group.go(home_position, wait=True)
    arm_group.stop()

def move_arm_to_pose(arm_group, pose):
    # Set the desired pose and plan
    arm_group.set_pose_target(pose)
    
    # Plan and execute
    plan = arm_group.plan()
    arm_group.go(wait=True)
    arm_group.stop()
    
def move_both_arms():
    # Initialize moveit
    left_arm_group, right_arm_group = initialize_moveit()
    
    # Move the arms to their home position
    rospy.loginfo("Moving left arm to home position...")
    move_arm_to_home_position(left_arm_group)
    
    rospy.loginfo("Moving right arm to home position...")
    move_arm_to_home_position(right_arm_group)

    # Example Pose for the left arm (set the target pose here)
    left_pose = left_arm_group.get_current_pose().pose
    left_pose.position.x += 0.1  # Move the arm in the x-direction for example
    left_pose.position.y += 0.1  # Move the arm in the y-direction for example
    
    # Example Pose for the right arm (set the target pose here)
    right_pose = right_arm_group.get_current_pose().pose
    right_pose.position.x -= 0.1  # Move the arm in the x-direction for example
    right_pose.position.y -= 0.1  # Move the arm in the y-direction for example
    
    # Move both arms to their desired poses
    rospy.loginfo("Moving left arm to the new pose...")
    move_arm_to_pose(left_arm_group, left_pose)
    
    rospy.loginfo("Moving right arm to the new pose...")
    move_arm_to_pose(right_arm_group, right_pose)

if __name__ == '__main__':
    try:
        # Start moving both arms
        move_both_arms()
        
    except rospy.ROSInterruptException:
        pass
