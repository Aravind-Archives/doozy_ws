
#-----------KINEMATIC INVERSION------------#

#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import math
import csv

# Global variable to store joint positions
joint_positions = []

# Open the CSV file in write mode
csv_file_path = '/home/robot/doozy_ws/src/robot_controller/scripts/my_path.csv'
csv_file = open(csv_file_path, 'w', newline='')
csv_writer = csv.writer(csv_file)

def joint_states_callback(msg):
    global joint_positions
    # Extract joint positions from the message
    joint_positions_1 = round(math.degrees(msg.position[0]), 2)
    joint_positions_2 = round(math.degrees(msg.position[1]), 2)
    joint_positions_3 = round(math.degrees(msg.position[2]), 2)
    joint_positions_4 = round(math.degrees(msg.position[3]), 2)
    joint_positions_5 = round(math.degrees(msg.position[4]), 2)
    joint_positions_6 = round(math.degrees(msg.position[5]), 2)
    joint_positions = [joint_positions_1, joint_positions_2, joint_positions_3, joint_positions_4, joint_positions_5, joint_positions_6]

def move_and_record(arm_group, pose_target):
    global joint_positions

    # Set a pose target for the arm
    arm_group.set_pose_target(pose_target)

    # Move the arm to the pose target
    plan = arm_group.go(wait=True)

    # Append the current joint positions to the CSV file
    csv_writer.writerow(joint_positions)

    # Clear the joint positions list after writing to the CSV file
    joint_positions = []

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('joint_states_listener1')

    # Initialize MoveIt! and the robot commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arduinobot_arm")

    # Wait for MoveIt! to initialize
    rospy.sleep(2.0)

    # Subscribe to the joint_states topic
    rospy.Subscriber("/arduinobot/joint_states", JointState, joint_states_callback)

    # Set a pose target for the arm and move
    pose_target1 = geometry_msgs.msg.Pose()
    pose_target1.position.x = 0.4
    pose_target1.position.y = 0.5
    pose_target1.position.z = 0.8
    pose_target1.orientation.x = 1.0
    pose_target1.orientation.y = 0.0
    pose_target1.orientation.z = 0.0
    pose_target1.orientation.w = -0.0
    move_and_record(arm_group, pose_target1)

    # Set another pose target for the arm and move
    pose_target2 = geometry_msgs.msg.Pose()
    pose_target2.position.x = 0.5
    pose_target2.position.y = 0.5
    pose_target2.position.z = 0.8
    pose_target2.orientation.x = 1.0
    pose_target2.orientation.y = 0.0
    pose_target2.orientation.z = 0.0
    pose_target2.orientation.w = -0.0
    move_and_record(arm_group, pose_target2)

    # Set another pose target for the arm and move
    pose_target3 = geometry_msgs.msg.Pose()
    pose_target3.position.x = 0.7
    pose_target3.position.y = 0.5
    pose_target3.position.z = 0.8
    pose_target3.orientation.x = 1.0
    pose_target3.orientation.y = 0.0
    pose_target3.orientation.z = 0.0
    pose_target3.orientation.w = -0.0
    move_and_record(arm_group, pose_target3)

    # Close the CSV file
    csv_file.close()
