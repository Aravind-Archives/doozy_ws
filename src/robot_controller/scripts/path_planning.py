#! /usr/bin/env python3
import time
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import csv
from tf.transformations import quaternion_from_euler
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import mplcursors

filename = 'co-ordinates.csv'

# Define the input and output file paths
# Replace with the path to your input CSV file
input_file_path = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_complex (copy).csv'
# Replace with the desired output file path
output_file_path = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_complex1.csv'

# Open the input CSV file and create the output CSV file
with open(input_file_path, 'r') as input_file, open(output_file_path, 'w', newline='') as output_file:
    csv_reader = csv.reader(input_file)
    csv_writer = csv.writer(output_file)

    for row in csv_reader:
        formatted_row = []
        for value in row:
            try:
                # Try to convert to float and then to string
                formatted_value = str(float(value))
                formatted_row.append(formatted_value)
            except ValueError:
                # If conversion fails, keep the original value
                formatted_row.append(value)

        csv_writer.writerow(formatted_row)

print("Conversion complete. Check the 'output.csv' file.")


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('ik1', anonymous=True)

        self._planning_group = "arduinobot_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_num_planning_attempts(5)  # Set a lower number of planning attempts
        self._group.set_planning_time(0.05)
        self._group.set_planner_id("RRTstar")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        
        global flag_plan

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        # self._group.set_goal_tolerance(0.001)
        self._group.set_num_planning_attempts(5)  # Set a lower number of planning attempts
        self._group.set_planning_time(0.05)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    # Position Offset:
    x_angle = input("Enter x-Orientation:")
    roll = math.radians(float(x_angle))
    y_angle = input("Enter y-Orientation:")
    pitch = math.radians(float(y_angle))
    z_angle = input("Enter z-Orientation:")
    yaw = math.radians(float(z_angle))
    quaternions = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    print(quaternions)

    # Orientation Offset:
    x_offset = input("Enter x-Position: ")
    y_offset = input("Enter y-Position: ")
    z_offset = input("Enter z-Position: ")
    print("The Offsets are:", x_offset, y_offset, z_offset)

    # Cycle Offset:
    input_values = []
    for i in range(3):
        user_input = input(f"Enter value {i + 1}: ")
        input_values.append(float(user_input))

    # Name of the existing CSV file and the new CSV file
    existing_csv_file = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_complex1.csv'
    new_csv_file = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_offset_waypoints.csv'

    # Read the existing CSV file and append the new row
    with open(existing_csv_file, 'r', newline='') as infile, open(new_csv_file, 'w', newline='') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)

        # Copy existing rows
        for row in reader:
            writer.writerow(row)

        # Append the new row
        writer.writerow(input_values)

    print(f"Data has been added to {new_csv_file}.")

    # Replace with your input CSV file
    input_file = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_offset_waypoints.csv'
    # Replace with your desired output CSV file
    output_file = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_waypoints.csv'

    # Function to remove commas and replace with spaces
    def remove_commas_replace_with_spaces(input_csv_file, output_csv_file):
        with open(input_csv_file, 'r') as infile, open(output_csv_file, 'w') as outfile:
            for line in infile:
                # Remove commas and replace with spaces
                modified_line = line.replace(',', ' ')  
                outfile.write(modified_line)

    if __name__ == '__main__':
        remove_commas_replace_with_spaces(input_file, output_file)
    print(
        f'Commas removed and replaced with spaces in "{input_file}" and saved to "{output_file}".')

#-------------------------------------------------------------------------------------------------------------------------------
    plt.ion()  # Turn on interactive mode
    fig = plt.figure(figsize=(10,5))
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title("POINTS FROM CAD (using Macros)")
    ax1.set_xlabel("X-axis (in mm)")
    ax1.set_ylabel("Y-axis (in mm)")
    ax1.set_zlabel("Z-axis (in mm)")
    x, y, z = [], [], []
    with open("/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_waypoints.csv", 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            waypoint = row[0].split()
            x.append(float(waypoint[0]))
            y.append(float(waypoint[1]))
            z.append(float(waypoint[2]))
    ax1.scatter(x, y, z, c='b', marker='o', label='Waypoints',
               alpha=0.5)  # Set alpha to control transparency
    ax1.set_xlim(min(x), max(x))
    ax1.set_ylim(min(y), max(y))
    ax1.set_zlim(min(z), max(z))
    ax1.legend()

    ax= fig.add_subplot(122, projection='3d')
    ax.set_title("ROBOT PATH TRACE")
    ax.set_xlabel("x-axis (in mm)")
    ax.set_ylabel("y-axis (in mm)")
    ax.set_zlabel("z-axis (in mm)")

    # Initialize empty lists for x, y, and z data
    x, y, z = [], [], []
    with open("/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_waypoints.csv", 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            waypoint = row[0].split()
            x.append(float(waypoint[0]) + 1000*(float(x_offset)))
            y.append(float(waypoint[1]) + 1000*(float(y_offset)))
            z.append(float(waypoint[2]) + 1000*(float(z_offset)))
    ax.scatter(x, y, z, c='b', marker='o', label='Waypoints',alpha=0.5)  # Set alpha to control transparency
    ax.set_xlim(min(x), max(x))
    ax.set_ylim(min(y), max(y))
    ax.set_zlim(min(z), max(z))

    line, = ax.plot([], [], [], c='r', marker='o', label='Visted Points')
    x, y, z = [], [], []
    ax.legend()
    # Initialize your UR5 object and other setup here
    ur5_pose_2 = geometry_msgs.msg.Pose()

    # Open the CSV file
    with open("/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_waypoints.csv", 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        cursor = mplcursors.cursor(line, hover=True)
        cursor.connect("add", lambda sel: sel.annotation.set_text(
            f"x: {x[int(sel.target.index)]:.2f}, y: {y[int(sel.target.index)]:.2f}, z: {z[int(sel.target.index)]:.2f}"))
        for row in csv_reader:
            waypoint = row[0].split()
            ur5_pose_2.position.x = float(waypoint[0]) / 1000 + float(x_offset)
            ur5_pose_2.position.y = float(waypoint[1]) / 1000 + float(y_offset)
            ur5_pose_2.position.z = float(waypoint[2]) / 1000 + float(z_offset)
            ur5_pose_2.orientation.x = quaternions[0]
            ur5_pose_2.orientation.y = quaternions[1]
            ur5_pose_2.orientation.z = quaternions[2]
            ur5_pose_2.orientation.w = quaternions[3]
            if not rospy.is_shutdown():
                ur5.go_to_pose(ur5_pose_2)
            else:
                print("----ROS node killed by the USER----")
                break
            if flag_plan==True and not rospy.is_shutdown():
                x.append((ur5_pose_2.position.x)*1000)
                y.append((ur5_pose_2.position.y)*1000)
                z.append((ur5_pose_2.position.z)*1000)

                # Update the line data and redraw the plot
                line.set_data(x, y)
                line.set_3d_properties(z)

                # Print the current coordinates
                print(f"x: {x[-1]}, y: {y[-1]}, z: {z[-1]}")
                plt.pause(0.016)  # Pause for 0.016 second to update the plot
            elif rospy.is_shutdown():
                print("----ROS node killed by the USER----")
                break
            else:
                pass
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()


# If we Publish the scene geometry, the respective object will be considered by the robot as 'COLLISION OBJECT"
# and the robot will plan the path according to the imported workobject (scene geometry).
# Even the tip of the tool touching the workobject is considered as collision
# and the solution for the path will not be found and raises an error.
# The orientation of the target according to the requirement will not be established with this collision-free planning,
# because, the orientation of TCP or the robot movement, depends on the orientation of the Targets!!!

# Now i did not publish the scene geometry here, so the robot does not consider the workobject!
