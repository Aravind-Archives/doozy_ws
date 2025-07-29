#! /usr/bin/env python3
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

class Ur5Moveit:
    def __init__(self):
        rospy.init_node('ik1', anonymous=True)

        self._planning_group = "arduinobot_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_num_planning_attempts(5)
        self._group.set_planning_time(0.05)
        self._group.set_planner_id("RRTstar")
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        rospy.loginfo('\033[94m' + f"Planning Frame: {self._planning_frame}" + '\033[0m')
        rospy.loginfo('\033[94m' + f"End Effector Link: {self._eef_link}" + '\033[0m')
        rospy.loginfo('\033[94m' + f"Group Names: {self._group_names}" + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    ur5 = Ur5Moveit()

    x_angle = input("Enter x-Orientation:")
    roll = math.radians(float(x_angle))
    y_angle = input("Enter y-Orientation:")
    pitch = math.radians(float(y_angle))
    z_angle = input("Enter z-Orientation:")
    yaw = math.radians(float(z_angle))
    quaternions = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    x_offset = input("Enter x-Position: ")
    y_offset = input("Enter y-Position: ")
    z_offset = input("Enter z-Position: ")

    # Read waypoints from cleaned CSV directly
    waypoint_file = '/home/robot/doozy_ws/src/robot_controller/scripts/cylinder_xy_waypoints.csv'
    waypoints = []

    with open(waypoint_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            waypoint = row[0].split()
            pose = geometry_msgs.msg.Pose()
            pose.position.x = float(waypoint[0]) / 1000 + float(x_offset)
            pose.position.y = float(waypoint[1]) / 1000 + float(y_offset)
            pose.position.z = float(waypoint[2]) / 1000 + float(z_offset)
            pose.orientation.x = quaternions[0]
            pose.orientation.y = quaternions[1]
            pose.orientation.z = quaternions[2]
            pose.orientation.w = quaternions[3]
            waypoints.append(copy.deepcopy(pose))

    (plan, fraction) = ur5._group.compute_cartesian_path(waypoints, 0.01, 0.0)

    if fraction > 0.9:
        ur5._group.execute(plan, wait=True)
        print("Executed the Cartesian trajectory successfully.")
    else:
        print("Could not compute full Cartesian path. Try adjusting waypoints or parameters.")

if __name__ == '__main__':
    main()
