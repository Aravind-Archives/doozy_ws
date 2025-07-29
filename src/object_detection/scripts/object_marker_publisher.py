#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "object"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = msg.pose
    marker.scale.x = 0.05  # size similar to Gazebo unit_box
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('object_marker_publisher')
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.Subscriber('/detected_object_pose', PoseStamped, pose_callback)
    rospy.spin()
