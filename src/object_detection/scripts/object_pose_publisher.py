#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel

class ObjectPosePublisher:
    def __init__(self):
        rospy.init_node("object_pose_publisher")

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.depth_image = None

        rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/depth_camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/depth_camera/image_raw", Image, self.rgb_callback)
        
        self.pose_pub = rospy.Publisher("/detected_object_pose", PoseStamped, queue_size=10)

        rospy.loginfo("Object pose publisher started.")
        rospy.spin()

    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("Depth callback error: %s", e)

    def rgb_callback(self, msg):
        if self.depth_image is None:
            return

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("RGB callback error: %s", e)
            return

        # Detect red object
        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.logwarn_throttle(5, "No object detected.")
            return

        # Pick largest contour
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Get depth
        depth = self.depth_image[cy, cx] * 0.001  # Convert from mm to meters if needed
        if depth == 0.0:
            rospy.logwarn_throttle(5, "Depth value is zero at (%d, %d)", cx, cy)
            return

        # Project 2D to 3D
        ray = self.camera_model.projectPixelTo3dRay((cx, cy))
        x = ray[0] * depth
        y = ray[1] * depth
        z = ray[2] * depth

        # Publish pose
        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tfFrame()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)
        rospy.loginfo_throttle(2, "Published object pose: (%.2f, %.2f, %.2f)", x, y, z)

if __name__ == "__main__":
    try:
        ObjectPosePublisher()
    except rospy.ROSInterruptException:
        pass
