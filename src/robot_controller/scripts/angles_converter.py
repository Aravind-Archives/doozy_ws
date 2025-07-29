#!usr/bin/env python3
import rospy
import math
from robot_controller.srv import AnglesConverter, AnglesConverterResponse

def convert_radians_to_degrees(req):
    res = AnglesConverterResponse()
    res.base = round((((req.base) * 360) / (math.pi / 2)) / 4, 2)
    res.shoulder = round((((req.shoulder) * 360) / (math.pi / 2)) / 4, 2)
    res.elbow = round((((req.elbow) * 360) / (math.pi / 2)) / 4, 2)
    res.forearm = round((((req.forearm) * 360) / (math.pi / 2)) / 4, 2)
    res.wrist = round((((req.wrist) * 360) / (math.pi / 2)) / 4, 2)
    res.finger = round((((req.finger) * 360) / (math.pi / 2)) / 4, 2)
    return res

def convert_degrees_to_radians(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in degrees
    # and returns the Result message as output with the angles in radians
    res = AnglesConverterResponse()
    res.base = float((((180-req.base)*math.pi) - ((math.pi/2)*360))/360)
    res.shoulder = float((((180-req.shoulder)*math.pi) - ((math.pi/2)*360))/360)
    res.elbow = float((((180-req.elbow)*math.pi) - ((math.pi/2)*360))/360)
    res.forearm =float((((180-req.forearm)*math.pi) - ((math.pi/2)*360))/360)
    res.wrist = float((((180-req.wrist)*math.pi) - ((math.pi/2)*360))/360)
    res.finger = float((((180-req.finger)*math.pi) - ((math.pi/2)*360))/360)
    return res

if __name__ == "__main__":
    rospy.init_node("angles_converter")
    radians_to_degrees = rospy.Service('/radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    rospy.spin()
    