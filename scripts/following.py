#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import cv2
from cv_bridge import CvBridge, CvBridgeError

class AutoTracking:

    def __init__(self):
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist)
        self.sub = rospy.Subscriber('face_position', Float32MultiArray, self.callback)
        self.center_bound = 5
        self.slope = .01
        self.vel_msg = Twist()

    def _does_not_detect_any_face(self, pos):
        return pos.data[2] < 1

    def _decision_making(self, pos):

        x = pos.data[0]
        self.vel_msg.angular.z = 0
        if x >= 50 or x <= -50:
            print("ValueError of face_position! Skip decision making")
            return 1

        if self._does_not_detect_any_face(pos):
            return 0

        if abs(x) <= self.center_bound:
            return 0

        self.vel_msg.angular.z = -x * self.slope

    def callback(self, data):
        self._decision_making(data)
        self.pub.publish(self.vel_msg)


def main(args):
    rospy.init_node('following', anonymous=True)
    AutoTracking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down ...")

if __name__ == "__main__":
    main(sys.argv)
