#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer

import cv2
from cv_bridge import CvBridge, CvBridgeError

class FaceDetector:

    def __init__(self):
        rospy.Rate(5)

        self.pub = rospy.Publisher('face_position', Float32MultiArray)
        self.pub_debug = rospy.Publisher('face_debug', Image, queue_size = 1)
        self.sub = rospy.Subscriber("bebop/image_raw", Image, self.callback)

        self.bridge = CvBridge()
        self.image = None
        self.image_debug = None
        self.face_info = None
        self.face_pos = Float32MultiArray()
        self.face_cascade = cv2.CascadeClassifier('/home/stephlin/haarcascade_frontalface_default.xml')

        self.skip_rate = 15
        self.counter = -1

    def _haar_cascade_face_detection(self):
        gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.face_info = self.face_cascade.detectMultiScale(gray_image)

        # only get the face of maximum size
        if len(self.face_info) > 0:
            max = gray_image.shape[0] * gray_image.shape[1] / 50
            argmax = -1
            for key, (x, y, w, h) in enumerate(self.face_info):
                if w*h > max:
                    max = w*h
                    argmax = key

            self.face_info = self.face_info[argmax] if argmax >= 0 else []

    def callback(self, data):

        self.counter = (self.counter + 1) % self.skip_rate
        if self.counter > 0:
            return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self._haar_cascade_face_detection()

        self.image_debug = self.image.copy()

        x, y, w, h = [-1, -1, 0, 0]

        if len(self.face_info) > 0:
            x, y, w, h = self.face_info
            cv2.rectangle(self.image_debug, (x, y), (x+w, y+h), (0, 255, 255), 10)

        face_x = float(x + w/2) / self.image.shape[1] * 100 - 50
        face_y = float(y + h/2) / self.image.shape[0] * 100 - 50

        self.face_pos.data = [face_x, face_y, w*h]
        self.pub.publish(self.face_pos)

        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(self.image_debug, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('face_detector', anonymous=True)
    FaceDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down ...")

if __name__ == "__main__":
    main(sys.argv)
