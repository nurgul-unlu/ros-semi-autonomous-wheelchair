#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("qr_detector")
bridge = CvBridge()

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    detector = cv2.QRCodeDetector()

    data, _, _ = detector.detectAndDecode(img)
    if data:
        rospy.loginfo("QR OKUNDU: " + data)

rospy.Subscriber("/c270/image_raw", Image, cb)
rospy.spin()

