#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import tf

# QR kod -> konum tablosu
positions = {
    "QR1": (0.0, 0.0, 0.0),
    "QR2": (1.0, 0.0, 1.57),
    "QR3": (0.0, 1.0, 3.14)
}

bridge = CvBridge()
pose_pub = None  # Node başlatılınca atanacak

def image_callback(msg):
    global pose_pub
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    decoded = decode(gray)
    for obj in decoded:
        text = obj.data.decode('utf-8')
        rospy.loginfo(f"QR Kod bulundu: {text}")

        if text in positions:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = positions[text][0]
            pose.pose.position.y = positions[text][1]

            quat = tf.transformations.quaternion_from_euler(0, 0, positions[text][2])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            pose_pub.publish(pose)
            rospy.loginfo(f"QR Kod: {text} → Pose yayınlandı")
        else:
            rospy.logwarn(f"Bilinmeyen QR kod: {text}")

if __name__ == "__main__":
    rospy.init_node("qr_reader")
    pose_pub = rospy.Publisher('/robot1/qr_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()
