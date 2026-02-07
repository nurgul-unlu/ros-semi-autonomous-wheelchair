#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    middle_index = len(msg.ranges) // 2
    distance = msg.ranges[middle_index]

    if distance == float('inf') or distance == 0.0:
        rospy.loginfo("Robot1 önünde engel yok veya ölçüm geçersiz.")
    else:
        rospy.loginfo(f"Robot1 önündeki engel mesafesi: {distance:.2f} m")

def listener():
    rospy.init_node('robot1_laser_distance_node')
    rospy.Subscriber("/robot1/scan_front", LaserScan, scan_callback)
    rospy.loginfo("Robot1 ön mesafe ölçümü başlatıldı...")
    rospy.spin()

if __name__ == '__main__':
    listener()
