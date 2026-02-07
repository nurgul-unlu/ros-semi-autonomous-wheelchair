#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback_front(data):
    # Ön taraftaki mesafe
    front_distance = min(data.ranges)  # min değer: en yakın engel
    print(f"[Ön] En yakın mesafe: {front_distance:.2f} m")

def callback_right(data):
    right_distance = min(data.ranges)
    print(f"[Sağ] En yakın mesafe: {right_distance:.2f} m")

def callback_left(data):
    left_distance = min(data.ranges)
    print(f"[Sol] En yakın mesafe: {left_distance:.2f} m")

rospy.init_node('distance_monitor')

rospy.Subscriber('/scan_front', LaserScan, callback_front)
rospy.Subscriber('/scan_right', LaserScan, callback_right)
rospy.Subscriber('/scan_left', LaserScan, callback_left)

rospy.spin()
