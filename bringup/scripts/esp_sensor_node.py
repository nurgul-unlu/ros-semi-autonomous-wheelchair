#!/usr/bin/env python3
import rospy
import serial
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class ESPSensorNode:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(port, baud, timeout=1)
        rospy.sleep(2)

        self.imu_pub   = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.odom_pub  = rospy.Publisher("/wheel_odom", Odometry, queue_size=10)

        self.prev_left = None
        self.prev_right = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = rospy.Time.now()

        # Fiziksel parametreler (kalibre edilebilir)
        self.ticks_per_rev = 4096
        self.wheel_radius  = 0.10   # metre
        self.wheel_base    = 0.50   # metre

    def spin(self):
        while not rospy.is_shutdown():
            line = self.ser.readline().decode().strip()

            if not line.startswith("DATA"):
                continue

            parts = line.split(',')
            yaw_imu = float(parts[1])
            encL = int(parts[2])
            encR = int(parts[3])

            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if self.prev_left is not None:

                # --- Encoder sarması düzeltme ---
                dL = encL - self.prev_left
                dR = encR - self.prev_right

                if dL > 2048: dL -= 4096
                if dL < -2048: dL += 4096
                if dR > 2048: dR -= 4096
                if dR < -2048: dR += 4096

                # --- Gerçek mesafe hesabı ---
                distL = (dL / self.ticks_per_rev) * 2 * math.pi * self.wheel_radius
                distR = (dR / self.ticks_per_rev) * 2 * math.pi * self.wheel_radius

                dist = (distL + distR) / 2.0
                dtheta = (distR - distL) / self.wheel_base

                self.yaw += dtheta
                self.x += dist * math.cos(self.yaw)
                self.y += dist * math.sin(self.yaw)

                q = quaternion_from_euler(0, 0, self.yaw)

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation = Quaternion(*q)

                self.odom_pub.publish(odom)

            # --- IMU yayın ---
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "base_link"
            imu.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw_imu))
            self.imu_pub.publish(imu)

            self.prev_left = encL
            self.prev_right = encR

if __name__ == "__main__":
    rospy.init_node("esp_sensor_node")
    ESPSensorNode().spin()
