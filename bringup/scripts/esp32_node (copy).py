#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math

def open_serial(port, baud):
    while not rospy.is_shutdown():
        try:
            rospy.loginfo(f"[esp32_node] {port} açılıyor...")
            ser = serial.Serial(port, baud, timeout=1)
            rospy.loginfo("[esp32_node] Seri port açıldı.")
            return ser
        except serial.SerialException as e:
            rospy.logwarn(f"[esp32_node] Seri port açılamadı: {e}. 2 sn sonra tekrar denenecek.")
            rospy.sleep(2.0)
    return None

def main():
    rospy.init_node("esp32_node")

    port = rospy.get_param("~port", "/dev/ttyACM1")
    baud = rospy.get_param("~baud", 115200)

    ser = open_serial(port, baud)
    if ser is None:
        return

    imu_pub  = rospy.Publisher("/imu/data", Imu, queue_size=20)
    odom_pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=20)
    tof_pub  = rospy.Publisher("/tof_front", Range, queue_size=10)

    frame_imu   = rospy.get_param("~imu_frame", "imu_link")
    frame_odom  = rospy.get_param("~odom_frame", "odom")
    frame_base  = rospy.get_param("~base_frame", "base_link")
    frame_tof   = rospy.get_param("~tof_frame", "link_on_sensor")

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                rate.sleep()
                continue

            # IMU:ax,ay,az,gx,gy,gz
            if line.startswith("IMU:"):
                try:
                    data_str = line.split(":", 1)[1]
                    ax, ay, az, gx, gy, gz = map(float, data_str.split(","))
                    msg = Imu()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = frame_imu

                    # ivme (m/s^2)
                    msg.linear_acceleration.x = ax
                    msg.linear_acceleration.y = ay
                    msg.linear_acceleration.z = az

                    # açısal hız (rad/s)
                    msg.angular_velocity.x = gx
                    msg.angular_velocity.y = gy
                    msg.angular_velocity.z = gz

                    imu_pub.publish(msg)
                except Exception as e:
                    rospy.logwarn(f"[esp32_node] IMU satırı parse edilemedi: '{line}' Hata: {e}")

            # ENC:x,y,th
            elif line.startswith("ENC:"):
                try:
                    data_str = line.split(":", 1)[1]
                    x, y, th = map(float, data_str.split(","))

                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = frame_odom
                    odom.child_frame_id = frame_base

                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.0

                    # Sadece yaw’dan quaternion
                    qz = math.sin(th / 2.0)
                    qw = math.cos(th / 2.0)
                    odom.pose.pose.orientation.z = qz
                    odom.pose.pose.orientation.w = qw

                    odom_pub.publish(odom)
                except Exception as e:
                    rospy.logwarn(f"[esp32_node] ENC satırı parse edilemedi: '{line}' Hata: {e}")

            # TOF:d
            elif line.startswith("TOF:"):
                try:
                    data_str = line.split(":", 1)[1]
                    d = float(data_str)
                    r = Range()
                    r.header.stamp = rospy.Time.now()
                    r.header.frame_id = frame_tof
                    r.radiation_type = Range.INFRARED
                    r.min_range = 0.05
                    r.max_range = 4.0
                    r.range = d
                    tof_pub.publish(r)
                except Exception as e:
                    rospy.logwarn(f"[esp32_node] TOF satırı parse edilemedi: '{line}' Hata: {e}")

            rate.sleep()

        except serial.SerialException as e:
            rospy.logerr(f"[esp32_node] Seri bağlantı koptu: {e}")
            ser.close()
            ser = open_serial(port, baud)
            if ser is None:
                break

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

