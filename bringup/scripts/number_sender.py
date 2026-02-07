#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import Int32

PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    rospy.init_node("ros_serial_bridge")

    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    pub = rospy.Publisher("/number_out", Int32, queue_size=10)

    rate = rospy.Rate(1)
    counter = 0

    while not rospy.is_shutdown():
        # ESP'ye gönder
        ser.write(f"{counter}\n".encode())

        rospy.loginfo(f"ESP'ye giden: {counter}")

        # ESP'den oku
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            rospy.loginfo(f"ESP'den gelen: {line}")

        counter += 1
        rate.sleep()

if __name__ == "__main__":
    main()

