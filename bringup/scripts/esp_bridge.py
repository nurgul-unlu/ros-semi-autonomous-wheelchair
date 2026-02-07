#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import Int32

PORT = "/dev/ttyACM0"
BAUD = 115200

def cb(msg):
    ser.write(f"{msg.data}\n".encode())

def main():
    global ser
    rospy.init_node("esp_bridge")

    pub = rospy.Publisher("/number_out", Int32, queue_size=10)
    rospy.Subscriber("/number_in", Int32, cb)

    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line.startswith("OUT:"):
                pub.publish(int(line.split(":")[1]))
        rate.sleep()

if __name__ == "__main__":
    main()

