#!/usr/bin/env python3
import rospy
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class CmdVelToESP:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(port, baud, timeout=1)
        rospy.sleep(2)

        self.front_dist = 999
        self.stop_dist = 0.10

        self.target_v = 0.0
        self.target_w = 0.0

        self.cur_left = 0
        self.cur_right = 0

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("/tof/front", Range, self.front_cb)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("cmd_vel -> ESP (STABLE + TURN)")

    def front_cb(self, msg):
        self.front_dist = msg.range

    def cmd_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

    def control_loop(self, event):

        # === TOF ACİL DUR ===
        if self.front_dist < self.stop_dist:
           self.cur_left = 0
           self.cur_right = 0
           self.send_pwm(0, 0)
           return

        base = int(self.target_v * 150)
        diff = int(self.target_w * 100)

        left_target  = base - diff
        right_target = base + diff

        MAX_PWM = 80
        left_target  = max(min(left_target,  MAX_PWM), -MAX_PWM)
        right_target = max(min(right_target, MAX_PWM), -MAX_PWM)

        RAMP = 10
        self.cur_left  += max(min(left_target  - self.cur_left,  RAMP), -RAMP)
        self.cur_right += max(min(right_target - self.cur_right, RAMP), -RAMP)

        self.send_pwm(self.cur_left, self.cur_right)
        
         
    def send_pwm(self, l, r):
        cmd = f"CMD,{l},{r}\n"
        self.ser.write(cmd.encode())

if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_esp")
    CmdVelToESP()
    rospy.spin()

