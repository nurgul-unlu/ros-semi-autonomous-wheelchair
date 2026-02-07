#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial

def open_serial(port, baud):
    while not rospy.is_shutdown():
        try:
            rospy.loginfo(f"[motor_control] {port} açılıyor...")
            ser = serial.Serial(port, baud, timeout=1)
            rospy.loginfo("[motor_control] Seri port açıldı.")
            return ser
        except serial.SerialException as e:
            rospy.logwarn(f"[motor_control] Seri port açılamadı: {e}. 2 sn sonra tekrar denenecek.")
            rospy.sleep(2.0)
    return None

class MotorControl:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyACM0")  # sensörden farklı port olsun
        self.baud = rospy.get_param("~baud", 115200)
        self.ser = open_serial(self.port, self.baud)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        rospy.loginfo("[motor_control] /cmd_vel dinleniyor.")

    def cmd_cb(self, msg):
        if self.ser is None:
            rospy.logwarn("[motor_control] Seri port yok, komut gönderilemiyor.")
            return

        lin = msg.linear.x
        ang = msg.angular.z

        data = f"SPEED:{lin:.3f},{ang:.3f}\n"
        try:
            self.ser.write(data.encode())
            # rospy.loginfo(f"[motor_control] Gönderildi: {data.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"[motor_control] Seri yazma hatası: {e}")
            try:
                self.ser.close()
            except:
                pass
            self.ser = open_serial(self.port, self.baud)

def main():
    rospy.init_node("motor_control_node")
    mc = MotorControl()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

