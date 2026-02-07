#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import smbus
import time

def read_axis(bus, addr, reg):
    """
    MLX90395 için örnek okuma fonksiyonu.
    Gerçekte sensörün datasheet'ine göre komutlar ayarlanmalı.
    Şimdilik basit bir demo mantığı bırakıyorum.
    """
    try:
        raw = bus.read_word_data(addr, reg)
        return raw
    except Exception as e:
        rospy.logwarn_throttle(1.0, f"[joy_control] I2C okuma hatası: {e}")
        return 2048  # ortalama değer gibi düşün

def main():
    rospy.init_node("joy_control_node")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    i2c_bus_id   = rospy.get_param("~bus_id", 1)
    mlx_addr     = rospy.get_param("~mlx_addr", 0x0C)
    max_lin_vel  = rospy.get_param("~max_linear", 0.5)   # m/s
    max_ang_vel  = rospy.get_param("~max_angular", 1.0)  # rad/s

    bus = smbus.SMBus(i2c_bus_id)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # Örnek register adresleri (datasheet'e göre değiştir)
        x_raw = read_axis(bus, mlx_addr, 0x27)
        y_raw = read_axis(bus, mlx_addr, 0x29)

        # 0–4095 arası varsayalım, ortası 2048
        x_norm = (x_raw - 2048) / 2048.0  # -1 … +1
        y_norm = (y_raw - 2048) / 2048.0  # -1 … +1

        cmd = Twist()
        cmd.linear.x  = y_norm * max_lin_vel      # ileri/geri
        cmd.angular.z = -x_norm * max_ang_vel     # sağ/sol

        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

