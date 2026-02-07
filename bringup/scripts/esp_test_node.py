#!/usr/bin/env python3
import serial
import time

PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_10:20:BA:72:80:34-if00"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("ROS: Seri hatta bağlandım.")

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print("ESP -> ROS:", line)

