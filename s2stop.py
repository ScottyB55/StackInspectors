import s2lidar
import time
import math
from matplotlib import pyplot as plt
import serial.tools.list_ports
# from pyserial import serial.tools.list_ports

serialPortName = ""
serialports = serial.tools.list_ports.comports()
for port in serialports:
	if port.device.startswith("/dev/ttyUSB"):
		serialPortName = port.device
print(serialports, serialPortName)

s2lidar.init(serialPortName)
s2lidar.stop()