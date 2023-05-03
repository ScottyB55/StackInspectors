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


def lidarprocess():
	s2lidar.init(serialPortName)
	time.sleep(2)

	for i in range(10):
		for y in range(10):
			print(i, y)
			scan = s2lidar.get_scan()
			X = []
			Y = []
			for s in scan:
				if s[2] != 0:
			    		X.append(s[1] * math.cos(math.radians(s[0])))
			    		Y.append(s[1] * math.sin(math.radians(s[0])))
			plt.clf()
			# plt.xlim([-1, 1])
			# plt.ylim([-1, 1])
			plt.scatter(X, Y)
			plt.pause(0.01)
			# time.sleep(5)
		# time.sleep(1)
	plt.show()
	print("stop")
	s2lidar.stop()
	print("done")

if __name__ == "__main__":
	lidarprocess()



