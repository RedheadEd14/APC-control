import numpy as np
#import algopy as ap
# from algopy import UTPM
from numpy import pi
import serial
import os
from time import time as now, sleep
import math as m
import matplotlib.pyplot as plt
import datetime
import slack_removal as slack
from jacobian import jacobianMatrix


def get_lidar_readings():  #reads lidar sensor data back
	l = []
	while (len(l) != 1):
		#bytesToRead = sz.ser_lidar1.inWaiting()
		#readings = sz.ser_lidar1.read(bytesToRead)
		# ser_encoder.flushInput()
		# sleep(.1)
		readings = ser_encoder.readline()
		l = []
		startread = False
		for t in reversed(readings.split()):  # read from most recent serial data
			if (t is '#'):
				startread = True
				continue
			if (startread):
				if (t is '$'):
					break
				else:
					try:
						l.append(float(t))
					except ValueError:
						pass
		l = l[::-1]  # Reverse readings to get in [x y z] order
		if (len(l) != 1):
			sleep(0.05)
		# print "LIDAR Readings:"
		# print l
	p = (l[0] * .001 + .035)
	return np.array(p)

def sensed_lidar():  #averages a set of 5 lidar readings
	avg = 0				
	size = 5
	ser_encoder.write("i")

	for i in range(5):
		read = get_lidar_readings()        
		if read < .17:  #throws away bad data
			read = 0
			size = size - 1
		avg = avg + read
	# print avg
	avg = avg/size
	return avg

if __name__=="__main__":

	
	ser = [0, 0]
	got_port = [0, 0]
	try:
		ser[0] = serial.Serial('/dev/ttyACM0', 57600)
	except Exception, e:
		print "No Arduino on ACM0"
		print str(e)
		got_port[0] = 0
	try:
		ser[0] = serial.Serial('/dev/ttyACM1', 57600)
	except Exception, e:
		print "No Arduino on ACM1"
		print str(e)
		got_port[0] = 0
	ser_encoder = ser[0]
	
	while (1):
		lidardata = sensed_lidar()
		print lidardata	
		sleep(.1)
