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
import slack_removal as slackn
from jacobian import jacobianMatrix

global arduino
#init serial port
if __name__=="__main__":

    try:
	    arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except Exception, e:
	    print "No Arduino on ACM0"
	    print str(e)
    try:
	    arduino = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    except Exception, e:
	    print "No Arduino on ACM1"
	    print str(e)

    sleep(3)
    if arduino.isOpen():
	    print "Port Open"

    while True:
  	  #   encoder output <i>
    	# length sensor output <j>
    	# Maxon command <k + number between 0-255>
    	# IQinentics speed command <m + number between 0-12>
    	# IQinentics brake command <n>
    	# IQinentics slack command <o>

	    command = input("What is your command, you lowblooded mongrel of unfortunate proportions? ")
	    arduino.write(command)
	    # arduino.write(6)
	    sleep(.1)
	    if(arduino.inWaiting() > 0):
		    resp = arduino.readline()
		    print "i got "
		    print resp

	    # sleep(.25)
	# Finish transmission with -1
    arduino.flush()
    arduino.write("-1")
    #done
    arduino.close()