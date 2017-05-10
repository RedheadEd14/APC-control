import pygame
from pygame import joystick
from pygame import event
from pygame.locals import *
import ckbot.logical as L # CKBot interface
from time import time as now, sleep
from numpy import pi
import math
import sys
import numpy as np
import spiral_zipper_1Arm as sz
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import collections
from jacobian import jacobianMatrix

# Specify module ID and the name we want to give each of them:
#modules = {0xE4: 'T1',0xEA:'T2',0xDF:'T3'}#,0xF8:'T4',
modules = {0xE4:'T1',0xEA:'T2', 0xDF:'T3',0xE3:'T4'}

if __name__=="__main__":

	print "Initializing Spiral Zipper Demo"  

	##### INITIALIZE CKBOT CLUSTER ###########
	if len(sys.argv) == 2:
		modules = {int(sys.argv[1]) : 'T1', int(sys.argv[2]) : 'T2', int(sys.argv[3]) : 'T3',int(sys.argv[4]) : 'T4'}
	c = L.Cluster()
	c.populate( len(modules), modules )

	# Limit module speed and torque for safety
	torque_limit = 0.75
	for m in c.itermodules():
		m.set_torque_limit( torque_limit )
		# If module is a servo, then also limit speed
		if not m.get_mode():
			m.set_speed( 10.0 )

	r_winch = 0.022
	looptime = 0.1		
	sz1 = sz.SpiralZipper(r_winch, c, looptime)

	#### SETS VELOCITY CONTROL GAINS ####
	Kp1 = 15 #proportional gains
	Kp2 = 5
	Kp3 = 5
	Kp = [Kp1,Kp2,Kp3]
    
	Ki1 = .1  #integral gains							
	Ki2 = .025
	Ki3 = .025
	Ki = [Ki1,Ki2,Ki3]

	Kd1 = 0.075
	Kd2 = 0.075
	Kd3 = 0.075
	Kd = [Kd1,Kd2,Kd3]

	sz1.set_Kp_Gains(Kp,0)
	sz1.set_Ki_Gains(Ki,0)
	sz1.set_Kd_Gains(Kd,0)
	
	sleep(.1)
	sensed_grav = sz1.get_encoder_readings() #initial orientation 
	angular_precision = .0001
	while(sensed_grav[1] >  angular_precision):
		sz1.update_state(c)
		sz1.update_goal([.001,.001,0],0)
		sz1.set_tether_speeds(0)
		sz1.actuate_Motors(c,0)
		sleep(.1)
		sensed_grav = sz1.get_encoder_readings() #initial orientation from the IMU
		print "1"
	while(sensed_grav[1] < -angular_precision):
		sz1.update_state(c)			
		sz1.update_goal([-.001,-.001,0],0)
		sz1.set_tether_speeds(0)
		sz1.actuate_Motors(c,0)
		sleep(.1)
		sensed_grav = sz1.get_encoder_readings() #initial orientation from the IMU
		print "2"
	while(sensed_grav[0] > angular_precision):
		sz1.update_state(c)			
		sz1.update_goal([.001,-.001,0],0)
		sz1.set_tether_speeds(0)
		sz1.actuate_Motors(c,0)
		sleep(.1)
		sensed_grav = sz1.get_encoder_readings() #initial orientation from the IMU
		print "3"
	while(sensed_grav[0] < -angular_precision):
		sz1.update_state(c)			
		sz1.update_goal([-.001,.001,0],0)
		sz1.set_tether_speeds(0)
		sz1.actuate_Motors(c,0)
		sleep(.1)
		sensed_grav = sz1.get_encoder_readings() #initial orientation from the IMU
		print "4"

	sz1.update_goal([0,0,0])
	sz1.set_tether_speeds(0)
	sz1.actuate_Motors(c,0)
	theta1 = c.at.T1.get_pos()
	theta2 = c.at.T2.get_pos()
	theta3 = c.at.T3.get_pos()
	theta4 = c.at.T4.get_pos()
	#theta_reading = [theta1,theta2,theta3]
	theta_reading = [theta1,theta2,theta3,theta4]
	sz1.theta_curr = theta_reading # update current theta	
	dtheta = sz1.get_CR_delta() # calculate the change in position since last call

	sz1.theta_prev = sz1.theta_curr
	sensed_grav1 = sz1.get_encoder_readings() #initial orientation from the IMU
	e = 0
	f = 0
	while(-0.085 < f < 0.1):
		sz1.update_state(c)			
		sz1.update_goal([-.005,-.005,0])
		sz1.set_tether_speeds(0)
		sz1.actuate_Motors(c,0)
		theta1 = c.at.T1.get_pos()
		theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		theta4 = c.at.T4.get_pos()
		#theta_reading = [theta1,theta2,theta3]
		theta_reading = [theta1,theta2,theta3,theta4]
		sz1.theta_curr = theta_reading # update current theta

		dtheta = sz1.get_CR_delta() # calculate the change in position since last call
		sz1.theta_prev = sz1.theta_curr
		e = e + dtheta[1]*r_winch
		sensed_grav2 = sz1.get_encoder_readings() #initial orientation from the IMU
		f = sensed_grav2[0] - sensed_grav1[0]
		print f

	c.at.T2.set_torque(0)
	c.at.T3.set_torque(0)
	print "f is %f" %f
	print "e is %f" %e
