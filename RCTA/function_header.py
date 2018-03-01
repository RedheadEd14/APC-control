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
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import collections
from jacobian import jacobianMatrix

class SpiralZipper:

	def __init__(sz, r_winch, c, looptime):
			
		#Lengths set the initial configuration of the system.
		# Lengths: array 1x4 [L0, L1, L2, L3]
		#create spiral zipper object

		sz.tether_subtract_CCW = True #sz ckbot direction: CCW subtracts tether and CW adds/releases tether.

		sz.thetaold = 0 #used in Jacobian function
		sz.Lold = [0,0,0,0]

		got_port = [1, 1]
		try:
			sz.ser_encoder = serial.Serial('/dev/ttyACM0', 57600)
		except Exception, e:
			print "No Arduino on ACM0"
			print str(e)
			got_port[0] = 0
		try:
			sz.ser_encoder = serial.Serial('/dev/ttyACM1', 57600)
		except Exception, e:
			print "No Arduino on ACM1"
			print str(e)
			got_port[1] = 0

		sleep(1)
		if sz.ser_encoder.isOpen():
			print "Port Open"
		#control loop speed to be enforced
		sz.looptime = looptime  
		sz.timeold = 0

		# set params for ckbot control
		sz.max_t = 4095
		sz.min_t = 0
		sz.limit_t = sz.max_t/2
		sz.radian_const = 2 * np.pi / sz.max_t #(rad/tick)
		#initializes robot geometric parameters. Constant
		sz.rb = 0.292 #0.2171          # circle radius inscribing the spiral zipper motors #.22 for the APC arm, .11 for the RCTA arm
		sz.rt = 0.035  	   				# radius of the spiral zipper column
		sz.r_winch = r_winch   # the radius of the winch used to wind tether.
		phi = 30 * pi/180

		# positions of the motors in xyz.
		p1 = .199*np.array([0,1, 0]) + np.array([0,0,0.01415])  
		p2 = sz.rb*np.array([np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,0.0221])
		p3 = sz.rb*np.array([-np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,0.0221])
		sz.p = [p1,p2,p3]

		# tether attachment points on the end effector assuming arm is straight upsz.OB
		ef1 = sz.rt*np.array([0,1, 0]) 
		ef2 = sz.rt*np.array([np.cos(pi/6),-np.sin(pi/6), 0])
		ef3 = sz.rt*np.array([-np.cos(pi/6),-np.sin(pi/6), 0])
		sz.ef = [ef1,ef2,ef3]

		# initializes robot state parameters.  Variable
		sz.L = [0,0,0,0]
		sz.L_pos_desired = [0,0,0,0]
		sz.L_vel_desired = [0,0,0,0]
		sz.L_vel_actual = [0,0,0,0]
		sz.errorsumP = [0,0,0,0]
		sz.errorsumV = [0,0,0,0]	
		sz.old_pos_error = np.array([0,0,0,0])
		sz.old_vel_error = np.array([0,0,0,0])
		sz.lold = [0,0,0]
		sz.OB = [0,0,0]
		sz.Lprev = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
		sz.R = np.array([[0,0,0],[0,0,0],[0,0,0]])
		
		c.at.T3.set_torque(0)
		c.at.T4.set_torque(0)
		sz.ser_encoder.write('m9')

		sleep(2)		
		sz.calibration()		

		sensed_grav = sz.get_encoder_readings() #initial orientation from the gimbal encoders
		sleep(.005)

		while (sensed_grav[0] > 1.57 or sensed_grav[0] < -1.57 or sensed_grav[1] < -1.57 or sensed_grav[1] > 1.57 ):
			sensed_grav = sz.get_encoder_readings() #takes the average of a set of IMU readings
			print sensed_grav
			print sensed_grav[0]
			print sensed_grav[1]
			print sensed_grav[2]

		# Initial Sensed Position.
		sz.sensed_pos = sz.rotate_encoder(sensed_grav[0], sensed_grav[1],sz.L[0])
		sz.L = sz.cart2tether_actual(sz.sensed_pos)

		# Initial Desired Position
		sz.L0_desired = sz.L[0]		
		sz.update_goal(sz.sensed_pos, 1) 
		#sz.set_tether_speeds(1)

		print "getting arm 1 readings"
		theta1 = sensed_grav[2]
		# theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		theta4 = c.at.T4.get_pos()
		sz.theta0 = [theta1,theta3,theta4] # initial encoder positions of the motors
		sz.theta_prev = sz.theta0 # used to update delta position
		sz.theta_curr = sz.theta0 # will hold the current theta of the system
		sz.column_number = 0

		print "sensed_orientation "
		print sensed_grav
		print "Sensed_Position "
		print sz.sensed_pos
		print "Tether_Length "
		print sz.L

		sz.sensed_pos_prev = sz.sensed_pos

	##### LIVE PLOT INITIALIZATION  #####
		sz.fig = plt.figure()
		sz.ax = sz.fig.add_subplot(111)

		sz.y1array = collections.deque([None] * 200, maxlen=200)
		sz.y2array = collections.deque([None] * 200, maxlen=200)
		sz.y3array = collections.deque([None] * 200, maxlen=200)
		sz.y1darray = collections.deque([None] * 200, maxlen=200)
		sz.y2darray = collections.deque([None] * 200, maxlen=200)
		sz.y3darray = collections.deque([None] * 200, maxlen=200)
		sz.xarray = collections.deque([None] * 200, maxlen=200)

		sz.l1, = sz.ax.plot(sz.xarray, sz.y1array, 'r-', label = "sensed x")
		sz.l2, = sz.ax.plot(sz.xarray, sz.y2array, 'b-', label = "sensed y")
		sz.l3, = sz.ax.plot(sz.xarray, sz.y3array, 'g-', label = "sensed z")
		sz.l1d, = sz.ax.plot(sz.xarray, sz.y1darray, 'r--', label = "desired x")
		sz.l2d, = sz.ax.plot(sz.xarray, sz.y2darray, 'b--', label = "desired y")
		sz.l3d, = sz.ax.plot(sz.xarray, sz.y3darray, 'g--', label = "desired z")
		plt.legend(bbox_to_anchor=(.8, .9), loc=2, borderaxespad=0.)
		sz.fig.canvas.draw()
		plt.show(block=False)

	def calibration(sz):
		choice = input("Would you like to recalibrate the arm? Type 1 to recalibrate, 0 to run program as is: ")
		while choice !=0 and choice != 1:
			choice = input("That is an invalid input. Please type 1 to recalibrate the arm length or 0 to run the program as is: ")
		if choice ==1:

			sz.L[0] = input("Please set the current length of arm in meters.") #measure from the base of the slider to the top of the band
			sz.L[0] = sz.L[0] + .129
			s = str(sz.L[0])
			file_obj = open("armLengthParams.txt", "w")
			file_obj.write(s)
		elif choice == 0:
			file_obj = open("armLengthParams.txt", "r")
			sz.L[0] = float(file_obj.read())
			# sz.update_state()
		file_obj.close()

	def vacuum_cleaner(sz):  #sends a command to the arduino that controls the vacuum.  Toggles it on and off
		sz.ser_encoder.write("r")

	def slack_removal(sz,c):
		slack.slack_remove(sz,c)
		sleep(.25)
		rotation = sz.get_encoder_readings()
		sz.sensed_pos = sz.rotate_encoder(rotation[0],rotation[1],sz.L[0])
		sz.L = sz.cart2tether_actual(sz.sensed_pos)  #tether length update based on IMU only
		sz.goal = sz.sensed_pos
		sz.sensed_pos_prev = sz.goal

	def set_PID_Gains(sz, Kp, Ki, Kd, remote_or_auto):
		if remote_or_auto == 0:
			sz.Kp_remote = Kp
			sz.Ki_remote = Ki
			sz.Kd_remote = Kd
		if remote_or_auto == 1:
			sz.Kp = Kp
			sz.Ki = Ki
			sz.Kd = Kd
	
	def get_Kp_Gains(sz, remote_or_auto):
		if remote_or_auto == 0:
			return sz.Kp_remote
		if remote_or_auto == 1:
			return sz.Kp

	def get_Ki_Gains(sz, remote_or_auto):
		if remote_or_auto == 0:
			return sz.Ki_remote
		if remote_or_auto == 1:
			return sz.Ki

	def get_Kd_Gains(sz,remote_or_auto):
		if remote_or_auto == 0:
			return sz.Kd_remote
		if remote_or_auto == 1:
			return sz.Kd

	#def get_goal(sz): 	#get goal information
	#	return sz.goal= xyz + np.dot(R,sz.ef[1])		

	def update_state (sz, c):
		#This function updates the position of the system based on encoder data and the IMU.
		rotation =sz.get_encoder_readings()
		# theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		theta4 = c.at.T4.get_pos()
		#theta_reading = [theta1,theta2,theta3]
		theta_reading = [rotation[2], theta3, theta4, rotation[3]]
		
		print "motor theta readings:"
		print theta_reading

		sz.theta_curr = theta_reading # update current theta
		dtheta = [0,0,0]
		# dtheta[0] = sz.T1_Rotation()
		dtheta[0] = theta_reading[0]-sz.theta_prev[0]
		dtheta[1:2] = sz.get_CR_delta() # calculate the change in position since last call

		sz.theta_prev = sz.theta_curr

		# devins_constant = .000926 #.04445/48
		#if height of band is ~1.8 in and there are 72 ticks per rev. Than
		# the following number is 1.8/72 * .0254
		devins_constant = .000640
		# devins_constant = .00061453888
		#rotation = sz.get_sensor_readings()
		sz.L[0] = sz.L[0] + dtheta[0] * devins_constant #change in radians * height change per radian gets change in height
		#sz.sensed_pos = sz.rotate(rotation[0],rotation[1],sz.L[0])

		if rotation[3] > 1 and rotation[3] < 32:
			#the height of the band is 1.8 in. 1.8 * .0254 = .04572
			estimated_height = (rotation[3] * 0.04572) + .1915
			if abs(estimated_height - sz.L[0]) < .1:
				sz.L[0] = (rotation[3] * 0.04572) + .1915
				sz.column_number = rotation[3]

		print "column_number is: "
		print sz.column_number

		sz.sensed_pos = sz.rotate_encoder(rotation[0],rotation[1],sz.L[0])

		sz.L = sz.cart2tether_actual(sz.sensed_pos)  #tether length update based on IMU only

		sz.L_vel_actual[0] = (dtheta[0] * devins_constant) / sz.looptime #gets the current speed of the system from encoder data
		# sz.L_vel_actual[1] = (dtheta[1] * sz.r_winch) / sz.looptime
		sz.L_vel_actual[2] = (dtheta[1] * sz.r_winch) / sz.looptime
		sz.L_vel_actual[3] = (dtheta[2] * sz.r_winch) / sz.looptime
		s = str(sz.L[0])
		file_obj = open("armLengthParams.txt", "w")
		file_obj.write(s)
		file_obj.close()
	
	def T1_Rotation(sz):
		encoder_max = 16383
		diff = sz.theta_curr[0] - sz.theta_prev[0]
		if abs(diff) > (encoder_max/2):
			if sz.theta_curr[0] >= sz.theta_prev[0]:
				diff = -1 * ((encoder_max - sz.theta_curr[0]) + (sz.theta_prev[0] - 0))
			else:
				diff = (encoder_max - sz.theta_prev[0]) + (sz.theta_curr[0] - 0)
		else:
			diff = diff

		diff  = diff * (pi)/(encoder_max) #number of ticks over total number of ticks per rev gets % of revolution.  Multiplied by 2*pi to get radians over 2 to get gear ratio.
		# print diff
		return diff

	def get_CR_delta(sz):
		#compare previous value and current value for modules
		# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
		#get current readings
		#Assume the current value of the sensor is stored and the previous value is correct.
		timeelapsed = now() - sz.timeold
		diff = [0.0,0.0,0.0,0.0]
		#diff = [0.0, 0.0]
		for i in range(2,3):
			diff[i] = sz.theta_curr[i]-sz.theta_prev[i]
			if abs(diff[i])>sz.limit_t: #calculate valid delta and switch signs to be correct direction
				if sz.theta_curr[i] >= sz.theta_prev[i]:# diff is +ve, so the solution should be -ve
					diff[i] = ((sz.max_t-sz.theta_curr[i])+(sz.theta_prev[i]-sz.min_t)+1)*(-1)
				else: # diff is negative, therfore the solution should be positive
					diff[i] = ((sz.max_t-sz.theta_prev[i])+(sz.theta_curr[i]-sz.min_t)+1)
			else:  #tether length update based on IMU only

				diff[i] = diff[i] #valid calculation

		#print "motor position change "
		#print np.array(diff)#*sz.radian_const
		
		sz.timeold = now()
		if timeelapsed < .2:
			return np.array(diff)*sz.radian_const # convert ticks to radians
		else:
			return [0.0,0.0,0.0,0.0]
			# return [0.0,0.0,0.0]


	def update_goal(sz,delta,mode):
		if mode == 0: # Velocity: adds a delta value to the current position to create a goal position
			if (sz.sensed_pos[0] == 0 and sz.sensed_pos[1] ==0):  #error rejection from IMU. probably obsolete
				sz.goal = sz.sensed_pos
			else:
				sz.L0_desired = sz.L[0] + delta[2]
				sz.goal = sz.goal + np.array([delta[0],delta[1],0]) 		
				sz.goal = (np.array(sz.goal)/np.linalg.norm(sz.goal))*sz.L0_desired
			sz.sensed_pos_prev = sz.sensed_pos
			# print "goal is"
			# print sz.goal
		else: 		 # Position: Takes given input and directly sets it as the goal.
			sz.goal = np.array([delta[0],delta[1],delta[2]])
		
		angleLimitNeg = 55 * pi/180  #minimum angle the arm can go to
		angleLimitPos = 30 * pi/180
		r = m.sqrt(sz.goal[0]**2 + sz.goal[1]**2)
		angle = m.atan(sz.goal[2]/r)  #will the desired target take the arm outside its safe zone?
		if angle < angleLimitNeg and sz.goal[1] < 0:  #if so, make the arm not go that direction.
			sz.goal = sz.sensed_pos
		elif angle < angleLimitPos:
			sz.goal = sz.sensed_pos
		print "angle is "
		print angle

	def set_tether_speeds(sz,mode):
		L_goal = sz.cart2tether_actual(sz.goal)
		if mode == 0: # Velocity: adds a delta value to the current position to create a goal position
			for i in range(4):
				sz.L_vel_desired[i] = (L_goal[i] - sz.L[i]) / sz.looptime
		else: 		 # Position: Takes given input and directly sets it as the goal.
			sz.L_pos_desired = L_goal 

	def cart2tether_actual(sz,xyz):
		#convert a cartesian goal to tether length goals. assumes the tethers go to points on the outside edge of the end effector.
		#returns a more precise estimate of tether length, but one that is inadmissible to the get_xyz_pos function
		#goal : 1x3 array [x,y,z]
			# L = 1x4 array [L0,L1,L2,L3] spiral zipper and each tether length
		#finds the axis-angle rotation matrix from the column's vertical pose.
		# theta = sz.angle_between([0,0,sz.L[0]], xyz)
		# k = np.cross([0,0,sz.L[0]],xyz)

		# if np.linalg.norm(k) != 0: #ensures k is a unit vector where its norm == 1
		# 	k = k/np.linalg.norm(k)

		# Xk = k[0]
		# Yk = k[1]
		# Zk = k[2]
		# v = 1 - m.cos(theta)

		# R = np.array( [[m.cos(theta) + ((Xk**2)*v)   , (Xk*Yk*v) - (Zk*m.sin(theta)), (Xk*Zk*v) + Yk*m.sin(theta)],\
		# 			   [((Yk*Xk*v) + Zk*m.sin(theta)), m.cos(theta) + ((Yk**2)*v)     , (Yk*Zk*v) - Xk*m.sin(theta)],\
		# 			   [(Zk*Xk*v) - Yk*m.sin(theta), (Zk*Yk*v) + Xk*m.sin(theta)  , m.cos(theta) + ((Zk**2)*v)   ]])

		#calculates position vector of the column tether attachment points in the world frame
		OB1 = xyz + np.dot(sz.R,sz.ef[0])  
		OB2 = xyz + np.dot(sz.R,sz.ef[1])		
		OB3 = xyz + np.dot(sz.R,sz.ef[2])
		sz.OB = [OB1, OB2, OB3]

		L0 = m.sqrt((xyz[0]**2+xyz[1]**2+xyz[2]**2)) # should just be sz.L[0] if not there is a math mistake
		L1 = np.linalg.norm(OB1 - sz.p[0])
		L2 = np.linalg.norm(OB2 - sz.p[1])
		L3 = np.linalg.norm(OB3 - sz.p[2])
		L = [L0,L1,L2,L3]
		return L

	    
	def rotate_encoder(sz,roll,pitch,r):
            v0 = np.array([0,0,r])
            R_roll = np.array([[1,0,0],[0,m.cos(pitch),-m.sin(pitch)],[0,m.sin(pitch),m.cos(pitch)]])
            R_pitch = np.array([[m.cos(roll),0,m.sin(roll)],[0,1,0],[-m.sin(roll),0,m.cos(roll)]])
            sz.R = np.dot(R_pitch,R_roll)
            v2 = np.dot(sz.R,v0)
            return v2	


	def angle_between(sz, v1, v2): #gets angle between two input vectors
		v1_u = v1/np.linalg.norm(v1) # unit vectors
		v2_u = v2/np.linalg.norm(v2)
		return np.arccos(np.dot(v1_u, v2_u))  


	def get_encoder_readings(sz):
		angle = []
		startread = False
		finishread = False
		while len(angle) < 4 :
			sz.ser_encoder.write('i')
			sleep(.002)
			if(sz.ser_encoder.inWaiting() > 0):
				readval = sz.ser_encoder.readline()
				readval = readval.split()
				try: 
					angle = [-float(readval[1]),float(readval[2]),float(readval[3]), float(readval[4])]
				except:
					print "Error! Recollecting angle data."
					print angle
		return angle

	def PID_pos_control(sz,remote_or_auto): #position PID control
		desired_pos = sz.L_pos_desired #creates a local variable from the object variable
		actual_pos = sz.L#_vel_actual
	
		Kp = sz.get_Kp_Gains(1) #gets PI gains from  the Main file
		Ki = sz.get_Ki_Gains(1) 		
		Kd = sz.get_Kd_Gains(1)

		error = np.array(desired_pos) - np.array(actual_pos)

		sz.errorsumP = sz.errorsumP + (np.array(desired_pos) - np.array(actual_pos))  #I control variable
		errorsummax = .01  #sets a limit on how large the I parameter can grow.  Needs to be tuned
		for i in range(4):  #antiwindup loop. Ensures I term never grows larger than the errorsum max
			if sz.errorsumP[i] > errorsummax:
				sz.errorsumP[i] = errorsummax
			elif sz.errorsumP[i] < -1*errorsummax:
				sz.errorsumP[i] = -1*errorsummax

		errordif = (error - sz.old_pos_error)/ sz.looptime
		
		t1 = Kp[0] * error[0] + Ki[0]*sz.errorsumP[0] + Kd[0] * errordif[0] #the PID control equation
		if desired_pos[0] ==0:  #if no command input. system shouldn't move from jitters in the controller
			sz.errorsumP[0] = 0
			t1 = 0
		
		t2 = Kp[1] * error[1] + Ki[1]*sz.errorsumP[1] + Kd[1] * errordif[1]
		if desired_pos[2] ==0:
			sz.errorsumP[2] = 0
			t2 = 0

		t3 = Kp[2] * error[2] + Ki[2]*sz.errorsumP[2] + Kd[2] * errordif[2]
		if desired_pos[3] ==0:
			sz.errorsumP[3] = 0
			t3 = 0

		t4 = Kp[2] * error[3] + Ki[2]*sz.errorsumP[3] + Kd[2] * errordif[3]
		if desired_pos[1] ==0:
			sz.errorsumP[1] = 0
			t4 = 0
		
		sz.old_pos_error = error
		#command_torques = [t1,t2,t3]
		command_torques = [t1,t2,t3,t4]
		#print "command_torques are: "
		#print command_torques
		for i in range(4):
		#for i in range(3):  #check that scales speed commands that are too large down to something the system can manage. Good for position control/step inputs

			if command_torques[i] > 0.875:
				factor = command_torques[i] / 0.875
				for j in range(4):
					command_torques[j] =  command_torques[j] / factor
			if command_torques[i] < -0.875:
				factor = command_torques[i] / -0.875
				for j in range(4):
					command_torques[j] = command_torques[j] / factor

		return command_torques

	def PID_vel_control(sz,remote_or_auto): #position PI control
		desired_vel = sz.L_vel_desired #creates a local variable from the object variable
		actual_vel = sz.L_vel_actual
		

		Kp = sz.get_Kp_Gains(0) #gets PI gains from  the Main file
		Ki = sz.get_Ki_Gains(0) 		
		Kd = sz.get_Kd_Gains(0)

		error = np.array(desired_vel)-np.array(actual_vel)  #P control variable

		sz.errorsumV = sz.errorsumV + (np.array(desired_vel) - np.array(actual_vel))  #I control variable
		errorsummax = .5  #sets a limit on how large the I parameter can grow.  Needs to be tuned
		for i in range(4):  #antiwindup loop. Ensures I term never grows larger than the errorsum max
			if sz.errorsumV[i] > errorsummax:
				sz.errorsumV[i] = errorsummax
			elif sz.errorsumV[i] < -1*errorsummax:
				sz.errorsumV[i] = -1*errorsummax
		
		errordif = (error - sz.old_vel_error)/ sz.looptime #D control variable

		t1 = Kp[0] * (error[0]) +  Ki[0]*sz.errorsumV[0] + Kd[0] * errordif[0] #the PID control equation
		if desired_vel[0] ==0:  #if no command input. system shouldn't move from jitters in the controller
			sz.errorsumV[0] = 0
			t1 = 0

		#because this is the tensioning system, it should always overshoot the speed on retraction commands, and undershoot it on extension commands to maintain system tension.
		# t2 = Kp[1] * (error[1]) + Ki[1]*sz.errorsumV[1] + Kd[1] * errordif[1]  
		t2 = Kp[1] * (error[1]) * -1
		if t2 > 1:
			t2 = 1
		if t2 < -1:
			t2 = -1
		# t2 = (sz.L_vel_desired[1] / sz.r_winch) / 29.32
		# if desired_vel[1] ==0:
		# 	sz.errorsumV[1] = 0
		# 	t2 = 0
		# t2 = t2 * -1 #positive/negative are flipped on this motor relative to others.

		t3 = Kp[2] * (error[2]) + Ki[2]*sz.errorsumV[2] + Kd[2] * errordif[2]
		if desired_vel[2] ==0:
			sz.errorsumV[2] = 0
			t3 = 0
		# if t3 < 0:
		# 	t3 = t3 * 1.05

		t4 = Kp[2] * (error[3]) + Ki[2]*sz.errorsumV[3] + Kd[2] * errordif[3]
		if desired_vel[3] ==0:
			sz.errorsumV[3] = 0
			t4 = 0
		# if t4 < 0:
		# 	t4 = t4 * 1.05
		sz.old_vel_error = error
		#command_torques = [t1,t2,t3]
		command_torques = [t1,t2,t3,t4]

		max_pwm = 1.0

		for i in range(4):
			if command_torques[i] > max_pwm:
				factor = command_torques[i] / max_pwm
				for j in range(4):
					command_torques[j] =  command_torques[j] / factor
			if command_torques[i] < -max_pwm:
				factor = command_torques[i] / -max_pwm
				for j in range(4):
					command_torques[j] = command_torques[j] / factor

			if abs(command_torques[i]) < .0625:#stops jitters when arm is supposed to be stationary
				command_torques[i] = 0

		return command_torques

	def actuate_Motors(sz, c, remote_or_auto):  #converts desired user input commands into useable servo command inputs

		if remote_or_auto == 0:
			command_torques = sz.PID_vel_control(remote_or_auto)
		else:
			command_torques = sz.PID_pos_control(remote_or_auto)

		for i in range(4):
			if(m.isnan(command_torques[i])): #guarantees that the motors won't vibrate due to I control instabilities
				command_torques[i]=0

		# if command_torques[0] < .1 and command_torques[0] > 0.005:  #increases length changing velocity commands if they are too small to help jump the system into motion and avoid stiction and tooth tolerance delays from ruining control
		# 	command_torques[0] = .1
		# if command_torques[0] > -.1 and command_torques[0] < -0.005:
		# 	command_torques[0] = -.1

 		#modification of the constants allows user to specify an extra torque input for the tethers when retracting.  this helps ensure the tethers never go slack.  Probably an obsolete command that can be removed.  Command also flips the sign to fit the system convention.
 	# 	if command_torques[0] == 0 and command_torques[1] < -0.01:
 	# 		command_torques[1] = command_torques[1] - .04
 	# 	if command_torques[0] == 0 and command_torques[2] < -0.01:
 	# 		command_torques[2] = command_torques[2] - .04
		# if command_torques[0] == 0 and command_torques[3] < -0.01:
 	# 		command_torques[3] = command_torques[3] - .04

		# c.at.T1.set_torque(-command_torques[0])
		# # c.at.T2.set_torque(command_torques[1])

		#actuation alterations for the column motor.  Limits up and down speed.
		max_upward_torque = 75 + 200 * sz.L[0]
		command_torques[0] = int(command_torques[0] * 255)
		print "desired column torque is : "
		print command_torques[0]
		if command_torques[0] > max_upward_torque:
			command_torques[0] = max_upward_torque
		elif command_torques[0] < -90:
			command_torques[0] = -90
		T1command = str('k' + repr(command_torques[0]))
		sz.ser_encoder.write(T1command)
		sleep(.001)

		#actuation alterations for the torque motor.  Scales motor torque based on column height
		minimum = 8*sz.L[0] + 3
		command_torques[1] = command_torques[1] * 12
		maximum = 12
		#also sets torque bounds for operation.  And torque should be high when retracting and low when extending.
		if command_torques[1] < minimum: #under pi-theta motion, the torque should never be lower than this to ensure proper tensioning.
			command_torques[1] = minimum
		elif command_torques[1] > maximum:  #cannot output a torque higher than max pwm
			command_torques[1] = maximum
		if command_torques[0] > 0:  #if extending the torque load should increase with respect to length, but be less than normal.
			command_torques[1] = .25
		if command_torques[0] < 0:  #if retracting the torque should be high to guarantee tension remains effectatious.
			command_torques[1] = maximum - 2
		T2command = str('m' + repr(command_torques[1]))
		sz.ser_encoder.write(T2command)
		'''if desired position = current position and error is not close to zero.  Lock the arm in place as hard as possible.'''

		c.at.T3.set_torque(command_torques[2])
		c.at.T4.set_torque(command_torques[3])
		print "command torques are :"
		print 

	def trajectory(sz,itime): #makes the arm go in a circle

		ftime = itime + 10
		ctime = now()

		x = sz.sensed_pos_prev[0]
		y = sz.sensed_pos_prev[1]
		z = sz.sensed_pos_prev[2]

		r = np.sqrt(x**2 + y**2) #+ .0001
		theta = m.atan2(y,x)
		thetagoal = theta + .2*z

		x = r*m.cos(thetagoal)
		y=  r*m.sin(thetagoal)
		sz.goal = [x,y,z]
		sz.sensed_pos_prev = sz.goal

		L_goal = sz.cart2tether_actual(sz.goal)

		for i in range(4):
			sz.L_vel_desired[i] = (L_goal[i] - sz.L[i]) / sz.looptime

	def closing_function(sz): #turns off the torque control motor
		T1command = str('k' + repr(0))
		sz.ser_encoder.write(T1command)
		sz.ser_encoder.write('m0')
		sleep(8)
		sz.ser_encoder.write('o')


	def plot_update(sz):  #live plot updating function
		sz.xarray.append(now())  
		sz.y1array.append(sz.sensed_pos[0])
		sz.y2array.append(sz.sensed_pos[1])
		sz.y3array.append(sz.sensed_pos[2])
		sz.y1darray.append(sz.goal[0])
		sz.y2darray.append(sz.goal[1])
		sz.y3darray.append(sz.goal[2])

		if len(sz.xarray) > 200:
			sz.xarray.popleft()
			sz.y1array.popleft()
			sz.y2array.popleft()
			sz.y3array.popleft()
			sz.y3darray.popleft()
			sz.y2darray.popleft()
			sz.y1darray.popleft()
		sz.l1.set_xdata(sz.xarray)
		sz.l1.set_ydata(sz.y1array)
		sz.l2.set_xdata(sz.xarray)
		sz.l2.set_ydata(sz.y2array)
		sz.l3.set_xdata(sz.xarray)
		sz.l3.set_ydata(sz.y3array)

		sz.l1d.set_xdata(sz.xarray)
		sz.l1d.set_ydata(sz.y1darray)
		sz.l2d.set_xdata(sz.xarray)
		sz.l2d.set_ydata(sz.y2darray)
		sz.l3d.set_xdata(sz.xarray)
		sz.l3d.set_ydata(sz.y3darray)

		sz.ax.relim() 
		sz.ax.autoscale_view(True,True,True)
		sz.fig.canvas.draw()