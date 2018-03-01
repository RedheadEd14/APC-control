#!/usr/bin/env python
'''
I recommend running this code in ipython via:
>> run main.py
or in python  via:
>>python main.py
AAAAAARRGH!!!
'''
# Import pygame, which handles joysticks and other inputs
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
import function_header as sz
from jacobian import jacobianMatrix

# Specify module ID and the name we want to give each of them:
#modules = {0xE4: 'T1',0xEA:'T2',0xDF:'T3'}#,0xF8:'T4',
modules = {0x02:'T3',0x01:'T4'}#,0xE7:'T2'

if __name__=="__main__":

	print "Initializing Spiral Zipper Demo"  

	##### INITIALIZE CKBOT CLUSTER ###########
	if len(sys.argv) == 2:
		modules = {int(sys.argv[2]) : 'T3',int(sys.argv[4]) : 'T4'}
	c = L.Cluster()
	c.populate( len(modules), modules )

	# Limit module speed and torque for safety
	# torque_limit = 0.95
	# for m in c.itermodules():
	# 	m.set_torque_limit( torque_limit )

	##### INITIALIZE REMOTE CONTROLLER ######
	pygame.init()
	joystick.init()
	joystick.Joystick(0).init()

	drivable = True
	x1_move = 0 #joystick xy movement variables
	x2_move = 0
	y1_move = 0
	y2_move = 0

	z1_move = 0 #joystick z movement variables
	z2_move = 0
	z1_flag = False 
	z2_flag = False

	########### INITIALIZE ARMS ############
	r_winch = 0.019 #radius of the pulleys on the motors
	# r_winch = 0.0225 #radius of the pulleys on the motors

	looptime = 0.1

	sz1 = sz.SpiralZipper(r_winch, c, looptime)
	sz1.plot_update()
	#### SETS VELOCITY CONTROL GAINS [T1, T2, T3] ####
	Kp = [10, 10, 1.875]  #proporional gains 
	Ki = [0, 0.01, 0.075] #integral gains
	Kd = [0, 0.1, 0.175] #derivative gains

	sz1.set_PID_Gains(Kp,Ki,Kd,0)

	#### SETS POSITION CONTROL GAINS [T1, T2, T3] ####
	Kp = [60, 15, 15] #proportional gains
	Ki = [0, 0, 0] #integral gains
	Kd = [0, 0, 0] #derivative gains

	sz1.set_PID_Gains(Kp,Ki,Kd,1)

	#### SETS JOYSTICK INPUT GAINS ####
	Kx = 0.05 
	Ky = 0.05
	Kz = 0.02

	#end_effector_old = np.matrix([[.25],[0],[.4]])
	#end_effector_des = np.matrix([[.25],[0],[.4]])
	#sz1.slack_removal(c)
	sensed_pos_prev = [0,0,0]
	flag = 0 #toggles trajectory motion with joystick motion.  Need to get rid of.
	mode = 0  #State machine flag
	sz1.errorsumV = [0,0,0,0]

	print "Spiral Zipper demo initialized"
	old_report = now()

	#################### MAIN LOOP #########################
	while True: 
		try:
			time_elapsed = now() - old_report
			if time_elapsed >= looptime:
				print "loop time is :" 
				print time_elapsed
				old_report = now()
				################### Joystick Input Case Structure ##################
				for evt in event.get(): 
					if evt.type == JOYAXISMOTION:   # Controller Joystick Inputs. Sets inputs to the Velocity Controller
						if evt.axis == 0:
							x1_move = evt.value    #arm 1. xy joystick control inputs
 						elif evt.axis == 1:
							y1_move = evt.value
 						elif evt.axis == 2:
							x2_move = evt.value    #arm 2. xy joystick control inputs
 						elif evt.axis == 3:
							y2_move = evt.value

					elif evt.type == JOYBUTTONDOWN:  #Controller Button inputs. Sets inputs to Position Controller. Toggles between V and P control.
						if evt.button is 0: #toggles arm between Velocity to Position Control.
							mode = 0
							sz1.errorsumV = [0,0,0,0]
							sz1.update_goal([0,0,0],mode)

						elif evt.button is 1: #Reinitializes outer loop control using the IMU.
							#sz1.L[0] = sz1.sensed_lidar()
							#rotation = sz1.get_sensor_readings()
							#sz1.sensed_pos = sz1.rotate(rotation[0],rotation[1],sz1.L[0])
							rotation = sz1.get_encoder_readings()
							sz1.sensed_pos = sz1.rotate_encoder(rotation[0],rotation[1],sz1.L[0])
							sz1.L = sz1.cart2tether_actual(sz1.sensed_pos)  #tether length update based on IMU only
							sz1.goal = sz1.sensed_pos
							sz1.sensed_pos_prev = sz1.goal

						elif evt.button is 2: #sets a desired position in Position control mode
							mode = 1 
							sz1.update_goal([-.1, 0, .35],mode) 

						elif evt.button is 3: #triggers velocity control
							initial_time = now()
							flag = not flag

						# elif evt.button is 4: #arm 1 up in Velocity control mode
						# 	z1_flag = not z1_flag
						# 	sz1.slack_removal(c)

						# 	if z1_flag:
						# 		z1_move = 1
						# 	else:
						# 		z1_move = 0

						# elif evt.button is 6: #arm 1 down in Velocity control mode
						# 	z1_flag = not z1_flag
						# 	if z1_flag:
						# 		z1_move = -1
						# 	else:
						# 		z1_move = 0

						# elif evt.button is 5: #arm 2 up in Velocity control mode
						# 	z2_flag = not z2_flag
						# 	if z2_flag:
						# 		z2_move = 1
						# 	else:
						# 		z2_move = 0

						# elif evt.button is 7: #arm 2 down in Velocity control mode
						# 	z2_flag = not z2_flag
						# 	if z2_flag == True:
						# 		z2_move = -1
						# 	else:
						# 		z2_move = 0

						elif evt.button is 8: #sets a desired position in Position control mode
							mode = 1
							sz1.update_goal([0, -0.1,0.35],mode) 

						elif evt.button is 9: #sets a desired position in Position control mode
							mode = 1 
							sz1.update_goal([0, 0.1,0.35],mode) 

						else:
							raise KeyboardInterrupt
				if drivable:

					sz1.update_state(c)
					
					######################## ROBOT STATE MACHINE ##############################
					if mode == 0: #Velocity control 
						
						if flag == 1: #BAD AND EVIL FLAG.  set flag that bypasses user input velocity commands for a precomputed path using the velocity controller
							sz1.trajectory(initial_time)
						else :
							Kx = (sz1.L[0] * .05 + .001)
							Ky = Kx
							sz1.update_goal([x2_move*Kx,-y2_move*Ky,y1_move*Kz],mode)
						remote_or_auto = 0  	#flag toggles features in the functions called below that are calibrated to the two different controllers

					else: #When arm is in position mode
						remote_or_auto = 1		

					if flag == 0: #BAD AND EVIL FLAG. Flag is only 1 when the system is in precomputed trajectory mode.
						sz1.set_tether_speeds(mode) 

					sz1.actuate_Motors(c,remote_or_auto) 
				
					####################### DATA MONITORING #################################

					# sensed_grav = sz1.get_encoder_readings()
					# # while (sensed_grav[0] == 0 and sensed_grav[1] == 0 and sensed_grav[2] == 0):
					# # 	sensed_grav = sz1.get_encoder_readings() #takes the average of a set of IMU readings
					# 	# print sensed_grav
					# sensed_pos = sz1.rotate_encoder(sensed_grav[0], sensed_grav[1],sz1.L[0])

					# velocity_measured = (sz1.sensed_pos -sz1.sensed_pos_prev) / sz1.looptime
					# sensed_pos_prev = sz1.sensed_pos


					# j_velocity = sz1.Jacobian_Math(sensed_pos)
					# print "Jacobian velocity is : "
					# print np.squeeze(j_velocity)

					# print "measured velocity is "
					# print velocity_measured

					print "position is "
					print sz1.sensed_pos
					# print "goal is"
					# print sz1.goal

					
					# print "arm tether velocities: "
					# print sz1.L_vel_actual

					#print "end effector position is : "
					#print end_effector_pos


					print "current tether lengths : "
					print sz1.L

					# print "arm tether velocities: "
					# print sz1.L_vel_actual
					#print "arm 1 desired tether velocities: "			
					#print s z1.L_vel_desired

					# Torques = [c.at.T1.get_torque(), 
					# 	   c.at.T2.get_torque(), 
					# 	   c.at.T3.get_torque()]

					# print "Torques are: "
					# print Torques

					########## Graphing ###########
					# updates liveplots based on IMU data and desired inputs.
					sz1.plot_update()
					
				else:
					for m in c.itermodules():
						m.set_torque(0) #not go_slack - causes arm to collapse
		#send 
						if m == 2:
							m.set_torque_mx(0)
		except KeyboardInterrupt or ValueError:
			# Break out of the loop
			print "Keyboard Interrupt detected"
			break
		'''except Exception,e:

			print "\n\nERROR DETECTED."
			print str(e)
			print "\n\n"
			sleep(1)
			# Break out of the loop
			pass
			#break'''

	#np.savetxt('/home/modlab/Desktop/data.txt', big_list, delimiter=' / ')
	print "Demo exiting, turning off all modules"
	# Turn the modules off before we exit for safety
	for m in c.itermodules():
		m.set_torque(0)

	print "Closing all data files..."
	sz1.closing_function()

	# sz1.data_store_measurement.close()
	# sz1.data_store_estimate.close()
	# sz1.data_store_predict.close()