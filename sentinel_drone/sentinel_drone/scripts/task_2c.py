#!/usr/bin/env python3


from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge as bridge
import cv2 
import numpy as np

class Edrone():
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_waypoint[self.iterator], y_waypoint[self.iterator], z_waypoint[self.iterator]]
		self.waypoint = [-4.5,4.5,21] 
		self.iterator = 0;
		self.delay = 0;
		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 0
		self.cmd.rcAUX2 = 0
		self.cmd.rcAUX3 = 0
		self.cmd.rcAUX4 = 0
		self.flag =1

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]
		self.Kp = [30.6,30.6,39.5]
		self.Ki = [0,0,0.0] #197
		self.Kd = [1200,1200,900] #1223
		self.prev_values = [0,0,0]
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.sample_freq = 28                    #control loop frequency of 28hz
		self.prevI = [0,0,0]                     #roll,pitch,throttle



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.pitch_err_publisher = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_err_publisher = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.alt_err_publisher = rospy.Publisher('/alt_error', Float64, queue_size=1)

		#------------------------------------------------------------------------------------------------------------

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, self.get_image)

		self.arm() # ARMING THE DRONE

	def get_image(self,img_msg):
		br = bridge()
		img = br.imgmsg_to_cv2(img_msg, "passthrough")
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# cv2.imshow("sd",img)
		# if cv2.waitKey(25) & 0xFF==ord('q'):
		# 	print("lihli image")
		# 	cv2.imwrite('/home/sid/test_sd.jpg', img)
		##do CV Stuff and give me x,y,z co 
		lower_yellow = np.array([0, 150, 150])
		upper_yellow = np.array([130, 255, 255])

		img = cv2.GaussianBlur(img, (3,3), 0)
		thresh = cv2.inRange(img, lower_yellow, upper_yellow)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
		opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

		contours, hierarchy = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		# list(contours).sort(key=lambda x: -cv2.contourArea(x))
		if len(contours)==0:
			print(-1, -1)
			if self.flag==1:
				self.waypoint = [-4.5,4.5,21] 
		    
		else:
			M = cv2.moments(contours[0])

			X = int(M['m10'] / M['m00'])
			Y = int(M['m01'] / M['m00'])
			height, width, n_channels = img.shape
			self.flag =0
			self.waypoint = [self.waypoint[0]-(width/2-X)/300,self.waypoint[1]+(height/2-Y)/300,21]
			print(width/2-X, height/2-Y)

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)


	# Whycon callback function
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	#---------------------------------------------------------------------------------------------------------------

	# Callback functions for PID Tuning package
	def altitude_set_pid(self,alt):
		pass
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3


	def pitch_set_pid(self,pitch):
		pass
		self.Kp[1] = pitch.Kp * 0.06 
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self, roll):
		pass
		self.Kp[0] = roll.Kp * 0.06 
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

	#----------------------------------------------------------------------------------------------------------------------
	def findWaypoint(self,x,y,step, counter):
	    if counter % 2:
        	y += step*(counter//2)*((-1)**(counter//2+1))
	    else:
	        x += step*(counter//2)*((-1)**(counter//2+1))
	    print(x, y)
	    return (x,y)


	def pid(self):
		#calculating error

		error = [0,0,0]
		error[0] = self.drone_position[0] - self.waypoint[0]
		error[1] = self.drone_position[1] - self.waypoint[1]
		error[2] = self.drone_position[2] - self.waypoint[2]
		print("error is: ",error)
		if [-0.5,-0.5,-0.5]<error<=[0.5,0.5,0.5] and self.flag==1:
			self.iterator+=1
			self.waypoint[0], self.waypoint[1] = self.findWaypoint(0, 0,4.2, self.iterator)
			pass
		#using clipping technique to control integral part of throttle as roll and pitch have no integral part
		I_throttle = (self.prevI[2] + error[2]) * self.Ki[2]
		if (error[2]>0 and I_throttle<0) or (error[2]<0 and I_throttle>0) or I_throttle<-50 or I_throttle>0 or error[2]<-1 or error[2]>1:
			I_throttle = 0;	
		I_roll = (self.prevI[0] + error[0]) * self.Ki[0]
		I_pitch = (self.prevI[1] + error[1]) * self.Ki[1]

		#PID formula
		out_throttle = (self.Kp[2]*error[2]) + (self.Kd[2]*(error[2]-self.prev_values[2])) + I_throttle
		out_roll = (self.Kp[0]*error[0]) + (self.Kd[0]*(error[0]-self.prev_values[0])) + I_roll
		out_pitch = (self.Kp[1]*error[1]) + (self.Kd[1]*(error[1]-self.prev_values[1])) + I_pitch

		#setting commands for drone from PID values
		self.cmd.rcThrottle = 1500 + int(out_throttle)
		self.cmd.rcRoll = 1500 - int(out_roll)
		self.cmd.rcPitch = 1500 + int(out_pitch)

		#updating previous error
		self.prev_values = error 

		#udating previous error sum for integral part
		self.prevI = [I_roll, I_pitch, I_throttle]
		

		#clipping output for drone requirements
		# print("Iter:",self.iterator)
		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]
		if self.cmd.rcPitch > self.max_values[1]:
			self.cmd.rcPitch = self.max_values[1]											
		if self.cmd.rcRoll > self.max_values[0]:
			self.cmd.rcRoll = self.max_values[0]
		if self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]
		if self.cmd.rcPitch < self.min_values[1]:
			self.cmd.rcPitch = self.min_values[1]											
		if self.cmd.rcRoll < self.min_values[0]:
			self.cmd.rcRoll = self.min_values[0]
	
		
		self.command_pub.publish(self.cmd)
		self.pitch_err_publisher.publish(error[2])
		self.roll_err_publisher.publish(error[0])
		self.pitch_err_publisher.publish(error[1])




if __name__ == '__main__':

	e_drone = Edrone()
	print("started node")
	r = rospy.Rate(e_drone.sample_freq) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	time.sleep(6)
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
