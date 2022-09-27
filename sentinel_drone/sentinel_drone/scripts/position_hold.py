#!/usr/bin/env python3


from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2,2,20] 

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]
		self.Kp = [5.22,5.28,0]
		self.Ki = [0.68,0.384,0]
		self.Kd = [187.8,190.8,0]
		# self.Kp = [0,0,0]
		# self.Ki = [0,0,0]
		# self.Kd = [0,0,0]
		self.prev_values = [0,0,0]
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.sample_freq = 15
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

		self.arm() # ARMING THE DRONE


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

	# Callback functions
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


	def pid(self):

		error = [0,0,0]
		error[0] = self.drone_position[0] - self.setpoint[0]
		error[1] = self.drone_position[1] - self.setpoint[1]
		error[2] = self.drone_position[2] - self.setpoint[2]
		I_throttle = (self.prevI[2] + error[2]) * self.Ki[2]
		I_roll = (self.prevI[0] + error[0]) * self.Ki[0]
		I_pitch = (self.prevI[1] + error[1]) * self.Ki[1]
		out_throttle = (self.Kp[2]*error[2]) + (self.Kd[2]*(error[2]-self.prev_values[2])) + I_throttle
		out_roll = (self.Kp[0]*error[0]) + (self.Kd[0]*(error[0]-self.prev_values[0])) + I_roll
		out_pitch = (self.Kp[1]*error[1]) + (self.Kd[1]*(error[1]-self.prev_values[1])) + I_pitch
		print("kp:",self.Kp)
		print("out_throttle",out_throttle)
		print("out_roll",out_roll)
		print("out_pitch",out_pitch)
		self.cmd.rcThrottle = 1500 + int(out_throttle)
		self.cmd.rcRoll = 1500 - int(out_roll)
		self.cmd.rcPitch = 1500 + int(out_pitch)
		self.prev_values = error 
		self.prevI = [I_pitch, I_roll, I_throttle]
		print("throttle",self.cmd.rcThrottle)
		print("Roll",self.cmd.rcRoll)
		print("pitch:",self.cmd.rcPitch)
		print("error:",error)
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
	#------------------------------------------------------------------------------------------------------------------------
		
		
		self.command_pub.publish(self.cmd)




if __name__ == '__main__':

	e_drone = Edrone()
	print("started node")
	r = rospy.Rate(e_drone.sample_freq) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
