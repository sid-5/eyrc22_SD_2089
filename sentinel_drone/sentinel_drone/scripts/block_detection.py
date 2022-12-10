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
from osgeo import gdal
from sentinel_drone.msg import Geolocation
import subprocess

task2D_img_path = "/home/rajas/Sentinel_drone/Task_2d/task2d.tif"
saved_imgs_path = "/home/rajas/Sentinel_drone/Task_2d/test2"
csv_path = "/home/rajas/Sentinel_drone/Task_2d/test2/yellow_box.csv"
f  = open(csv_path, "w") #PATH

class Edrone():
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_waypoint[self.iterator], y_waypoint[self.iterator], z_waypoint[self.iterator]]
		self.waypoint = [0,0,21] 
		self.counter = 0
		self.delay = 0
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
		self.capture_flag = 0
		self.waypoint_flag = 0
		self.img_counter= 0
		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]
		self.Kp = [20.6,20.6,39.5]
		self.Ki = [0,0,0.0] #197
		self.Kd = [1200,1200,900] #1223
		self.prev_values = [0,0,0]
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.sample_freq = 28                    #control loop frequency of 28hz
		self.prevI = [0,0,0]                     #roll,pitch,throttle
		self.prev = [0,0]
		self.waypoint_queue = []
		self.keypoints = [0,0]
		self.ds = gdal.Open(task2D_img_path)
		self.base_img = cv2.imread(task2D_img_path)
		self.base_img = cv2.cvtColor(self.base_img, cv2.COLOR_BGR2GRAY)
        # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
		self.xoff, self.a, self.b, self.yoff, self.d, self.e = self.ds.GetGeoTransform()

		self.f = open(csv_path, "w") #PATH
		self.f.write(f'obj_id,lat,lon\n')
		self.f.close()

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.pitch_err_publisher = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_err_publisher = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.alt_err_publisher = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.loation_pblisher = rospy.Publisher('/geolocation', Geolocation, queue_size=1)

		#------------------------------------------------------------------------------------------------------------

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, self.get_image)

		self.arm() # ARMING THE DRONE

	def pixel2coord(self, x, y):
		"""Returns global coordinates from pixel x, y coords"""
		xp = self.a * x + self.b * y + self.xoff
		yp = self.d * x + self.e * y + self.yoff
		return(xp, yp)

	def pixel2coord_forBoxCentre(self, x, y, image_path):
		"""Returns global coordinates from pixel x, y coords"""
		ds = gdal.Open(f'{image_path}.tif')
        # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
		xoff, a, b, yoff, d, e = ds.GetGeoTransform()
		xp = a * x + b * y + xoff
		yp = d * x + e * y + yoff
		return(xp, yp)

	def validImage(self, image_mask):
		contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		M = cv2.moments(contours[0])
		X = int(M['m10'] / M['m00'])
		Y = int(M['m01'] / M['m00'])
		if (X>280 and X<360) and (Y>200 and Y<280):
			return True
		else:
			return False

	def geoReference(self, image_path):
		"""
		image -> captured by drone.
		"""
		image = cv2.imread(image_path)
		#img1 = cv2.imread('/home/rajas/Sentinel_drone/Task_2d/task2d.tif')  # CHECK THE PATH to the map.
		#img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
		img2 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		#sift
		orb = cv2.ORB_create(50000)

		keypoints_1, descriptors_1 = orb.detectAndCompute(self.base_img,None)
		keypoints_2, descriptors_2 =orb.detectAndCompute(img2,None)

		#feature matching
		bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

		matches = bf.match(descriptors_1,descriptors_2)
		matches = sorted(matches, key = lambda x:x.distance)

		query = "gdal_translate "

		for m in matches[:8]:
			x_base, y_base = (keypoints_1[m.queryIdx].pt)
			x_img, y_img = (keypoints_2[m.trainIdx].pt)
			x_new, y_new = self.pixel2coord(x_base, y_base)
			query += f"-gcp {x_img} {y_img} {x_new} {y_new} "
		query += f"-of GTiff {image_path} map-with-gcps.tif"
		cmd1 = query.split(" ")
		for i in cmd1:
			if i == '':
				print(i)
				cmd1.remove(i)
		subprocess.run(cmd1)
			#EXECUTE THE QUERIES FOR GEOREF -> THEN OPEN THE GEOTIFF FILE -> PIXEL2COORD on the centre point wrt translated geotiff file.
		t_srs = "+proj=longlat +ellps=WGS84"
		cmd2 = ["gdalwarp", "-overwrite", "map-with-gcps.tif",f"{image_path}.tif", "-t_srs", t_srs]
		subprocess.run(cmd2)
		print("GDAL Warp done...")
	
	def get_image(self,img_msg):
		br = bridge()
		img = br.imgmsg_to_cv2(img_msg, "passthrough")
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		lower_yellow = np.array([0, 150, 150])
		upper_yellow = np.array([130, 255, 255])
		image = img.copy()
		img = cv2.GaussianBlur(img, (3,3), 0)
		thresh = cv2.inRange(img, lower_yellow, upper_yellow)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
		opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

		contours, hierarchy = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		height, width, n_channels = img.shape
		# list(contours).sort(key=lambda x: -cv2.contourArea(x))
		if len(contours)>0:
			M = cv2.moments(contours[0])
			X = int(M['m10'] / M['m00'])
			Y = int(M['m01'] / M['m00'])
			if self.capture_flag and self.validImage(opening):
				new_img_path = saved_imgs_path+"/"+str(self.img_counter)+".jpg"
				cv2.imwrite(new_img_path, image)
				print("Img saved")
				self.capture_flag=0
				self.geoReference(new_img_path)
				lat, lon = self.pixel2coord_forBoxCentre(X, Y,new_img_path)
				print(X, Y, lat, lon)
				f = open(csv_path, "a") #PATH
				f.write(f"{self.img_counter},{lat},{lon}\n")
				f.close()
				data = Geolocation()
				data.objectid = f"{self.img_counter}"
				data.lat, data.long  = lat, lon
				self.loation_pblisher.publish(data)
				self.img_counter+=1

				# HE IMPLEMENT KARTA YEIL KA?

				# print("Length: ",len(self.waypoint_queue))
				# if len(self.waypoint_queue):
				# 	print(self.waypoint_queue[0][0], ", ", self.waypoint[0])
				# 	print(self.waypoint_queue[0][1] ,", ", self.waypoint[1])
				# if len(self.waypoint_queue) and abs(self.waypoint_queue[0][0] - self.waypoint[0]) < 2 and abs(self.waypoint_queue[0][1] - self.waypoint[1]) < 2:
				# 	self.waypoint_queue.pop(0)
				# 	print("**inside thresh check ",self.waypoint_queue)
				# self.findWaypoint(7)
				# return
 
			self.flag =0
			self.waypoint_flag = 1
			if self.waypoint_flag:
				self.waypoint = [self.waypoint[0]-((width/2-X)/2600),self.waypoint[1]-((height/2-Y)/3100),21]
				# print(self.waypoint)
				self.waypoint_flag = 0
		else:
			self.waypoint_flag = 1


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
	def findWaypoint(self, step):
		if len(self.waypoint_queue)==0:
			
			self.counter += 1

			if self.counter % 2:
				# self.waypoint[1] += step*(self.counter//2)*((-1)**(self.counter//2+1))
				waypoint_1 = self.keypoints[1] + step*(self.counter//2)*((-1)**(self.counter//2+1))
				print(f'{self.keypoints} is old corner point')
				print(f'{self.keypoints[0]},{waypoint_1} is new corner point')
				print(f'{self.counter} is current counter')
				if waypoint_1>self.keypoints[1]:
					for i in range(self.keypoints[1], waypoint_1, 7): #check step
						self.waypoint_queue.append((self.keypoints[0], i))
				else:
					for i in range(self.keypoints[1], waypoint_1, -7):
						self.waypoint_queue.append((self.keypoints[0], i))
				self.waypoint_queue.append((self.keypoints[0], waypoint_1))
				self.keypoints[1] = waypoint_1
			else:
				waypoint_0 = self.keypoints[0] + step*(self.counter//2)*((-1)**(self.counter//2+1))
				print(f'{self.keypoints} is old corner point')
				print(f'{waypoint_0},{self.keypoints[1]} is new corner point')
				print(f'{self.counter} is current counter pnt')
				if waypoint_0>self.keypoints[0]:
					for i in range(self.keypoints[0], waypoint_0, 7):
						self.waypoint_queue.append((i, self.keypoints[1]))
				else:
					for i in range(self.keypoints[0], waypoint_0, -7):
						self.waypoint_queue.append((i, self.keypoints[1]))
				self.waypoint_queue.append((waypoint_0, self.keypoints[1]))
				self.keypoints[0] = waypoint_0
		self.prev = [self.waypoint[0], self.waypoint[1]]
		print(self.waypoint_queue)
		self.waypoint[0], self.waypoint[1] = self.waypoint_queue.pop(0)
	    

	def pid(self):
		#calculating error

		error = [0,0,0]
		error[0] = self.drone_position[0] - self.waypoint[0]
		error[1] = self.drone_position[1] - self.waypoint[1]
		error[2] = self.drone_position[2] - self.waypoint[2]
		
		if (error[0]>-0.2 and error[0]<0.2) and (error[1]>-0.2 and error[1]<0.2) and (error[2]>-0.2 and error[2]<0.2):
			print(self.counter, self.waypoint, self.flag)
			if  self.flag:
				self.findWaypoint(7)
				return
			else:
				self.capture_flag = 1
				self.waypoint[0:2] = self.prev
				self.findWaypoint(7)
				self.flag = 1
				return
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