#!/usr/bin/env python3

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from sentinel_drone.msg import Geolocation
import rospy
import time


def location_callback(msg):
	rospy.loginfo(f"received data {msg.lat}")
def qgis_lat():
	rospy.init_node('qgis')
	rospy.Subscriber('/geolocation', Geolocation, location_callback)
	rospy.spin()

if __name__ == '__main__':
	print("started qgis node")
	while not rospy.is_shutdown():
		qgis_lat()
		
