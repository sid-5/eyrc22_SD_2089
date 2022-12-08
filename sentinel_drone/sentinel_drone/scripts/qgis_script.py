#!/usr/bin/env python3

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from sentinel_drone.msg import Geolocation
import rospy
import time



canvas = iface.mapCanvas()


def location_callback(msg):
	rospy.loginfo(f"received data {msg.lat}")
	pnt = QgsPointXY(msg.long, msg.lat)
	m = QgsVertexMarker(canvas)
	m.setCenter(pnt)
	m.setColor(QColor('Black'))
	m.setIconType(QgsVertexMarker.ICON_CIRCLE)
	m.setIconSize(12)
	m.setPenWidth(1)
	m.setFillColor(QColor(0, 200, 0))

def qgis_lat():
	rospy.init_node('qgis')
	rospy.Subscriber('/geolocation', Geolocation, location_callback)
	rospy.spin()

if __name__ == '__main__':
	print("started qgis node")
	while not rospy.is_shutdown():
		qgis_lat()
		
