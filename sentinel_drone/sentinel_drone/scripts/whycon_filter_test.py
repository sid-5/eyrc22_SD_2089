#!/usr/bin/env python3

#To use this first run task_4a.launch file and then rosrun this node


from edrone_client.msg import *
import rospy
from geometry_msgs.msg import PoseArray,Pose
filter_val =0.05
filtered = PoseArray()
p = Pose()
p.position.x=0
p.position.y =1
p.position.z = 2
filtered.poses.append(p)

inp = [0,0,0]
pub = rospy.Publisher("filter_z", PoseArray, queue_size=10 )

def main():	
	rospy.init_node("whycon_test")
	
	rospy.Subscriber("/whycon/poses", PoseArray, fil)
	while not rospy.is_shutdown():
		pub.publish(filtered)

def fil(msg):
	inp = [msg.poses[0].position.x,msg.poses[0].position.y,msg.poses[0].position.z]
	filtered.poses[0].position.x = inp[0]*filter_val + (1-filter_val)*filtered.poses[0].position.x
	filtered.poses[0].position.y = inp[1]*filter_val + (1-filter_val)*filtered.poses[0].position.y
	filtered.poses[0].position.z = inp[2]*filter_val + (1-filter_val)*filtered.poses[0].position.z

if __name__ == "__main__":
	main()
