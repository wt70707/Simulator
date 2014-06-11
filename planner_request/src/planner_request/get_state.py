#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

class Staterobot():
	def __init__(self,topic):
		self.position=[]
		#self.topic=topic
		#return position
		rospy.Subscriber(topic,Odometry,self.callback)
	def callback(self,data):
		self.position=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
	def getstate(self):
		return self.position	


if __name__=='__main__':
	obj=Staterobot('/ground_truth/state')
	rospy.init_node('spiri_state',anonymous=True)
	try:
		#print obj.position
		print obj.getstate()		
		rospy.spin()	
	except KeyboardInterrupt:
		print 'shutting down'
		
