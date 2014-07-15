#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
class Staterobot():
	def __init__(self,topic):
		self.state=PoseStamped()
		rospy.Subscriber(topic,Odometry,self.callback)
	def callback(self,data):
		
		self.state.pose=data.pose.pose
		self.state.header=data.header
	def getstate(self):
		return self.state	


if __name__=='__main__':
	obj=Staterobot('/ground_truth/state')
	rospy.init_node('spiri_state',anonymous=True)
	try:
		#print obj.position
		print obj.getstate()		
		rospy.spin()	
	except KeyboardInterrupt:
		print 'shutting down'
		
