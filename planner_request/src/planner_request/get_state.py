#!/usr/bin/env python
import roslib
roslib.load_manifest('hector_uav_msgs')
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3,Vector3Stamped, PointStamped
from sensor_msgs.msg import Imu,NavSatFix
from hector_uav_msgs.msg import Altimeter
# for conversion between datatype
from tf.transformations import euler_from_quaternion
class Staterobot():
	def __init__(self):
		self.state=PoseStamped()
		self.orientation=Quaternion()
		self.altimeter_height=0.0
		self.pressure_height=0.0
		self.gps=NavSatFix()
		self.gps_vel=Vector3()
		rospy.Subscriber('/ground_truth/state',Odometry,self.callback_state)
		rospy.Subscriber('/raw_imu',Imu,self.callback_imu)
		rospy.Subscriber('/altimeter',Altimeter,self.callback_altimeter)
		rospy.Subscriber('/pressure_height',PointStamped,self.callback_pressure)
		rospy.Subscriber('/fix',NavSatFix,self.callback_gps)
		rospy.Subscriber('/fix_velocity',Vector3Stamped,self.callback_gps_vel)
	def callback_state(self,data):

		self.state.pose=data.pose.pose
		self.state.header=data.header

	def callback_imu(self,data):
		self.orientation=data.orientation

	def callback_altimeter(self,data):
		self.altimeter_height=data.altitude

	def callback_pressure(self,data):
		self.pressure_height=data.point.z

	def callback_gps(self,data):
		self.gps.latitude=data.latitude
		self.gps.longitude=data.longitude
		self.gps.altitude=data.altitude

	def callback_gps_vel(self,data):
		self.gps_vel=data.vector
	def getstate(self):
		# this can be used internally otherwise we can return 7 values. This is useful if the use needs all the 7 values at the same timestep.
		return self.state

	def getposition(self):

		return self.state.pose.position
	def getorientation(self,format=0):
		if format==1:

			return self.state.pose.orientation
		else:

			euler=Vector3()
			temp=euler_from_quaternion([self.state.pose.orientation.x,self.state.pose.orientation.y,self.state.pose.orientation.z,self.state.pose.orientation.w])
			euler.x=temp[0];euler.y=temp[1];euler.z=temp[2]
			return euler
	def getorientation_imu(self,format=0):
	  # user wants it in quaternion
		if format==1:

			return self.orientation
		else:

			euler=Vector3()
			temp=euler_from_quaternion([self.state.pose.orientation.x,self.state.pose.orientation.y,self.state.pose.orientation.z,self.state.pose.orientation.w])
			euler.x=temp[0];euler.y=temp[1];euler.z=temp[2]
			return euler

	def getheight_altimeter(self):
		# returns one float
		return self.altimeter_height

	def getheight_pressure(self):
		# return one float
		return self.pressure_height
	def getgpsdata(self):
		# return lat,long and height
		return self.gps

	def getgpsvel(self):
		return self.gps_vel

if __name__=='__main__':
	obj=Staterobot()
	rospy.init_node('spiri_state',anonymous=True)
	try:
		#print obj.getstate()
		print obj.getgpsvel()
		rospy.spin()
	except KeyboardInterrupt:
		print 'shutting down'
