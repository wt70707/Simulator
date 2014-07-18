#!/usr/bin/env python

## @package planner_request
# API for Spiri
# @author Rohan Bhargava
# @version 1.1.1

import roslib
roslib.load_manifest('hector_uav_msgs')
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3,Vector3Stamped, PointStamped, Transform, Pose
from sensor_msgs.msg import Imu,NavSatFix
from hector_uav_msgs.msg import Altimeter
from moveit_commander import MoveGroupCommander
# to send goals
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
# for conversion between datatype
from tf.transformations import euler_from_quaternion

## Class defining all the functions to control Spiri
class Staterobot():

	## Constructor
	## @todo The callback needs to be called before the user can send goals.
	def __init__(self):

		self.state=Pose() #PoseStamped()
		self.orientation=Quaternion()
		self.altimeter_height=0.0
		self.pressure_height=0.0
		self.gps=NavSatFix()
		self.gps_vel=Vector3()
		rospy.init_node('spiri_api',anonymous=True)

		rospy.Subscriber('/ground_truth/state',Odometry,self.callback_state)
		rospy.Subscriber('/raw_imu',Imu,self.callback_imu)
		rospy.Subscriber('/altimeter',Altimeter,self.callback_altimeter)
		rospy.Subscriber('/pressure_height',PointStamped,self.callback_pressure)
		rospy.Subscriber('/fix',NavSatFix,self.callback_gps)
		rospy.Subscriber('/fix_velocity',Vector3Stamped,self.callback_gps_vel)

		time.sleep(1.0)

	## Callback function for the state topic
	# @param self object pointer
	# @param data Contains the data published on the topic
	def callback_state(self,data):



		#self.state.pose=data.pose.pose
		#self.state.header=data.header
		#print data
		self.state = data.pose.pose

	## Callback function for the imu topic

	def callback_imu(self,data):

		self.orientation=data.orientation

	## Callback function for the altimeter topic


	def callback_altimeter(self,data):
		self.altimeter_height=data.altitude

	## Callback function for the pressure_height topic

	def callback_pressure(self,data):

		self.pressure_height=data.point.z

	## Callback function for the fix topic

	def callback_gps(self,data):

		self.gps.latitude=data.latitude
		self.gps.longitude=data.longitude
		self.gps.altitude=data.altitude


	## Callback function for the fix_velocity topic

	def callback_gps_vel(self,data):

		self.gps_vel=data.vector
	def getstate(self):
		# this can be used internally otherwise we can return 7 values. This is useful if the use needs all the 7 values at the same timestep.
		return self.state

	## API function to read the position x,y,z of Spiri

	## Returns an object.

	## Position can be accessed using obj.x,obj.y,obj.z
	# @return Object
	def get_position(self):

		return self.state.pose.position


	## API function to read the orientation of Spiri
	## This function gives the orientation by fusing data from IMU, GPS and other sensors


	## Returns an object.
	## Depending upon the format
	## Euler - Orientation can be accessed using obj.x,obj.y,obj.z
	## Quaternion - obj.x,obj.y,obj.z,obj.w
  # @param format 1=Quaternion, 0=Euler
	def get_orientation(self,format=0):

		if format==1:

			return self.state.pose.orientation
		else:

			euler=Vector3()
			temp=euler_from_quaternion([self.state.orientation.x,self.state.orientation.y,self.state.orientation.z,self.state.orientation.w])
			euler.x=temp[0];euler.y=temp[1];euler.z=temp[2]
			return euler


	## API function to read the orientation of Spiri. This function gives the orientation reported by the IMU

	## Depending upon the format
	## Euler - Orientation can be accessed using obj.x,obj.y,obj.z
	## Quaternion - obj.x,obj.y,obj.z,obj.w
	# @return object
	# @param format euler=Euler quat=Quaternion

	def get_orientation_imu(self,format='euler'):

	  # user wants it in quaternion
		if format=='quat':

			return self.orientation
		else:

			euler=Vector3()
			temp=euler_from_quaternion([self.state.orientation.x,self.state.orientation.y,self.state.orientation.z,self.state.orientation.w])
			euler.x=temp[0];euler.y=temp[1];euler.z=temp[2]
			return euler

	## API function to read the altitude of Spiri. This function gives the altitude reported by the Altimeter

	# @return float


	def get_height_altimeter(self):

		# returns one float
		return self.altimeter_height

	## API function to read the altitude of Spiri. This function gives the altitude reported by the Pressure sensor

	# @return float


	def get_height_pressure(self):

		# return one float
		return self.pressure_height

	## API function to read the GPS data of Spiri. This function gives the latitude, longitude, altitude reported by the GPS
  ## Data can be accessed using obj.latitude, obj.longitude, obj.altitude

	# @return object

	def get_gps_data(self):

		# return lat,long and height
		return self.gps

	## API function to read the GPS velocity of Spiri. This function gives the velocity reported by the GPS.
	## Data can be accessed using obj.x, obj.y, obj.z

	# return object.



	def get_gps_vel(self):

		return self.gps_vel

	## API function to send goals to Spiri. This function will send a goal with respect to the world.
	## The start position of Spiri is 0,0
	## @todo Return based something on success or failure
	## @todo This function should also take in GPS coordinates
	# @param x  coordinate in x axis
	# @param y coordinate in y axis
	# @param z coordinate in z axis
	def send_goal(self,x,y,z):




		group=MoveGroupCommander('spiri')
		group.set_planner_id('PRMkConfigDefault')
		#group.set_workspace([-10.0,10.0,-10.0,10.0,-10.0,10.0])
		state=self.getstate()
		start_state=RobotState()

		transform=Transform()
		transform.translation=state.position
		transform.rotation=state.orientation
		# is this our case? Need to test
		if len(start_state.multi_dof_joint_state.transforms)==0:
			start_state.multi_dof_joint_state.transforms.append(transform)
			start_state.multi_dof_joint_state.joint_names.append('virtual_join')
		# if there are other joints in the system
		else:
			start_state.multi_dof_joint_state.transforms=[]
			start_state.multi_dof_joint_state.transforms.append(transform)
			start_state.multi_dof_joint_state.joint_names=[]
			start_state.multi_dof_joint_state.joint_names.append('virtual_join')
		start_state.joint_state.header.frame_id='/nav'
		start_state.multi_dof_joint_state.header.frame_id='/nav'
		# the start state has to be RobotState.
		group.set_start_state(start_state)

		# goal oncly accepts a list
		#print state.orientation
		#print self.state
		goal=[0.0,0.0,0.0,state.orientation.x,state.orientation.y,state.orientation.z,state.orientation.w]
		#print goal
		goal[0]=x
		goal[1]=y
		goal[2]=z
		group.set_joint_value_target(goal)
		plan=group.plan()

		#time.sleep(1.0)
		group.execute(plan)
		# need to return success or failure

	##API function to send goals to Spiri. This function will send a goal with respect to the start position.

	##Example-:
	##Suppose Spiri is at 1,1,1 and we call this function send_gaol_relative(1,0,0).

	##Spiri will move 1 m in x direction.
	## @todo Return based something on success or failure
	# @param x  distance in metres in x axis
	# @param y  distance in metres in y axis
	# @param z distance in metres in z axis
	def send_goal_relative(self,x,y,z):

		group=MoveGroupCommanderlove('spiri')
		group.set_planner_id('PRMkConfigDefault')
		#group.set_workspace([-10.0,10.0,-10.0,10.0,-10.0,10.0])
		state=self.getstate()
		start_state=RobotState()

		transform=Transform()
		transform.translation=state.position
		transform.rotation=state.orientation
		# is this our case? Need to test
		if len(start_state.multi_dof_joint_state.transforms)==0:
			start_state.multi_dof_joint_state.transforms.append(transform)
			start_state.multi_dof_joint_state.joint_names.append('virtual_join')
		# if there are other joints in the system
		else:
			start_state.multi_dof_joint_state.transforms=[]
			start_state.multi_dof_joint_state.transforms.append(transform)
			start_state.multi_dof_joint_state.joint_names=[]
			start_state.multi_dof_joint_state.joint_names.append('virtual_join')
		start_state.joint_state.header.frame_id='/nav'
		start_state.multi_dof_joint_state.header.frame_id='/nav'
		# the start state has to be RobotState.
		group.set_start_state(start_state)

		# goal oncly accepts a list
		#print state.orientation
		#print self.state
		goal=[0.0,0.0,0.0,state.orientation.x,state.orientation.y,state.orientation.z,state.orientation.w]

		#print goal
		goal[0]=state.position.x+x
		goal[1]=state.position.y+y
		goal[2]=state.position.z+z
		group.set_joint_value_target(goal)
		plan=group.plan()

		#time.sleep(1.0)
		group.execute(plan)
		# should return something
