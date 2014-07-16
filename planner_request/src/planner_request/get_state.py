#!/usr/bin/env python
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
class Staterobot():
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
		# its a hack. The callback needs to be called before the user can send goals. Need to find  a better way to do this.
		time.sleep(1.0)
	def callback_state(self,data):

		#self.state.pose=data.pose.pose
		#self.state.header=data.header
		#print data
		self.state = data.pose.pose

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


	def send_goal(self,x,y,z):
		#x=float(x)
		#y=float(y)
		#z=float(z)
		#print type(x),type(y),type(z)

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
		goal=[0.0,0.0,0.0,state.orientation.x,state.orientation.y,state.orientation.z,state.orientation.w]#self.state.pose.position.y,self.state.pose.position.z,self.state.pose.orientation.x,self.state.pose.orientation.y,self.state.pose.orientation.z,self.state.pose.orientation.w]

		#print goal
		goal[0]=x
		goal[1]=y
		goal[2]=z
		group.set_joint_value_target(goal)
		plan=group.plan()

		#time.sleep(1.0)
		group.execute(plan)
		# need to return success or failure
	def send_goal_relative(self,x,y,z):
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
		goal=[0.0,0.0,0.0,state.orientation.x,state.orientation.y,state.orientation.z,state.orientation.w]#self.state.pose.position.y,self.state.pose.position.z,self.state.pose.orientation.x,self.state.pose.orientation.y,self.state.pose.orientation.z,self.state.pose.orientation.w]

		#print goal
		goal[0]=state.position.x+x
		goal[1]=state.position.y+y
		goal[2]=state.position.z+z
		group.set_joint_value_target(goal)
		plan=group.plan()

		#time.sleep(1.0)
		group.execute(plan)
		# should return something
if __name__=='__main__':
	obj=Staterobot()
	rospy.init_node('spiri_state',anonymous=True)
	try:
		#print obj.getstate()
		#obj.send_goal(0,0,1)
		print obj.state
		rospy.spin()
	except KeyboardInterrupt:
		print 'shutting down'
