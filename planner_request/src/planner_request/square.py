#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('action_controller')
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from action_controller.msg import *
import actionlib
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Transform
class Staterobot():
	def __init__(self,topic):
		#self.position=[]
		self.state=Odometry()
		self.position=RobotState()
		self.transform=Transform()
		#return position
		rospy.Subscriber(topic,Odometry,self.callback)
	def callback(self,data):
		#self.state=data.pose.pose.positio
		self.transform.translation=data.pose.pose.position
		self.transform.rotation=data.pose.pose.orientation		
		self.state=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
		if len(self.position.multi_dof_joint_state.transforms)==0:
			self.position.multi_dof_joint_state.transforms.append(self.transform)
			self.position.multi_dof_joint_state.joint_names.append('virtual_join')
		else:
			self.position.multi_dof_joint_state.transforms=[]
			self.position.multi_dof_joint_state.transforms.append(self.transform)
			self.position.multi_dof_joint_state.joint_names=[]
			self.position.multi_dof_joint_state.joint_names.append('virtual_join')
		self.position.joint_state.header.frame_id='/nav'
		self.position.multi_dof_joint_state.header.frame_id='/nav'
	
	def getposition(self):
		return self.position	

	def getstate(self):
		return self.state

obj=Staterobot('/ground_truth/state')
counter=0
def callback(data):
	global counter
	t=data.status
	group=MoveGroupCommander('spiri')
	group.set_planner_id('PRMkConfigDefault')
	goal=group.get_current_joint_values()
	#print t.status	
	if(t.status==3):
		counter=counter+1
		time.sleep(5)
		print 'completed one goal'
		# this is a robot state message 		
		start=obj.getposition()
		group.set_start_state(start)
	
		#goal=group.get_current_joint_values()
		print start
		# a list
		goal=obj.getstate()
		if counter==1:
			print 'move in y direction'
			goal[1]=goal[1]+2.0
		elif counter==2:
			print 'move in x direction'
			goal[0]=goal[0]+2.0
		elif counter==3:
			print 'move back in y direction'
			goal[1]=goal[1]-2.0
		elif counter==4:
			print 'move back in x direction'
			goal[0]=goal[0]-2.0
		#group.set_start_state(goal)		
		#goal[0]=goal[0]+2.0
		print goal
		group.set_joint_value_target(goal)
		plan=group.plan()
		time.sleep(5.0)
		group.execute(plan)
		
	if(t.status==2):
		print 'timeout'
		
def callback_imu(data):
	global obj
	print obj.getstate()


def listener():
	
	rospy.init_node('spiri_goal',anonymous=True)
	
	#print obj.getstate()
	rospy.Subscriber('multi_dof_joint_trajectory_action/result',MultiDofFollowJointTrajectoryActionResult,callback)
	#rospy.Subscriber('/raw_imu',Imu,callback_imu)	
	try:
	
		rospy.spin()
	except KeyboardInterrupt:
		print 'shutting down'


if __name__=='__main__':
	listener()
	#group=MoveGroupCommander('spiri')
	#group.set_planner_id('PRMkConfigDefault')
	#goal=obj.getstate()
	#goal_2=RobotState()
	
	#t=Transform()
	#t.translation.z=1.0
	#goal_2.multi_dof_joint_state.transforms.append(t)
	#goal_2.multi_dof_joint_state.joint_names.append('virtual_join')
	#goal_2.multi_dof_joint_state.header.frame_id='/nav'
	#goal_2.joint_state.header.frame_id='/nav'
	#print goal_2
 	#print group.get_random_joint_values()	
	#group.set_start_state(goal_2)
	#goal=group.get_current_joint_values()
	#goal[0]=1.0
	#goal[2]=1.0
	#group.set_joint_value_target(goal)
	#group.plan()
		
		
