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
from planner_request import figures
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
waygoals=[]
goal=[]


def callback(data):
	#print 'inside the callback'

	global counter
	global waygoals
	global goal
	global figure
	global axis
	t=data.status
	group=MoveGroupCommander('spiri')
	group.set_planner_id('PRMkConfigDefault')

	#goal=group.get_current_joint_values()
	#print t.status


	if(t.status==3):
		counter=counter+1
		time.sleep(1)
		#print 'completed'+str(counter)+'goal'
		# this is a robot state message
		start=obj.getposition()
		group.set_start_state(start)

		#goal=group.get_current_joint_values()
		#print start
		# a list

		if counter==1:
			goal=obj.getstate()
			if figure=='square':
				#print 'its a sqaure'
				if axis=='None':
					#print 'no axis specifed'
					waygoals=figures.square_compute((goal[0],goal[1],goal[2]),(goal[0]+2,goal[1]+2,goal[2]+2))
					#print waygoals
				elif axis=='x':
					waygoals=figures.square_compute_vertical((goal[0],goal[1],goal[2]),(goal[0]+2,goal[1]+2,goal[2]+2),'x')
				elif axis=='y':
					waygoals=figures.square_compute_vertical((goal[0],goal[1],goal[2]),(goal[0]+2,goal[1]+2,goal[2]+2),'y')
			if figure=='triangle':
				if axis=='None':
					waygoals=figures.triangle_compute((goal[0],goal[1],goal[2]),1,2)
				if axis=='x':
					waygoals=figures.triangle_compute_vertical((goal[0],goal[1],goal[2]),1,2,'x')

				if axis=='y':
					waygoals=figures.triangle_compute_vertical((goal[0],goal[1],goal[2]),1,2,'y')
			if figure=='sine wave':
					if axis=='None':
						waygoals=figures.horizontal_sine_wave((goal[0],goal[1],goal[2]))
					if axis=='x':
						waygoals=figures.vertical_sine_wave((goal[0],goal[1],goal[2]),axis='x')
					if axis=='y':
						waygoals=figures.vertical_sine_wave((goal[0],goal[1],goal[2]),axis='y')


			#waygoals=figures.vertical_sine_wave((goal[0],goal[1],goal[2]),axis='x')

		#goal_1=goal
		#print 'waygoal',waygoals
		goal[0]=waygoals[counter-1][0]
		goal[1]=waygoals[counter-1][1]
		goal[2]=waygoals[counter-1][2]
		#group.set_start_state(goal)
		#goal[0]=goal[0]+2.0
		#print goal[0],goal[1],goal[2]

		group.set_joint_value_target(goal)
		plan=group.plan()
		#time.sleep(2.0)
		group.execute(plan)
		if counter==len(waygoals):
			rospy.signal_shutdown('completed the figure')

	if(t.status==2):
		print 'timeout'

def callback_imu(data):
	global obj
	#print obj.getstate()

figure="None"
axis="None"
def listener():

	rospy.init_node('spiri_goal',anonymous=True)
	global figure
	global axis
	#print obj.getstate()
	figure=rospy.get_param('figure')
	#print figure
	axis=rospy.get_param('axis')
	#print axis
	rospy.Subscriber('multi_dof_joint_trajectory_action/result',MultiDofFollowJointTrajectoryActionResult,callback)

	rospy.Subscriber('/raw_imu',Imu,callback_imu)
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
