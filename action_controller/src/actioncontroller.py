#! /usr/bin/env python

## @package action_controller
# Listens to trajectory and generates the appropriate velocity to move the Quadcopter.
# @author Rohan Bhargava
# @version 1.1.1
import roslib

#roslib.load_manifest('actionlib_tutorials')
roslib.load_manifest('action_controller')

import rospy
import actionlib
import actionlib_tutorials.msg
import moveit_msgs.msg
from actionlib.exceptions import *

import action_controller.msg
import trajectory_msgs.msg
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist, PoseStamped,Pose
from nav_msgs.msg import Odometry
import time

from planner_request import get_state
from planner_request import pid
from collections import deque

## Class for generating velocity for Spiri
class actioncontroller(object):
  ## Constructor
	def __init__(self,name):

		self.state=Pose()
		self._action_name=name
		self.traj=trajectory_msgs.msg.MultiDOFJointTrajectory()
		self._as=actionlib.ActionServer(self._action_name,ActionSpec=action_controller.msg.MultiDofFollowJointTrajectoryAction,goal_cb=self.goalcb,cancel_cb=self.cancelcb,auto_start=False)
		self.pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
		rospy.Subscriber('/ground_truth/state',Odometry,self.callback)
		self.lastPosition=Transform()
		self.vel=Twist()
		self.flag=True
		self.has_active_goal=False
		self.active_goal=actionlib.ServerGoalHandle()
		self._as.start()
		self.state=Pose()
		self.non_counter=0
		self.agg_scale=0.0
	## Callback function for the State topic.	This function will be called whenever a new message is published on a document
	# @param self Object pointer
	# @param data Contains data published on the topic
	def callback(self,data):

		self.state=data.pose.pose


	def goalcb(self,gh):
		if self.has_active_goal:
			self.pub.publish(Twist())
			active_goal.set_canceled()
			self.has_active_goal=False


		gh.set_accepted('goal accepted')
	        self.traj=gh.get_goal().trajectory
		self.active_goal=gh
		self.has_active_goal=True
		#print 'I am in the goal callback'
		#self.executetraj()
		goal=self.compute_goal()
		#self.publishvel_goal(goal)
		self.pid_vel(goal)
	def cancelcb(self,gh):
		if(self.active_goal==gh):
			self.pub.publish(Twist())
			self.active_goal.set_canceled()
			self.has_active_goal=False
			#print 'Cancelling the goal'




	def executetraj(self):
		traj_points=Transform()
		for i in range(len(self.traj.points)):
			traj_points=self.traj.points[i].transforms[0]
			#print traj_points.transforms[0].translation
			if(i!=0):
				self.flag=self.publishvel(traj_points,False)
				if i==len(self.traj.points)-1:
					if self.flag:
						self.publishvel(traj_points,False)
					else:
						self.publishvel(traj_points,True)

			self.pub.publish(Twist())
			if self.flag:
				self.lastPosition.translation=traj_points.translation
				self.lastPosition.rotation=traj_points.rotation
		self.active_goal.set_succeeded()
		#print ("goal has been reached")
		self.has_active_goal=False


	def publishvel_goal(self,goal):
		counter=0
		#for i in range(len(goal)):
		while counter<len(goal):
			#self.robot_state()
			#print 'state is',self.state
			#print 'your state should be',goal[counter-1]
			#if self.state==goal[i-1]:
			#	#print 'I have following the trajectory'

			#flag=True


			self.vel.linear.x=goal[counter].position.x-self.state.pose.position.x
			#self.vel.linear.x=self.pid_vel(goal[counter])
			self.vel.linear.y=goal[counter].position.y-self.state.pose.position.y
			self.vel.linear.z=goal[counter].position.z-self.state.pose.position.z
			#print "calculated the cmd vel topic"

			# this loop is just for the agressive behaviour
			if (abs(self.vel.linear.x)>=self.agg_scale or abs(self.vel.linear.y)>=self.agg_scale or abs(self.vel.linear.z)>=self.agg_scale):
				#print self.vel
				self.pub.publish(self.vel)
				#print"Published the commands"
				time.sleep(1.0)
			#self.robot_state()
			flag=self.checkgoal(goal[counter])
			if flag:
				counter=counter+1
				#return True
			else:
				self.non_counter=self.non_counter+1
				#print 'didnt reach previous goal'
			# unnecessary
			if self.non_counter>5:
				rospy.signal_shutdown('too many tries')
			self.pub.publish(Twist())
		self.active_goal.set_succeeded()
		#print 'goal has been reached'
		self.has_active_goal=False

	def publishvel(self,traj_points,anyway):
		self.vel.linear.x=traj_points.translation.x-self.lastPosition.translation.x
		self.vel.linear.y=traj_points.translation.y-self.lastPosition.translation.y
		self.vel.linear.z=traj_points.translation.z-self.lastPosition.translation.z
		#print "calculated the cmd vel topic"
		if (anyway or self.vel.linear.x>=0.5 or self.vel.linear.y>=0.5 or self.vel.linear.z>=0.5):
			self.pub.publish(self.vel)
			#print "Published the commands"
			time.sleep(1.0)
			return True
		else:
			return False
	def publishrotation(self,traj_points,start):
		self.vel.linear.x=0.0
		self.vel.linear.y=0.0
		self.vel.linear.z=0.0
		self.vel.angular.x=0.0
		self.vel.angular.y=0.0
		if start:
			self.vel.angular.z=0-self.traj_points.rotation.z
		else:
			self.vel.angular.z=self.traj_points.rotation.z
		self.pub.publish(self.vel)
		time.sleep(abs(self.vel.angular.z*3.0))
		self.vel.angular.z=0.0





	def compute_goal(self):
		goal=deque()
		#self.robot_state()
		goal.append(self.state)

		for i in range(1,len(self.traj.points)):
			temp=Pose()
			temp.position.x=goal[i-1].position.x+(self.traj.points[i].transforms[0].translation.x-self.traj.points[i-1].transforms[0].translation.x)
			temp.position.y=goal[i-1].position.y+(self.traj.points[i].transforms[0].translation.y-self.traj.points[i-1].transforms[0].translation.y)
			temp.position.z=goal[i-1].position.z+(self.traj.points[i].transforms[0].translation.z-self.traj.points[i-1].transforms[0].translation.z)
			goal.append(temp)
		#print self.traj.points
		#print goal
		return goal


	def checkgoal(self,goal):
		if abs(self.state.position.x-goal.position.x)<1.0 and abs(self.state.position.y-goal.position.y)<1.0 and abs(self.state.position.z-goal.position.z)<1.0:
			return True
		return False


	def pid_vel(self,goal):
		kp=rospy.get_param('kp',2.0)
		ki=rospy.get_param('ki',0.0)
		kd=rospy.get_param('kd',2.0)

		p=pid.PID([kp,kp,kp],[ki,ki,ki],[kd,kd,kd])
		'''
		for i in range(len(goal)):
			p.setPoint(temp_goal.position.z)
			#print 'completed goal',i
			for i in range(10):
				self.robot_state()
				temp=p.update(self.state.position.z)
				self.vel.linear.z=min(0.5,temp)
				#print self.vel.linear.z
				self.pub.publish(self.vel)
				time.sleep(1.0)
		'''
		#self.robot_state()

		#time_of_last_cycle=self.state.header.stamp.secs
		#while True:

		#for i in range(len(goal)):
		while len(goal)>0:
			temp_goal=goal.popleft()
			p.setPoint(temp_goal)

			while abs(self.state.position.x-temp_goal.position.x)>0.1 or abs(self.state.position.y-temp_goal.position.y)>0.1 or abs(self.state.position.z-temp_goal.position.z)>0.1:
				#dt=self.state.header.stamp.secs-time_of_last_cycle
				#time_of_last_cycle=self.state.header.stamp.secs
				temp=p.update(self.state)
				self.vel.linear.x=min(1.0,temp[0])
				self.vel.linear.y=min(1.0,temp[1])
				self.vel.linear.z=min(1.0,temp[2])
				#self.vel=min(1.0,temp)
				#self.vel.linear.z=temp
				#print'vel',self.vel
				self.pub.publish(self.vel)
				#time.sleep(1.0) # might not be required
				#self.pub.publish(Twist())
				#self.robot_state()
				#printkp,kd
			#print 'state',self.state.pose
			#print 'goal',temp_goal
			#print 'sub goal completed',i

		#time.sleep(1.0)
		self.pub.publish(Twist())
		self.active_goal.set_succeeded()
		#print 'goal has been reached'
		self.has_active_goal=False
#robot=get_state.Staterobot()
if __name__ == '__main__':

	rospy.init_node('multi_dof_joint_trajectory_action')
	#print rospy.get_name()

	actioncontroller(rospy.get_name())
	rospy.spin()
