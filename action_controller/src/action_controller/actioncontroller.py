#! /usr/bin/env python

## @package action_controller
# Listens to trajectory and sends it spiri controller
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


from spiri_api import pid
from collections import deque

## Class to listen for trajectories from moveit_simple_controller manager.
class actioncontroller(object):
  ## Constructor
	def __init__(self,name):

		self.state=Pose()
		self._action_name=name
		self.traj=trajectory_msgs.msg.MultiDOFJointTrajectory()
		self._as=actionlib.ActionServer(self._action_name,ActionSpec=action_controller.msg.MultiDofFollowJointTrajectoryAction,goal_cb=self.goalcb,cancel_cb=self.cancelcb,auto_start=False)
		self.pub=rospy.Publisher('command',trajectory_msgs.msg.MultiDOFJointTrajectory,queue_size=1)
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
		
	## Callback function for the State topic.This function will be called whenever a new message is published on a document
	# @param self Object pointer
	# @param data Contains data published on the topic
	def callback(self,data):

		self.state=data.pose.pose
		
		if rospy.get_param('/has_active_goal',True)==True:
		  return
		if rospy.get_param('/execution_completed',False)==False:
		  return
		  
		self.active_goal.set_succeeded()
		rospy.set_param('/execution_completed',False)
		

	def goalcb(self,gh):
		


		gh.set_accepted('goal accepted')
	        self.traj=gh.get_goal().trajectory
		self.active_goal=gh
		rospy.set_param('/has_active_goal',True)
		print 'I am in the goal callback'
		self.pub.publish(self.traj)
		#self.active_goal.set_succeeded()
		#self.executetraj()
		#goal=self.compute_goal()
		#rospy.set_param('cution',False)
		#self.publishvel_goal(goal)
		#self.pid_vel(goal)
	def cancelcb(self,gh):
		print 'cancelling the goal'
		if(self.active_goal==gh):
			#self.pub.publish(Twist())
			self.active_goal.set_canceled()
			self.has_active_goal=False
			#print 'Cancelling the goal'
			rospy.set_param('/has_active_goal',False)
			#self.has_active_goal=False


	
if __name__ == '__main__':

	rospy.init_node('multi_dof_joint_trajectory_action')
	#print rospy.get_name()

	actioncontroller(rospy.get_name())
	rospy.spin()
