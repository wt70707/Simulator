#! /usr/bin/env python

## @package spiri_controller
# Listens to trajectory and generates the appropriate velocity to move the Quadcopter. 
# @todo This should also convert cmd_vel to pitch
# @author Rohan Bhargava
# @version 1.1.1
import roslib

#roslib.load_manifest('actionlib_tutorials')
roslib.load_manifest('action_controller')

import rospy
import trajectory_msgs.msg
import actionlib
from spiri_api import pid
from spiri_api import spiri_api_python
from collections import deque
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
def compute_goal(traj):
  global spiri_obj
  global state
  current_state=state
  goal=deque()
  #spiri_obj=spiri_api_python()
  #current_state=spiri_obj.get_state()
  
  goal.append(current_state)

  for i in range(1,len(traj.points)):
	  temp=Pose()
	  temp.position.x=goal[i-1].position.x+(traj.points[i].transforms[0].translation.x-traj.points[i-1].transforms[0].translation.x)
	  temp.position.y=goal[i-1].position.y+(traj.points[i].transforms[0].translation.y-traj.points[i-1].transforms[0].translation.y)
	  temp.position.z=goal[i-1].position.z+(traj.points[i].transforms[0].translation.z-traj.points[i-1].transforms[0].translation.z)
	  goal.append(temp)
 
  return goal

  
def pid_vel(goal):
  global state
  current_state=state
  rospy.loginfo("Publishing vel")
  #spiri_obj=spiri_api_python()
  kp=rospy.get_param('kp',2.0)
  ki=rospy.get_param('ki',0.0)
  kd=rospy.get_param('kd',2.0)

  pid_object=pid.PID([kp,kp,kp],[ki,ki,ki],[kd,kd,kd])
  pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
  vel=Twist()
  while len(goal)>0 and rospy.get_param('/has_active_goal')==True:
	  temp_goal=goal.popleft()
	  pid_object.setPoint(temp_goal)
	  #current_state=spiri_obj.get_state()
	  while abs(current_state.position.x-temp_goal.position.x)>0.1 or abs(current_state.position.y-temp_goal.position.y)>0.1 or abs(current_state.position.z-temp_goal.position.z)>0.1:
		  
		  current_state=state
		  temp=pid_object.update(current_state)
		  vel.linear.x=min(1.0,temp[0])
		  vel.linear.y=min(1.0,temp[1])
		  vel.linear.z=min(1.0,temp[2])
		  
		  pub.publish(vel)
		  
  pub.publish(Twist())
  
  #self.active_goal.set_succeeded()
  #rospy.set_param('execution',True)
  #print 'goal has been reached'
  rospy.set_param('/has_active_goal',False)
  rospy.set_param('/execution_completed',True)


def vel_callback(traj):
  rospy.loginfo("Recieved Trajectory")
  goal=compute_goal(traj)
  pid_vel(goal)
  rospy.loginfo("Completed trajectory")
  
  
def state_callback(data):
  global state
  state=data.pose.pose


def spiri_listener():
  rospy.init_node('spiri_command_listener',anonymous=True)
  rospy.Subscriber("command",trajectory_msgs.msg.MultiDOFJointTrajectory,vel_callback)
  rospy.Subscriber("/ground_truth/state",Odometry,state_callback)
  rospy.spin()
  
state=Pose()

if __name__ == '__main__':
  
	  
  spiri_listener()
