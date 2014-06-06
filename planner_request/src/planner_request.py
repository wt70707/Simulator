#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander

if __name__ =='__main__':
	group=MoveGroupCommander('spiri')
	group.set_planner_id('PRMkConfigDefault')
	group.set_workspace([-2.0,10.0,-2.0,10.0,-2.0,10.0])
	goal=group.get_current_joint_values()
	print group.get_planning_frame()	
	#goal[2]=2.0
	#print goal
 	#group.set_joint_value_target(goal)
	#group.plan()
