#!/usr/bin/env python

import rospy 
from moveit_commander import MoveGroupCommander
import time


if __name__=='__main__':
	
	group=MoveGroupCommander('spiri')
	group.set_planner_id('PRMkConfigDefault')
	#assuming we are on teh ground as this a script for takeoff
	goal=group.get_current_joint_values()
	goal[2]=goal[2]+1.0
	group.set_joint_value_target(goal)
	plan=group.plan()
	time.sleep(2.0)
	group.execute(plan)
