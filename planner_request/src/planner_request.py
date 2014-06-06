#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Constraints, JointConstraint


if __name__ =='__main__':
	#con=Constraints()
	#constriants are only possible in moveit_commander 0.5.6 --> indigo
	#con.name='test'
	#k=JointConstraint()
	#k.position=2.0
	#k.tolerance_above=1.0
	#k.tolerance_below=1.0
	#k.weight=1.0
	#k.joint_name='virtual_join'
	#k=['virtual_join',2.0,1.0,1.0,2.0]
	#con.joint_constraints.append(k)	
	#print con
	#if type(con) is Constraints:
	#	print 'succes'
	#else:
	#	print 'failure'
	
	group=MoveGroupCommander('spiri')
	group.set_planner_id('PRMkConfigDefault')
	#group.set_path_constraints(con)	
	#print conversions.msg_to_string(con)
	#group.set_workspace([-2.0,10.0,-2.0,10.0,-2.0,10.0])
	goal=group.get_current_joint_values()
	#print group.get_planning_frame()	
	goal[2]=2.0
	#print goal
 	group.set_joint_value_target(goal)
	group.plan()
