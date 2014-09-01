#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from moveit_msgs.msg import PlanningSceneWorld
import time

def talker():
    pub = rospy.Publisher('/planning_scene_world', PlanningSceneWorld, queue_size=1)
    rospy.init_node('clear_ocotmap', anonymous=True)
    r = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    for i in range(5):

	    msg=PlanningSceneWorld()
	    msg.octomap.header.stamp=rospy.Time.now()
	    msg.octomap.header.frame_id='/nav'
	

	    rospy.loginfo(msg)
	    pub.publish(msg)
	    #time.sleep(1.0)
	    r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
