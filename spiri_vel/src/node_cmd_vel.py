#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_node', anonymous=True)
    r = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
	vel=Twist()
	vel.linear.x=rospy.get_param('/cmd_vel/x',0)
	vel.linear.y=rospy.get_param('/cmd_vel/y',0)
	vel.linear.z=rospy.get_param('/cmd_vel/z',0)
	pub.publish(vel)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass