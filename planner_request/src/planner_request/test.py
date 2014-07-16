#!/usr/bin/env python
import rospy
from planner_request import get_state
import time


if __name__=='__main__':
  spiri=get_state.Staterobot()
  #rospy.init_node('spiri_state',anonymous=True)
  try:
    #print obj.getstate()
    #print spiri.state.pose.orientation.w
    #time.sleep(1.0)
    spiri.send_goal_relative(0,0,1)
    #print 'succeeded'

    #rospy.spin()
  except KeyboardInterrupt:
    print 'shutting down'
#spiri=get_state.Staterobot()
#print spiri.state
#spiri.send_goal(0,0,1)
