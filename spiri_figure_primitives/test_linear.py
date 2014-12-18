#!/bin/python

""" 

Author: Trevor Gamblin
Date: 2014-12-17

This file is a test bed for flying linear trajectories with Spiri in Gazebo

"""

from Trajectory import *
from Linear import *
import rospy
import math

def main():
  
  linear1 = Line(length = 1.0, max_vel = 1, numsteps = 16, pub_rate = 2)
  #linear1.ros_publish_line(mode = 1)
  #linear1.stop_all_vel()
  #linear1.ros_publish_line(mode = 2)
  #linear1.stop_all_vel()
  linear1.ros_publish_line(mode = 3)
  linear1.stop_all_vel()
 
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
