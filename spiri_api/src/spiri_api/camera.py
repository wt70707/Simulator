#!/usr/bin/env python

## @package spiri_api
# Camera interface between User and Spiri. The messages are returned in Opencv format.
# @author Rohan Bhargava
# @version 1.2.0
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import cv2
## Class defining all the functions for perception
class camera_interface():

  ## Constructor
  def __init__(self):
    self.left_image=Image()
    self.right_image=Image()
    self.bottom_image=Image()
    self.bridge=CvBridge()
    rospy.init_node('spiri_api_camera',anonymous=True)
    rospy.Subscriber('/stereo/left/image_raw',Image,self.callback_left)
    rospy.Subscriber('/stereo/right/image_raw',Image,self.callback_right)
    rospy.Subscriber('/downward_cam/camera/image',Image,self.callback_bottom)
    time.sleep(1.0)

  def callback_left(self,data):
    self.left_image=data

  def callback_right(self,data):
    self.right_image=data

  def callback_bottom(self,data):
    self.bottom_image=data


  def get_left_image(self):
    try:
        self.cv_image_left=self.bridge.imgmsg_to_cv2(self.left_image,"bgr8")
        return self.cv_image_left
    except CvBridgeError,e:
        return e


  def get_right_image(self):
    try:
        self.cv_image_right=self.bridge.imgmsg_to_cv2(self.right_image,"bgr8")
        return self.cv_image_right
    except CvBridgeError,e:
        return e


  def get_bottom_image(self):
    try:
        self.cv_image_bottom=self.bridge.imgmsg_to_cv2(self.bottom_image,"bgr8")
        return self.cv_image_bottom
    except CvBridgeError,e:
        return e

  def save_left_image(self,path='./left.png'):
    cv2.imwrite(path,self.get_left_image())

  def save_right_image(self,path='./right.png'):
    cv2.imwrite(path,self.get_right_image())

  def save_bottom_image(self,path='./bottom.png'):
    cv2.imwrite(path,self.get_bottom_image())

  ## Functions to record video for a given time
  # @todo the sleep function is a hack. Need to find a better way
  def save_left_video(self,path='./left_video.avi',period='10'):
    out = cv2.VideoWriter(path,cv2.cv.CV_FOURCC('F','F','V','1'),20.0,(240,320))
    counter=0
    # this is not working
    while counter<int(period):
      print 'writing video',counter
      img_left=self.get_left_image()
      print img_left.shape
      cv2.imwrite('./left'+str(counter)+'.png',img_left)
      #print img_left
      out.write(img_left)

      counter=counter+1
      if cv2.waitKey(1) & 0xFF == ord('q'):
		    break
      time.sleep(1.0)
    out.release()
    print 'video is saved'
