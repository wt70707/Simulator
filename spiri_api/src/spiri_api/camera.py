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
import numpy as np
import threading
from spiri_api import get_state
from multiprocessing import Process
## Class defining all the functions for perception
class camera_interface():

  ## Constructor
  def __init__(self):
    #threading.Thread.__init__(self)
    #self.t1=threading.Thread(target=self.callback_threading)
    #self.t1.setDaemon(False)
    #self.t1.start()
    self.left_image=Image()
    self.right_image=Image()
    self.bottom_image=Image()
    self.bridge=CvBridge()
    self.number=0
    self.cv_image_left=np.zeros((10,10))
    self.cv_image_right=np.zeros((10,10))
    self.cv_image_bottom=np.zeros((10,10))
    rospy.Subscriber('/stereo/left/image_raw',Image,self.callback_left)
    rospy.Subscriber('/stereo/right/image_raw',Image,self.callback_right)
    rospy.Subscriber('/downward_cam/camera/image',Image,self.callback_bottom)
    #self.latest=False
    #rospy.init_node('spiri_api_camera',anonymous=True)
    #self.start()
    #state_robot=get_state.Staterobot()
  ## start the threads
  # @todo Can create threads for every subscriber
  def callback_threading(self):

    rospy.Subscriber('/stereo/left/image_raw',Image,self.callback_left)
    rospy.Subscriber('/stereo/right/image_raw',Image,self.callback_right)
    rospy.Subscriber('/downward_cam/camera/image',Image,self.callback_bottom)

  #time.sleep(1.0)

  def callback_left(self,data):
    #self.latest=False
    try:

        self.cv_image_left=self.bridge.imgmsg_to_cv2(data,"bgr8")

        self.number=self.number+1
        print 'in the callback',self.number
        #self.latest=True
        #self.e.set()
    except CvBridgeError,e:
        print e
  def callback_right(self,data):
    try:
        self.cv_image_right=self.bridge.imgmsg_to_cv2(data,"bgr8")

    except CvBridgeError,e:
        print e

  def callback_bottom(self,data):
    try:
        self.cv_image_bottom=self.bridge.imgmsg_to_cv2(data,"bgr8")

    except CvBridgeError,e:
        print e


  def get_left_image(self):
    time.sleep(0.0)
    if self.cv_image_left.shape==(10,10):
      return 0,None
    else:
      return 1,self.cv_image_left


  def get_right_image(self):
    #time.sleep(1.0)
    if self.cv_image_right.shape==(10,10):
      return 0,None
    else:
      return 1,self.cv_image_right


  def get_bottom_image(self):
    if self.cv_image_bottom.shape==(10,10):
      return 0,None
    else:
      return 1,self.cv_image_bottom

  def save_left_image_threading(self,path):
    t2=Process(target=self.save_left_image,args=(path,))
    t2.start()
    t2.join()
  def save_left_image(self,path='./left.png'):
    #self.t1=threading.Thread(target=self.save_left_image,args=(path,))
    #self.t1.start()
    #time.sleep(1.0)
    #self.t1.start()
    #self.t1.join()

    #print 'got the image',self.number
    #print 'inside the loop'

    ret,img=self.get_left_image()
    while ret==0:
        ret,img=self.get_left_image()

    cv2.imwrite(path,img)
    print 'image saved'
    #return True


  def save_right_image(self,path='./right.png'):
    time.sleep(1.0)
    cv2.imwrite(path,self.get_right_image())

  def save_bottom_image(self,path='./bottom.png'):
    time.sleep(1.0)
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
