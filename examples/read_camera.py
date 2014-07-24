from spiri_api import camera
from spiri_api import get_state
import cv2
import threading
import time
from multiprocessing import Process

import sys, select, os
spiri=get_state.Staterobot()
spiri_camera=camera.camera_interface()
	
				
#print 'testing'
#while 1:
#	cv2.imshow('test',spiri_camera.get_left_image())
#	cv2.waitKey(1)
#print spiri_camera.get_right_image()
#print spiri_camera.get_left_image()
#print spiri_camera.get_bottom_image()
	


#print 'saved first image'




#time.sleep(5.0)
#for i in range(10):
#for i in range(10):
#	print 'in loop'

#spiri.send_goal_relative(0,0,1)

#p=Process(target=spiri_camera.save_left_image,args=('/home/rohan/Documents/start.png',))#+str(i)+'.png')
#p.start()
#p.join()
print 'saving the first image'
spiri_camera.save_left_image('/home/rohan/Documents/start.png')
atom
#p1=Process(target=spiri.send_goal_relative,args=(0,0,-1,))
#p1.start()
#p1.join()
#spiri.send_goal_relative(0,0,-1)
while True:
	
	data=spiri.get_orientation_imu()
	if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
		line = raw_input()
		break
print data
print 'saving the last image'
spiri_camera.save_left_image('/home/rohan/Documents/finish.png')
#p2=Process(target=spiri_camera.save_left_image,args=('/home/rohan/Documents/finish.png',))
#p2.start()
#p2.join()
#print 'saved last image'
#t1=threading.Thread(target=test)
#t1.start()
#p=Process(target=test)
#p1=Process(target=goal)
#p.start()
#p.join()
#p1.start()


#for i in range(10):
#	print True
	#time.sleep(1.0)
