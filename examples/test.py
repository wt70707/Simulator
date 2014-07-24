from spiri_api import get_state
import time
import threading 
import sys, select, os
spiri=get_state.Staterobot()


#param 10000000
def read_imu():
	
	while True:
		
		data=spiri.get_orientation_imu()
		if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
			line = raw_input()
			break
		print data
'''
while True:
	
	data=spiri.get_orientation_imu()
	if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
		line = raw_input()
		break
print data
'''
p1=threading.Thread(target=read_imu)
p1.start()


