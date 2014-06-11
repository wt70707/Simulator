#!/usr/bin/env python
import numpy as np

#return waypoints as a list

		
def square_compute(start,diag_start):
	waypoints=[]
	waypoints=[(start[0],diag_start[1],start[2]),(diag_start[0],diag_start[1],start[2]),(diag_start[0],start[1],start[2]),(start[0],start[1],start[2])]
	return waypoints	

def square_compute_vertical(start,diag_start,axis='x'):
	waypoints=[]
	if axis=='x':

		waypoints=[(start[0],start[1],diag_start[2]),(diag_start[0],start[1],diag_start[2]),(diag_start[0],start[1],start[2]),(start[0],start[1],start[2])]
	if axis=='y':
		waypoints=[(start[0],start[1],diag_start[2]),(start[0],diag_start[1],diag_start[2]),(start[0],diag_start[1],start[2]),(start[0],start[1],start[2])]
	return waypoints
def triangle_compute(start,height,base):
	waypoints=[]
	waypoints=[(start[0]+base/2,start[1]+height,start[2]),(start[0]+base,start[1],start[2]),(start[0],start[1],start[2])]
	return waypoints

def triangle_compute_vertical(start,height,base,axis='x'):
	waypoints=[]
	if axis=='x':
	
		waypoints=[(start[0]+base/2,start[1],start[2]+height),(start[0]+base,start[1],start[2]),(start[0],start[1],start[2])]
	if axis=='y':
		waypoints=[(start[0],start[1]+base/2,start[2]+height),(start[0],start[1]+base,start[2]),(start[0],start[1],start[2])]		
	return waypoints

def horizontal_sine_wave(start,base=np.pi):
	waypoints=[]
	x=np.linspace(start[0],start[0]+base,10)
	y=np.sin(x)
	for i in range(len(x)):
		waypoints.append((x[i],y[i]))
	return waypoints
	
	
