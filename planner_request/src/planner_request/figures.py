#!/usr/bin/env python


#return waypoints as a list

		
def square_compute(start,diag_start):
	waypoints=[]
	waypoints=[(start[0],diag_start[1]),(diag_start[0],diag_start[1]),(diag_start[0],start[1]),(start[0],start[1])]
	return waypoints	

