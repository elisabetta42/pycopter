#!/usr/bin/env python
import sys
import rospy
import numpy
import math
import random
def separation(drone_list,current_drone):
	current_drone_xyz=current_drone.xyz
	alt_d = 4
	v=numpy.array([0.0,0.0,0.0])
	radius=500
	n_count=0
	for drone in drone_list:
		if drone.tag!=current_drone.tag:
			 other_drone_xyz=drone.xyz
			 x1=other_drone_xyz[0]
			 x2=current_drone_xyz[0]
			 y1=other_drone_xyz[1]
			 y2=current_drone_xyz[1]
			 distance=math.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))
			 print "distance"
			 print distance
			 print
			 if distance < radius:
				print "I am in the right distance"
			 	v[0] += x1 - x2
			 	v[1] += y1 - y2
			 	n_count=n_count+1
			
	#Divide by the number of neibourgh
	if n_count==0:	
		print "I have noone near me"	
		return current_drone_xyz			
	v[0]=v[0]/n_count
	v[1]=v[1]/n_count
	#calculate magnitude
	magnitude = math.sqrt(pow((v[0]), 2)+pow((v[1]), 2))
	v=v/magnitude
			
	#Opposite direction
	v[0]*=-1
	v[1]*=-1
	v[2]=current_drone_xyz[2]
	return v
	#current_drone.set_a_2D_alt_lya(v[0:2],-alt_d)	

def cohesion(drone_list,current_drone):
		        current_drone_xyz=current_drone.xyz
			alt_d = 4	
			comulative_distance=0
			v=numpy.array([0.0,0.0])
			position=current_drone_xyz
			#radius
			radius=1000	
			n_count=0			
    			for drone in drone_list:
				if drone.tag!=current_drone.tag:
					  other_drone_xyz=drone.xyz
			 		  x1=other_drone_xyz[0]
			 		  x2=current_drone_xyz[0]
			 		  y1=other_drone_xyz[1]
			                  y2=current_drone_xyz[1]
			                  distance=math.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))
					 
					 #Finding neibourghs
					  if distance < radius:
					 	v[0] += x1 
						v[1] += y1						
					        n_count=n_count+1
						comulative_distance=comulative_distance+distance
						
			if n_count==0:
				return position	
			#return center of mass	
			v[0]=v[0]/n_count
			v[1]=v[1]/n_count
			comulative_distance=comulative_distance/n_count
			
			#calculate directio for center of mass
			v[0]=v[0] - current_drone.xyz[0]
			v[1]=v[1] - current_drone.xyz[1]
			
			#Normalize vector using min-max normalization
			magnitude = math.sqrt(pow((v[0]), 2)+pow((v[1]), 2))
			v=v/magnitude
			return v

def flocking(drone_list,current_drone):
	alt_d=4
	vres=numpy.array([0.0,0.0])
	my_separation=separation(drone_list,current_drone)
	my_cohesion=cohesion(drone_list,current_drone)
	vres[0]=my_separation[0] + my_cohesion [0]
	vres[1]=my_separation[1] + my_cohesion [1]
	current_drone.set_v_2D_alt_lya(vres[0:2],-alt_d)	

	 	
		
			
			
