#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState
import sys
import rospy
import numpy
import math
import random
from std_msgs.msg import Float64

def separation(uav_name,blockListDict):
			model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    			input_drone_coordinates = model_coordinates(uav_name, "")	

			c_x=input_drone_coordinates.pose.position.x
 			c_y=input_drone_coordinates.pose.position.y
			
			v=numpy.array([0.0,0.0])
		        position=numpy.array([c_x,c_y])
			
			radius=2	
			n_count=0
			
    			for block in blockListDict.itervalues():
    				blockName = str(block._name)
    				if blockName!=uav_name:
					 resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
					 x1=resp_coordinates.pose.position.x
					 x2=input_drone_coordinates.pose.position.x
					 y1=resp_coordinates.pose.position.y
					 y2=input_drone_coordinates.pose.position.y
					 distance=math.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))
					 #Finding neibourghs
					 if distance < radius:
					 	v[0] += x1 - x2
						v[1] += y1 - y2
					        n_count=n_count+1
			
			#Divide by the number of neibourgh
			if n_count==0:	
				print "There is anyone around me"	
				return position			
			v[0]=v[0]/n_count
			v[1]=v[1]/n_count
			#calculate magnitude
			magnitude = math.sqrt(pow((v[0]), 2)+pow((v[1]), 2))
			print "magnitude"
			print magnitude
			v=v/magnitude
			
			#Opposite direction
			v[0]*=-2.5
			v[1]*=-2.5

			return v		 	
					 
def cohesion(uav_name,blockListDict):
		        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    			input_drone_coordinates = model_coordinates(uav_name, "")	
			c_x=input_drone_coordinates.pose.position.x
 			c_y=input_drone_coordinates.pose.position.y
			comulative_distance=0
			v=numpy.array([0.0,0.0])
			position=numpy.array([c_x,c_y])

			#radius
			radius=50	
			n_count=0			
    			for block in blockListDict.itervalues():
    				blockName = str(block._name)				
    				if blockName!=uav_name:
					 resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
					 x1=resp_coordinates.pose.position.x
					 x2=input_drone_coordinates.pose.position.x
					 y1=resp_coordinates.pose.position.y
					 y2=input_drone_coordinates.pose.position.y
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
			v[0]=v[0] - input_drone_coordinates.pose.position.x
			v[1]=v[1] - input_drone_coordinates.pose.position.y
			
			#Normalize vector using min-max normalization
			magnitude = math.sqrt(pow((v[0]), 2)+pow((v[1]), 2))
			v=v/magnitude
			
			#if comulative_distance < 3:
			#	return 0.5*v
		        #if comulative_distance > 5:
			#	return 2*v
			#if comulative_distance > 10:
			#	return 3*v
			#if comulative_distance > 15:
			#	return 4*v
			
			return v

def temperature_sensor(uav_name):
	value=[]
	a=600
	b=4
	model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    	input_drone_coordinates = model_coordinates(uav_name, "")	
	c_x=input_drone_coordinates.pose.position.x
 	c_y=input_drone_coordinates.pose.position.y
	fire_x=0.0
	fire_y=0.0
	distance=math.sqrt(pow((c_x - fire_x), 2) + pow((c_y - fire_y), 2))
	print "distance:"
	print distance
	temperature=a/(distance+b)
	print "temperature"
	print temperature
	value.append(temperature)
	value.append(distance)
	return value

 def flocking(self):
	separation=rules.separation("iris_1",self.blockListDict)
	cohesion=rules.cohesion("iris_1",self.blockListDict)
	self.vres[0]=separation[0] + (0.4*cohesion [0])+self.x_off
	self.vres[1]=separation[1] + (0.4*cohesion [1])+self.y_off
	return self.vres




	
	
