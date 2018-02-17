#!/usr/bin/env python
import math
import numpy
import numpy as np
def temperature_sensor(c_x,c_y,fire_x,fire_y):
	value=[]
	a=10000
	b=6
	distance=math.sqrt(pow((c_x - fire_x), 2) + pow((c_y - fire_y), 2))
	temperature=a/(distance+b)
	value.append(temperature)
	return temperature

	
def create_array(x_grid, y_grid,z_grid):
	#print x_grid
	#z=numpy.meshgrid(range(15))	
	print z_grid
	for i in range(0,15):
		for j in range(0,15):
	            value=temperature_sensor(i,j,0.0,0.0)
		    #print i,j
		    z_grid[i][j]=value
	return z_grid





def draw_grid(ax):
        x, y = numpy.meshgrid(range(-15,15), range(-15,15))
	point  = np.array([1, 2, 3])
        normal = np.array([1, 1, 2])
	d = -point.dot(normal)
	z = (-normal[0] * x - normal[1] * y - d) * 1. /normal[2]
	
	
	
	for i in range(-15,15):	
		for j in range(-15,15):
		 	z[i+15,j+15]=(temperature_sensor(i,j,0.0,0.0))
	ax.plot_surface(x, y, z/50, color="orange",alpha=0.3)
	
			
