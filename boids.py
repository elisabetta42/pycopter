#!/usr/bin/env python
from pvector import PVector
import numpy
def flocking(boids,current_drone):
    alt_d=4
    vres=numpy.array([0.0,0.0])
    my_separation = separation(boids,current_drone)
    #ali = align(boids,current)     
    my_cohesion = cohesion(boids,current_drone)
    vres[0]=my_separation[0] + my_cohesion [0]
    vres[1]=my_separation[1] + my_cohesion [1]
    current_drone.set_v_2D_alt_lya(vres[0:2],-alt_d)
    

def separation(boids,current_drone):
	alt_d=4
	desiredseparation=50
	steer = PVector(0.0, 0.0)
	count = 0
	position=current_drone.xyz
	c_vector=PVector(position[0],position[1])
	for other in boids:
		if other.tag!=current_drone.tag:
			other_position=other.xyz
			o_vector=PVector(other_position[0],other_position[1])
			distance=c_vector.distance(o_vector)
			if distance < desiredseparation:
				c_vector.subVector(o_vector)
				c_vector.normalize()
				c_vector.divScalar(distance)
				steer.addVector(c_vector)
				count=count+1
        if count > 0:
      		steer.divScalar(count)

	vres=steer.return_as_vector()
	#current_drone.set_a_2D_alt_lya(vres[0:2],-alt_d)
	return vres
def cohesion(boids,current_drone):
	neighbordist = 80
	sum_vector = PVector(0, 0)
	count = 0
	position=current_drone.xyz
	c_vector=PVector(position[0],position[1])
	for other in boids:
		if other.tag!=current_drone.tag:
			other_position=other.xyz
			o_vector=PVector(other_position[0],other_position[1])
			distance=c_vector.distance(o_vector)
			if distance < neighbordist:
				sum_vector.addVector(o_vector)
				count=count+1
	if count > 0:
      		sum_vector.divScalar(count)
      		return seek(sum_vector,current_drone.xyz).return_as_vector()
        else:
      		return PVector(0, 0).return_as_vector()

def seek(target,position):
    desired = target
    c_position=PVector(position[0],position[1])
    desired.subVector(c_position);  #A vector pointing from the position to the target
    #Scale to maximum speed
    desired.normalize();
    #desired.mult(maxspeed);
    return desired
  
    
			
	


	
		
    
		


