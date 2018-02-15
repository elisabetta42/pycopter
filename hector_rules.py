#!/usr/bin/env python
from pvector import PVector
import numpy
import math
def flocking(agents, current,radius, kva, ks, kc, ke):

    neighbor_count = 0;
    velAvg = PVector(0,0)
    centroid = PVector(0,0)
    separation = PVector(0,0)
    cohesion = PVector(0,0)
    desired_velocity = PVector(0,0)
    position=PVector(current.xyz[0],current.xyz[1])
    theta=0
    alt_d=8
    limitX=15
    limitY=15
    #We check all the agents on the screen.
    #Any agent closer than radius units is a neighbor.
    for it in agents:
        neighbor = PVector(it.xyz[0],it.xyz[1])
	relative_position=PVector(0,0)
	neighbor.subVector(position)
        relative_position.addVector(neighbor)
	#relative_position=PVector(relativePosition[0],relativePosition[1])

        if relative_position.normalize() < radius:
            #We have found a neighbor
            neighbor_count=neighbor_count+1

            #We add all the positions
            #centroid += it->getPosition();
	    it_position=PVector(it.xyz[0],it.xyz[1])
	    centroid.addVector(it_position)
	    it_velocity=PVector(it.v_ned[0],it.v_ned[1])
            #We add all the velocities
            velAvg.addVector(it_velocity)

            #Vector pointing at the opposite direction w.r.t. your 
            #neighbor
            #separation -= relativePosition;
	    separation.subVector(relative_position)
        

    centroid.divScalar(neighbor_count) # All the positions over the num of neighbors
    velAvg.divScalar(neighbor_count) # All the velocities over the numb of neighbors

    #Relative position of the agent w.r.t. centroid
    centroid.subVector(position)
    cohesion.addVector(centroid)

    #In order to compare the following vectors we normalize all of them,
    # so they have the same magnitude. Later on with the gains 
    #kva, ks and kc we assing which vectors are more important.
    velAvg.normalize();
    cohesion.normalize();
    separation.normalize();

    if neighbor_count == 1:
        desired_velocity = velocity
    else:
	vel_avg=velAvg.return_as_vector()
	v_separation=separation.return_as_vector()
	v_cohesion=cohesion.return_as_vector()
	v_target=tend_to_place(agents,current)
	desired_velocity=kva*vel_avg + ks*v_separation + kc*v_cohesion + kc*v_target
	desiredVel=PVector(desired_velocity[0],desired_velocity[1])
        #desired_velocity +=kva*velAvg + ks*separation + kc*cohesion;

    error_theta = math.atan2(desired_velocity[1], desired_velocity[0])-theta
    error_theta=ke*error_theta
    #updateUnicycle(0, ke*error_theta);
    current.set_v_2D_alt_lya(error_theta,-alt_d)
	
    
    #if position.x < 0 or position.x > limitX or position.y < 0 or position.y or limitY:
    #	current.set_v_2D_alt_lya(-error_theta,-alt_d)

def tend_to_place(agents, current):
	target=PVector(5,5)
	position=PVector(current.xyz[0],current.xyz[1])
	target.subVector(position)
	target.divScalar(100)
	target.normalize()
	return 2*target.return_as_vector()	



