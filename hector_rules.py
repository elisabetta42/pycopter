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
    obstacle = PVector(0,0)
    desired_velocity = PVector(0,0)
    position=PVector(current.xyz[0],current.xyz[1])
    theta=0
    alt_d=10
    limitX=15
    limitY=15
    #We check all the agents on the screen.
    #Any agent closer than radius units is a neighbor.
    for it in agents:
        neighbor = PVector(it.xyz[0],it.xyz[1])
	relative_position=PVector(0,0)
	neighbor.subVector(position)
        #relative_position.addVector(neighbor)
	#relative_position=PVector(relativePosition[0],relativePosition[1])
	d=math.sqrt(pow((position.x - neighbor.x), 2) + pow((position.y - neighbor.y), 2))
        if d/10 < radius:
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
        if neighbor_count==0:
	   velocity=PVector(current.v_ned_d[0],current.v_ned_d[1])
	   return velocity.return_as_vector()

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
       #desired_velocity = velocity
	desiredVel=PVector(current.v_ned_d[0],current.v_ned_d[1])
	desired_velocity=desiredVel.return_as_vector()
	
    else:
	vel_avg=velAvg.return_as_vector()
	v_separation=separation.return_as_vector()
	v_cohesion=cohesion.return_as_vector()
	v_target=tend_to_place(agents,current)
	random_walk=randomWalkb(position.x,position.y)
	v_bound_position=bound_position(17,-17,17,-17,position.x,position.y,3)
	#avoid_vector=getAvoidAvoids(position,1)
	avoid_vector=apply_force(position,obstacle,current)
	desired_velocity=kva*vel_avg + ks*v_separation + kc*v_cohesion
	#+5*ke*v_target
	#kva*vel_avg + ks*v_separation + 5*kc*v_cohesion + 3*ke*v_bound_position+ke*v_target
        #kva*vel_avg + ks*v_separation + kc*v_cohesion +ke*v_bound_position+ks*avoid_vector
	desiredVel=PVector(desired_velocity[0],desired_velocity[1])
        #desired_velocity +=kva*velAvg + ks*separation + kc*cohesion;
	
    	if(desiredVel.magnitude()>2):
		desiredVel.normalize()
		desiredVel.mulScalar(2)
    desired_vel=desiredVel.return_as_vector()
    current.set_v_2D_alt_lya(desired_vel,-alt_d)
	
    
    #if position.x < 0 or position.x > limitX or position.y < 0 or position.y or limitY:
    #	current.set_v_2D_alt_lya(-error_theta,-alt_d)

def tend_to_place(agents, current):
	target=PVector(0,0)
	position=PVector(current.xyz[0],current.xyz[1])
	target.subVector(position)
	target.divScalar(50)
	target.normalize()
	return target.return_as_vector()	

def randomWalkb(x,y):   
        new = numpy.random.randint(1,4)
        if new == 1:
            x += 1
        elif new == 2:
            y += 1
        elif new ==3 :
            x += -1
        else :
            y += -1
    	new_position=PVector(x,y)
    	new_position.normalize()  
    	return new_position.return_as_vector()

def bound_position(Xmin,Xmax,Ymin,Ymax,x,y,constant):
	v=PVector(0,0)

	if x <= Xmin:
		v.x = constant
	elif x >= Xmax:
		v.x = -constant
		
	if y <= Ymin :
		v.y = constant
	elif y >= Ymax :
		v.y = -constant
		
	return v.return_as_vector()

def getAvoidAvoids(position,avoid_radius):
    relative_position=PVector(0,0)
    diff=PVector(0,0)
    steer = PVector(0, 0)
    obstacle_position=PVector(0.0,0.0)
    count = 0
    obstacle_position.subVector(position)
    relative_position.addVector(obstacle_position)
    d=math.sqrt(pow((position.x - obstacle_position.x), 2) + pow((position.y - obstacle_position.y), 2))
   #for obstacle : obstacles :     
      # If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if relative_position.normalize() < avoid_radius:
        #Calculate vector pointing away from neighbor
	position.subVector(obstacle_position)
        diff.addVector(position)
        diff.normalize();
        diff.divScalar(d);        #Weight by distance
        steer.addVector(diff);
        #count++;            // Keep track of how many    
    return steer.return_as_vector()

def apply_force(other,position,current_drone):
        """Apply a simple short range repulsive force from another particle on
        this particle."""
        return apply_force_from_coords(other.x, other.y,position,current_drone)

def apply_force_from_coords(ox, oy,position,current_drone):
        """Apply a simple short range repulsive force from a particle at
        the given coordinates on this particle."""
	ax=0
	ay=0
	radius=4
	empty=PVector(0,0)      
	dx = ox - position.x
        dy = oy - position.y
        if dx == dy == 0:
           return empty.return_as_vector()# no directional force from particle at same location
        r2 = max(dx * dx + dy * dy, current_drone.min_r2)
        if r2 > radius:
            return empty.return_as_vector()# out of force range
        r = math.sqrt(r2)

        # Very simple short range repulsive force
        coef = (1 - current_drone.cutoff / r) / r2 / current_drone.mass
        ax += coef * dx
        ay += coef * dy
	direction=PVector(ax,ay)
	direction.normalize()
	
	
	return direction.return_as_vector()





