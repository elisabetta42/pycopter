import random
import math
def get_neighbour(current,drone_list,radius):
	position=current.xyz
	neighbours=[]
	for drone in drone_list:
		if drone.tag!=current.tag:
			neighbour=drone.xyz
			d=math.sqrt(pow((position[0] - neighbour[0]), 2) + pow((position[1] -neighbour[1]), 2))
			if d<radius:
				neighbours.append(drone)
        return neighbours
def get_drone_position(drone,percentage):
	 if rand()<=percentage:
		return drone.xyz
	 else: 
		return None

	

		
	
		
		
	
