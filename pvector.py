#!/usr/bin/env python
import math
import numpy
class PVector:
 x=0
 y=0
 def __init__(self,x,y):
	self.x=x
	self.y=y
 def set(self,x,y):
 	self.x=x
	self.y=y
 
 def addVector(self,v):
    	self.x += v.x
    	self.y += v.y
 
 def subVector(self,v):
    	self.x -= v.x
    	self.y -= v.y
 
 def subTwoVector(self,v, v2):
    	tmp=PVector(0.0,0.0)
    	v.x -= v2.x
    	v.y -= v2.y
    	tmp.set(v.x, v.y)
        return tmp

 def subScalar(self,s):
       self.x -= s
       self.y -= s

 def mulVector(self,v):
       self.x *= v.x
       self.y *= v.y

 def mulScalar(self,s):
      self.x *= s
      self.y *= s

 def divVector(self,v):
      self.x /= v.x
      self.y /= v.y

 def divScalar(self,s):
      self.x /= s
      self.y /= s


 def limit(self,max):
      size = magnitude()

      if size > max: 
      	set(self.x / size, self.y / size)
   
 def distance(self,v):
    dx = self.x - v.x
    dy = self.y - v.y
    dist = math.sqrt(dx*dx + dy*dy)
    return dist

 def dotProduct(self,v):
    dot = self.x * v.x + self.y * v.y
    return dot

 def magnitude(self):
    return math.sqrt(self.x*self.x + self.y*self.y);

 def setMagnitude(self,x):
    normalize()
    mulScalar(x)

 def angleBetween(self,v):
    if self.x == 0 and self.y == 0 :
	return 0.0
    if v.x == 0 and v.y == 0:
	return 0.0

    dot = self.x * v.x + self.y * v.y;
    v1mag = math.sqrt(self.x * self.x + self.y * self.y)
    v2mag = math.sqrt(v.x * v.x + v.y * v.y)
    amt = dot / (v1mag * v2mag)
   
    if amt <= -1: 
        return PI
    elif amt >= 1: 
        return 0
    
    tmp = acos(amt)
    return tmp


 def normalize(self):
    m = self.magnitude()

    if m > 0: 
	arg_1=(self.x / m)
	arg_2=(self.y / m)
        self.set(arg_1, arg_2)
    else:
        self.set(self.x, self.y)

 def return_as_vector(self):
	vector=numpy.array([0.0,0.0])	
	vector[0]=self.x
	vector[1]=self.y
	return vector






		
		
 
