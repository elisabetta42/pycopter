#!/usr/bin/env python
from scipy import linalg as la
import matplotlib.pyplot as pl
import numpy as np
import formation_distance as form
import quadrotor as quad
import quadlog
import animation as ani
#import pycopter_rules as rules
import boids as rules

# Quadrotor
m = 0.65 # Kg
l = 0.23 # m
Jxx = 7.5e-3 # Kg/m^2
Jyy = Jxx
Jzz = 1.3e-2
Jxy = 0
Jxz = 0
Jyz = 0
J = np.array([[Jxx, Jxy, Jxz], \
              [Jxy, Jyy, Jyz], \
              [Jxz, Jyz, Jzz]])
CDl = 9e-3
CDr = 9e-4
kt = 3.13e-5  # Ns^2
km = 7.5e-7   # Ns^2
kw = 1/0.18   # rad/s

# Initial conditions
att_0 = np.array([0.0, 0.0, 0.0])
pqr_0 = np.array([0.0, 0.0, 0.0])
xyz1_0 = np.array([1.0, 1.2, 0.0])
xyz2_0 = np.array([1.2, 2.0, 0.0])
xyz3_0 = np.array([-1.1, 2.6, 0.0])

xyz4_0 = np.array([3.0, 3.2, 0.0])
xyz5_0 = np.array([4.2, 4.0, 0.0])
xyz6_0 = np.array([-3.2, 4.6, 0.0])
v_ned_0 = np.array([0.0, 0.0, 0.0])
w_0 = np.array([0.0, 0.0, 0.0, 0.0])

# Setting quads
q1 = quad.quadrotor(1, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz1_0, v_ned_0, w_0)

q2 = quad.quadrotor(2, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz2_0, v_ned_0, w_0)

q3 = quad.quadrotor(3, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz3_0, v_ned_0, w_0)

q4 = quad.quadrotor(4, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz4_0, v_ned_0, w_0)

q5 = quad.quadrotor(5, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz5_0, v_ned_0, w_0)

q6 = quad.quadrotor(6, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz6_0, v_ned_0, w_0)
quad_list=[]
quad_list.append(q1)
quad_list.append(q2)
quad_list.append(q3)

quad_list.append(q4)
quad_list.append(q5)
quad_list.append(q6)

# Formation Control
# Shape
side = 8
Btriang = np.array([[1, 0, -1],[-1, 1, 0],[0, -1, 1]])
dtriang = np.array([side, side, side])

# Motion
mu = 0e-2*np.array([1, 1, 1])
tilde_mu = 0e-2*np.array([1, 1, 1])

fc = form.formation_distance(2, 1, dtriang, mu, tilde_mu, Btriang, 5e-2, 5e-1)

# Simulation parameters
tf = 2500
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 100

# Data log
q1_log = quadlog.quadlog(time)
q2_log = quadlog.quadlog(time)
q3_log = quadlog.quadlog(time)
Ed_log = np.zeros((time.size, fc.edges))

# Plots
quadcolor = ['r', 'g', 'b']
pl.close("all")
pl.ion()
fig = pl.figure(0)
axis3d = fig.add_subplot(111, projection='3d')

init_area = 5
s = 2

# Desired altitude and heading
alt_d = 4
q1.yaw_d =  -np.pi
q2.yaw_d =  -np.pi
q3.yaw_d =  -np.pi

q4.yaw_d =  -np.pi
q5.yaw_d =  -np.pi
q6.yaw_d =  -np.pi

for t in time:
    # Simulation
    #X = np.append(q1.xyz[0:2], np.append(q2.xyz[0:2], q3.xyz[0:2]))
    #V = np.append(q1.v_ned[0:2], np.append(q2.v_ned[0:2], q3.v_ned[0:2]))
    #U = fc.u_acc(X, V)

    #q1.set_a_2D_alt_lya(U[0:2], -alt_d)
    #q2.set_a_2D_alt_lya(U[2:4], -alt_d)
    #q3.set_a_2D_alt_lya(U[4:6], -alt_d)
    # Simulation
    rules.flocking(quad_list,q1)
    q1.step(dt)

    rules.flocking(quad_list,q2)
    q2.step(dt)

    rules.flocking(quad_list,q3)
    q3.step(dt)

    rules.flocking(quad_list,q4)
    q4.step(dt)

    rules.flocking(quad_list,q5)
    q5.step(dt)

    rules.flocking(quad_list,q6)
    q6.step(dt)
   
    # Animation
    if it%frames == 0:

        axis3d.cla()
        ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q3.xyz, q3.Rot_bn(), quadcolor[0])
	
	ani.draw3d(axis3d, q4.xyz, q1.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q5.xyz, q2.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q6.xyz, q3.Rot_bn(), quadcolor[0])

        axis3d.set_xlim(-15, 15)
        axis3d.set_ylim(-15, 15)
        axis3d.set_zlim(0, 15)
        axis3d.set_xlabel('South [m]')
        axis3d.set_ylabel('East [m]')
        axis3d.set_zlabel('Up [m]')
        axis3d.set_title("Time %.3f s" %t)
        pl.pause(0.5)
        pl.draw()
	
	
        #pl.figure(1)
        #pl.clf()
        #ani.draw2d(1, X, fc, quadcolor)
        #ani.draw_edges(1, X, fc, -1)
        #pl.xlabel('South [m]')
        #pl.ylabel('West [m]')
        #pl.title('2D Map')
        #pl.xlim(-s*init_area, s*init_area)
        #pl.ylim(-s*init_area, s*init_area)
        #pl.grid()
        #pl.pause(0.001)
        #pl.draw()
    it+=1
 
# Stop if crash
    if (q1.crashed == 1):
        break
        
