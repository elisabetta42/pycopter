#!/usr/bin/env python
from scipy import linalg as la
import matplotlib.pyplot as pl
import numpy as np
import numpy
import formation_distance as form
import quadrotor as quad
import quadlog
import animation as ani
#import pycopter_rules as rules
import hector_rules as rules
import temperature_function as temperature
from sys import exit

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

q4_log = quadlog.quadlog(time)
q5_log = quadlog.quadlog(time)
q6_log = quadlog.quadlog(time)

Ed_log = np.zeros((time.size, fc.edges))

# Plots
quadcolor = ['r', 'g', 'b']
pl.close("all")
pl.ion()
fig = pl.figure(0)
axis3d = fig.add_subplot(111, projection='3d')
#ax= fig.add_subplot(111, projection='3d')
#ax.set_xlim(-15, 15)
#ax.set_ylim(-15, 15)
#ax.cla()
temperature.draw_grid(axis3d)

init_area = 5
s = 2
# Make data
#u = np.linspace(0, 2 * np.pi, 100)
#v = np.linspace(0, np.pi, 100)
#x = range(-15,15)
#y = range(-15,15)
#x_1,y_1,z = temperature.create_array(15,15,-15,-15)
# create x,y
#x, y = numpy.meshgrid(range(15), range(15))
#x=numpy.asarray(x)
#y=numpy.asarray(y)
#x, y, z = np.meshgrid(range(-15,15), range(-15,15),range(-15,15))
#z = temperature.create_array(x,y,z)
#z=numpy.asarray(z)
#pl.pause(0)
#exit(0)

# Desired altitude and heading
alt_d = 4
q1.yaw_d =  -np.pi
q2.yaw_d =  -np.pi
q3.yaw_d =  -np.pi

q4.yaw_d =  -np.pi
q5.yaw_d =  -np.pi
q6.yaw_d =  -np.pi
radius=1000
for t in time:
    # Simulation
    #X = np.append(q1.xyz[0:2], np.append(q2.xyz[0:2], q3.xyz[0:2]))
    #V = np.append(q1.v_ned[0:2], np.append(q2.v_ned[0:2], q3.v_ned[0:2]))
    #U = fc.u_acc(X, V)

    #q1.set_a_2D_alt_lya(U[0:2], -alt_d)
    #q2.set_a_2D_alt_lya(U[2:4], -alt_d)
    #q3.set_a_2D_alt_lya(U[4:6], -alt_d)
    # Simulation
    rules.flocking(quad_list,q1,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q1.step(dt)
    q1.same_position(quad_list)

    rules.flocking(quad_list,q2,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q2.step(dt)
    q2.same_position(quad_list)

    rules.flocking(quad_list,q3,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q3.step(dt)
    q3.same_position(quad_list)


    rules.flocking(quad_list,q4,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q4.step(dt)
    q4.same_position(quad_list)


    rules.flocking(quad_list,q5,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q5.step(dt)
    q5.same_position(quad_list)


    rules.flocking(quad_list,q6,radius,numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2),numpy.random.uniform(0.9, 1.2))
    q6.step(dt)
    q6.same_position(quad_list)
   
   
    # Animation
    if it%frames == 0:

        axis3d.cla()
	

        ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q3.xyz, q3.Rot_bn(), quadcolor[0])
	
	ani.draw3d(axis3d, q4.xyz, q4.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q5.xyz, q5.Rot_bn(), quadcolor[0])
	ani.draw3d(axis3d, q6.xyz, q6.Rot_bn(), quadcolor[0])

        axis3d.set_xlim(-15, 15)
        axis3d.set_ylim(-15, 15)
        axis3d.set_zlim(0, 15)
        axis3d.set_xlabel('South [m]')
        axis3d.set_ylabel('East [m]')
        axis3d.set_zlabel('Up [m]')
        axis3d.set_title("Time %.3f s" %t)
	
	temperature.draw_grid(axis3d)
        pl.pause(0.001)
        pl.draw()
	
	
         # Log
    	q1_log.xyz_h[it, :] = q1.xyz
    	q1_log.att_h[it, :] = q1.att
    	q1_log.w_h[it, :] = q1.w
    	q1_log.v_ned_h[it, :] = q1.v_ned
    	q1_log.xi_g_h[it] = q1.xi_g
    	q1_log.xi_CD_h[it] = q1.xi_CD

	 # Log
    	q2_log.xyz_h[it, :] = q2.xyz
    	q2_log.att_h[it, :] = q2.att
    	q2_log.w_h[it, :] = q2.w
    	q2_log.v_ned_h[it, :] = q2.v_ned
    	q2_log.xi_g_h[it] = q2.xi_g
    	q2_log.xi_CD_h[it] = q2.xi_CD


 	# Log
    	q3_log.xyz_h[it, :] = q3.xyz
    	q3_log.att_h[it, :] = q3.att
    	q3_log.w_h[it, :] = q3.w
    	q3_log.v_ned_h[it, :] = q3.v_ned
    	q3_log.xi_g_h[it] = q3.xi_g
    	q3_log.xi_CD_h[it] = q3.xi_CD


	  # Log
    	q4_log.xyz_h[it, :] = q4.xyz
    	q4_log.att_h[it, :] = q4.att
    	q4_log.w_h[it, :] = q4.w
    	q4_log.v_ned_h[it, :] = q4.v_ned
    	q4_log.xi_g_h[it] = q4.xi_g
    	q4_log.xi_CD_h[it] = q4.xi_CD

	 # Log
    	q5_log.xyz_h[it, :] = q5.xyz
    	q5_log.att_h[it, :] = q5.att
    	q5_log.w_h[it, :] = q5.w
    	q5_log.v_ned_h[it, :] = q5.v_ned
    	q5_log.xi_g_h[it] = q5.xi_g
    	q5_log.xi_CD_h[it] = q5.xi_CD


 	# Log
    	q6_log.xyz_h[it, :] = q6.xyz
    	q6_log.att_h[it, :] = q6.att
    	q6_log.w_h[it, :] = q6.w
    	q6_log.v_ned_h[it, :] = q6.v_ned
    	q6_log.xi_g_h[it] = q6.xi_g
    	q6_log.xi_CD_h[it] = q6.xi_CD


    it+=1
 
# Stop if crash
    if (q1.crashed == 1):
        break


pl.figure(1)
pl.plot(time, -q1_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q1_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q1_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(2)
pl.plot(time, -q2_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q2_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q2_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(3)
pl.plot(time, -q3_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q3_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q3_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(4)
pl.plot(time, -q4_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q4_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q4_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(5)
pl.plot(time, -q5_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q5_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q5_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(6)
pl.plot(time, -q6_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q6_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q6_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()




pl.pause(0)






        
