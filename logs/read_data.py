#!/usr/bin/env python

import numpy as np 
import matplotlib.pyplot as plt
import rospy
from pylab import genfromtxt  


# data
data = genfromtxt("frep_5.txt")
data1 = genfromtxt("fattr_5.txt")
data2 = genfromtxt("current_5.txt")

# Obstaculos
#obstacles = np.array ([[1.,1.],[2., 1.],[3., 3.]])
obstacles = genfromtxt("obst_5.txt")
#x_obst,y_obst = obstacles.T
# Goal
goal = np.array([4.0,4.0])
x_goal, y_goal = goal.T

# Size
ns = data.shape[0]
nsd = data1.shape[0]

# step 
step_rep = 80
step_attr = 80

#traj = plt.plot(data[:,0], data[:,1],'b--',linewidth = 4.0,label = "Trayectoria")
plt.figure()
#plt.title("Repulsive force applied to the mobile robot")
#Q1 = plt.quiver(data2[:,1],data2[:,2],data[:,0], data[:,1], units = 'width', pivot = 'mid'
#		, width = 0.001, scale=1 / 0.15)
Q1 = plt.quiver(data2[:,1],data2[:,2],data[0:ns:step_rep,0], data[0:ns:step_rep,1], units = 'width', pivot = 'mid'
		, width = 0.002, scale=1 / 0.15)
#qk = plt.quiverkey(Q1, 0.9, 0.9,1, r'$1 \frac{m}{s}$', 
#		labelpos='E',coordinates='figure')
plt.scatter (data2[:,1], data2[:,2], color = 'b', alpha = 1.0, label = "Trajectory")
#plt.scatter(x_obst,y_obst,s = 200, c="g", alpha = 1.0, marker ='o', label = "Obstacles")
plt.scatter(obstacles[:,0],obstacles[:,1],s = 10, c="g", alpha = 1.0, marker ='o', label = "Obstacles")
goal = plt.scatter(x_goal,y_goal,s = 200, c="r", alpha = 1.0, marker ='v', label = "Desired position")
plt.grid(color = 'k',  linestyle='-', linewidth=0.1)
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend(loc='upper left')
#plt.savefig('/home/jm95/Documents/TESIS/image_tesis/APF/repulsivo1.png')

#=========================================================================
plt.figure()
#plt.title("Attractive force applied to the mobile robot")                  
#Q2 = plt.quiver(data2[:,1],data2[:,2],data1[:,0], data1[:,1], units = 'width', pivot = 'mid'
#		, width = 0.001, scale=1 / 0.15)
Q2 = plt.quiver(data2[:,1],data2[:,2],data1[0:nsd:step_attr,0], data1[0:nsd:step_attr,1], units = 'width', pivot = 'mid'
		, width = 0.001, scale=1 / 0.15)
plt.scatter (data2[:,1], data2[:,2], color = 'b', alpha = 1.0, label = "Trajectory")
#plt.scatter(x_obst,y_obst,s = 200, c="g", alpha = 1.0, marker ='v', label = "Obstaculos")
goal = plt.scatter(x_goal,y_goal,s = 200, c="r", alpha = 1.0, marker ='v', label = "Desired position")
plt.grid(color = 'k',  linestyle='-', linewidth=0.1)
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend(loc='upper left')
#plt.savefig('/home/jm95/Documents/TESIS/image_tesis/APF/atractivo1.png')
#====================================================================================
plt.figure()
#plt.title("Navigation force applied to the mobile robot")
Q3 = plt.quiver(data2[:,1],data2[:,2],data1[0:nsd:step_attr,0]+data[0:ns:step_rep,0], data1[0:nsd:step_attr,1]+data[0:ns:step_rep,1], units = 'width', pivot = 'mid'
		, width = 0.001, scale=1 / 0.15)
plt.scatter (data2[:,1], data2[:,2], color = 'b', alpha = 1.0, label = "Trajectory")
#plt.scatter(x_obst,y_obst,s = 200, c="g", alpha = 1.0, marker ='o', label = "Obstacles")
plt.scatter(obstacles[:,0],obstacles[:,1],s = 10, c="g", alpha = 1.0, marker ='o', label = "Obstacles")
goal = plt.scatter(x_goal,y_goal,s = 200, c="r", alpha = 1.0, marker ='v', label = "Desired position")
plt.grid(color = 'k',  linestyle='-', linewidth=0.1)
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend(loc='upper left')
#plt.savefig('/home/jm95/Documents/TESIS/image_tesis/APF/navegacion1.png')
#qk = plt.quiverkey(Q, 0.9, 0.9, 2, r'$2 \frac{m}{s}$', labelpos='E',
#                   coordinates='figure')
plt.show()
