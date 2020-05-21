#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion as q2e
import math as m
import numpy as np

#initialize a node
rospy.init_node("plot_odom_node")
#states variable vectors in global frame
state_x = []
state_y = []
state_theta = []

#state variable vectors in local frame
transformed_x = []
transformed_y = []
transformed_theta = []

#state variables initially in global frame
init_set = False #initial state set flag
ini_x = 0
ini_y = 0
ini_theta = 0

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def transform_pose(x,y,theta):
    global ini_x, ini_y
    R = np.array([[m.cos(ini_theta), -m.sin(ini_theta)],
                  [m.sin(ini_theta), m.cos(ini_theta)]])
    t = np.array([[ini_x],
                  [ini_y]])
    #Homogenous representation of ini pose in global frame
    Htop = np.hstack((R,t))
    Hbot = np.array([0,0,1])
    print(Htop)
    print(Hbot)
    H = np.vstack((Htop, Hbot))
    print(H)

    #We need the inverse transform
    H_inv =  np.linalg.inv(H)

    # #this will be used to transform x,y
    homo_vector = np.array([[x],
                            [y],
                            [1]])
    trans_vect = H_inv.dot(homo_vector)
    #
    final_state = [trans_vect[0], trans_vect[1], wrapToPi(theta - ini_theta)]
    return final_state

def odom_cb(msg):
    global state_x, state_y, transformed_x, transformed_y, init_set

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ori = msg.pose.pose.orientation
    theta = q2e([ori.x, ori.y, ori.z, ori.w])[2]

    if not init_set:
        init_set = True
        ini_x = x;
        ini_y = y;
        ini_theta = theta
    else:
        tx, ty, ttht = transform_pose(x,y,theta)
        transformed_x.append(tx)
        transformed_y.append(ty)

    state_x.append(x)
    state_y.append(y)


rospy.Subscriber('/pioneer2dx/odom', Odometry, odom_cb)
rospy.spin()

#plot these values
plt.plot(state_x, state_y,'co',label='original')
plt.plot(transformed_x, transformed_y, 'go', label="transformed")
plt.legend()
plt.axis('equal')
plt.show()

"""
CONCLUSION OF THIS EXPERIMENT:
THE ODOM WAS IN THE LOCAL FRAME TO BEGIN with
NO transformation to the original frame needed

"""
