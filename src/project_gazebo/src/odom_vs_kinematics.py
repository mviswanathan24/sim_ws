#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion as q2e
import math as m
import numpy as np

#initialize a node
rospy.init_node("odom_vs_kinematics_node")
#odom vectors in local frame
odom_x = []
odom_y = []
odom_theta = []

#model vectors  in local frame
model_x = []
model_y = []
model_theta = []

#first call flag
first_call = True
prev_x = 0
prev_y = 0
prev_theta = 0


def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def update(x,y,theta,v,w):
    dt = 0.01
    x = x + v*m.cos(theta)*dt
    y = y + v*m.sin(theta)*dt
    theta = theta + w*dt

    return([x,y,theta])

def odom_cb(msg):
    global odom_x, odom_y, model_x, model_y, first_call, prev_x, prev_y, prev_theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ori = msg.pose.pose.orientation
    theta = q2e([ori.x, ori.y, ori.z, ori.w])[2]

    if first_call:
        first_call = False
        prev_x = x
        prev_y = y
        prev_theta = theta
        model_x.append(x)
        model_y.append(y)
    else:
        #for model
        new_x, new_y, new_theta = update(prev_x, prev_y, prev_theta,0.5,-0.25)
        model_x.append(new_x)
        model_y.append(new_y)
        prev_x = new_x
        prev_y = new_y
        prev_theta = new_theta

    odom_x.append(x)
    odom_y.append(y)

rospy.Subscriber('/pioneer2dx/odom', Odometry, odom_cb)
rospy.spin()

#plot these values
plt.plot(odom_x, odom_y,'c-',label='odom')
plt.plot(model_x, model_y, 'g-', label="model")
plt.legend()
plt.axis('equal')
plt.show()
