#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

rospy.init_node("periodic_subscriber_node")

data = rospy.wait_for_message('/pioneer2dx/imu', Imu)
print(data)
