#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("move_in_circle_node")
# create a publisher
cmd_pub = rospy.Publisher("/pioneer2dx/cmd_vel",Twist,queue_size=10)
rate = rospy.Rate(10)
cmd = Twist()

def publish_vel():
    cmd.linear.x = 0.5 #0.0
    cmd.angular.z = -0.25

    while not rospy.is_shutdown():
        cmd_pub.publish(cmd)
        rate.sleep()

def stop():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    cmd_pub.publish(cmd)

if __name__ == "__main__":
    try:
        rospy.on_shutdown(stop)
        publish_vel()
    except rospy.ROSInterruptException:
        pass
