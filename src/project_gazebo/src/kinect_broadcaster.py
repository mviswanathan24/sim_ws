#!/usr/bin/env python
import rospy
import tf

if __name__ == "__main__":
    rospy.init_node("kinect_broadcaster_node")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.25, 0.0, 0.3),
                         (-0.5 , 0.5, -0.5, 0.5),
                         rospy.Time.now(),
                         "kinect_link",
                         "pioneer2dx/chassis")
