#!/usr/bin/env python
import rospy
import tf

if __name__ == "__main__":
    rospy.init_node("hokuyo_broadcaster_node")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.32),
                         (0.0 , 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "hokuyo_link",
                         "pioneer2dx/chassis")
