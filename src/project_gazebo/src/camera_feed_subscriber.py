#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# intialize the node
rospy.init_node("camer_feed_subscriber_node")

def image_cb(msg):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


def camera_feed_sub():
    rospy.Subscriber('/pioneer2dx/pioneer2dx/camera/image_raw',Image,image_cb)

if __name__ == "__main__":
    try:
        camera_feed_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
