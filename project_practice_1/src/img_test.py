#!/usr/bin/env python
import rospy 
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import math

bridge=CvBridge()
def callback(image):
    print 'got an image'
    global bridge
    try:
        cv_image=bridge.imgmsg_to_cv2(image,"bgr8")
    except CvBridgeError as e:
        print(e)
    print(cv_image.shape)
def call(depth):
    global bridge
    try:
        cv_depth=bridge.imgmsg_to_cv2(depth,"16UC1")
    except CvBridgeError as e:
        print(e)
    print(cv_depth.shape)


def main(args):
    rospy.init_node('depth_test',anonymous=True)
    image_sub=rospy.Subscriber("/camera/color/image_raw",Image,callback)
    depth_sub=rospy.Subscriber("/camera/depth/image_rect_raw",Image,call)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    #make_model
    main(sys.argv)