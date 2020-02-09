#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy 
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np

def callback(data):
    res=np.array(data.data).reshape((2,2))
    print rospy.get_name(), "I heard "
    print(res)
def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats",numpy_msg(Floats),callback)
    rospy.spin()

if __name__ == '__main__':
    listener()