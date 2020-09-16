#!/usr/bin/env python
import time
from std_msgs.msg import Int16,String
#from tf.transformations import euler_from_quaternion, quaternion_from_euler,euler_matrix,quaternion_matrix
from meerkat_camera.msg import ccam_kf,ccam_rl
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tensorflow._api.v1.compat.v2.config

bridge = CvBridge()

def call_back_kf(i):
    try:
        data = rospy.wait_for_message('/position_kf', ccam_kf, timeout=1)
        data2 = rospy.wait_for_message('/chatter', String, timeout=1)
        data3=rospy.wait_for_message('/camera/color/image_raw',Image,timeout=2)
        image=bridge.imgmsg_to_cv2(data3, "bgr8")
        x=data.x
        y=data.y
        yaw=data.yaw
        cv.imwrite(image,str(i)+'.png')
        print(data,data2)
    except:
        print('error')

def main():
    rospy.init_node("callback_node")
    a=time.time()
    #obj_position=kinect_rgb_img_get.get_pose_class_state()
    for i in range(100000):
        call_back_kf(i)
        b=time.time()
        print(b-a)
        a=b
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__=='__main__':
    main()
