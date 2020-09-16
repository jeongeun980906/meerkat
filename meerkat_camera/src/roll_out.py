#!/usr/bin/env python
import time
from std_msgs.msg import Int16,String
#from tf.transformations import euler_from_quaternion, quaternion_from_euler,euler_matrix,quaternion_matrix
from meerkat_camera.msg import ccam_kf,ccam_rl
import rospy
import random
import numpy as np
import cv2
from math import cos,sin,pi,sqrt,atan2

class Move():
    def __init__(self):
        self.pub_servo_angle = rospy.Publisher('/servoang', Int16, queue_size=5)
        self.pub_servo_vel = rospy.Publisher('/servovel', Int16, queue_size=5)
        self.x=0
        self.y=0
        self.yaw=0
        self.past=90
    
    def move(self,vel,ang):

        msg_vel=Int16()
        msg_angle=Int16()
        if vel==1:
            msg_vel=86
        elif vel==0:
            msg_vel=91
        else:
            msg_vel=110
        if self.past==(15*(ang-2)+90):
            if ang==0:
                msg_angle=60*2-self.past-5
            elif ang==1:
                msg_angle=75*2-self.past-5
            elif ang==2:
                msg_angle=90*2-self.past-5
            elif ang==3:
                msg_angle=105*2-self.past-5
            elif ang==4:
                msg_angle=120*2-self.past-5
        else:
            if ang==0:
                msg_angle=60
            elif ang==1:
                msg_angle=75
            elif ang==2:
                msg_angle=90
            elif ang==3:
                msg_angle=105
            elif ang==4:
                msg_angle=120
        self.past=(15*(ang-2)+90)
        for _ in range(10):
            self.pub_servo_angle.publish(msg_angle)
            self.pub_servo_vel.publish(msg_vel)
        time.sleep(1)
    
    def roll_out(self):
        self.move(0,2)
        self.call_back_kf()
        self.move(1,2)
        self.call_back_kf()
        # self.move(1,2)
        # self.call_back_kf()
        self.move(1,4)
        self.call_back_kf()
        self.move(1,4)
        self.call_back_kf()
        self.move(1,2)
        self.call_back_kf()
        self.move(1,2)
        self.call_back_kf()
        self.move(1,0)
        self.call_back_kf()
        self.move(1,0)
        self.call_back_kf()
        # self.move(1,0)
        # self.call_back_kf()
        self.move(0,2)
        self.call_back_kf()
        self.move(0,2)
        self.call_back_kf()
        self.move(0,2)
        self.call_back_kf()
        self.move(0,2)
        self.call_back_kf()
        print('done')

def main():
    rospy.init_node("roll_out_node")
    sub=Sub()
    sub.roll_out()
    #obj_position=kinect_rgb_img_get.get_pose_class_state()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__=='__main__':
    main()
