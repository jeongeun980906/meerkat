#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import numpy as np
import time
import math
from std_srvs.srv import Empty
from cv_bridge import CvBridge,CvBridgeError

inf=99999
direction=0
left=0
right=0
theta=0
bridge=CvBridge()

def preprocessing(ranges):
    left=np.min(ranges[30:150])
    right=np.min(ranges[210:330])
    return left,right
    
def scan_callback(scan):
    ranges=scan.ranges
    #print(scan)
    ranges=np.array(ranges)
    global right
    global left
    left,right=preprocessing(ranges)
    #print(left,right)

def filter_color(rgb_image,lower_bound_color,upper_bound_color):
    hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)
    mask=cv2.inRange(hsv_image,lower_bound_color,upper_bound_color)
    return mask
def get_contours(binary_image):
    _,contours,hierachy=cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours,hierachy

def contour(rgb_image,contours,hierarchy):   
    #print(hierarchy)
    num=0
    pix_x=[]
    pix_y=[]    
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        x,y,w,h=cv2.boundingRect(c)
        if area>300:
            cv2.drawContours(rgb_image,[c],-1,(0,255,255),1)
            cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
            if hierarchy[0][index][3]==-1:
                pix_x.append(x+w/2)
                pix_y.append(y+h/2)
                num+=1
    cv2.imshow("RGB Image Contours",rgb_image)
    #print(new_img_array.shape)
    return pix_x,pix_y,num

def handle_pixel(pix_x,pix_y):
    global direction
    if pix_x.shape == None:
        direction=0
    else:
        pix_x=np.array(pix_x)
        pix_y=np.array(pix_y)
        max_x=np.max(pix_x)
    
        if max_x<300:
            direction=1 #front
        elif max_x<340:
            direction=2 #rear
        else:
            drection=3 #middle


def image_callback(ros_image):
    global bridge
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    Lower=(40,50,90)
    Upper=(70,255,255)
    mask=filter_color(cv_image,Lower,Upper)
    cv2.imshow('mask',mask)
    countours,hierarchy=get_contours(mask)
    pix_x,pix_y,num=contour(cv_image,countours,hierarchy)
    handle_pixel(pix_x,pix_y)
    
def move_back():
    global direction
    loop_rate=rospy.Rate(100)
    vel_msg=Twist()
    velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    while direction!=3:
        if direction==0 or direction==2:
            vel_msg=transformation[5]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
    
        if direction==1:
            vel_msg=transformation[2]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

    vel_msg=transformation[6]
    velocity_publisher.publish(vel_msg)
    loop_rate.sleep()
def move_forward():
    global direction
    loop_rate=rospy.Rate(100)
    vel_msg=Twist()
    velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    while direction!=3:
        if direction==0 or direction==1:
            vel_msg=transformation[2]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
    
        if direction==2:
            vel_msg=transformation[5]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

    vel_msg=transformation[6]
    velocity_publisher.publish(vel_msg)
    loop_rate.sleep()

def transformation(action):
    vel_msg=Twist()
    vel_msg.angular.y=0
    vel_msg.angular.x=0
    vel_msg.linear.z=0
    vel_msg.linear.y=0
    if action==0:
        vel_msg.linear.x=0.
        vel_msg.angular.z=-0.674218
    elif action==1:
        vel_msg.linear.x=0.2
        vel_msg.angular.z=-0.740741 #right
    elif action==2:
        vel_msg.linear.x=0.2
        vel_msg.angular.z=0.0
    elif action==3:
        vel_msg.linear.x=-0.2
        vel_msg.angular.z=0.740741 #left
    elif action==4:
        vel_msg.linear.x=0.2
        vel_msg.angular.z=0.674218
    elif action==5:
        vel_msg.linear.x=-0.2
        vel_msg.angular.z=0.00
    elif action==6:
        vel_msg.linear.x=0.0
        vel_msg.angular.z=0.0
    return vel_msg

if __name__ == "__main__":
    try:
        rospy.init_node('meekat_wall')
        velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
        scan_sub=rospy.Subscriber('/scan',LaserScan,scan_callback)
        image_sub=rospy.Subscriber('/usb_cam',Image,image_callback)
        action=[0,1,2,3,4,5]
        done=False
        global left
        global direction
        go_back=False
        loop_rate=rospy.Rate(100)
        if left>0.3:
            if direction==3:
                go_back=True
            vel_msg=transformation[0]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
            rospy.sleep(1)
            while left<0.25:
                if direction==3:
                    go_back=True
                vel_msg=transformation[2]
                velocity_publisher.publish(vel_msg)
                loop_rate.sleep()
            if direction==3:
                go_back=True
            vel_msg=transformation[4]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
            rospy.sleep(1)
            vel_msg=transformation[6]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
        if left<0.3 and left>0.21:
            if direction==3:
                go_back=True
            vel_msg=transformation[1]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
            rospy.sleep(0.3)
            while left<0.23:
                if direction==3:
                    go_back=True
                vel_msg=transformation[2]
                velocity_publisher.publish(vel_msg)
                loop_rate.sleep()
            if direction==3:
                go_back=True
            vel_msg=transformation[3]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
            rospy.sleep(0.3)
            vel_msg=transformation[6]
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
        else:
            pass
        if go_back:
            move_back()
        else:
            move_forward()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")