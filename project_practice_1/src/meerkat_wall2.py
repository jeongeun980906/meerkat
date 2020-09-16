#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
from std_srvs.srv import Empty
inf=99999
x=0.0
y=0.0
w=0.0
ranges=np.zeros((1,360),dtype=np.float)

def preprocessing(ranges,theta):
    #print(theta)
    if theta>=0 and theta<45:
        left=np.min(ranges[45-theta:135-theta])
        right=np.min(ranges[225-theta:315-theta])    
    elif theta>=45 and theta<135:
        left1=np.min(ranges[405-theta:])
        left2=np.min(ranges[:135-theta])
        left=np.min([left1,left2])
        right=np.min(ranges[225-theta:315-theta])
    elif theta>=135 and theta<215:
        left=np.min(ranges[405-theta:495-theta])
        right=np.min(ranges[225-theta:315-theta])
    elif theta>=215 and theta<315:
        left=np.min(ranges[405-theta:495-theta])
        right1=np.min(ranges[585-theta:])
        right2=np.min(ranges[:315-theta])
        right=np.min([right1,right2])
    else:
        left=np.min(ranges[405-theta:495-theta])
        right=np.min(ranges[585-theta:675-theta])

    return left,right
    
def scan_callback(scan):
    ranges1=scan.ranges
    #print(scan)
    ranges1=np.array(ranges1)
    global ranges
    ranges=ranges1
    #print(left,right)

def transformation(action):
    vel_msg=Twist()
    vel_msg.angular.y=0
    vel_msg.angular.x=0
    vel_msg.linear.z=0
    vel_msg.linear.y=0
    if action==0:
        vel_msg.linear.x=0.05
        vel_msg.angular.z=-0.337109
    elif action==1:
        vel_msg.linear.x=0.05
        vel_msg.angular.z=-0.371747 #right
    elif action==2:
        vel_msg.linear.x=0.05
        vel_msg.angular.z=0.0
    elif action==3:
        vel_msg.linear.x=-0.05
        vel_msg.angular.z=0.371747 #left
    elif action==4:
        vel_msg.linear.x=0.05
        vel_msg.angular.z=0.337109
    elif action==5:
        vel_msg.linear.x=-0.05
        vel_msg.angular.z=0.00
    elif action==6:
        vel_msg.linear.x=0.0
        vel_msg.angular.z=0.0
    return vel_msg
def move(action):
    velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    vel_msg=transformation(action)
    loop_rate=rospy.Rate(100)
    for i in range(20):
        #rospy.loginfo("moves")
        #print(i)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    vel_msg=transformation(6)
    rospy.sleep(0.883)
    for i in range(20):
        #rospy.loginfo("moves")
        #print(i)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    rospy.loginfo('move_done')
def odometry(action,w_w):
    dx=0.0
    dy=0.0
    dw=0.0
    if action==0:
        dx=0.048857
        dy=0.0091875
        dw=-0.371747
    elif action==1:
        dx=0.049046
        dy=0.008344
        dw=-0.337109
    elif action==2:
        dx=0.05
    elif action==3:
        dx=0.049046
        dy=-0.008344
        dw=0.337109
    elif action==4:
        dx=0.048857
        dy=-0.0091875
        dw=0.371747
    elif action==6:
        dx=-0.05
    dx_t=dx*math.cos(w_w)-dy*math.sin(w_w)
    dy_t=dx*math.sin(w_w)+dy*math.cos(w_w)
    return dx_t,dy_t,dw

if __name__ == "__main__":
    try:
        rospy.init_node('meekat_cmd_vel')
        velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
        scan_sub=rospy.Subscriber('/scan',LaserScan,scan_callback)
        action=[0,1,2,3,4,5]
        while (1): 
            rospy.sleep(1)
            global x
            global y
            global w
            global ranges
            theta=int((-math.degrees(w))%360)
            left,right=preprocessing(ranges,theta)
            print(theta)
            print(left)
            if left>0.3:
                print('hi')
                move(action[4])
                rospy.sleep(0.5)
                dx,dy,dw=odometry(action[4],w)
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                x+=dx
                y+=dy
                w+=dw
                while left>0.25:
                    move(action[2])
                    rospy.sleep(0.5)
                    dx,dy,dw=odometry(action[2],w)
                    x+=dx
                    y+=dy
                    w+=dw
                    theta=int((math.degrees(w))%360)
                    left,right=preprocessing(ranges,theta)
                    print(x,y,theta)
                    print(left)
                move(action[3])
                rospy.sleep(0.5)
                dx,dy,dw=odometry(action[3],w)
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                print(x,y,theta)
                print(left)
                move(action[1])
                rospy.sleep(0.5)
                dx,dy,dw=odometry(action[1],w)
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                print(x,y,theta)
                print(left)
                move(action[0])
                rospy.sleep(0.5)
                dx,dy,dw=odometry(action[0],w)
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                print(x,y,theta)
                print(left)
                reset=rospy.ServiceProxy('/gazebo/reset_world',Empty)
                reset()
                break
                
            elif left<0.25 and left>0.22:
                move(action[3])
                global x
                global y
                global w
                global ranges
                dx,dy,dw=odometry(action[3])
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                move(action[2])
                dx,dy,dw=odometry(action[2])
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                move(action[1])
                dx,dy,dw=odometry(action[1])
                x+=dx
                y+=dy
                w+=dw
                theta=int((math.degrees(w))%360)
                left,right=preprocessing(ranges,theta)
                print(theta)
                print(left)
            elif left<0.1:
                move(action[0])
                
            elif left>0.1 and left<0.19:
                move(action[1])
                #move(action[2])
            elif left>0.19 and left<0.21:
                move(action[2])
                break
            
            theta=int((math.degrees(w))%360)
            
            rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")