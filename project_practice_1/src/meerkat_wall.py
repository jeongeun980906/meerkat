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
theta=0
right=0.0
left=0.0
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
    ranges=scan.ranges
    #print(scan)
    ranges=np.array(ranges)
    global right
    global left
    left,right=preprocessing(ranges,theta)
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
        vel_msg.linear.z=0.0
    elif action==3:
        vel_msg.linear.x=-0.05
        vel_msg.linear.z=0.371747 #left
    elif action==4:
        vel_msg.linear.x=0.05
        vel_msg.linear.z=0.337109
    elif action==5:
        vel_msg.linear.x=-0.05
        vel_msg.linear.z=0.00
    elif action==6:
        vel_msg.linear.x=0.0
        vel_msg.linear.z=0.0
    return vel_msg
def move(action):
    velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
    vel_msg=transformation(action)
    loop_rate=rospy.Rate(100)
    for i in range(20):
        rospy.loginfo("moves")
        print(i)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    vel_msg=transformation(6)
    rospy.sleep(0.883)
    for i in range(20):
        rospy.loginfo("moves")
        print(i)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
def odometry(action):
    x=0.0
    y=0.0
    z=0.0
    if action==0:
        x=0.048857
        y=0.0091875
        w=-0.371747
    elif action==1:
        x=0.049046
        y=0.008344
        w=-0.337109
    elif action==2:
        x=0.5
    elif action==3:
        x=0.049046
        y=-0.008344
        w=0.337109
    elif action==5:
        x=0.048857
        y=-0.0091875
        w=0.371747
    elif action==6:
        x=-0.05
    x_t=x*math.cos(w)-y*math.sin(w)
    y_t=x*math.sin(w)+y*math.cos(w)
    return x_t,y_t,w

if __name__ == "__main__":
    try:
        rospy.init_node('meekat_cmd_vel')
        velocity_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=100)
        scan_sub=rospy.Subscriber('/scan',LaserScan,scan_callback)
        action=[0,1,2,3,4,5]
        move(action[0])
        global x
        global y
        global w
        global theta
        dx,dy,dw=odometry(action[0])
        x+=dx
        y+=dy
        w+=dw
        theta=int((-math.degrees(w))%360)
        print(theta)
        print(left)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")