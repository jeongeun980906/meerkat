#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist,Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from goal2 import Respawn
from data import scan as Check

class Env():
    def __init__(self):
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = True
        self.action_size = 5
        self.respawn_goal=Respawn()
        self.position_x = 0.0
        self.position_y = 0.0
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

    def move(self,action):
        vel_msg=Twist()
        vel_msg.angular.y=0
        vel_msg.angular.x=0
        vel_msg.linear.z=0
        vel_msg.linear.y=0
        if action==0:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=-0.674218
        elif action==1:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=-0.740741 #right
        elif action==2:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=0.0
        elif action==3:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=0.740741 #left
        elif action==4:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=0.674218
        for _ in range(10):
            self.pub_cmd_vel .publish(vel_msg)
    
    def transformation(self,pose_x,pose_y,yaw):
        self.position_x=round(self.goal_x*math.cos(yaw)+self.goal_y*math.sin(yaw)-pose_x*math.cos(yaw)-pose_y*math.sin(yaw),3)
        self.position_y=round(-self.goal_x*math.sin(yaw)+self.goal_y*math.cos(yaw)+pose_x*math.sin(yaw)-pose_y*math.cos(yaw),3)

    def getOdometry(self,odom):
        pose=Pose()
        pose= odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.transformation(pose.x,pose.y,yaw)

    def getState(self,scan):
        scan_range=[]
        done = False

        for i in range(len(scan.ranges)):
            if i%2==0:
                if scan.ranges[i] == float('Inf'):
                    scan_range.append(3.5)
                elif np.isnan(scan.ranges[i]):
                    scan_range.append(0)
                else:
                    scan_range.append(scan.ranges[i])
        check=Check(scan_range)
        done=check.range()
        distance=round(math.sqrt(self.position_x**2+self.position_y**2),2)
        if distance<0.1:
            self.goal=True
            done=True
        return scan_range,done

    def Reward(self,done):
        distance=round(math.sqrt(self.position_x**2+self.position_y**2),2)
        origin=round(math.sqrt((self.goal_x+1.5)**2+(self.goal_y+1.5)**2),2)
        theta=math.degrees(math.atan2(self.position_y,self.position_x))/90
        #print(5*(origin-distance)/origin,theta)
        reward=5*(origin-distance)/origin-abs(theta)
        if done:
            rospy.loginfo('done')
            reward=-150
        if self.goal==True:
            reward=500
            rospy.loginfo('goal!')
        return reward
    def step(self, action):
        self.move(action)
        rospy.sleep(0.5) #hyperparameter
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/m2wr/laser/scan', LaserScan, timeout=5)
            except:
                pass
        scan_range, done = self.getState(data)
        point=[self.position_x,self.position_y]
        reward=self.Reward(done)
        return np.asarray(scan_range),np.asarray(point), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
	done=False
	if self.goal==True:
            self.goal_x, self.goal_y=self.respawn_goal.getPosition()
	    self.goal=False
        print('goal is ',self.goal_x,' ',self.goal_y)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/m2wr/laser/scan', LaserScan, timeout=5)
            except:
                pass
       
        scan_range, done = self.getState(data)
        point=[self.position_x,self.position_y]
        return np.asarray(scan_range),np.asarray(point)
