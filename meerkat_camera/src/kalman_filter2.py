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
        try:
            ax,ay=minput()
        except:
            return
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
        return ax,ay

class KalmanFilter(object):
    def __init__(self):
        #x=[x y th x_dot y_dot th_dot]
        #y=[x y th th_dot]
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,0,0,1]])
        self.Q = np.array([0.01,0.01,0.05,0.02,0.02,0.02])
        self.R = np.array([[0.01,0,0,0],[0,100.0,0,0],[0,0,1.3,0],[0,0,0,0.3]])
        self.P = np.eye(6)
        self.x = np.zeros((6,1))
        self.x[0]=2.0
        self.x[1]=-0.5
        self.x[2]=pi

    def predict(self, u ,dt):
        theta=self.x[2]
        #print(u)
        self.F = np.array([[1,0,0,dt,0,0],[0,1,0,0,dt,0],[0,0,1,0,0,dt],\
            [0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        G=np.array([[dt**2/2,0],[0,dt**2/2],[0,0],[dt,0],[0,dt],[0,0]])
        inp=np.array([[cos(theta)*u[0]-sin(theta)*u[1]],[sin(theta)*u[0]+cos(theta)*u[1]]])
        T=np.dot(G,inp)
       # print('input',T)
        self.x = np.dot(self.F, self.x) + T
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(6)
        #print((np.dot(K, self.R)).shape,K.T.shape)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


def minput():
    data2 = rospy.wait_for_message('/chatter', String, timeout=2)
    data=data2.data
    d3=data.split('D')
    d4=d3[1].split('E')
    d5=d4[1].split('F')
    ax=float(d4[0])
    ay=float(d5[0])
    ax=-ax/(32750*9.8)
    ay=-ay/(32750*9.8)
   # print(ay,ax)
    return ay,ax

def measurement():
    try:
        data1 = rospy.wait_for_message('/position_kf', ccam_kf, timeout=1)
        data2 = rospy.wait_for_message('/chatter', String, timeout=2)
        x=data1.x
        y=data1.y
        yaw=data1.yaw
        data=data2.data
        d2=data.split('C')
        d3=d2[1].split('D')
        gz=float(d3[0])
        gz=-gz/(131*180)*pi
    #    print(gz)
        return x,y,yaw,gz
    except:
            print('error')
            return

kf = KalmanFilter()
move=Move()

def step(vel,ang):
    a=time.time()
    ax,ay=move.move(vel,ang)
    b=time.time()
    print('---------------')
    kf.predict(np.array([ax,ay]),b-a)
    x,y,yaw,gz=measurement()
    print('---measurement----')
    kf.update([[x],[y],[yaw],[gz]])
    x=kf.x
    print(x)
    

def main():
    rospy.init_node("roll_out_node")
    _,_=move.move(0,2)
    #_,_=move.move(1,2)
    x=kf.x
    print(x)
    step(1,2)
    step(1,4)
    step(1,4)
    step(1,4)
    step(1,4)
    step(1,2)
    step(1,2)
    step(1,0)
    step(1,0)
    step(0,2)
    step(0,2)
    print('done')
    print(kf.P)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__=='__main__':
    main()




       