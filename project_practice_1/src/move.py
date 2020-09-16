#!/usr/bin/env python
from go_num.srv import go_num
from go_num.srv import go_numRequest
from go_num.srv import go_numResponse
from project_practice_1.srv import project_practice1
from project_practice_1.srv import project_practice1Request
from project_practice_1.srv import project_practice1Response
import rospy
import time
import sys
from sensor_msgs.msg import LaserScan
import numpy as np
inf=99999
ranges=np.zeros((1,360),dtype=np.float)

def scan_callback(scan):
    ranges1=scan.ranges
    #print(scan)
    ranges1=np.array(ranges1)
    global ranges
    ranges=ranges1

def go_straight():
    global ranges
 #   left_range=ranges[40:140]
    right_range=ranges[220:320]
  #  mlr= left_range[left_range != 0]
    mrr=  right_range[right_range != 0]
  #  left=mlr.min()
    right=mlr.min()
    if right>0.6:
        #왼쪽으로 조금
    elif right<0.3:
        #오른쪽으로 조금
    else:
        #직진

def project_practice1_client(req):
    rospy.wait_for_service('num_detection')
    try:
        num_detection = rospy.ServiceProxy('num_detection', project_practice1)
        resp1 = num_detection(req)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    req = input("숫자를 입력하세요: ")
    req=float(req)
    res=False
    while(res==False):
        for _ in range(10):
            response = project_practice1_client(req)
            res=response.result
            if res==True:
                break
        if res==True:
            break
        go_straight()
        rospy.sleep(0.5)
    
def handler(req):
    go=req.go
    if go:
        main()
        return go_numRequest(True)
    else:
        return go_numRequest(False)

if __name__ == "__main__":
    rospy.init_node('move',anonymous=True)
    server=rospy.Service('go_num',go_num,handler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"