#!/usr/bin/env python

from project_practice_1.srv import project_practice1
from project_practice_1.srv import project_practice1Request
from project_practice_1.srv import project_practice1Response
import rospy
import sys

def handler(req):
    input_num=req.input
    input_num=float(input_num)
    res=213.0
    if res==input_num:
        detect=True
            #depth
        x=0.0
        y=0.0
            
        return project_practice1Response(detect,x,y)
    detect=False
    x=0.0
    y=0.0
    
    return project_practice1Response(detect,x,y)

def main(args):
    rospy.init_node('preprocessing',anonymous=True)
    server=rospy.Service('num_detection',project_practice1,handler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__ == '__main__':
    main(sys.argv)