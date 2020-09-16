#!/usr/bin/env python

from project_practice_1.srv import project_practice1
from project_practice_1.srv import project_practice1Request
from project_practice_1.srv import project_practice1Response
import rospy
import sys

def project_practice1_client(req):
    rospy.wait_for_service('num_detection')
    try:
        num_detection = rospy.ServiceProxy('num_detection', project_practice1)
        resp1 = num_detection(req)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
def main():
    req=213
    rospy.init_node('test_server',anonymous=True)
    response = project_practice1_client(req)
    res=False
    while(res==False):
        res=response.result
        #go stra
if __name__ == "__main__":
    main()