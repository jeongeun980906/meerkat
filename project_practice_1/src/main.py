#!/usr/bin/env python

from go_num.srv import go_num
from go_num.srv import go_numRequest
from go_num.srv import go_numResponse
import rospy
import sys

def go_client(req):
    rospy.wait_for_service('go_num')
    try:
        response = rospy.ServiceProxy('go_num', go_num)
        resp1 = response(req)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def go1(go):
    response = go_client(go)
    done=response.done
    return done

if __name__ == "__main__":
    rospy.init_node('main',anonymous=True)