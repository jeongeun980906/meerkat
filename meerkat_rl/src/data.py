#!/usr/bin/env python

import rospy
import numpy as np
import math

class scan():
    def __init__(self,data):
        self.data=data
    def range(self):
        done=False
        for index,scan_data in enumerate(self.data):
            if index<25.34:
                if scan_data<(0.19*math.tan(math.radians(index))+0.03):
                    done=True
                    break
            elif index<154.66 and index>25.43:
                if scan_data<(0.09*math.tan(abs(math.radians(90-index)))+0.05):
                    done=True
                    break
            elif index>154.66 and index<205.34:
                if scan_data<(0.19*math.tan(abs(math.radians(180-index)))+0.05):
                    done=True
                    break
            elif index>205.34 and index<334.66:
                if scan_data<(0.09*math.tan(abs(math.radians(270-index)))+0.05):
                    done=True
                    break
            else:
                if scan_data<(0.19*math.tan(abs(math.radians(360-index)))+0.05):
                    done=True
                    break
        return done
