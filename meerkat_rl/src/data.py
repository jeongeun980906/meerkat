#!/usr/bin/env python

import rospy
import numpy as np
import math

class scan():
    def __init__(self,data):
        self.data=data
    def range(self):
        for scan_data in self.data:
            