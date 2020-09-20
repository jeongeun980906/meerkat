#!/usr/bin/env python

import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self):
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y

    def getPosition(self):
        position_check=True
        while position_check:
            goal_x = random.randrange(-12, 13) / 10.0
            goal_y = random.randrange(-12, 13) / 10.0
            if abs(goal_x - self.obstacle_1[0]) <= 0.5 and abs(goal_y - self.obstacle_1[1]) <= 0.5:
                position_check = True
            elif abs(goal_x - self.obstacle_2[0]) <= 0.5 and abs(goal_y - self.obstacle_2[1]) <= 0.5:
                position_check = True
            elif abs(goal_x - self.obstacle_3[0]) <= 0.5 and abs(goal_y - self.obstacle_3[1]) <= 0.5:
                position_check = True
            elif abs(goal_x - self.obstacle_4[0]) <= 0.5 and abs(goal_y - self.obstacle_4[1]) <= 0.5:
                position_check = True
            elif abs(goal_x - 0.0) <= 0.5 and abs(goal_y - 0.0) <= 0.5:
                position_check = True
            else:
                position_check = False

            if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                position_check = True

            self.goal_position.position.x = goal_x
            self.goal_position.position.y = goal_y

        time.sleep(0.5)

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
