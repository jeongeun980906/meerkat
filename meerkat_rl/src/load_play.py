#!/usr/bin/env python
import os
import rospy
import numpy as np
import random
import time
import sys
from collections import deque
import tensorflow as tf
from environment import Env
from keras import Model
from keras import layers,Input
from keras.optimizers import Adam
import matplotlib.pyplot as plt

if __name__ == '__main__':
    rospy.init_node('meerkat_rl_load')
    scan_size = 360
    point_size=2
    action_size = 5
    time=0
    env = Env()
    model=tf.keras.models.load_model('/home/kasimov/catkin_ws/src/meerkat_rl/save_model/model_1000.hdf5')
    scan,point = env.reset()
    while True:
        done = False    
        q_value = model.predict([scan.reshape(1, len(scan)),point.reshape(1,len(point))])
        action=np.argmax(q_value[0])
        scan,point, reward, done = env.step(action)
        time+=1
        print('time:',time,'reward:',reward)
        if done:
            break
