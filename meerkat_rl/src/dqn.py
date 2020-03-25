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
EPISODES = 5000

class DQNAgent():
    def __init__(self, scan_size,point_size, action_size):
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.dirPath = self.dirPath.replace('meerkat_rl/src', 'meerkat_rl/save_model/model_')
        self.load_model=False
        self.load_episode = 0

        self.scan_size = scan_size
        self.point_size=point_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 200
        self.discount_factor = 0.99
        self.learning_rate = 0.0025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=10000)
        self.q_value=np.zeros(self.action_size)

        self.model = self.buildModel()
        self.target_model = self.buildModel()

        self.updateTargetModel()

        if self.load_model:
            self.model.set_weights(tf.keras.models.load_model(self.dirPath + str(self.load_episode) + ".h5").get_weights())

    def buildModel(self):
        scan_input=Input(shape=(self.scan_size,),name='scan')
        point_input=Input(shape=(self.point_size,),name='point')
        s1=layers.Dense(128,activation='relu',kernel_initializer='lecun_uniform')(scan_input)
        s2=layers.Dense(64,activation='relu',kernel_initializer='lecun_uniform')(s1)
        s3=layers.Dense(32,activation='relu',kernel_initializer='lecun_uniform')(s2)
        p1=layers.Dense(32,activation='relu',kernel_initializer='lecun_uniform')(point_input)
        concatenated=layers.concatenate([s3,p1],axis=-1)
        c2=layers.Dense(16,activation='relu',kernel_initializer='lecun_uniform')(concatenated)
        value=layers.Dense(self.action_size,activation='linear',kernel_initializer='lecun_uniform')(c2)
        model=Model([scan_input,point_input],value)
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        model.summary()
        return model

    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self,scan,point):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict([scan.reshape(1, len(scan)),point.reshape(1,len(point))])
            self.q_value = q_value
            return np.argmax(q_value[0])

    def appendMemory(self, scan, point, action, reward, next_scan,next_point, done):
        self.memory.append((scan, point, action, reward, next_scan,next_point, done))

    def trainModel(self):
        mini_batch = random.sample(self.memory, self.batch_size)
        X1_batch = np.empty((0, self.scan_size), dtype=np.float64)
        X2_batch = np.empty((0, self.point_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):
            scans = mini_batch[i][0]
            points=mini_batch[i][1]
            actions = mini_batch[i][2]
            rewards = mini_batch[i][3]
            next_scans = mini_batch[i][4]
            next_points=mini_batch[1][5]
            dones = mini_batch[i][6]

            q_value = self.model.predict([scans.reshape(1, len(scans)), points.reshape(1,len(points))])
            #q_value=q_value[0]
            self.q_value = q_value
            
            next_target = self.target_model.predict([next_scans.reshape(1, len(scans)),next_points.reshape(1,len(points))])
            #next_target=next_target[0]
            next_q_value = self.getQvalue(rewards, next_target, dones)
            #print(self.q_value,next_q_value)
            X1_batch = np.append(X1_batch, np.array([scans.copy()]), axis=0)
            X2_batch = np.append(X2_batch, np.array([points.copy()]), axis=0)
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if dones:
                X1_batch = np.append(X1_batch, np.array([next_scans.copy()]), axis=0)
                X2_batch = np.append(X2_batch, np.array([next_points.copy()]), axis=0)
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)
            
        self.model.fit([X1_batch,X2_batch], Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)

if __name__ == '__main__':
    rospy.init_node('meerkat_rl')
    scan_size = 360
    point_size=2
    action_size = 5

    env = Env()

    agent = DQNAgent(scan_size, point_size,action_size)
    scores, episodes = [], []
    global_step = 0
    start_time = time.time()

    for e in range(agent.load_episode + 1, EPISODES):
        done = False
        scan,point = env.reset()
        #print(state)
        score = 0
        for t in range(agent.episode_step):
            print('step')
            print(scan.shape,point)
            action = agent.getAction(scan,point)

            next_scan,next_point, reward, done = env.step(action)
            print('reward:' ,reward)
            agent.appendMemory(scan,point, action, reward, next_scan,next_point, done)

            if len(agent.memory) >= agent.train_start:
                rospy.loginfo('train starts')
                agent.trainModel()

            score += reward
            scan = next_scan
            point=next_point
            if t > 500:
                rospy.loginfo("Time out.")
                done = True

            if e % 10 == 0:
                agent.model.save(agent.dirPath + str(e) + '.h5')
            if done:
                scores.append(score)
                episodes.append(e)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              e, score, len(agent.memory), agent.epsilon, h, m, s)
                break

            global_step += 1
            print(global_step)
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")
                agent.updateTargetModel()

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
