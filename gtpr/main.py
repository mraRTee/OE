from cmath import sin
import gym
import FolderB
import numpy as np
import os
import math
import sys
import random

def perform_sinus(env, obs):
    '''
    This is only perform sinus presentation
    '''
    amplitude = 1.2
    frequency = 20.0
    time = np.arange(0, 10000, 0.001)
    sinus = amplitude * np.sin(2 * np.pi * frequency * time)
    while(True):
        for t in range(10000):
            action = [sinus[t],sinus[t],sinus[t]]
            obs = env.step(action)
            env.render()

def train(env, obs):
    
    #q_table = env.q_table_zeroing(state_space = 100000, action_state = 4)
    q_table = np.random.uniform(low=-10,high=10,size=([10000]+[4]))

    while(True):
        for t in range(10000):
            #action = np.random.uniform(low=-10, high=10, size=3)
            action = q_table[random.randint(0, 9999)][:-1]
            print(action)
            #print(action)
            obs, reward, done, _ = env.step_train(action)
            print("obs: ",obs)
            # print("reward: ",reward)
            env.render()
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                print(reward)
                input()
                obs = env.reset()
                break

def main():
    env = gym.make('GTPR-v0')
    obs = env.reset()
    if sys.argv[1] == "train":
        train(env, obs)
    elif sys.argv[1] == "sin":
        perform_sinus(env, obs)
    env.close()


if __name__ == "__main__":
    main()