from cmath import sin
import gym
import FolderB
import numpy as np
import os
import math
import sys


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
    while(True):
        for t in range(10000):
            action = env.action_space.sample()
            #action = [sinus[t],sinus[t],sinus[t]]
            obs, reward, done, info = env.step_train(action)
            env.render()
            if done:
                print("Episode finished after {} timesteps".format(t+1))
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