from cmath import sin
import gym
import FolderB
import numpy as np
import os

env = gym.make('GTPR-v0')
# env.qposvel_zeroing()
# env.ball_init()
asd = np.sin(range(10000))
obs = env.reset()
while(True):
    for t in range(1000):
        #action = env.action_space.sample()
        action = [asd[t],asd[t],asd[t]]
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            obs = env.reset()
            break
env.close()




# from cmath import sin
# import gym
# import FolderB
# import numpy as np

# env = gym.make('GTPR-v0')
# #obs = env.reset()
# # env.qposvel_zeroing()
# # env.ball_init()
# asd = np.sin(range(10000))
# # action = [asd[t],asd[t],asd[t]]
# while(True):
#     #obs = env.reset()
#     for t in range(1000):
#         action = env.action_space.sample()
#         # action = [asd[t],asd[t],asd[t]]
#         obs, reward, done, info = env.step(action)
#         env.render()
#         #if done:
#         #    print("Episode finished after {} timesteps".format(t+1))
#         #    break
# env.close()


# import gym
# import FolderB

# env = gym.make('GTPR-v0')
# obs = env.reset()
# while(True):
#     env.ball_init()
#     for t in range(10000000):
#         action = env.action_space.sample()
#         print("action: ",action)
#         observation = env.observation_space.sample()
#         print("observation: ",observation)
#         obs, reward, done, info = env.step(action)
#         env.render()
#         env.setter() #invertedPendulum-v2
#         if done:
#             print("Episode finished after {} timesteps".format(t+1))
#             break
# env.close()




# import gym
# from network import DQN

# import FolderB
# print("Sajat kornyezet inditasa!")
# #env = gym.make('GTPR-v0')
# #env= lambda obs:0
# env = gym.make('InvertedPendulum-v2')
# while(True):
#     obs = env.reset()
#     for t in range(10000000):
#         #env.setter() #invertedPendulum-v2
#         action = env.action_space.sample()
#         obs, reward, done, info = env.step(action)
#         env.render()
#         lfsz=DQN(env.action_space,128)
#         print(lfsz.build_model())
#         input()
#         if done:
#             print("Episode finished after {} timesteps".format(t+1))
#             break
# env.close()
