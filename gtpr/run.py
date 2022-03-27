import gym
import FolderB
print("Sajat kornyezet inditasa!")
env = gym.make('GTPR-v0')
print("egy")
#env = gym.make('HalfCheetah-v3')
print("ketto")
while(True):
    observation = env.reset()
    for t in range(1000):
        env.render()
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
#env.close()
