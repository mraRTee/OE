import gym
import numpy as np
print("asdasdasdasd")
# hyperparameters
learning_rate = 0.1
discount_factor = 0.99
num_episodes = 5000
max_steps_per_episode = 200

# create the environment
env = gym.make('CartPole-v1')

# define the state and action spaces
num_states = env.observation_space.shape[0]
num_actions = env.action_space.n

# initialize the Q-table
Q = np.zeros((num_states, num_actions*2))

# helper function to discretize the state space
def discretize_state(state):
    state_min = env.observation_space.low
    state_max = env.observation_space.high
    state_bins = [np.linspace(state_min[i], state_max[i], num=10) for i in range(num_states)]
    return tuple([np.digitize(state[i], state_bins[i]) - 1 for i in range(num_states)])

# loop over episodes
for episode in range(num_episodes):
    # reset the environment and get the initial state
    state = env.reset()
    state = discretize_state(state)
    
    # loop over steps in the episode
    for t in range(max_steps_per_episode):
        # choose an action using the epsilon-greedy policy
        if np.random.uniform() < 0.1:
            action = env.action_space.sample()
        else:
            action = np.argmax(Q[state, :])
            
        # take the chosen action and observe the next state and reward
        next_state, reward, done, _ = env.step(action)
        next_state = discretize_state(next_state)
        
        # update the Q-table using the Q-learning rule
        Q[state, action] = (1 - learning_rate) * Q[state, action] + learning_rate * (reward + discount_factor * np.max(Q[next_state, :]))
        
        # update the state
        state = next_state
        
        # end the episode if the cartpole has fallen over
        if done:
            break
    
    # print the total reward for the episode
    print(f"Episode {episode + 1} finished after {t + 1} steps with total reward {t + 1}")
