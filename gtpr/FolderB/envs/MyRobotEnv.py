import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import os
import time
import mujoco_py as mjp
from dm_control import suite,mujoco

class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    falling_flag = False
    decking_counter = 0
    biggest = []
    biggest_numbers_of_events = []
    elapsed_time = 0
    reward = 0
    previous_dist = 0
    all_ball_height_list = []
    highest_balls_list = []
    counter = 0
    counter_list = []
    distance = []
    reward_list = []
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
        frame_skip = 1
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)
        
        
    def reset(self):
        self.sim.reset()
        print("COUNTER: ", self.decking_counter)
        self.decking_counter = 0
        self.biggest = 0
        self.biggest_numbers_of_events = []
        self.elapsed_time = time.time()
        self.reward = 0
        return self.get_observation()
        
    def get_observation(self):
        qpos = self.sim.data.qpos
        qvel = self.sim.data.qvel
        return np.concatenate([qpos, qvel])
    
    def dimension_getter(self, dimension):
        if dimension == "x":
            return 2
        """ TODO
        if dimension == "y":
            return 2
        if dimension == "z":
            return 2
        """
    
    def get_obs_my(self, body, dimension):
        """
        Getting back the bodies coordinates.    \n
        Possible bodies:                        \n
        'ball', 'base_link', 'platform'         \n
        'A1', 'U1', 'U1L', 'U1R', 'L1',         \n
        'A2', 'U2', 'U2L', 'U2R', 'L2',         \n
        'A3', 'U3', 'U3L', 'U3R', 'L3', 'WL1-1',\n
        """
        
        self.dimension_getter(dimension)
        return self.sim.data.get_body_xpos(body)[dimension]

    def diff(self, body_1, body_2, dim_1,  dim_2):
        dim_1 = self.dimension_getter(dim_1)
        dim_2 = self.dimension_getter(dim_2)
        body_1 = self.get_obs_my(body_1, dim_1)
        body_2 = self.get_obs_my(body_2, dim_2)
        return True if body_1 > body_2 else False

    def step(self, action):

        self.sim.data.ctrl[:] = action
        self.falling_flag = False
        self.previous_dist = 0
        self.sim.step()
        self.elapsed_time = time.time()
        return self.get_observation(), 0, False, {}
    
    def reward_function(self, platform_height, ball_height):
        distance_to_target = abs(platform_height - ball_height)
        diff = self.diff('ball', 'platform', "x", "x")
        elapsed_time_in_reward = time.time()-self.elapsed_time
        self.all_ball_height_list.append(ball_height)

        # Ball going upwards 
        if (self.previous_dist < ball_height) and (self.previous_dist != 0 ) and self.falling_flag == True:
            print("EMELKEDIK")
            self.falling_flag = False
            # Decking counting up
            self.decking_counter += 1

        # Slow down
        # time.sleep(0.04)

        # Ball going down
        if self.previous_dist > ball_height and self.falling_flag == False:
            print("ESIK")
            self.highest_balls_list.append(ball_height)
            self.counter_list.append(self.counter)
            self.falling_flag = True
            # Counting height
            self.distance.append(abs(platform_height - ball_height))

        if self.decking_counter:
            self.reward += self.highest_balls_list[-1]** 0.02
            self.reward += self.decking_counter ** 0.2
            self.reward += elapsed_time_in_reward ** 0.2

        # Getting the latest height of the ball
        self.counter += 1
        self.previous_dist = ball_height
        return self.reward, self.previous_dist

    def step_train(self, action):
        done = False

        if self.diff('platform', 'ball', "x", "x"):
            done = True
            print("utolsó magas labda: ", self.highest_balls_list[-1])
            print("dekázás: ", self.decking_counter)

        self.sim.data.ctrl[:] = action
        self.sim.step()
        reward, self.previous_dist = self.reward_function(self.get_obs_my('platform', 2), self.get_obs_my('ball', 2))

        return self.get_observation(), reward, done, {}, self.highest_balls_list, self.all_ball_height_list, self.counter_list
    
    def q_table_update(self, state_space, action_state):

        # HINT:
        #  max_future_q = np.max(q_table[new_discrete_state])

        # current_q = q_table[discrete_state + (action,)]

        # new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)

        # q_table[discrete_state + (action,)] = new_q
        # retrun q_table
        pass

    def q_table_zeroing(self, state_space, action_state):
        # state space is amount of elements, action space is the size of elements. 
        # state_space = 10
        # action_state = 2
        q_table = np.zeros((state_space, action_state))
        print(q_table)


        # q_table = np.zeros((10000,3))
        # for i in range(10000):
        #     for j in range(3):
        #         q_table[i][j] = np.random.uniform(low=-5, high=0)

  