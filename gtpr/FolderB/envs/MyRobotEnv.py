import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import os
import time
import mujoco_py as mjp
from dm_control import suite,mujoco

class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    falling_flag = False
    step_counter = 0
    decking_counter = 0
    above_BONUS_ALTITUDE_DIFF = False
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
    prev_deck = 0
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
        self._max_episode_steps = 1000
        frame_skip = 1
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)
        
        
    def reset(self):
        # self.sim.reset()
        print("COUNTER: ", self.decking_counter)
        self.above_BONUS_ALTITUDE_DIFF = False
        self.decking_counter = 0
        self.biggest = 0
        self.biggest_numbers_of_events = []
        self.elapsed_time = time.time()
        self.reward = 0
        self.prev_deck = 0
        
        
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.01, high=0.01
        )
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=-0.01, high=0.01
        )
        self.set_state(qpos, qvel)
        return self.get_observation()
        
    def get_observation(self):
        # EZEK AZ AKTUÁTOROKHOZ TARTOZÓ JOINT-OK POZÍVIÓJA ÉS SEBESSÉGE
        # print(self.sim.data.qpos[7]) # A1 qpos
        # print(self.sim.data.qpos[17]) # A2 qpos
        # print(self.sim.data.qpos[27]) # A3 qpos)
        # print(self.sim.data.qvel[6]) # A1 qvel
        # print(self.sim.data.qvel[16]) # A2 qvel
        # print(self.sim.data.qvel[27]) # A3 qvel
        return np.concatenate([self.sim.data.qpos, self.sim.data.qvel]).ravel()
    
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
        self.do_simulation(action, self.frame_skip)
        observations = self.get_observation()
        done = False

        observations, reward, done = self.reward_function(self.get_obs_my('platform', 2), self.get_obs_my('ball', 2), done)
        return observations, reward, done, {}

    
    def reward_function(self, platform_height, ball_height, done):
        self.step_counter += 1
        distance_to_target = abs(platform_height - ball_height)
        diff = self.diff('ball', 'platform', "x", "x")
        if diff < .2:
            reward = -25
            done = True
        else:
            if distance_to_target >= .16:
                done = False
                if not self.above_BONUS_ALTITUDE_DIFF:
                    reward = 50
                    self.above_BONUS_ALTITUDE_DIFF = True
                    self.step_counter = 0
                else:
                    reward = 0
            else:   #ball is above the platform but lower than the relative height threshold
                if self.above_BONUS_ALTITUDE_DIFF:
                    self.above_BONUS_ALTITUDE_DIFF = False
                reward = -0.1
                done = False

        # max step num is 800
        if self.step_counter >= 800:
            done = True

        # info = {"eef position: ": self.observation[6:9], \
        #         "ball position: ": self.observation[12:15]}


        return self.get_observation(), reward, done

    def reward_function2(self, platform_height, ball_height, done):
        distance_to_target = abs(platform_height - ball_height)
        diff = self.diff('ball', 'platform', "x", "x")
        if diff:
            reward = -25
            done = True

        # elapsed_time_in_reward = time.time()-self.elapsed_time
        # self.all_ball_height_list.append(ball_height)

        # Ball going up
        if (self.previous_dist < ball_height) and (self.previous_dist != 0 ) and self.falling_flag == True:
            # print("EMELKEDIK")
            self.falling_flag = False
            # Decking counting up - have to set this treshhold because there are some little movement up and down in the environment
            if self.distance[-1]>.2:
                self.decking_counter += 1

        # Slow down
        # time.sleep(0.04)

        # Ball going down
        if self.previous_dist > ball_height and self.falling_flag == False:
            # print("ESIK")
            self.highest_balls_list.append(ball_height)
            # self.counter_list.append(self.counter)
            self.falling_flag = True
            # Counting height
            self.distance.append(abs(platform_height - ball_height))


        if self.decking_counter > self.prev_deck:
            # self.reward += self.decking_counter
            if self.decking_counter >=2:
                print('WOOOOW: ',self.decking_counter)
                self.reward += 100*self.decking_counter

        if done is False:
            if self.distance:
                self.reward += self.distance[-1]*0.00000001
            # print(self.reward)
        else:
            if self.decking_counter < 2:
                self.reward -= 1000
        #     print(self.reward)
        #     if self.decking_counter:
        #         self.reward -= .00001/self.decking_counter
        #     print("a")
        #     print(self.decking_counter)
        #     print(self.prev_deck)
        #     print(done)
        #     print(self.reward)
        #     print("b")
        # Getting the latest height of the ball
        # self.counter += 1
        self.previous_dist = ball_height
        self.prev_deck = self.decking_counter
        return self.reward, done


    # def step_train(self, action):
    #     done = False

    #     if self.diff('platform', 'ball', "x", "x"):
    #         done = True
    #         print("utolsó magas labda: ", self.highest_balls_list[-1])
    #         print("dekázás: ", self.decking_counter)

    #     self.sim.data.ctrl[:] = action
    #     self.sim.step()
    #     reward, self.previous_dist = self.reward_function(self.get_obs_my('platform', 2), self.get_obs_my('ball', 2))

    #     return self.get_observation(), reward, done, {}, self.highest_balls_list, self.all_ball_height_list, self.counter_list
  