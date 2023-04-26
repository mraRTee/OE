import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import os
import time
import mujoco_py as mjp
from dm_control import suite,mujoco

class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
        frame_skip = 1
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)
        
    def reset(self):
        self.sim.reset()
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

        # This line is responsible to control with SINUS
        self.sim.data.ctrl[:] = action
        #self.sim.step()
        return self.get_observation(), 0, False, {}
    
    def reward_function(self, platform_height, ball_height):
        # Az állapot tartalmazza a labda magasságát és sebességét stb.
        distance_to_target = abs(platform_height - ball_height)
        diff = self.diff('ball', 'platform', "x", "x")

        if diff:
            reward = distance_to_target / platform_height
        else:
            reward = 0.01
        return reward

    def step_train(self, action):
        # This line is responsible to control with SINUS
        
        done = False
        if self.diff('platform', 'ball', "x", "x"):
            done = True
        self.sim.step()
        print(self.reward_function(self.get_obs_my('platform', 2), self.get_obs_my('ball', 2)))
        return self.get_observation(), 0, done, {}
    