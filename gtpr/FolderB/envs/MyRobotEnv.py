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
        
    def step(self, action):
        # a1 = self.sim.model.get_joint_qpos_addr('A1')
        # a2 = self.sim.model.get_joint_qpos_addr('A2')
        # a3 = self.sim.model.get_joint_qpos_addr('A3')
        # sim_state = self.sim.get_state()
        # print(action[0])
        # if action[0]>500:
        #     sim_state.qpos[a1]=.4
        #     self.sim.set_state(sim_state)
        # else:
        #     sim_state.qpos[a1]=0
        #     self.sim.set_state(sim_state)

        print(self.sim.model.get_joint_qpos_addr('A1'))
        
        self.sim.step()
        return self.get_observation(), 0, False, {}
    
    def step_sinus(self, action):
        # This line is responsible to control with SINUS
        self.sim.data.ctrl[:] = action
        self.sim.step()
        return self.get_observation()
    