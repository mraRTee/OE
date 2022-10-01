import numpy as np
import os
from gym import utils, error, spaces
from gym.envs.mujoco import mujoco_env
from mujoco_py import MjViewer, functions


class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = '' # Absolute path to your .xml MuJoCo scene file 
                        # OR.
        FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
        frame_skip = 5
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)

    def step(self, a):
        # Carry out one step 
        # Don't forget to do self.do_simulation(a, self.frame_skip)
        xposbefore = self.get_body_com("base_link")[0]
        self.do_simulation(a, self.frame_skip)
        # xposafter = self.get_body_com("base_link")[0]
        # forward_reward = (xposafter - xposbefore) / self.dt
        # ctrl_cost = 0.5 * np.square(a).sum()
        # contact_cost = (
        #     0.5 * 1e-3 * np.sum(np.square(np.clip(self.sim.data.cfrc_ext, -1, 1)))
        # )
        # survive_reward = 1.0
        # reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        # state = self.state_vector()
        # notdone = np.isfinite(state).all() and state[2] >= 0.2 and state[2] <= 1.0
        # done = not notdone

        ob = self._get_obs()
        reward = 0
        done = False
        return (
            ob,
            reward,
            done,
            # dict(
            #     reward_forward=forward_reward,
            #     reward_ctrl=-ctrl_cost,
            #     reward_contact=-contact_cost,
            #     reward_survive=survive_reward,
            # ),
            1,
        )

    def _get_obs(self):
        return np.concatenate(
            [
                self.sim.data.qpos.flat[2:],
                self.sim.data.qvel.flat,
                np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
            ]
        )

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.1, high=0.1
        )
        qvel = self.init_qvel + self.np_random.standard_normal(self.model.nv) * 0.1
        self.set_state(qpos, qvel)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5