import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import mujoco_py
import os
import time


import mujoco_py as mjp



from dm_control import suite,mujoco


cnter=0

A1_qpos = 7
A2_qpos = 17
A3_qpos = 28

# Ez nem tudom mennyire igaz
A1_qvel = 6
A2_qvel = 16
A3_qvel = 27
class Static():
    cnt1=0
    cnt2=0
    x=4
    y=4
class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
    
        utils.EzPickle.__init__(self)
        FILE_PATH = '' # Absolute path to your .xml MuJoCo scene file 
                        # OR.
        FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
        frame_skip = 2
        #mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)
        
        model = mjp.load_model_from_path("/home/rot1bp/Desktop/work_custom_mujoco_project/OE/gtpr/FolderB/envs/compiled.xml")
        sim = mjp.MjSim(model)
        # másik mujoco az adatok kinyerésére
        #model = mujoco.MjModel.from_xml_path(FILE_PATH)
        #data = mujoco.MjData(model)

        
        #print(data.joint('A1').qpos) # ez működik, de ez a másik mujoco
        
# https://mujoco.readthedocs.io/en/2.1.2/python.html

    
    def move(self,y):
        print("move")
        #self.sim.data.qpos[A1_qpos] = 0.2
        #self.sim.data.qpos[A2_qpos] = 0.2
        #self.sim.data.qpos[A3_qpos] = 0.2
        self.sim.data.qvel[A1_qvel] = 0.5
        self.sim.data.qvel[A2_qvel] = 0.5
        self.sim.data.qvel[A3_qvel] = 0.5


    
    def zero(self,y):
        print("zero")
        #self.sim.data.qpos[A1_qpos] = 0
        #self.sim.data.qpos[A2_qpos] = 0
        #self.sim.data.qpos[A3_qpos] = 0
        self.sim.data.qvel[A1_qvel] = -0.5
        self.sim.data.qvel[A2_qvel] = -0.5
        self.sim.data.qvel[A3_qvel] = -0.5

    def step(self, a):
        #print(self.sim.get_state())
        #print(self.sim.model.joint_name2id('A1'))
        #print(self.sim.model.joint_name2id('A2'))
        #print(self.sim.model.joint_name2id('A3'))
        print(self.sim.model.get_joint_qvel_addr("A1"))
        print(self.sim.model.get_joint_qvel_addr("A2"))
        print(self.sim.model.get_joint_qvel_addr("A3"))
        #print("fasz")
        #print(self.data.get_body_xpos('A1'))

        #print(self.sim.model.jnt_qposadr[fasz])
        #input()
        # gpos és qvel-eket ki kéne találni
        vec_1 = self.get_body_com("ball") - self.get_body_com("A1")
        vec_2 = self.get_body_com("ball") - self.get_body_com("base_link")
        ball = self.get_body_com("ball")
        platform = self.get_body_com("WL1-1")
        A1 = self.get_body_com("A1")
        A2 = self.get_body_com("A2")
        A3 = self.get_body_com("A3")
        actuators_pos = self.sim.data.qpos[:]
        actuators_vel = self.sim.data.qvel[:]



        #notdone = False

        # REWARD? 
        reward_near = -np.linalg.norm(vec_1)
        reward_dist = -np.linalg.norm(vec_1)
        reward_ctrl = -np.square(a).sum()
        reward = reward_dist + 0.1 * reward_ctrl + 0.5 * reward_near
            
        if Static.cnt2%1000 == 0:
            Static.cnt1+=1
        if Static.cnt1 == len(self.sim.data.qvel):
            Static.cnt1 = 0
        Static.cnt2+=1
        if Static.cnt2<500:
            print(Static.cnt1)
            self.move(Static.cnt1)
        else:
            print(Static.cnt1)
            self.zero(Static.cnt1)
        if Static.cnt2==1000:
            Static.cnt2=0
        
        
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        # notdone until ball is over the platform
        print(ball)
        print(platform)
        notdone = (ball[2] >= platform[2])
        print(notdone)
        done = not notdone
        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

    # def reset_model(self):
    #     qpos = self.init_qpos

    #     self.goal_pos = np.asarray([0, 0])
    #     while True:
    #         self.cylinder_pos = np.concatenate(
    #             [
    #                 self.np_random.uniform(low=-0.3, high=0, size=1),
    #                 self.np_random.uniform(low=-0.2, high=0.2, size=1),
    #             ]
    #         )
    #         if np.linalg.norm(self.cylinder_pos - self.goal_pos) > 0.17:
    #             break

    #     qpos[-4:-2] = self.cylinder_pos
    #     qpos[-2:] = self.goal_pos
    #     qvel = self.init_qvel + self.np_random.uniform(
    #         low=-0.005, high=0.005, size=self.model.nv
    #     )
    #     qvel[-4:] = 0
    #     self.set_state(qpos, qvel)
    #     return self._get_obs()

    def reset_model(self):
         print("reset")
         qpos = self.init_qpos + self.np_random.uniform(
             size=self.model.nq, low=-0.1, high=0.1
         )
         qvel = self.init_qvel + self.np_random.standard_normal(self.model.nv) * 0.1
         self.set_state(qpos, qvel)
         return self._get_obs()

    def _get_obs(self):
        data = self.sim.data
        return np.concatenate(
            [
                data.qpos.flat[:7],
                data.qvel.flat[:7],
                self.get_body_com("A1"),
                self.get_body_com("A2"),
                self.get_body_com("ball"),
            ]
        )

#######################################################################################################################################


# import numpy as np
# import os
# from gym import utils, error, spaces
# from gym.envs.mujoco import mujoco_env
# from mujoco_py import MjViewer, functions


# cunt=0
# class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
#     def __init__(self):
#         utils.EzPickle.__init__(self)
#         FILE_PATH = '' # Absolute path to your .xml MuJoCo scene file 
#                         # OR.
#         FILE_PATH = os.path.join(os.path.dirname(__file__), "compiled.xml")
#         frame_skip = 5
#         mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)
#         self.A1 = 3
#         self.A2 = 17
#         self.A3 = 31



#     # self.sim.data.qpos[0:3] - labda
#     # self.sim.data.qpos[3] - A1 aktuátor
#     # self.sim.data.qpos[17] - A2
#     # self.sim.data.qpos[31] - A2
#     # self.sim.data.qpos[4] - teteje talán vagy az A1 platformnál lévő középrésze VAGY a munkaháromszög?
#     # self.sim.data.qpos[5] - valami káosz nem tom mi lehet 
#     def setter(self):
#         global cunt
#         cunt+=1
#         print("QPOS!!!!!!!!!!!!!!!!!!!!!!")
#         b = self.sim.data.qpos[3]
#         i=0
#         self.sim.data.qpos[0] = 0.4
#         self.sim.data.qpos[1] = 0.4
#         self.sim.data.qpos[2] = 0.4
#         if cunt<100:
#             self.sim.data.qpos[self.A1] = 0.4
#             self.sim.data.qpos[self.A2] = 0.4
#             self.sim.data.qpos[self.A3] = 0.4
#         if cunt>200:
#             self.sim.data.qpos[self.A1] = 0
#             self.sim.data.qpos[self.A1] = 0
#             self.sim.data.qpos[self.A1] = 0
#             if cunt==300:
#                 cunt=0
#         self.sim.data.ctrl[:]
        


#     def step(self, a):
#         # Carry out one step 
#         # Don't forget to do self.do_simulation(a, self.frame_skip)
#         xposbefore = self.get_body_com("base_link")[0]
#         self.do_simulation(a, self.frame_skip)
#         # xposafter = self.get_body_com("base_link")[0]
#         # forward_reward = (xposafter - xposbefore) / self.dt
#         # ctrl_cost = 0.5 * np.square(a).sum()
#         # contact_cost = (
#         #     0.5 * 1e-3 * np.sum(np.square(np.clip(self.sim.data.cfrc_ext, -1, 1)))
#         # )
#         # survive_reward = 1.0
#         # reward = forward_reward - ctrl_cost - contact_cost + survive_reward
#         # state = self.state_vector()
#         # notdone = np.isfinite(state).all() and state[2] >= 0.2 and state[2] <= 1.0
#         # done = not notdone
#         ob = self._get_obs()
#         reward = 0
#         done = False
#         return (
#             ob,
#             reward,
#             done,
#             # dict(
#             #     reward_forward=forward_reward,
#             #     reward_ctrl=-ctrl_cost,
#             #     reward_contact=-contact_cost,
#             #     reward_survive=survive_reward,
#             # ),
#             1,
#         )
#     def _get_obs(self):
#         return np.concatenate(
#             [
#                 self.sim.data.qpos.flat[2:],
#                 self.sim.data.qvel.flat,
#                 np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
#             ]
#         )
#     def reset_model(self):
#         print("reset")
#         input()
#         #qpos = self.init_qpos + self.np_random.uniform(
#         #    size=self.model.nq, low=-0.1, high=0.1
#         #)
#         #qvel = self.init_qvel + self.np_random.standard_normal(self.model.nv) * 0.1
#         #self.set_state(qpos, qvel)
#         return self._get_obs()
#     def viewer_setup(self):
#         self.viewer.cam.distance = self.model.stat.extent * 0.5