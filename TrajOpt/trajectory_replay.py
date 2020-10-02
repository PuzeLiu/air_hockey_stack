import numpy as np
import torch
import quaternion
import time

from air_hockey.environments.iiwa_envs.air_hockey_env import AirHockeyBase
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch

from mushroom_rl.core import Core
from mushroom_rl.algorithms import Agent

class DummyTrajectoryExecuter(Agent):
    def __init__(self, n_actions):
        self._n_actions = n_actions
        tcp_pos = torch.tensor([0., 0., 0.3455]).double()
        tcp_quat = torch.tensor([0., 0., 0., 1.]).double()
        self.kinematics = KinematicsTorch(tcp_pos, tcp_quat)
        self.q = np.load('q.npy')
        self.index = 0

    def draw_action(self, state):
        # time.sleep(1/240.)
        q = self.q[self.index]
        if self.index < self.q.shape[0]-1:
            self.index += 1
        return q

    def episode_start(self):
        pass

    def fit(self, dataset):
        pass

env = AirHockeyBase(debug_gui=True)
agent = DummyTrajectoryExecuter(env.info.action_space.shape[0])

core = Core(agent, env)
dataset = core.evaluate(n_episodes=1, render=False)