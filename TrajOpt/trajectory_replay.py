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
        self.q = np.load('generic_optimization/result/trajectory.npy')
        self.index = 0

    def draw_action(self, state):
        time.sleep(1/50.)
        q = self.q[self.index, 0]
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

states = np.empty((len(dataset), dataset[0][0].shape[0]))
actions = np.empty((len(dataset), dataset[0][1].shape[0]))
for i in range(len(dataset)):
    states[i] = dataset[i][0]
    actions[i] = dataset[i][1]
diff = states[1:, [10, 12, 14, 16, 18, 20, 22]] - actions[:-1]

t = np.arange(states.shape[0]) / 240.

import matplotlib.pyplot as plt
plt.figure()
plt.plot(t[1:], diff)
plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'])

plt.figure()
plt.plot(t, np.linalg.norm(states[:, [-3, -2]], axis=1))

plt.show()