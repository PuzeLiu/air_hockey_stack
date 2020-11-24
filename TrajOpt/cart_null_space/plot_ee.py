import os
import rosbag
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import torch
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch

dir = os.path.abspath(__file__ + "/../data")
file = "2020-11-24-14-59-18"
bag = rosbag.Bag(os.path.join(dir, file + ".bag"))

tcp_pos = torch.tensor([0., 0., 0.59])
tcp_quat = torch.tensor([1., 0., 0., 0.])
kinematics = KinematicsTorch(tcp_pos=tcp_pos,
                             tcp_quat=tcp_quat)

pose = []

for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_states":
        q = torch.tensor(msg.position).double()
        x = kinematics.forward_kinematics(q).detach().numpy()
        pose.append(x)

pose = numpy.array(pose)

fig, axes = plt.subplots(3)
axes[0].plot(pose[:, 0])
axes[1].plot(pose[:, 1])
axes[2].plot(pose[:, 2])
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(pose[:, 0], pose[:, 1], pose[:, 2])
plt.show()
