import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
import torch
from iiwas_kinematics.kinematics_torch import KinematicsTorch

input_dir = os.path.abspath(__file__ + "/..")
file_name = "2020-12-10-18-53-58.bag"

positions = []
velocities = []
accelerations = []
time = []

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/front_iiwa/joint_trajectory_controller/command":
        for i in range(len(msg.points)):
            positions.append(msg.points[i].positions)
            velocities.append(msg.points[i].velocities)
            accelerations.append(msg.points[i].accelerations)
            time.append(msg.points[i].time_from_start.to_sec())

positions = np.array(positions)
velocities = np.array(velocities)
accelerations = np.array(accelerations)
time = np.array(time)

print("total Points: ", time.shape)
kinematics = KinematicsTorch(torch.tensor([0., 0., 0.5]).double(), torch.Tensor([1.0, 0., 0., 0.]).double())
for i in range(7):
    fig, axes = plt.subplots(2, 1)
    axes[0].scatter(time, positions[:, i], s=2)
    axes[1].scatter(time, velocities[:, i], s=2)
    axes[1].plot(time, np.ones_like(time) * kinematics.joint_vel_limits[i, 0].detach().numpy(), 'r-.')
    axes[1].plot(time, np.ones_like(time) * kinematics.joint_vel_limits[i, 1].detach().numpy(), 'r-.')
    fig.suptitle("Joint " + str(i+1))


fig1, axes1 = plt.subplots(3, 1)
x = []
y = []
z = []
for i, pos_i in enumerate(positions):
    x_i = kinematics.forward_kinematics(torch.tensor(pos_i))
    x.append(x_i[0].item())
    y.append(x_i[1].item())
    z.append(x_i[2].item())
axes1[0].plot(time, x)
axes1[1].plot(time, y)
axes1[2].plot(time, z)
fig1.suptitle("XYZ")

fig, axis = plt.subplots(1, 1)
axis.plot(x, y)
axis.scatter(0.8, 0.2, c='r', marker='x')

plt.show()