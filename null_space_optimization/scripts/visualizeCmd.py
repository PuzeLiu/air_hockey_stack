import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
import torch
from iiwas_kinematics.kinematics_torch import KinematicsTorch

input_dir = os.path.abspath("/home/puze/Desktop")
file_name = "2020-12-30-17-51-48.bag"

desired_positions = []
desired_velocities = []
actual_positions = []
actual_velocities = []
error_positions = []
error_velocities = []
time = []

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_torque_trajectory_controller/state":
        desired_positions.append(msg.desired.positions)
        desired_velocities.append(msg.desired.velocities)
        actual_positions.append(msg.actual.positions)
        actual_velocities.append(msg.actual.velocities)
        error_positions.append(msg.error.positions)
        error_velocities.append(msg.error.velocities)
        time.append(msg.header.stamp.to_sec())

desired_positions = np.array(desired_positions)
desired_velocities = np.array(desired_velocities)
actual_positions = np.array(actual_positions)
actual_velocities = np.array(actual_velocities)
error_positions = np.array(error_positions)
error_velocities = np.array(error_velocities)


kinematics = KinematicsTorch(torch.tensor([0., 0., 0.515]).double(), torch.Tensor([1.0, 0., 0., 0.]).double())
for i in range(7):
    fig, axes = plt.subplots(2, 1)
    axes[0].plot(time, desired_positions[:, i], label="desired")
    axes[0].plot(time, actual_positions[:, i], label='actual')
    axes[1].plot(time, desired_velocities[:, i], label="desired")
    axes[1].plot(time, actual_velocities[:, i], label='actual')
    fig.suptitle("Joint " + str(i+1))


fig1, axes1 = plt.subplots(3, 1)
x_actual = []
x_desired = []

for i, pos_i in enumerate(actual_positions):
    x_i_actual = kinematics.forward_kinematics(torch.tensor(actual_positions[i]))
    x_i_desired = kinematics.forward_kinematics(torch.tensor(desired_positions[i]))
    x_actual.append(x_i_actual[:3].detach().numpy())
    x_desired.append(x_i_desired[:3].detach().numpy())

x_actual = np.array(x_actual)
x_desired = np.array(x_desired)
axes1[0].plot(time, x_desired[:, 0], label="desired")
axes1[0].plot(time, x_actual[:, 0], label="actual")

axes1[1].plot(time, x_actual[:, 1], label="desired")
axes1[1].plot(time, x_desired[:, 1], label="actual")
axes1[2].plot(time, x_actual[:, 2], label="desired")
axes1[2].plot(time, x_desired[:, 2], label="actual")
plt.legend()
fig1.suptitle("XYZ")

fig, axis = plt.subplots(1, 1)
axis.plot(x_desired[:, 0], x_desired[:, 1], label="desired")
axis.plot(x_actual[:, 0], x_actual[:, 1], label="actual")
axis.scatter(0.8, -0.2, c='r', marker='x')

plt.show()