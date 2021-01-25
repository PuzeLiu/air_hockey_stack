import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import JointTrajectory
import torch
from iiwas_kinematics import KinematicsTorch

input_dir = os.path.abspath("/home/puze/air_hockey_record")
file_name = "2021-01-25-19-21-08.bag"

desired_positions = []
desired_velocities = []
actual_positions = []
actual_velocities = []
error_positions = []
error_velocities = []
desired_cart_pos = []
actual_cart_pos = []
time = []

kinematics = KinematicsTorch(torch.tensor([0., 0., 0.515]), torch.tensor([0., 0., 0., 1.]))

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_back/joint_position_trajectory_controller/state":
        msg:JointTrajectory
        desired_positions.append(msg.desired.positions)
        desired_velocities.append(msg.desired.velocities)
        actual_positions.append(msg.actual.positions)
        actual_velocities.append(msg.actual.velocities)
        error_positions.append(msg.error.positions)
        error_velocities.append(msg.error.velocities)
        time.append(msg.header.stamp.to_sec())
        desired_cart_pos.append(kinematics.forward_kinematics(torch.tensor(msg.desired.positions)).numpy()[:3])
        actual_cart_pos.append(kinematics.forward_kinematics(torch.tensor(msg.actual.positions)).numpy()[:3])

desired_positions = np.array(desired_positions)
desired_velocities = np.array(desired_velocities)
actual_positions = np.array(actual_positions)
actual_velocities = np.array(actual_velocities)
error_positions = np.array(error_positions)
error_velocities = np.array(error_velocities)
desired_cart_pos = np.array(desired_cart_pos)
actual_cart_pos = np.array(actual_cart_pos)
time = np.array(time)

for i in range(7):
    fig, axes = plt.subplots(2)
    fig.suptitle("Joint_" + str(i+1))
    axes[0].plot(time, desired_positions[:, i])
    axes[0].plot(time, actual_positions[:, i])
    axes[0].plot(time, error_positions[:, i])
    axes[1].plot(time, desired_velocities[:, i])
    axes[1].plot(time, actual_velocities[:, i])
    axes[1].plot(time, error_velocities[:, i])

fig, axes = plt.subplots(3)
fig.suptitle("Cartesian Position")
axes[0].plot(time, desired_cart_pos[:, 0])
axes[0].plot(time, actual_cart_pos[:, 0])
axes[1].plot(time, desired_cart_pos[:, 1])
axes[1].plot(time, actual_cart_pos[:, 1])
axes[2].plot(time, desired_cart_pos[:, 2])
axes[2].plot(time, actual_cart_pos[:, 2])

plt.figure()
plt.plot(desired_cart_pos[:, 0], desired_cart_pos[:, 1])
plt.plot(actual_cart_pos[:, 0], actual_cart_pos[:, 1])

plt.show()