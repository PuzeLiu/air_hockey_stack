import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from sensor_msgs.msg import JointState
import torch
from iiwas_kinematics import KinematicsTorch

input_dir = os.path.abspath("/home/puze/air_hockey_record")
file_name = "2021-01-25-19-21-08.bag"

actual_positions = []
actual_velocities = []
actual_cart_pos = []
time = []

kinematics = KinematicsTorch(torch.tensor([0., 0., 0.515]), torch.tensor([0., 0., 0., 1.]))

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_back/joint_states":
        msg:JointState
        if (len(msg.position) == 7):
            actual_positions.append(msg.position)
            actual_velocities.append(msg.velocity)
            time.append(msg.header.stamp.to_sec())
            actual_cart_pos.append(kinematics.forward_kinematics(torch.tensor(msg.position)).numpy()[:3])

actual_positions = np.array(actual_positions)
actual_velocities = np.array(actual_velocities)
actual_cart_pos = np.array(actual_cart_pos)
time = np.array(time)

for i in range(7):
    fig, axes = plt.subplots(2)
    fig.suptitle("Joint_" + str(i+1))
    axes[0].plot(time, actual_positions[:, i])
    axes[1].plot(time, actual_velocities[:, i])

fig, axes = plt.subplots(3)
fig.suptitle("Cartesian Position")
axes[0].plot(time, actual_cart_pos[:, 0])
axes[1].plot(time, actual_cart_pos[:, 1])
axes[2].plot(time, actual_cart_pos[:, 2])

plt.figure()
plt.plot(actual_cart_pos[:, 0], actual_cart_pos[:, 1])

plt.show()