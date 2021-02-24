import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import JointTrajectory
from iiwas_kinematics_py import Kinematics

input_dir = os.path.abspath("/home/puze/Desktop")
file_name = "2021-02-24-20-16-15.bag"

desired_positions = []
desired_velocities = []
actual_positions = []
actual_velocities = []
error_positions = []
error_velocities = []
desired_cart_pos = []
actual_cart_pos = []
time = []

kinematics = Kinematics(np.array([0., 0., 0.515]), np.array([0., 0., 0., 1.]))

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_torque_trajectory_controller/state":
        msg: JointTrajectory
        desired_positions.append(msg.desired.positions)
        desired_velocities.append(msg.desired.velocities)
        actual_positions.append(msg.actual.positions)
        actual_velocities.append(msg.actual.velocities)
        error_positions.append(msg.error.positions)
        error_velocities.append(msg.error.velocities)
        time.append(msg.header.stamp.to_sec())
        desired_cart_pos.append(kinematics.forward_kinematics(np.array(msg.desired.positions))[0])
        actual_cart_pos.append(kinematics.forward_kinematics(np.array(msg.actual.positions))[0])

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
    fig.suptitle("Joint_" + str(i + 1))
    axes[0].scatter(time, desired_positions[:, i], s=3, label='desired')
    axes[0].scatter(time, actual_positions[:, i], s=3, label='actual')
    axes[0].legend()
    # axes[0].plot(time, error_positions[:, i])
    axes[1].scatter(time, desired_velocities[:, i], s=3, label='desired')
    axes[1].scatter(time, actual_velocities[:, i], s=3, label='actual')
    axes[1].legend()
    # axes[1].plot(time, error_velocities[:, i])

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
