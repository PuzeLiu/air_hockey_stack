import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import JointTrajectory
from iiwas_kinematics_py import Kinematics

input_dir = os.path.abspath("/home/puze/Desktop/real_trajectory")
file_name = "2021-03-01-20-24-54.bag"

desired_positions = []
desired_velocities = []
desired_accelerations = []
actual_positions = []
actual_velocities = []
actual_accelerations = []
error_positions = []
error_velocities = []
error_accelerations = []
desired_cart_pos = []
actual_cart_pos = []
time = []

kinematics = Kinematics(np.array([0., 0., 0.515]), np.array([0., 0., 0., 1.]))

bag = rosbag.Bag(os.path.join(input_dir, file_name))
t_start = bag.get_start_time()
i = 0
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_position_trajectory_controller/state":
        msg: JointTrajectory
        i += 1
        if(i%1 == 0):
            desired_positions.append(msg.desired.positions)
            desired_velocities.append(msg.desired.velocities)
            desired_accelerations.append(msg.desired.accelerations)
            actual_positions.append(msg.actual.positions)
            actual_velocities.append(msg.actual.velocities)
            actual_accelerations.append(msg.actual.accelerations)
            error_positions.append(msg.error.positions)
            error_velocities.append(msg.error.velocities)
            error_accelerations.append(msg.error.accelerations)
            time.append(msg.header.stamp.to_sec() - t_start)
            desired_cart_pos.append(kinematics.forward_kinematics(np.array(msg.desired.positions))[0])
            actual_cart_pos.append(kinematics.forward_kinematics(np.array(msg.actual.positions))[0])


desired_positions = np.array(desired_positions)
desired_velocities = np.array(desired_velocities)
desired_accelerations = np.array(desired_accelerations)
actual_positions = np.array(actual_positions)
actual_velocities = np.array(actual_velocities)
actual_accelerations = np.array(actual_accelerations)
error_positions = np.array(error_positions)
error_velocities = np.array(error_velocities)
error_accelerations = np.array(error_accelerations)
desired_cart_pos = np.array(desired_cart_pos)
actual_cart_pos = np.array(actual_cart_pos)
time = np.array(time)

for i in range(7):
    fig, axes = plt.subplots(3)
    fig.suptitle("Joint_" + str(i + 1))
    axes[0].scatter(time, desired_positions[:, i], s=3, label='desired')
    axes[0].scatter(time, actual_positions[:, i], s=3, label='actual')
    axes[0].legend()
    # axes[0].plot(time, error_positions[:, i])
    axes[1].scatter(time, desired_velocities[:, i], s=3, label='desired')
    axes[1].scatter(time, actual_velocities[:, i], s=3, label='actual')
    axes[1].legend()
    # axes[1].plot(time, error_velocities[:, i])
    axes[2].scatter(time, desired_accelerations[:, i], s=3, label='desired')
    # axes[2].scatter(time, actual_accelerations[:, i], s=3, label='actual')
    axes[2].legend()

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
