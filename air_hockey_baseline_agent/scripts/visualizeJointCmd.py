import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import JointTrajectory
from iiwas_kinematics_py import Kinematics

input_dir = os.path.abspath("/home/puze/Desktop/real_trajectory")
file_name = "2021-03-01-20-34-33.bag"

command_positions = []
command_velocities = []
command_accelerations = []
time = []

kinematics = Kinematics(np.array([0., 0., 0.515]), np.array([0., 0., 0., 1.]))

bag = rosbag.Bag(os.path.join(input_dir, file_name))
t_start = bag.get_start_time()
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_position_trajectory_controller/command":
        msg: JointTrajectory
        positions_i = []
        velocities_i = []
        accelerations_i = []
        t_i = []
        for point in  msg.points:
            positions_i.append(point.positions)
            velocities_i.append(point.velocities)
            accelerations_i.append(point.accelerations)
            t_i.append(point.time_from_start.to_sec())

        positions_i = np.array(positions_i)
        velocities_i = np.array(velocities_i)
        accelerations_i = np.array(accelerations_i)
        t_i = np.array(t_i)

        command_positions.append(positions_i)
        command_velocities.append(velocities_i)
        command_accelerations.append(accelerations_i)
        time.append(t_i)

# for i in range(command_positions.__len__()):
i = 1
for j in range(7):
    fig, axes = plt.subplots(3)
    fig.suptitle("Joint_" + str(j + 1))
    axes[0].scatter(time[i], command_positions[i][:, j], s=3, label='desired')
    axes[0].legend()
    # axes[0].plot(time, error_positions[:, i])
    axes[1].scatter(time[i], command_velocities[i][:, j], s=3, label='desired')
    axes[1].legend()
    # axes[1].plot(time, error_velocities[:, i])
    # axes[2].scatter(time[i], command_accelerations[i][:, j], s=3, label='desired')
    # axes[2].scatter(time, actual_accelerations[:, i], s=3, label='actual')
    axes[2].legend()

plt.show()
