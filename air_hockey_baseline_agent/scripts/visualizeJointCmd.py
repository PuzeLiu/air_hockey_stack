import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag
from iiwas_kinematics_py import Kinematics
from trajectory_msgs.msg import JointTrajectory

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

def read_cmd(bag_dir, idx):
    bag = rosbag.Bag(bag_dir)
    t_start = bag.get_start_time()
    command_positions = []
    command_velocities = []
    command_accelerations = []
    time = []

    for topic, msg, t in bag.read_messages():
        if topic == "/iiwa_front/joint_position_trajectory_controller/command":
            msg: JointTrajectory
            positions_i = []
            velocities_i = []
            accelerations_i = []
            t_i = []
            for point in msg.points:
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
    return command_positions[idx], command_velocities[idx], command_accelerations[idx], time[idx]

def main():
    input_dir = os.path.abspath("/home/default/bag_logs/real_trajectory")
    file_opt = "2021-12-16-15-19-41.bag"
    #file_ik = "IK_3.bag"

    kinematics = Kinematics(np.array([0., 0., 0.515]), np.array([0., 0., 0., 1.]))
    pos_limit = np.array([170., 120., 170., 120., 170., 120., 175.])
    vel_limit = np.array([85., 85., 100., 75., 130., 135., 135.])

    pos_opt, vel_pot, _, t_opt = read_cmd(os.path.join(input_dir, file_opt), idx=0)
    #pos_ik, vel_ik, _, t_ik = read_cmd(os.path.join(input_dir, file_ik), idx=1)

    # for i in range(command_positions.__len__()):
    fig, axes = plt.subplots(2, 4, figsize=(16, 8), sharex=True, sharey=True)
    fig.delaxes(axes[1, 3])
    axes2 = np.empty((2, 4), dtype=object)
    for j in range(7):
        axes[j // 4, j % 4].plot(t_opt, np.rad2deg(pos_opt[:, j]), lw=3, color='tab:blue')
        #axes[j // 4, j % 4].plot(t_ik, np.rad2deg(pos_ik[:, j]), lw=3, color='tab:green')
        axes[j // 4, j % 4].plot([t_opt[0], t_opt[-1]], [-pos_limit[j], -pos_limit[j]], c='tab:red', ls='--', lw=2)
        axes[j // 4, j % 4].plot([t_opt[0], t_opt[-1]], [pos_limit[j], pos_limit[j]], c='tab:red', ls='--', lw=2)
        axes[j // 4, j % 4].set_yticks([-170, -120, 0, 120, 170])
        axes[j // 4, j % 4].set_yticklabels([-170, -120, 0, 120, 170], fontsize=20)
        axes[j // 4, j % 4].set_title('Joint ' + str(j + 1), fontsize=20)

        axes2[j // 4, j % 4] = axes[j // 4, j % 4].twinx()
        axes2[j // 4, j % 4].plot(t_opt, np.rad2deg(vel_pot[:, j]), lw=3, color='tab:orange')
        #axes2[j // 4, j % 4].plot(t_ik, np.rad2deg(vel_ik[:, j]), lw=3, color='tab:brown')
        axes2[j // 4, j % 4].plot([t_opt[0], t_opt[-1]], [-vel_limit[j], -vel_limit[j]], c='tab:purple', ls='--', lw=2)
        axes2[j // 4, j % 4].plot([t_opt[0], t_opt[-1]], [vel_limit[j], vel_limit[j]], c='tab:purple', ls='--', lw=2)

        if j == 3:
            axes2[j // 4, j % 4].set_ylim([-110 - 10, 110 + 10])
            axes2[j // 4, j % 4].set_yticks([-135, -100, -75, 0, 75, 100, 135])
            axes2[j // 4, j % 4].set_yticklabels([-135, -100, -75, 0, 75, 100, 135], fontsize=20)
            axes2[j // 4, j % 4].set_ylabel('velocity', fontsize=20)
        elif j == 6:
            axes2[j // 4, j % 4].set_ylim([-135 - 10, 135 + 10])
            axes2[j // 4, j % 4].set_yticks([-135, 0, 135])
            axes2[j // 4, j % 4].set_yticklabels([-135, 0, 135], fontsize=20)
            axes2[j // 4, j % 4].set_ylabel('velocity', fontsize=20)
        else:
            axes2[j // 4, j % 4].set_yticks([])
            axes2[j // 4, j % 4].set_yticklabels([])

    for j in range(1, 7):
        axes2[0, 0].get_shared_y_axes().join(axes2[0, 0], axes2[j // 4, j % 4])

    axes[0, 0].set_ylabel('position', fontsize=20)
    axes[1, 0].set_ylabel('position', fontsize=20)
    axes[0, 0].set_yticks([-170, -120, 0, 120, 170])
    axes[1, 0].set_yticklabels([-170, -120, 0, 120, 170], fontsize=20)
    axes[1, 0].set_xlabel('t', fontsize=20)
    axes[1, 1].set_xlabel('t', fontsize=20)
    axes[1, 2].set_xlabel('t', fontsize=20)
    plt.plot([], [], lw=3, color='tab:blue', label='Position Optimized')
    plt.plot([], [], lw=3, color='tab:orange', label='Velocity Optimized')
    plt.plot([], [], lw=3, color='tab:green', label='Position IK')
    plt.plot([], [], lw=3, color='tab:brown', label='Velocity IK')
    plt.plot([], [], lw=3, color='tab:red', label='Position Limit')
    plt.plot([], [], lw=3, color='tab:purple', label='Velocity Limit')
    plt.subplots_adjust(0.1, 0.1, 0.9, 0.9)
    plt.legend(loc='center left', bbox_to_anchor=(1.4, 0.5), fontsize=15)
    #plt.savefig("/home/puze/Dropbox/PHD/AirHockey/IROS/optimized_trajectory/" + "command.pdf")
    plt.show()


if __name__ == '__main__':
    main()