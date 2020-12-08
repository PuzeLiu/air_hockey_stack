import os
import copy
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal
import rosbag

from utils import makedirs, plot_trajectory, read_bag, plot_velocity


def collision_detection(puck_pose_):
    check_steps = [1, 3, 5, 10]
    max_step = int(np.max(check_steps))
    detection_idx = []
    i = max_step
    while i < puck_pose_.shape[0]:
        count = 0
        for j in check_steps:
            i_prev = np.clip(i - j, 0, puck_pose_.shape[0] - 1)
            i_post = np.clip(i + j, 0, puck_pose_.shape[0] - 1)
            p_diff_prev = (puck_pose_[i] - puck_pose_[i_prev])[:2]
            p_diff_post = (puck_pose_[i_post] - puck_pose_[i])[:2]
            p_diff_prev_mag = np.linalg.norm(p_diff_prev)
            p_diff_post_mag = np.linalg.norm(p_diff_post)

            # check if start movement
            if p_diff_prev_mag < 1e-3 and p_diff_post_mag > 1e-3:
                count += 1
            elif p_diff_prev_mag * p_diff_post_mag > 1e-5 and \
                    np.dot(p_diff_prev, p_diff_post) / (p_diff_prev_mag * p_diff_post_mag) < np.cos(np.deg2rad(20)):
                count += 1
        if count >= 3:
            detection_idx.append(i)
        i += 1

    i = 1
    collision_idx = [0]
    point_collect = [detection_idx[0]]
    while i < len(detection_idx):
        if detection_idx[i] - detection_idx[i - 1] == 1:
            point_collect.append(detection_idx[i])
        else:
            collision_idx.append(int(np.mean(point_collect)))
            point_collect = [detection_idx[i]]
        i += 1
    collision_idx.append(int(np.mean(point_collect)))
    collision_idx.append(puck_pose_.shape[0] - 1)
    return collision_idx

def remove_outlier(traj):
    if traj.shape[0] > 16:
        sos = signal.butter(4, 0.125, output='sos')
        traj_filt = signal.sosfiltfilt(sos, traj[:, :2], axis=0)

        noise = traj[:, :2] - traj_filt
        noise_conf_interv = 3 * np.std(noise, axis=0)
        noise_mean = np.ones_like(noise) * np.mean(noise, axis=0)

        outlier_idx, = np.where(np.any(np.abs(noise) > noise_mean + noise_conf_interv, axis=1))
        traj = np.delete(traj, outlier_idx, axis=0)
        # print(outlier_idx)

        # fig, axes = plt.subplots(2, 2)
        # axes[0, 0].plot(traj[:, 0])
        # axes[0, 0].plot(traj_filt[:, 0])
        #
        # axes[1, 0].plot(noise[:, 0])
        # axes[1, 0].plot((noise_mean - noise_conf_interv)[:, 0], 'r-.')
        # axes[1, 0].plot((noise_mean + noise_conf_interv)[:, 0], 'r-.')
        #
        # axes[0, 1].plot(traj[:, 1])
        # axes[0, 1].plot(traj_filt[:, 1])
        #
        # axes[1, 1].plot(noise[:, 1])
        # axes[1, 1].plot((noise_mean - noise_conf_interv)[:, 1], 'r-.')
        # axes[1, 1].plot((noise_mean + noise_conf_interv)[:, 1], 'r-.')
        # plt.show()

    return traj


def cut_trajectory(puck_tf, output_dir, idx):
    idx = [0, 31, 61, 91, 191, 346, 356, 605, 635, 658]
    pd.DataFrame(puck_tf).to_csv(os.path.join(output_dir, "data"), index=False)
    count = 0
    for interv_idx in range(len(idx) - 1):
        traj_i = puck_tf[idx[interv_idx] + 3:idx[interv_idx + 1] - 3]
        if traj_i.shape[0] > 16:
            traj = remove_outlier(traj_i)
            pd.DataFrame(traj).to_csv(os.path.join(output_dir, "traj_{}.csv".format(count)), index=False)
            count += 1
    pd.DataFrame(idx).to_csv(os.path.join(output_dir, "cutting_point.csv"), index=False)


def main():
    input_dir = os.path.abspath(__file__ + "/../data")
    file_name = "2020-11-03-14-53-29"
    output_dir = os.path.join(input_dir, file_name)
    makedirs(output_dir)

    bag = rosbag.Bag(os.path.join(input_dir, file_name + ".bag"))

    mallet_tf, puck_tf, table_tf = read_bag(bag)

    idx = collision_detection(puck_tf)
    print("Detected Collision: \n", idx)
    plot_trajectory(puck_tf, puck_tf[:, -1], table_tf, idx)
    plt.show()

    cut_trajectory(puck_tf, output_dir, idx)

    ###################################################
    #            Plot Velocity magnitude              #
    ###################################################
    diff = puck_tf[1:, :2] - puck_tf[:-1, :2]
    diff_t = puck_tf[1:, -1] - puck_tf[:-1, -1]
    fig, axes = plt.subplots(3)
    axes[0].plot(diff[:, 0] / diff_t)
    axes[0].plot(np.zeros_like(diff[:, 0]), 'r-.')
    axes[0].set_title('X')
    axes[1].plot(diff[:, 1] / diff_t)
    axes[1].plot(np.zeros_like(diff[:, 0]), 'r-.')
    axes[1].set_title('y')
    axes[2].plot(np.linalg.norm(diff[:, :2], axis=-1) / diff_t)
    axes[2].plot(np.zeros_like(diff[:, 0]), 'r-.')
    axes[2].set_title('Magnitude')
    plt.show()



if __name__ == "__main__":
    main()



