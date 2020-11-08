import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def read_bag(bag):
    mallet_poses = []
    puck_poses = []
    table_poses = []
    count_table = 0
    count_puck = 0
    count_mallet = 0
    for topic, msg, t in bag.read_messages():

        t_start = bag.get_start_time()
        t_i = t.to_sec() - t_start
        if topic == 'tf':
            if msg.transforms[0].child_frame_id == "Mallet":
                count_mallet += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
                if len(mallet_poses) == 0 or not np.equal(np.linalg.norm(mallet_poses[-1][:2] - pose_i[:2]), 0):
                    mallet_poses.append(pose_i)

            elif msg.transforms[0].child_frame_id == "Puck":
                count_puck += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
                if len(puck_poses) == 0 or not np.equal(np.linalg.norm(puck_poses[-1][:2] - pose_i[:2]), 0):
                    puck_poses.append(pose_i)

            elif msg.transforms[0].child_frame_id == "Table":
                count_table += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
                if len(table_poses) == 0 or not np.equal(np.linalg.norm(table_poses[-1][2] - pose_i[2]), 0):
                    table_poses.append(pose_i)
    print("Found puck TF: {}, used: {}.".format(count_puck, len(puck_poses)))
    print("Found mallet TF: {}, used: {}.".format(count_mallet, len(mallet_poses)))
    print("Found table TF: {}, used: {}.".format(count_table, len(table_poses)))

    mallet_poses = np.array(mallet_poses)
    puck_poses = np.array(puck_poses)
    table_poses = np.array(table_poses)

    return mallet_poses, puck_poses, table_poses

def makedirs(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def read_data_from_dirs(dirs):
    data = []
    for dir in dirs:
        dir = os.path.join(os.path.abspath(__file__ + "../../data"), dir)
        file_id = 0
        while os.path.exists(os.path.join(dir, "traj_{}.csv".format(file_id))):
            traj_i = pd.read_csv(os.path.join(dir, "traj_{}.csv".format(file_id))).to_numpy()
            data.append(traj_i)
            file_id += 1
    return data

def plot_trajectory(puck_pose, t, table_pose=None, idx=None):
    # plot separate axis
    fig, axes = plt.subplots(3)
    axes[0].set_title("X - Direction")
    axes[1].set_title("Y - Direction")
    axes[2].set_title("Z - Direction")
    if not idx is None:
        for i in range(len(idx) - 1):
            axes[0].plot(puck_pose[idx[i]:idx[i + 1] + 1, -1], puck_pose[idx[i]:idx[i + 1] + 1, 0])
            axes[0].scatter(puck_pose[idx[i], -1], puck_pose[idx[i], 0], s=20)
            axes[0].text(puck_pose[idx[i], -1], puck_pose[idx[i], 0], str(idx[i]))

            axes[1].plot(puck_pose[idx[i]:idx[i + 1] + 1, -1], puck_pose[idx[i]:idx[i + 1] + 1, 1])
            axes[1].scatter(puck_pose[idx[i], -1], puck_pose[idx[i], 1], s=20)
            axes[1].text(puck_pose[idx[i], -1], puck_pose[idx[i], 1], str(idx[i]))

            axes[2].plot(puck_pose[idx[i]:idx[i + 1] + 1, -1], puck_pose[idx[i]:idx[i + 1] + 1, 2])
            axes[2].scatter(puck_pose[idx[i], -1], puck_pose[idx[i], 2], s=20)
            axes[2].text(puck_pose[idx[i], -1], puck_pose[idx[i], 2], str(idx[i]))
    else:
        axes[0].plot(puck_pose[:, -1], puck_pose[:, 0])
        axes[1].plot(puck_pose[:, -1], puck_pose[:, 1])
        axes[2].plot(puck_pose[:, -1], puck_pose[:, 2])

    axes[0].text(puck_pose[idx[-1], -1], puck_pose[idx[-1], 0], str(idx[-1]))
    axes[1].text(puck_pose[idx[-1], -1], puck_pose[idx[-1], 1], str(idx[-1]))
    axes[2].text(puck_pose[idx[-1], -1], puck_pose[idx[-1], 2], str(idx[-1]))

    # plot scatter in x-y plane
    fig, axes = plt.subplots()
    axes.scatter(puck_pose[:, 0], puck_pose[:, 1], marker=".", s=10)
    axes.set_title("X - Y plane")
    if not table_pose is None:
        plt.scatter(table_pose[:, 0], table_pose[:, 1], color='r', marker='x')

    # plot cutted trajectory
    fig, axes = plt.subplots()
    for i in range(len(idx) - 1):
        axes.plot(puck_pose[idx[i]: idx[i + 1] + 1, 0], puck_pose[idx[i]: idx[i + 1] + 1, 1])
        axes.scatter(puck_pose[idx[i], 0], puck_pose[idx[i], 1], marker=".", s=20)
        axes.text(puck_pose[idx[i], 0], puck_pose[idx[i], 1], str(idx[i]))
    axes.text(puck_pose[idx[-1], 0], puck_pose[idx[-1], 1], str(idx[-1]))
    axes.set_xlim([0.55, 2.70])
    axes.set_ylim([0.35, 1.40])
    axes.set_aspect(1.0)
    plt.draw()

def plot_velocity(puck_pose):
    diff = puck_pose[1:, :2] - puck_pose[:-1, :2]
    diff_t = puck_pose[1:, -1] - puck_pose[:-1, -1]
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
    plt.draw()
