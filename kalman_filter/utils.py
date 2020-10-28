import os

import matplotlib.pyplot as plt


def makedirs(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def plot_trajectory(puck_pose, t, table_pose=None, idx=None):
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

    fig, axes = plt.subplots()
    axes.scatter(puck_pose[:, 0], puck_pose[:, 1], marker=".", s=10)
    axes.set_title("X - Y plane")
    if not table_pose is None:
        plt.scatter(table_pose[:, 0], table_pose[:, 1], color='r', marker='x')

    axes.set_xlim([0.45, 2.50])
    axes.set_ylim([0.35, 1.40])
    axes.set_aspect(1.0)
    plt.draw()
