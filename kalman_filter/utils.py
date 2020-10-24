import os
import matplotlib.pyplot as plt

def makedirs(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def plot_trajectory(puck_pose, t, table_pose=None, mallet_pose=None):
    fig, axes = plt.subplots(3)
    axes[0].set_title("X - Direction")
    axes[0].plot(t, puck_pose[:, 0])
    axes[1].set_title("Y - Direction")
    axes[1].plot(t, puck_pose[:, 1])
    axes[2].set_title("Z - Direction")
    axes[2].plot(t, puck_pose[:, 2])

    plt.figure()
    plt.plot(puck_pose[:, 0], puck_pose[:, 1])
    plt.title("X - Y plane")
    if not table_pose is None:
        plt.scatter(table_pose[:, 0], table_pose[:, 1], color='r', marker='x')

    plt.xlim([0.45, 2.40])
    plt.ylim([0.35, 1.40])

    plt.show()

