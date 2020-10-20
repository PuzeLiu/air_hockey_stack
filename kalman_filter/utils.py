import os
import matplotlib.pyplot as plt

def makedirs(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def plot_trajectory(pose, t):
    fig, axes = plt.subplots(3)
    axes[0].set_title("X - Direction")
    axes[0].plot(t, pose[:, 0])
    axes[1].set_title("Y - Direction")
    axes[1].plot(t, pose[:, 1])
    axes[2].set_title("Z - Direction")
    axes[2].plot(t, pose[:, 2])

    plt.figure()
    plt.plot(pose[:, 0], pose[:, 1])
    plt.title("X - Y plane")
    plt.show()

