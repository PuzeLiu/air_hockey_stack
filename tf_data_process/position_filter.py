import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import pandas as pd

from utils import read_data_from_dirs

def main():
    dirs = ["2020-11-03-14-52-31",
            "2020-11-03-14-53-29",
            "2020-11-03-14-54-16",
            "2020-11-03-14-54-29"]

    data = read_data_from_dirs(dirs)

    profiles = []

    for i, traj_i in enumerate(data):
        pos_x = traj_i[:, 0]
        pos_y = traj_i[:, 1]

        if pos_x.shape[0] > 15:
            b, a = signal.ellip(4, 0.1, 120, 0.125)
            pos_gust_x = signal.filtfilt(b, a, pos_x, method="gust")
            pos_gust_y = signal.filtfilt(b, a, pos_y, method="gust")

            sos = signal.butter(4, 0.125, output='sos')
            pos_sos_x = signal.sosfiltfilt(sos, pos_x)
            pos_sos_y = signal.sosfiltfilt(sos, pos_y)

            noise_x = pos_sos_x - pos_x
            noise_y = pos_sos_y - pos_y

            profiles.append({"pos_x": pos_x, "pos_y": pos_y,
                             "pos_gust_x": pos_gust_x, "pos_gust_y": pos_gust_y,
                             "noise_x": noise_x, "noise_y": noise_y,
                             "pos_sos_x": pos_sos_x, "pos_sos_y": pos_sos_y})

            fig = plt.figure()
            axes = fig.subplots(2, 2)
            axes[0, 0].plot(traj_i[:, -1], pos_x, label="origin")
            axes[0, 0].plot(traj_i[:, -1], pos_gust_x, 'b-', linewidth=4, label='gust')
            axes[0, 0].plot(traj_i[:, -1], pos_sos_x, 'r-', linewidth=1.5, label='sos')
            axes[0, 0].set_title("X Direction")

            axes[0, 1].plot(traj_i[:, -1], pos_y, label="origin")
            axes[0, 1].plot(traj_i[:, -1], pos_gust_y, 'b-', linewidth=4, label='gust')
            axes[0, 1].plot(traj_i[:, -1], pos_sos_y, 'r-', linewidth=1.5, label='sos')
            axes[0, 1].set_title("Y Direction")

            mean_noise_x = np.mean(noise_x)
            axes[1, 0].plot(traj_i[:, -1], noise_x)
            axes[1, 0].plot(traj_i[:, -1], np.ones(noise_x.shape[0]) * mean_noise_x, 'r-', label="mean")
            axes[1, 0].set_title("Noise, Variance: " + str(np.var(noise_x)))
            fig.suptitle("X {}-th Trajectory:".format(i))

            mean_noise_y = np.mean(noise_y)
            axes[1, 1].plot(traj_i[:, -1], noise_y)
            axes[1, 1].plot(traj_i[:, -1], np.ones(noise_y.shape[0]) * mean_noise_y, 'r-', label="mean")
            axes[1, 1].set_title("Noise, Variance: " + str(np.var(noise_y)))
            fig.suptitle("Y {}-th Trajectory:".format(i))

            plt.show()

    noise_total_x = []
    noise_total_y = []
    for profile in profiles:
        noise_total_x.extend(noise_x)
        noise_total_y.extend(noise_y)

    var_x = np.var(noise_total_x)
    var_y = np.var(noise_total_y)
    print(var_x, var_y)


if __name__ == "__main__":
    main()