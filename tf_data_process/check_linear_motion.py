import os
import matplotlib.pyplot as plt
# plt.switch_backend('agg')
import numpy as np
from scipy import signal
import pandas as pd

def main():
    subfolder = "2020-11-03-14-54-29"
    input_dir = os.path.abspath(__file__ + "/../data")
    output_dir = os.path.join(input_dir, subfolder)

    file_id = 0

    profiles = []

    # writer = SummaryWriter(output_dir)

    while os.path.exists(os.path.join(output_dir, "traj_{}.csv".format(file_id))):
        traj_i = pd.read_csv(os.path.join(output_dir, "traj_{}.csv".format(file_id))).to_numpy()
        velocity = (traj_i[1:, :2] - traj_i[:-1, :2]) / (traj_i[1:, -1:] - traj_i[:-1, -1:])
        vel_mag = np.linalg.norm(velocity, axis=1)

        if vel_mag.shape[0] > 15:
            b, a = signal.ellip(4, 0.1, 120, 0.125)
            vel_gust = signal.filtfilt(b, a, vel_mag, method="gust")

            sos = signal.butter(4, 0.125, output='sos')
            vel_sos = signal.sosfiltfilt(sos, vel_mag)

            noise = vel_sos - vel_mag

            profiles.append({"traj": traj_i, "vel": vel_mag, "gust": vel_gust, "sos": vel_sos, "noise": noise})

            fig = plt.figure()
            axes = fig.subplots(2)
            axes[0].plot(traj_i[1:, -1], vel_mag, label="origin")
            axes[0].plot(traj_i[1:, -1], vel_gust, 'b-', linewidth=4, label='gust')
            axes[0].plot(traj_i[1:, -1], vel_sos, 'r-', linewidth=1.5, label='sos')
            axes[0].set_title("Velocity Magnitude")

            mean_noise = np.mean(noise)
            axes[1].plot(traj_i[1:, -1], noise)
            axes[1].plot(traj_i[1:, -1], np.ones(noise.shape[0]) * mean_noise, 'r-', label="mean")
            axes[1].set_title("Noise, Variance: " + str(np.var(noise)))
            fig.suptitle("Velocity Profile {}-th Trajectory:".format(file_id))
            plt.show()
            # writer.add_figure('matplotlib', fig, file_id)

        file_id += 1

    fig, axes = plt.subplots(2)
    fig.suptitle("Velocity Profile with Filter")
    for profile in profiles:
        axes[0].plot(profile["traj"][1:, -1], profile["vel"], label="origin")
        axes[0].plot(profile["traj"][1:, -1], profile["gust"], 'b-', linewidth=4, label='gust')
        axes[0].plot(profile["traj"][1:, -1], profile["sos"], 'r-', linewidth=1.5, label='sos')

        noise = profile["noise"]
        mean_noise = np.mean(noise)
        axes[1].plot(profile["traj"][1:, -1], noise)
        axes[1].plot(profile["traj"][1:, -1], np.ones(noise.shape[0]) * mean_noise, 'r-', label="mean")
        fig.suptitle("Velocity Profile {}-th Trajectory:".format(file_id))
    plt.show()
    # writer.add_figure("Total", fig, file_id)
    # writer.close()


if __name__ == "__main__":
    main()
