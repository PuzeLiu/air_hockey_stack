import os
import matplotlib.pyplot as plt
import numpy as np

def main():
    subfolder = "2020-10-22-19-45-13"
    input_dir = os.path.abspath(__file__ + "/../data/rosbag")

    file_id = 0

    fig, axes = plt.subplots()
    while os.path.exists(os.path.join(input_dir, subfolder, "{}.npy".format(file_id))):
        traj_i = np.load(os.path.join(input_dir, subfolder, "{}.npy".format(file_id)))
        velocity = (traj_i[1:, :2] - traj_i[:-1, :2]) / (traj_i[1:, -1:] - traj_i[:-1, -1:])
        axes.plot(np.linalg.norm(velocity, axis=1))
        file_id += 1
    plt.show()


if __name__ == "__main__":
    main()
