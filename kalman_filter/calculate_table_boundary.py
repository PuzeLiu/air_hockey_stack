import os
import numpy as np
import matplotlib.pyplot as plt


def plot_bound(data, point):
    # 1:down, 2: right, 3: top, 4: left
    plt.figure()
    for data_i in data:
        plt.plot(data_i[:, 0], data_i[:, 1])
    plt.scatter(point[:, 0], point[:, 1])
    plt.show()


if __name__ == "__main__":
    parent_dir = os.path.abspath(__file__ + "/../data/rosbag")
    # can be 1, 2, 3, 4
    bound_id = 1
    traj_dir = os.path.join(parent_dir, "bound_{}".format(bound_id))

    id = 0
    data = []
    point = []
    while os.path.exists(os.path.join(traj_dir, "data_{}.npy".format(id))):
        data_i = np.load(os.path.join(traj_dir, "data_{}.npy".format(id)))
        data.append(data_i)

        if bound_id is 1:
            point_i = data_i[np.argmin(data_i[:, 1]), :2]
        elif bound_id is 2:
            point_i = data_i[np.argmax(data_i[:, 0]), :2]
        elif bound_id is 3:
            point_i = data_i[np.argmax(data_i[:, 1]), :2]
        elif bound_id is 4:
            point_i = data_i[np.argmin(data_i[:, 0]), :2]
        point.append(point_i)
        id += 1

    point = np.array(point)
    plot_bound(data, point)

