import numpy as np
import rospy
import rosbag
import os
import matplotlib.pyplot as plt


def plot_innovation(innovation):
    fig, axes = plt.subplots(2, 2)
    axes[0, 0].plot(innovation[:, 0])
    axes[0, 0].set_title("Innovation X")
    axes[0, 1].plot(innovation[:, 1])
    axes[0, 1].set_title("Innovation Y")

    axes[1, 0].plot(innovation[:, 2])
    axes[1, 0].plot()
    axes[1, 0].set_title("Innovation Normalized Squared")

    # Calculate auto correlation
    autocorrelation = []
    for tau in range(100):
        count = 0
        sum = 0
        for k in range(innovation.shape[0] - tau):
            count += 1
            sum += np.dot(innovation[tau, :2], innovation[tau+k, :2])
        autocorrelation.append(sum / count)

    axes[1, 1].plot(autocorrelation)
    axes[1, 1].set_title("Innovation whitenoise (autocorrelation)")

    plt.show()

def main():
    file_name = "2020-11-03-14-54-29" + ".bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../../data"), file_name)
    bag = rosbag.Bag(file_path)
    innovation = []
    for topic, msg, t in bag.read_messages():
        innovation.append([msg.x, msg.y, msg.normalized])

    innovation = np.array(innovation)
    plot_innovation(innovation)

if __name__ == "__main__":
    main()
