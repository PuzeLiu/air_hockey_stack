import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import MultiDOFJointTrajectory

input_dir = os.path.abspath("/home/default/bag_logs/real_trajectory/")
file_name = "2021-12-16-15-19-41.bag"

positions = []
velocities = []
time = []

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/cartesian_trajectory":
        msg:MultiDOFJointTrajectory
        positions_i = []
        velocities_i = []
        time_i = []
        for i in range(msg.points.__len__()):
            positions_i.append([msg.points[i].transforms[0].translation.x, msg.points[i].transforms[0].translation.y,
                              msg.points[i].transforms[0].translation.z])
            velocities_i.append([msg.points[i].velocities[0].linear.x, msg.points[i].velocities[0].linear.y,
                              msg.points[i].velocities[0].linear.z])
            # accelerations.append([msg.points[i].accelerations[0].linear.x, msg.points[i].accelerations[0].linear.y,
            #                   msg.points[i].accelerations[0].linear.z])
            time_i.append(msg.points[i].time_from_start.to_sec())

        positions_i = np.array(positions_i)
        velocities_i = np.array(velocities_i)
        time_i = np.array(time_i)

        positions.append(positions_i)
        velocities.append(velocities_i)
        time.append(time_i)


idx = 0
fig, axes = plt.subplots(3, 1)
axes[0].scatter(time[idx], positions[idx][:, 0], label="X", s=5)
axes[0].set_title("X")
axes[1].scatter(time[idx], positions[idx][:, 1], label="Y", s=5)
axes[1].set_title("Y")
axes[2].scatter(time[idx], positions[idx][:, 2], label="Z", s=5)
axes[2].set_title("Z")
fig.suptitle("Position")

fig1, axes1 = plt.subplots(3, 1)
axes1[0].scatter(time[idx], velocities[idx][:, 0], s=5)
axes1[0].set_title("X")
axes1[1].scatter(time[idx], velocities[idx][:, 1], s=5)
axes1[1].set_title("Y")
axes1[2].scatter(time[idx], np.linalg.norm(velocities[idx], axis=1), s=5)
axes1[2].set_title("Magnitude")
fig1.suptitle("Velocities")

# fig2, axes2 = plt.subplots(3, 1)
# axes2[0].scatter(time, accelerations[:, 0], s=5)
# axes2[0].set_title("X")
# axes2[1].scatter(time, accelerations[:, 1], s=5)
# axes2[1].set_title("Y")
# axes2[2].scatter(time, np.linalg.norm(accelerations, axis=1), s=5)
# axes2[2].set_title("Magnitude")
# fig2.suptitle("Acceleration")

fig3, axes3 = plt.subplots(1, 1)
axes3.scatter(positions[idx][:, 0], positions[idx][:, 1], s=5)
axes3.set_xlim([0.5, 1.5])
axes3.set_ylim([-0.45, 0.45])
axes3.set_aspect(aspect=1)

plt.show()