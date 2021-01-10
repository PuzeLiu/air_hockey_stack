import numpy as np
import matplotlib.pyplot as plt
import os
import rosbag
from trajectory_msgs.msg import MultiDOFJointTrajectory

input_dir = os.path.abspath("/home/puze/Desktop")
file_name = "2021-01-09-14-43-53.bag"

positions = []
velocities = []
accelerations = []
time = []

bag = rosbag.Bag(os.path.join(input_dir, file_name))
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/cartesian_trajectory":
        msg:MultiDOFJointTrajectory
        for i in range(msg.points.__len__()):
            positions.append([msg.points[i].transforms[0].translation.x, msg.points[i].transforms[0].translation.y,
                              msg.points[i].transforms[0].translation.z])
            velocities.append([msg.points[i].velocities[0].linear.x, msg.points[i].velocities[0].linear.y,
                              msg.points[i].velocities[0].linear.z])
            # accelerations.append([msg.points[i].accelerations[0].linear.x, msg.points[i].accelerations[0].linear.y,
            #                   msg.points[i].accelerations[0].linear.z])
            time.append(msg.header.stamp.to_sec() + msg.points[i].time_from_start.to_sec())

positions = np.array(positions)
velocities = np.array(velocities)
accelerations = np.array(accelerations)
time = np.array(time)

fig, axes = plt.subplots(3, 1)
axes[0].scatter(time, positions[:, 0], label="X", s=5)
axes[0].set_title("X")
axes[1].scatter(time, positions[:, 1], label="Y", s=5)
axes[1].set_title("Y")
axes[2].scatter(time, positions[:, 2], label="Z", s=5)
axes[2].set_title("Z")
fig.suptitle("Position")

fig1, axes1 = plt.subplots(3, 1)
axes1[0].scatter(time, velocities[:, 0], s=5)
axes1[0].set_title("X")
axes1[1].scatter(time, velocities[:, 1], s=5)
axes1[1].set_title("Y")
axes1[2].scatter(time, np.linalg.norm(velocities, axis=1), s=5)
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
axes3.scatter(positions[:, 0], positions[:, 1], s=5)

plt.show()