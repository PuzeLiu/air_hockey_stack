import os
import rosbag
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import torch
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch

dir = os.path.abspath(__file__ + "/../data")
file = "2020-12-03-21-00-54"
bag = rosbag.Bag(os.path.join(dir, file + ".bag"))

tcp_pos = torch.tensor([0., 0., 0.5])
tcp_quat = torch.tensor([1., 0., 0., 0.])
kinematics = KinematicsTorch(tcp_pos=tcp_pos,
                             tcp_quat=tcp_quat)
t_0 = bag.get_start_time()
pose_actual = []
t_actual = []
pose_cmd = []
t_cmd = []
for topic, msg, t in bag.read_messages():
    if topic == "/iiwa_front/joint_states":
        q = torch.tensor(msg.position).double()
        x = kinematics.forward_kinematics(q).detach().numpy()
        pose_actual.append(x)
        t_actual.append(t.to_sec() - t_0)
    if topic == "/iiwa_front/joint_trajectory_controller/command":
        q_cmd = torch.tensor(msg.points[0].positions).double()
        x_cmd = kinematics.forward_kinematics(q_cmd).detach().numpy()
        pose_cmd.append(x_cmd)
        t_cmd.append(t.to_sec() - t_0)

pose_actual = numpy.array(pose_actual)
pose_cmd = numpy.array(pose_cmd)
t_actual = numpy.array(t_actual)
t_cmd = numpy.array(t_cmd)

fig, axes = plt.subplots(3)
axes[0].plot(t_actual[:], pose_actual[:, 0], label='actual')
axes[1].plot(t_actual[:], pose_actual[:, 1], label='actual')
axes[2].plot(t_actual[:], pose_actual[:, 2], label='actual')
axes[0].plot(t_cmd[:], pose_cmd[:, 0], label='desired')
axes[1].plot(t_cmd[:], pose_cmd[:, 1], label='desired')
axes[2].plot(t_cmd[:], pose_cmd[:, 2], label='desired')
plt.legend()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(pose_actual[:, 0], pose_actual[:, 1], pose_actual[:, 2], label='actual')
ax.plot(pose_cmd[:, 0], pose_cmd[:, 1], pose_cmd[:, 2], label='desired')
plt.legend()
plt.show()
