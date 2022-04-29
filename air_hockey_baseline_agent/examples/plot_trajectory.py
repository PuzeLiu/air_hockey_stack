import os
from copy import deepcopy
import rosbag
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import JointTrajectory
import pinocchio as pino

description_path = rospkg.RosPack().get_path("air_hockey_description")
urdf_file = os.path.join(description_path, "urdf/iiwa_striker.urdf")
pino_model = pino.buildModelFromUrdf(urdf_file)
pino_data = pino_model.createData()
pino_frame_id = pino_model.getFrameId("F_striker_tip")

qp_bag = rosbag.Bag("qp_opt.bag")

qp_cart_positions = []
qp_cart_velocities = []
qp_joint_positions = []
qp_joint_velocities = []
qp_cart_fk_positions = []
qp_time = []
for topic, msg, t in qp_bag.read_messages():
    if topic == "cartesian_trajectory":
        msg: MultiDOFJointTrajectory
        for i in range(msg.points.__len__()):
            qp_cart_positions.append([msg.points[i].transforms[0].translation.x, msg.points[i].transforms[0].translation.y,
                                msg.points[i].transforms[0].translation.z])
            qp_cart_velocities.append([msg.points[i].velocities[0].linear.x, msg.points[i].velocities[0].linear.y,
                                 msg.points[i].velocities[0].linear.z])
            qp_time.append(msg.points[i].time_from_start.to_sec())

        qp_cart_positions = np.array(qp_cart_positions)
        qp_cart_velocities = np.array(qp_cart_velocities)
        qp_time = np.array(qp_time)
    if topic == "joint_trajectory":
        msg: JointTrajectory
        for i in range(msg.points.__len__()):
            qp_joint_positions.append(msg.points[i].positions)
            qp_joint_velocities.append(msg.points[i].velocities)
            pino.forwardKinematics(pino_model, pino_data, np.array(msg.points[i].positions))
            qp_cart_fk_positions.append(deepcopy(pino.updateFramePlacement(pino_model, pino_data, pino_frame_id).translation))

        qp_joint_positions = np.array(qp_joint_positions)
        qp_joint_velocities = np.array(qp_joint_velocities)
        qp_cart_fk_positions = np.array(qp_cart_fk_positions)

aqp_bag = rosbag.Bag("aqp_opt.bag")
aqp_cart_positions = []
aqp_cart_velocities = []
aqp_joint_positions = []
aqp_joint_velocities = []
aqp_cart_fk_positions = []
aqp_time = []
for topic, msg, t in aqp_bag.read_messages():
    if topic == "cartesian_trajectory":
        msg: MultiDOFJointTrajectory
        for i in range(msg.points.__len__()):
            aqp_cart_positions.append([msg.points[i].transforms[0].translation.x, msg.points[i].transforms[0].translation.y,
                                      msg.points[i].transforms[0].translation.z])
            aqp_cart_velocities.append([msg.points[i].velocities[0].linear.x, msg.points[i].velocities[0].linear.y,
                                       msg.points[i].velocities[0].linear.z])
            aqp_time.append(msg.points[i].time_from_start.to_sec())
        aqp_cart_positions = np.array(aqp_cart_positions)
        aqp_cart_velocities = np.array(aqp_cart_velocities)
        aqp_time = np.array(aqp_time)
    if topic == "joint_trajectory":
        msg: JointTrajectory
        for i in range(msg.points.__len__()):
            aqp_joint_positions.append(msg.points[i].positions)
            aqp_joint_velocities.append(msg.points[i].velocities)
            pino.forwardKinematics(pino_model, pino_data, np.array(msg.points[i].positions))
            aqp_cart_fk_positions.append(deepcopy(pino.updateFramePlacement(pino_model, pino_data, pino_frame_id).translation))

        aqp_joint_positions = np.array(aqp_joint_positions)
        aqp_joint_velocities = np.array(aqp_joint_velocities)
        aqp_cart_fk_positions = np.array(aqp_cart_fk_positions)


fig, ax = plt.subplots(1)
ax.scatter(qp_cart_positions[:, 0], qp_cart_positions[:, 1], s=3, label='qp_plan')
ax.scatter(qp_cart_fk_positions[:, 0], qp_cart_fk_positions[:, 1], s=5, label='qp_opt')
ax.scatter(aqp_cart_positions[:, 0], aqp_cart_positions[:, 1], s=3, label='qp_plan')
ax.scatter(aqp_cart_fk_positions[:, 0], aqp_cart_fk_positions[:, 1], s=5, label='aqp_opt')
ax.legend()
ax.set_aspect(1)

fig, ax = plt.subplots(3)
ax[0].plot(qp_time, qp_cart_positions[:, 0])
ax[1].plot(qp_time, qp_cart_positions[:, 1])
ax[2].plot(qp_time, qp_cart_positions[:, 2])
ax[0].plot(aqp_time, aqp_cart_positions[:, 0])
ax[1].plot(aqp_time, aqp_cart_positions[:, 1])
ax[2].plot(aqp_time, aqp_cart_positions[:, 2])
fig.suptitle("Cartesian Positions")

fig, ax = plt.subplots(3)
ax[0].plot(qp_time, qp_cart_velocities[:, 0])
ax[1].plot(qp_time, qp_cart_velocities[:, 1])
ax[2].plot(qp_time, qp_cart_velocities[:, 2])
ax[0].plot(aqp_time, aqp_cart_velocities[:, 0])
ax[1].plot(aqp_time, aqp_cart_velocities[:, 1])
ax[2].plot(aqp_time, aqp_cart_velocities[:, 2])
fig.suptitle("Cartesian Velocity")

fig, ax = plt.subplots(4, 2)
for i in range(7):
    ax[int(i / 2), i % 2].plot(qp_time, qp_joint_positions[:, i], label='qp_joint')
    ax[int(i / 2), i % 2].plot(aqp_time, aqp_joint_positions[:, i], label='aqp_joint')
    ax[int(i / 2), i % 2].legend()
fig.suptitle("Joint Positions")

fig, ax = plt.subplots(4, 2)
for i in range(7):
    ax[int(i / 2), i % 2].plot(qp_time, qp_joint_velocities[:, i], label='qp_joint')
    ax[int(i / 2), i % 2].plot(aqp_time, aqp_joint_velocities[:, i], label='aqp_joint')
    ax[int(i / 2), i % 2].legend()
fig.suptitle("Joint Velocities")

plt.show()
