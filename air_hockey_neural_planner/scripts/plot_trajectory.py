import os.path

import numpy as np
import matplotlib.pyplot as plt
import rosbag

import pinocchio as pino
from scipy.signal import butter, filtfilt


def cal_d(fs, dts):
    dfs = np.zeros_like(fs)
    dfs[1: -1] = (fs[2:, :] - fs[:-2, :]) / (dts[2:] - dts[:-2])
    dfs[0] = dfs[1]
    dfs[-1] = dfs[-2]
    return dfs


def get_vel_acc(t, q_m):
    # Position
    b, a = butter(6, 4, fs=1000)
    q_m_filter = q_m

    # Velocity
    dq_m = cal_d(q_m, t)
    dq_m_filter = filtfilt(b, a, dq_m, axis=0)

    # Acceleration
    ddq_m = cal_d(dq_m_filter, t)
    ddq_m_filter = filtfilt(b, a, ddq_m, axis=0)

    return q_m_filter, dq_m_filter, ddq_m_filter, q_m, dq_m, ddq_m


def read_bag(bag, duration):
    time = []
    joint_state_desired = []
    joint_state_actual = []
    joint_state_error = []
    puck_pose = []
    puck_time = []

    for topic, msg, t_bag in bag.read_messages():
        if topic in ["/iiwa_front/adrc_trajectory_controller/state",
                     "/iiwa_front/bspline_adrc_joint_trajectory_controller/state",
                     "/iiwa_front/joint_feedforward_trajectory_controller/state"]:
            n_joints = len(msg.joint_names)
            time.append(msg.header.stamp.to_sec())
            if len(msg.desired.positions) != 7:
                msg.desired.positions += (0.,)
            joint_state_desired.append(np.concatenate(
                [msg.desired.positions, msg.desired.velocities, msg.desired.accelerations, msg.desired.effort]))
            joint_state_actual.append(np.concatenate(
                [msg.actual.positions, msg.actual.velocities, msg.actual.accelerations, msg.actual.effort]))
            joint_state_error.append(np.concatenate(
                [msg.error.positions, msg.error.velocities, msg.error.accelerations, msg.error.effort]))
        elif topic == "/tf":
            data = msg.transforms[0]
            frame_id = data.child_frame_id
            if frame_id == "Puck":
                t = data.transform.translation
                dx = 0.19
                dy = 0.813
                dz = 0.03
                puck_pose.append([t.x - dx, t.y - dy, t.z - dz])
                puck_time.append(data.header.stamp.to_sec())

    return np.array(time), np.array(joint_state_desired), np.array(joint_state_actual), np.array(joint_state_error), \
           np.array(puck_pose), np.array(puck_time)


root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
# bag_path = os.path.join(package_dir, "test_bspline_adrc_ff.bag")
# bag_path = os.path.join(package_dir, "bags/qddotend.bag")
bag_path = os.path.join(package_dir, "tilted_c1_t1.bag")

bag_file = rosbag.Bag(bag_path)

robot_file = os.path.join(root_dir, "manifold_planning", "iiwa_striker_new.urdf")
# robot_file = os.path.join(root_dir, "manifold_planning", "iiwa.urdf")
pino_model = pino.buildModelFromUrdf(robot_file)
pino_data = pino_model.createData()
pino_positions = np.zeros(pino_model.nq)
pino_velocities = np.zeros(pino_model.nq)

t, desired, actual, error, puck, puck_time = read_bag(bag_file, 10000)

q, dq, ddq, q_, dq_, ddq_ = get_vel_acc(t[:, np.newaxis], actual[:, :7])

# applied_torque_estimate = []
ee_pos_des = np.zeros((t.shape[0], 3))
ee_pos_actual = np.zeros((t.shape[0], 3))
# frame_id = pino_model.getFrameId("F_striker_joint_link")
frame_id = pino_model.getFrameId("F_striker_mallet")
for i, _ in enumerate(t):
    pino_positions[:7] = desired[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_des[i] = pino_data.oMf[frame_id].translation.copy()

    pino_positions[:7] = actual[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_actual[i] = pino_data.oMf[frame_id].translation.copy()

# detect hitting idx
vxpuck = np.diff(puck[:, 0], axis=0)
vypuck = np.diff(puck[:, 0], axis=0)
vpuck = np.sqrt(vxpuck ** 2 + vypuck ** 2)
hit_idx = np.where(vpuck > 0.01)[0][0]
puck_time_hit = puck_time[hit_idx] - puck_time[0]
time_zeroed = t - t[0]
time_hit_idx = np.argmin(np.abs(puck_time_hit - time_zeroed))

plt.plot(ee_pos_des[:, 0], ee_pos_des[:, 1], 'b', label="desired")
plt.plot(ee_pos_actual[:, 0], ee_pos_actual[:, 1], 'r', label="actual")
plt.plot(puck[:, 0], puck[:, 1], 'g', label="puck")
plt.show()

fig_1, axes_1 = plt.subplots(3, 2)
fig_2, axes_2 = plt.subplots(3, 2)
fig_3, axes_3 = plt.subplots(3, 2)
fig_4, axes_4 = plt.subplots(3, 2)
fig_1.suptitle("Positions errors")
fig_2.suptitle("Positions")
fig_3.suptitle("Velocities")
fig_4.suptitle("Accelerations")


def plot_actual_desired(ax, i, actual, desired):
    ax[int(i / 2), i % 2].plot(t, actual[:, i], label="actual")
    ax[int(i / 2), i % 2].plot(t[time_hit_idx], actual[time_hit_idx, i], 'rx')
    ax[int(i / 2), i % 2].plot(t, desired[:, i], label="desired")
    ax[int(i / 2), i % 2].plot(t[time_hit_idx], desired[time_hit_idx, i], 'rx')
    ax[int(i / 2), i % 2].legend()
    ax[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))


for i in range(6):
    axes_1[int(i / 2), i % 2].plot(t, error[:, i])
    axes_1[int(i / 2), i % 2].plot(t[time_hit_idx], error[time_hit_idx, i], 'rx')
    axes_1[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))
    plot_actual_desired(axes_2, i, q, desired[:, :7])
    plot_actual_desired(axes_3, i, dq, desired[:, 7:14])
    plot_actual_desired(axes_4, i, ddq, desired[:, 14:21])
#
# fig, axes = plt.subplots(3)
# for i in range(3):
#    axes[i].plot(t, ee_pos_des[:, i])
#    axes[i].plot(t, ee_pos_actual[:, i])
#
# fig, ax = plt.subplots(1)
# ax.plot(ee_pos_des[:, 0], ee_pos_des[:, 1])
# ax.plot(ee_pos_actual[:, 0], ee_pos_actual[:, 1])
#
plt.show()
