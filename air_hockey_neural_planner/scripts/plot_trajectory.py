import os.path

import numpy as np
import matplotlib.pyplot as plt
import rosbag

import pinocchio as pino
from scipy.signal import butter, filtfilt
from scipy.fft import fft

from manifold_planning.utils.manipulator import Iiwa

def cal_d(fs, dts):
    dfs = np.zeros_like(fs)
    dfs[1: -1] = (fs[2:, :] - fs[:-2, :]) / (dts[2:] - dts[:-2])
    dfs[0] = dfs[1]
    dfs[-1] = dfs[-2]
    return dfs


def get_vel_acc(t, q_m, iiwa):
    # Position
    bv, av = butter(6, 40, fs=1000)
    b, a = butter(6, 40, fs=1000)
    bt, at = butter(6, 30, fs=1000)
    #bv, av = butter(6, 4, fs=1000)
    #b, a = butter(6, 4, fs=1000)
    q_m_filter = q_m

    # Velocity
    dq_m = cal_d(q_m, t)
    dq_m_filter = filtfilt(bv, av, dq_m.copy(), axis=0)

    # Acceleration
    ddq_m = cal_d(dq_m_filter, t)
    ddq_m_filter = filtfilt(b, a, ddq_m.copy(), axis=0)

    torque = iiwa.rnea(q_m.astype(np.float32), dq_m.astype(np.float32),
                       ddq_m.astype(np.float32)).numpy()[0]
    # Torque
    tau_m = torque
    tau_m_filter = filtfilt(bt, at, tau_m.copy(), axis=0)

    return q_m_filter, dq_m_filter, ddq_m_filter, tau_m_filter, q_m, dq_m, ddq_m, tau_m


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
bag_path = os.path.join(package_dir, "bags/straight_c1_t1_3.bag")
#bag_path = os.path.join(package_dir, "bags/tilted_c1_t1.bag")

bag_file = rosbag.Bag(bag_path)

robot_file = os.path.join(root_dir, "manifold_planning", "iiwa_striker_new.urdf")
iiwa_file = os.path.join(root_dir, "manifold_planning", "iiwa.urdf")
pino_model = pino.buildModelFromUrdf(robot_file)
pino_data = pino_model.createData()
pino_positions = np.zeros(pino_model.nq)
pino_velocities = np.zeros(pino_model.nq)

t, desired, actual, error, puck, puck_time = read_bag(bag_file, 10000)

iiwa = Iiwa(iiwa_file)
desired_torque = iiwa.rnea(desired[:, :7].astype(np.float32), desired[:, 7:14].astype(np.float32), desired[:, 14:21].astype(np.float32)).numpy()[0]


q, dq, ddq, torque, q_, dq_, ddq_, torque_ = get_vel_acc(t[:, np.newaxis], actual[:, :7], iiwa)

# applied_torque_estimate = []
ee_pos_des = np.zeros((t.shape[0], 3))
ee_pos_actual = np.zeros((t.shape[0], 3))
# frame_id = pino_model.getFrameId("F_striker_joint_link")
frame_id = pino_model.getFrameId("F_striker_mallet")
centrifugal_coriolis = []
for i, _ in enumerate(t):
    pino_positions[:7] = desired[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_des[i] = pino_data.oMf[frame_id].translation.copy()

    pino_positions[:7] = actual[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_actual[i] = pino_data.oMf[frame_id].translation.copy()

    #qi = np.pad(q[i], [[0, 2]])
    #dqi = np.pad(q[i], [[0, 2]])
    qi = np.pad(desired[i, :7], [[0, 2]])
    dqi = np.pad(desired[i, 7:14], [[0, 2]])
    centrifugal_coriolis.append(pino.nonLinearEffects(pino_model, pino_data, qi, dqi) - pino.nonLinearEffects(pino_model, pino_data, qi, np.zeros_like(dqi)))

centrifugal_coriolis = np.array(centrifugal_coriolis)

# detect hitting idx
vxpuck = np.diff(puck[:, 0], axis=0)
vypuck = np.diff(puck[:, 0], axis=0)
vpuck = np.sqrt(vxpuck ** 2 + vypuck ** 2)
hit_idx = np.where(vpuck > 0.01)[0][0]
puck_time_hit = puck_time[hit_idx] - puck_time[0]
time_zeroed = t - t[0]
time_hit_idx = np.argmin(np.abs(puck_time_hit - time_zeroed))


plt.figure()
plt.plot(ee_pos_des[:, 0], ee_pos_des[:, 1], 'b', label="desired")
plt.plot(ee_pos_actual[:, 0], ee_pos_actual[:, 1], 'r', label="actual")
plt.plot(puck[:, 0], puck[:, 1], 'g', label="puck")
plt.legend()
plt.show()

plt.figure()
plt.subplot(121)
plt.plot(t, ee_pos_des[:, 0], label="desired_x")
plt.plot(t, ee_pos_actual[:, 0], label="actual_x")
plt.subplot(122)
plt.plot(t, ee_pos_actual[:, 1], label="actual_y")
plt.plot(t, ee_pos_des[:, 1], label="desired_y")
plt.legend()

fig_1, axes_1 = plt.subplots(3, 2)
fig_2, axes_2 = plt.subplots(3, 2)
fig_3, axes_3 = plt.subplots(3, 2)
fig_4, axes_4 = plt.subplots(3, 2)
fig_5, axes_5 = plt.subplots(3, 2)
fig_6, axes_6 = plt.subplots(3, 2)
fig_1.suptitle("Positions errors")
fig_2.suptitle("Positions")
fig_3.suptitle("Velocities")
fig_4.suptitle("Accelerations")
fig_5.suptitle("Torques")
fig_6.suptitle("Centrifugal")


def plot_actual_desired(ax, i, actual, desired):
    ax[int(i / 2), i % 2].plot(t, actual[:, i], label="actual")
    ax[int(i / 2), i % 2].plot(t[time_hit_idx], actual[time_hit_idx, i], 'rx')
    ax[int(i / 2), i % 2].plot(t, desired[:, i], label="desired")
    ax[int(i / 2), i % 2].plot(t[time_hit_idx], desired[time_hit_idx, i], 'rx')
    ax[int(i / 2), i % 2].legend()
    ax[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))


for i in range(6):
    axes_1[int(i / 2), i % 2].plot(t, error[:, i])
    axes_1[int(i / 2), i % 2].plot(t[time_hit_idx], np.abs(error[time_hit_idx, i]), 'rx')
    axes_1[int(i / 2), i % 2].plot(t, 1e-3*np.abs(centrifugal_coriolis[:, i]))
    axes_1[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))
    axes_6[int(i / 2), i % 2].plot(t, centrifugal_coriolis[:, i])
    axes_6[int(i / 2), i % 2].plot(t[time_hit_idx], centrifugal_coriolis[time_hit_idx, i], 'rx')
    axes_6[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))
    plot_actual_desired(axes_2, i, q, desired[:, :7])
    plot_actual_desired(axes_3, i, dq, desired[:, 7:14])
    plot_actual_desired(axes_4, i, ddq, desired[:, 14:21])
    plot_actual_desired(axes_5, i, torque, desired_torque)
    axes_5[int(i / 2), i % 2].plot(t, centrifugal_coriolis[:, i] + torque[:, i])

#fig_6, axes_6 = plt.subplots(3, 2)
#fig_7, axes_7 = plt.subplots(3, 2)
#fig_6.suptitle("Velocities FFT")
#fig_7.suptitle("Accelerations FFT")
#idxs = np.where(np.logical_and(72.75 < t, t < 73.6))[0]
#for i in range(6):
#    axes_6[int(i / 2), i % 2].plot(fft(dq[idxs, i]), 'r', label="filtered")
#    axes_6[int(i / 2), i % 2].plot(fft(dq_[idxs, i]), 'g', label="unfiltered")
#    axes_6[int(i / 2), i % 2].plot(fft(desired[idxs, i+7]), 'b', label="desired")
#    axes_6[int(i / 2), i % 2].legend()
#    axes_6[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))
#    axes_7[int(i / 2), i % 2].plot(fft(ddq[idxs, i]), 'r', label="filtered")
#    axes_7[int(i / 2), i % 2].plot(fft(ddq_[idxs, i]), 'g', label="unfiltered")
#    axes_7[int(i / 2), i % 2].plot(fft(desired[idxs, i+14]), 'b', label="desired")
#    axes_7[int(i / 2), i % 2].legend()
#    axes_7[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))


plt.show()
