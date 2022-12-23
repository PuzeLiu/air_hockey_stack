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
                     "/iiwa_front/bspline_ff_joint_trajectory_controller/state",
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
#bag_path = os.path.join(package_dir, "bags/straight_c1_t1_3.bag")
#bag_path = os.path.join(package_dir, "bags/tilted_c1_t1.bag")
# bag_path = os.path.join(package_dir, "88_right.bag")
# bag_path = os.path.join(package_dir, "105_left.bag")
# bag_path = os.path.join(package_dir, "115_right.bag")
# bag_path = os.path.join(package_dir, "2022-07-27-15-48-47.bag")
#bag_path = os.path.join(package_dir, "132_right3.bag")
#bag_path = os.path.join(package_dir, "128_right3.bag")
# bag_path = os.path.join(package_dir, "b59_left1a.bag")
#bag_path = os.path.join(package_dir, "b59_right1a.bag")
#bag_path = os.path.join(package_dir, "neural/2022-08-01-17-43-17.bag")
bag_path = os.path.join(package_dir, "bags/ours_opt_lp/E0.bag")

mul = 1.

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
mass_matricies = []
energy = []
dkwii = []
ee_des_vxy = []
joint_idx = pino_model.getFrameId("F_striker_tip")
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
    J = pino.computeFrameJacobian(pino_model, pino_data, qi, joint_idx, pino.LOCAL_WORLD_ALIGNED)[:3, :6]
    ee_des_vxy.append(J @ desired[i, :6])
    centrifugal_coriolis.append(pino.nonLinearEffects(pino_model, pino_data, qi, dqi) - pino.nonLinearEffects(pino_model, pino_data, qi, np.zeros_like(dqi)))
    m = pino.crba(pino_model, pino_data, qi)[:6, :6]
    mass_matricies.append(m)
    dqtm = desired[i, 7:13, np.newaxis].T @ m
    energy.append((dqtm @ desired[i, 7:13, np.newaxis])[0, 0] / 2)
    dkwii.append([dqi[j] * dqtm[0, j] for j in range(6)])
    a = 0

centrifugal_coriolis = np.array(centrifugal_coriolis)
mass_matricies = np.stack(mass_matricies, axis=0)
energy = np.array(energy)
dkwii = np.array(dkwii)
ee_des_vxy = np.stack(ee_des_vxy, axis=0)

# detect hitting idx
vxpuck = np.diff(puck[:, 0], axis=0)
vypuck = np.diff(puck[:, 0], axis=0)
vpuck = np.sqrt(vxpuck ** 2 + vypuck ** 2)
hit_idx = np.where(vpuck > 0.01)[0][0]
puck_time_hit = puck_time[hit_idx] - puck_time[0]
puck_time_zeroed = puck_time - puck_time[0]
time_zeroed = t - t[0]
time_hit_idx = np.argmin(np.abs(puck_time_hit - time_zeroed))
#time_hit_idx = 0

plt.figure()
plt.plot(ee_pos_des[:, 0], mul * ee_pos_des[:, 1], 'b', label="desired")
plt.plot(ee_pos_actual[:, 0], mul * ee_pos_actual[:, 1], 'r', label="actual")
plt.plot(puck[:, 0], mul * puck[:, 1], 'g', label="puck")
plt.legend()
#plt.xlim(0.6, 1.1)
#plt.ylim(0.0, 0.5)
plt.show()
#assert False

ee_act_vx = np.diff(ee_pos_actual[:, 0], axis=0) / np.diff(t)
ee_act_vy = np.diff(ee_pos_actual[:, 1], axis=0) / np.diff(t)
ee_des_vx = np.diff(ee_pos_des[:, 0], axis=0) / np.diff(t)
ee_des_vy = np.diff(ee_pos_des[:, 1], axis=0) / np.diff(t)
puck_vx = np.diff(puck[:, 0], axis=0) / np.diff(puck_time)
puck_vy = np.diff(puck[:, 1], axis=0) / np.diff(puck_time)

plt.figure()
plt.subplot(131)
plt.plot(time_zeroed[:-1], ee_des_vx, label="desired_x")
plt.plot(time_zeroed[:-1], ee_act_vx, label="actual_x")
plt.plot(puck_time_zeroed[:-1], puck_vx, label="puck_x")
plt.legend()
plt.subplot(132)
plt.plot(time_zeroed[:-1], ee_des_vy, label="desired_y")
plt.plot(time_zeroed[:-1], ee_act_vy, label="actual_y")
plt.plot(puck_time_zeroed[:-1], puck_vy, label="puck_y")
plt.subplot(133)
#plt.plot(time_zeroed[:-1], np.abs(ee_des_vy) / (np.abs(ee_des_vx) + 1e-4), label="desired_y")
#plt.plot(time_zeroed[:-1], np.abs(ee_act_vy) / (np.abs(ee_act_vx) + 1e-4), label="actual_y")
plt.plot(time_zeroed, np.abs(ee_des_vxy[:, 1]) / (np.abs(ee_des_vxy[:, 0]) + 1e-4), label="actual_y")
plt.plot(puck_time_zeroed[:-1], np.abs(puck_vy) / (np.abs(puck_vx) + 1e-4), label="puck_y")
plt.legend()

plt.figure()
plt.subplot(121)
plt.plot(time_zeroed, ee_pos_des[:, 0], label="desired_x")
plt.plot(time_zeroed, ee_pos_actual[:, 0], label="actual_x")
plt.plot(puck_time_zeroed, puck[:, 0], label="puck_x")
plt.legend()
plt.subplot(122)
plt.plot(time_zeroed, ee_pos_des[:, 1], label="desired_y")
plt.plot(time_zeroed, ee_pos_actual[:, 1], label="actual_y")
plt.plot(puck_time_zeroed, puck[:, 1], label="puck_y")
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
    axes_1[int(i / 2), i % 2].plot(t, np.abs(error[:, i]), 'r', label=f"err_{i}")
    axes_1[int(i / 2), i % 2].plot(t[time_hit_idx], np.abs(error[time_hit_idx, i]), 'rx')
    axes_1[int(i / 2), i % 2].plot(t, 1e-3*np.abs(centrifugal_coriolis[:, i]), 'b', label=f"cc_{i}")
    axes_1[int(i / 2), i % 2].plot(t, 1e-3*np.abs(dkwii[:, i]), 'm', label=f"dkwii_{i}")
    axes_1[int(i / 2), i % 2].plot(t, 1e-3*np.abs(energy), 'g', label=f"energy")
    axes_1[int(i / 2), i % 2].set_title("joint_{}".format(i + 1))
    axes_1[int(i / 2), i % 2].legend()
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
