import os.path

import numpy as np
import matplotlib.pyplot as plt
import rosbag

import pinocchio as pino
from scipy.signal import butter, filtfilt
from scipy.fft import fft

from manifold_planning.utils.manipulator import Iiwa
from manifold_planning.utils.feasibility import check_if_plan_valid


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
    # bv, av = butter(6, 4, fs=1000)
    # b, a = butter(6, 4, fs=1000)
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


def compute_vel_acc_tau(q, qd, qd_dot, qd_ddot):
    iiwa = Iiwa(iiwa_file)
    desired_torque = iiwa.rnea(qd, qd_dot, qd_ddot).numpy()[0]

    q, dq, ddq, torque, q_, dq_, ddq_, torque_ = get_vel_acc(t[:, np.newaxis], q, iiwa)
    return q, dq, ddq, torque, qd, qd_dot, qd_ddot, desired_torque


def compute_end_effector(pino_model, pino_data, q, qd):
    ee_pos_des = np.zeros((t.shape[0], 3))
    ee_pos_actual = np.zeros((t.shape[0], 3))
    frame_id = pino_model.getFrameId("F_striker_mallet")
    pino_positions = np.zeros(pino_model.nq)
    for i, _ in enumerate(t):
        pino_positions[:7] = qd[i]
        pino.forwardKinematics(pino_model, pino_data, pino_positions)
        pino.updateFramePlacement(pino_model, pino_data, frame_id)
        ee_pos_des[i] = pino_data.oMf[frame_id].translation.copy()

        pino_positions[:7] = q[i]
        pino.forwardKinematics(pino_model, pino_data, pino_positions)
        pino.updateFramePlacement(pino_model, pino_data, frame_id)
        ee_pos_actual[i] = pino_data.oMf[frame_id].translation.copy()
    return np.array(ee_pos_actual), np.array(ee_pos_des)


def detect_hitting_idx(puck, puck_time, t):
    vxpuck = np.diff(puck[:, 0], axis=0)
    vypuck = np.diff(puck[:, 0], axis=0)
    vpuck = np.sqrt(vxpuck ** 2 + vypuck ** 2)
    hit_idx = np.where(vpuck > 0.01)[0][0]
    puck_time_hit = puck_time[hit_idx] - puck_time[0]
    time_zeroed = t - t[0]
    time_hit_idx = np.argmin(np.abs(puck_time_hit - time_zeroed))
    return time_hit_idx, hit_idx


def check_if_scored(puck, puck_time, puck_time_hit_idx):
    x_in_range = np.logical_and(2.42 < puck[:, 0], puck[:, 0] < 2.44)
    y_in_range = np.logical_and(-0.125 < puck[:, 1], puck[:, 1] < 0.125)
    xy_in_range = np.logical_and(x_in_range, y_in_range)
    time_in_range = puck_time < puck_time[puck_time_hit_idx] + 2.
    if_scored = np.logical_and(xy_in_range, time_in_range)
    return np.any(if_scored)


def read_bag(bag):
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
    desired = np.array(joint_state_desired).astype(np.float32)
    actual = np.array(joint_state_actual).astype(np.float32)
    error = np.array(joint_state_error).astype(np.float32)
    puck_pose = np.array(puck_pose).astype(np.float32)
    t = np.array(time)
    puck_time = np.array(puck_time)
    t -= t[0]
    puck_time -= puck_time[0]
    t = t.astype(np.float32)
    puck_time = puck_time.astype(np.float32)
    return t, actual, desired, error, puck_pose, puck_time


root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
# bag_path = os.path.join(package_dir, "test_bspline_adrc_ff.bag")
# bag_path = os.path.join(package_dir, "bags/qddotend.bag")
bag_path = os.path.join(package_dir, "bags/straight_c1_t1_3.bag")
#bag_path = os.path.join(package_dir, "bags/tilted_c1_t1_3.bag")
bag_file = rosbag.Bag(bag_path)

robot_file = os.path.join(root_dir, "manifold_planning", "iiwa_striker_new.urdf")
iiwa_file = os.path.join(root_dir, "manifold_planning", "iiwa.urdf")
pino_model = pino.buildModelFromUrdf(robot_file)
pino_data = pino_model.createData()

result = {}

t, actual, desired, error, puck, puck_time = read_bag(bag_file)
q, dq, ddq, torque, qd, qd_dot, qd_ddot, torqued = compute_vel_acc_tau(actual[:, :7], desired[:, :7],
                                                                       desired[:, 7:14], desired[:, 14:21])
ee, ee_d = compute_end_effector(pino_model, pino_data, q, qd)

time_hit_idx, puck_time_hit_idx = detect_hitting_idx(puck, puck_time, t)

scored = check_if_scored(puck, puck_time, puck_time_hit_idx)
valid = check_if_plan_valid(ee_d, qd, qd_dot, qd_ddot, torqued)

result["scored"] = scored
result["valid"] = valid
print(result)

#for i in range(6):
#    plt.plot(t, qd_ddot[:, i], label=f"q_ddot_{i}")
#plt.legend()
#plt.show()


# plt.plot(puck[:, 0], puck[:, 1])
# plt.show()
