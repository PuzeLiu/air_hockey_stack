import os
import os.path
import pickle
import numpy as np
import matplotlib.pyplot as plt
import rosbag
from glob import glob

import pinocchio as pino
from scipy.signal import butter, filtfilt
from scipy.fft import fft
from scipy.interpolate import interp1d

from manifold_planning.utils.manipulator import Iiwa
from manifold_planning.utils.feasibility import check_if_plan_valid, compute_cartesian_losses


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


def compute_vel_acc_tau(iiwa_file, t, q, qd, qd_dot, qd_ddot):
    iiwa = Iiwa(iiwa_file)
    desired_torque = iiwa.rnea(qd, qd_dot, qd_ddot).numpy()[0]

    q, dq, ddq, torque, q_, dq_, ddq_, torque_ = get_vel_acc(t[:, np.newaxis], q, iiwa)
    return q, dq, ddq, torque, qd, qd_dot, qd_ddot, desired_torque


def compute_end_effector(pino_model, pino_data, q, qd):
    ee_pos_des = np.zeros((q.shape[0], 3))
    ee_pos_actual = np.zeros((q.shape[0], 3))
    frame_id = pino_model.getFrameId("F_striker_mallet")
    pino_positions = np.zeros(pino_model.nq)
    for i, _ in enumerate(q):
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
    nonzero_vel = np.where(vpuck > 0.01)
    if not len(nonzero_vel[0]):
        return None, None
    hit_idx = nonzero_vel[0][0]
    puck_time_hit = puck_time[hit_idx] - puck_time[0]
    time_zeroed = t - t[0]
    time_hit_idx = np.argmin(np.abs(puck_time_hit - time_zeroed))
    return time_hit_idx, hit_idx


def check_if_scored(puck, puck_time, puck_time_hit_idx):
    time_in_range = np.logical_and(puck_time[puck_time_hit_idx] < puck_time, puck_time < puck_time[puck_time_hit_idx] + 1.5)
    puck_x = puck[time_in_range, 0]
    puck_y = puck[time_in_range, 1]
    xs = interp1d(np.linspace(0., 1., len(puck_x)), puck_x)
    ys = interp1d(np.linspace(0., 1., len(puck_y)), puck_y)
    x = xs(np.linspace(0., 1., 300))
    y = ys(np.linspace(0., 1., 300))
    x_in_range = np.logical_and(2.42 < x, x < 2.44)
    #x_in_range = np.logical_and(2.435 < puck[time_in_range, 0], puck[time_in_range, 0] < 2.44)
    #y_in_range = np.logical_and(-0.125 < puck[:, 1], puck[:, 1] < 0.125)
    #y_in_range = np.logical_and(-0.09335 < puck[time_in_range, 1], puck[time_in_range, 1] < 0.09335)
    #y_in_range = np.logical_and(-0.09335 < y, y < 0.09335)
    y_in_range = np.logical_and(-0.105 < y, y < 0.105)
    #y_in_range = np.logical_and(-0.125 < y, y < 0.125)
    vx_sign = np.diff(x, axis=0) > 0
    xy_in_range = np.logical_and(x_in_range, y_in_range)
    if_scored = np.logical_and(xy_in_range[1:], vx_sign)
    #if_scored = np.logical_and(x_in_range, y_in_range)

    #print("SEP")
    #print(np.any(xy_in_range))
    #print(np.any(if_scored))
    #plt.plot(x, y)
    #plt.show()
    return np.any(if_scored)


def compute_puck_velocity(puck, puck_time, puck_time_hit_idx):
    xyz = puck[puck_time_hit_idx:]
    t = puck_time[puck_time_hit_idx:]
    vxy = np.diff(xyz[:, :2], axis=0)
    vth = np.arctan2(vxy[:, 1], vxy[:, 0])
    dt = np.diff(t)
    v = np.sqrt(np.sum(vxy ** 2, axis=-1)) / (dt + 1e-8)
    # plt.subplot(131)
    # plt.plot(xyz[:, 0], xyz[:, 1])
    # plt.subplot(132)
    # plt.plot(t[1:], v)
    # plt.subplot(133)
    # plt.plot(t[1:], vth)
    # plt.show()
    v_mag = np.median(v[:10])
    v_th = np.median(vth[:10])
    return v_mag, v_th


def compute_planned_velocity(velocity):
    v_mag = np.sqrt(np.sum(velocity ** 2))
    v_angle = np.arctan2(velocity[1], velocity[0])
    return v_mag, v_angle


def compute_movement_and_hitting_duration(dq, t, time_hit_idx):
    dq_v = np.sum(np.abs(dq), axis=-1)
    moving_idx = np.where(dq_v > 0.02)[0]
    t0 = t[moving_idx[0]]
    tf = t[moving_idx[-1]]
    th = t[time_hit_idx]
    movement_duration = tf - t0
    hitting_duration = th - t0
    return movement_duration, hitting_duration


def read_bag(bag):
    time = []
    joint_state_desired = []
    joint_state_actual = []
    joint_state_error = []
    puck_pose = []
    puck_time = []
    planned_hit_cartesian_velocity = None
    planned_hit_joint_velocity = None
    planning_time = None
    success = None
    desired_hit_angle = None
    desired_hit_point = None
    planned_hitting_time = None
    planned_motion_time = None

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
        elif topic == "/neural_planner/status":
            planned_hit_cartesian_velocity = np.array(msg.planned_hit_cartesian_velocity)
            planned_hit_joint_velocity = np.array(msg.planned_hit_joint_velocity)
            planning_time = msg.planning_time
            planned_hitting_time = msg.planned_hitting_time
            planned_motion_time = msg.planned_motion_time
            success = msg.success
        elif topic == "/neural_planner/plan_trajectory":
            desired_hit_angle = msg.hit_angle
            hp = msg.hit_point
            desired_hit_point = np.array([hp.x, hp.y, hp.z])
    desired = np.array(joint_state_desired).astype(np.float32)
    actual = np.array(joint_state_actual).astype(np.float32)
    error = np.array(joint_state_error).astype(np.float32)
    puck_pose = np.array(puck_pose).astype(np.float32)
    t = np.array(time)
    puck_time = np.array(puck_time)
    t_off = np.minimum(t[0], puck_time[0])
    #t -= t_off
    #puck_time -= t_off
    t -= t[0]
    puck_time -= puck_time[0]
    t = t.astype(np.float32)
    puck_time = puck_time.astype(np.float32)
    return dict(t=t, actual=actual, desired=desired, error=error, puck_pose=puck_pose, puck_time=puck_time,
                planning_time=planning_time, planned_hit_cartesian_velocity=planned_hit_cartesian_velocity,
                planned_hit_joint_velocity=planned_hit_joint_velocity, desired_hit_angle=desired_hit_angle,
                desired_hit_point=desired_hit_point, success=success, planned_hitting_time=planned_hitting_time,
                planned_motion_time=planned_motion_time)


root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
# bag_path = os.path.join(package_dir, "test_bspline_adrc_ff.bag")
# bag_path = os.path.join(package_dir, "bags/qddotend.bag")
# bag_path = os.path.join(package_dir, "bags/straight_c1_t1_3.bag")
# bag_path = os.path.join(package_dir, "bags/tilted_c1_t1_3.bag")
#bag_path = os.path.join(package_dir, "test.bag")



def compute_metrics(bag_path):
    #bag_path = os.path.join(package_dir, "bags", bag_path)
    #bag_path = os.path.join(package_dir, "bags/ours_nn/K9.bag")
    bag_file = rosbag.Bag(bag_path)

    robot_file = os.path.join(root_dir, "manifold_planning", "iiwa_striker_new.urdf")
    iiwa_file = os.path.join(root_dir, "manifold_planning", "iiwa.urdf")
    pino_model = pino.buildModelFromUrdf(robot_file)
    pino_data = pino_model.createData()

    result = {}

    bag_dict = read_bag(bag_file)
    q, dq, ddq, torque, qd, qd_dot, qd_ddot, torqued = compute_vel_acc_tau(iiwa_file,
                                                                           bag_dict["t"],
                                                                           bag_dict["actual"][:, :7],
                                                                           bag_dict["desired"][:, :7],
                                                                           bag_dict["desired"][:, 7:14],
                                                                           bag_dict["desired"][:, 14:21])
    plt.plot(bag_dict["t"], qd_dot[..., :2], 'r')
    plt.plot(bag_dict["t"], dq[..., :2], 'm')
    moving = np.linalg.norm(qd_dot, axis=-1) > 1e-2
    q = q[moving]
    dq = dq[moving]
    ddq = ddq[moving]
    torque = torque[moving]
    qd = qd[moving]
    qd_dot = qd_dot[moving]
    qd_ddot = qd_ddot[moving]
    torqued = torqued[moving]
    t = bag_dict["t"][moving]
    plt.plot(t, qd_dot[..., :2], 'b')
    plt.plot(t, dq[..., :2], 'g')
    plt.show()
    ee, ee_d = compute_end_effector(pino_model, pino_data, q, qd)

    time_hit_idx, puck_time_hit_idx = detect_hitting_idx(bag_dict["puck_pose"], bag_dict["puck_time"], bag_dict["t"])
    if time_hit_idx is None:
        result["puck_initial_pose"] = np.median(bag_dict["puck_pose"][:10], axis=0)
        result["scored"] = 0
        result["valid"] = 0
        return result
    movement_time, hitting_time = compute_movement_and_hitting_duration(dq, bag_dict["t"], time_hit_idx)

    puck_velocity_magnitude, puck_velocity_angle = compute_puck_velocity(bag_dict["puck_pose"], bag_dict["puck_time"],
                                                                         puck_time_hit_idx)
    planned_hit_velocity_magnitude, planned_hit_velocity_angle = compute_planned_velocity(
        bag_dict["planned_hit_cartesian_velocity"])
    scored = check_if_scored(bag_dict["puck_pose"], bag_dict["puck_time"], puck_time_hit_idx)
    valid = check_if_plan_valid(ee_d, qd, qd_dot, qd_ddot, torqued)
    desired_z_loss, desired_xlow_loss, desired_xhigh_loss, desired_ylow_loss, desired_yhigh_loss = compute_cartesian_losses(
        ee_d)
    actual_z_loss, actual_xlow_loss, actual_xhigh_loss, actual_ylow_loss, actual_yhigh_loss = compute_cartesian_losses(ee)

    result["scored"] = int(scored)
    result["valid"] = int(valid)
    result["planning_time"] = bag_dict["planning_time"]
    result["planned_hitting_time"] = bag_dict["planned_hitting_time"]
    result["planned_motion_time"] = bag_dict["planned_motion_time"]
    result["actual_hitting_time"] = hitting_time
    result["actual_motion_time"] = movement_time
    result["puck_initial_pose"] = np.median(bag_dict["puck_pose"][:10], axis=0)

    result["puck_velocity_magnitude"] = puck_velocity_magnitude
    result["planned_puck_velocity_magnitude"] = planned_hit_velocity_magnitude
    result["puck_actual_vs_planned_velocity_magnitude_error"] = puck_velocity_magnitude - planned_hit_velocity_magnitude

    result["puck_actual_vs_planned_velocity_angle_error"] = puck_velocity_angle - planned_hit_velocity_angle
    result["puck_planned_vs_desired_angle_error"] = bag_dict["desired_hit_angle"] - planned_hit_velocity_angle
    result["desired_hit_angle"] = bag_dict["desired_hit_angle"]
    result["puck_velocity_angle"] = puck_velocity_angle
    result["planned_puck_velocity_angle"] = planned_hit_velocity_angle

    dt = np.diff(t)
    integral = lambda x: np.sum(np.abs(x)[1:] * dt)
    #reduce_integral = lambda x: integral(np.sum(np.abs(x), axis=-1))
    reduce_integral = lambda x: integral(np.linalg.norm(x, axis=-1))
    result["planned_z_error"] = integral(desired_z_loss)
    result["planned_xlow_error"] = integral(desired_xlow_loss)
    result["planned_xhigh_error"] = integral(desired_xhigh_loss)
    result["planned_ylow_error"] = integral(desired_ylow_loss)
    result["planned_yhigh_error"] = integral(desired_yhigh_loss)
    result["actual_z_error"] = integral(actual_z_loss)
    result["actual_xlow_error"] = integral(actual_xlow_loss)
    result["actual_xhigh_error"] = integral(actual_xhigh_loss)
    result["actual_ylow_error"] = integral(actual_ylow_loss)
    result["actual_yhigh_error"] = integral(actual_yhigh_loss)

    result["joint_trajectory_error"] = reduce_integral(bag_dict["error"][moving])
    result["cartesian_trajectory_error"] = reduce_integral(ee_d - ee)

    print(result)
    return result

# dir_path = os.path.join(package_dir, "bags/baseline_opt_lp/")
dir_path = os.path.join(package_dir, "bags/ours_opt_lp/")
sp = dir_path.replace("bags", "results")
os.makedirs(sp, exist_ok=True)
for p in glob(dir_path + "*.bag"):
    d = compute_metrics(p)
    save_path = p[:-3] + "res"
    save_path = save_path.replace("bags", "results")
    with open(save_path, 'wb') as fh:
        pickle.dump(d, fh)


# for i in range(6):
#    plt.plot(t, qd_ddot[:, i], label=f"q_ddot_{i}")
# plt.legend()
# plt.show()


# plt.plot(puck[:, 0], puck[:, 1])
# plt.show()
