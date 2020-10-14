import os
import numpy as np
import torch
import matplotlib.pyplot as plt

from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch
from TrajOpt.spline.cubic_spline import CubicSpline


if __name__ == "__main__":
    output_dir = "result"
    result_file = "result_[0.9,0.2]_optimal_final.npy"
    result_summary = np.load(os.path.join(output_dir, result_file), allow_pickle='TRUE').item()
    result_summary['dq_0'] = np.zeros(7)
    result_tensor = torch.from_numpy(result_summary['result']).view((-1, 7))
    t_f = result_summary['t_f']

    kinematics = KinematicsTorch(tcp_pos=torch.tensor(result_summary['tcp_pos']).double(),
                                 tcp_quat=torch.tensor(result_summary['tcp_quat']).double())
    cubic_spline = CubicSpline(n_dim=result_summary['q_0'].shape[0],
                               x_0=torch.tensor(result_summary['q_0']).double(),
                               x_i=result_tensor[:-2].double(),
                               x_f=result_tensor[-2].double(),
                               b_0=torch.tensor(result_summary['dq_0']).double(),
                               b_f=result_tensor[-1].double(),
                               t_f=t_f,
                               bound_type="velocity")

    # cubic_spline.plot()

    step_size = 1/240.
    trajectories, t = cubic_spline.get_trajectory(step_size)
    np.save(os.path.join(output_dir, "trajectory.npy"), trajectories)

    trajectories = np.load("result/initial.npy")
    trajectories = torch.tensor(trajectories)

    ee_pos = torch.empty(trajectories.shape[0], 3)
    ee_vel = torch.empty(trajectories.shape[0], 3)
    for i in range(trajectories.shape[0]):
        ee_pos[i] = kinematics.forward_kinematics(trajectories[i, 0])[:3]
        ee_vel[i] = (kinematics.get_jacobian(trajectories[i, 0])[:3] @ trajectories[i, 1])

    ee_pos = ee_pos.detach().numpy()
    ee_vel = ee_vel.detach().numpy()

    fig, axes = plt.subplots(3)
    fig.suptitle("Position")
    axes[0].set_title("X")
    axes[0].plot(t, ee_pos[:, 0])
    axes[1].set_title("Y")
    axes[1].plot(t, ee_pos[:, 1])
    axes[2].set_title("Z")
    axes[2].plot(t, ee_pos[:, 2])
    axes[2].plot(t, torch.ones_like(t) * 0.1915, color='r', linestyle='-.', label='Table Height')
    axes[2].legend()

    fig, axes = plt.subplots(2, 2)
    fig.suptitle("Velocity, maximum velocity:{:.3f}".format(np.linalg.norm(ee_vel[-1, :2])))
    axes[0][0].set_title("X")
    axes[0][0].plot(t, ee_vel[:, 0])
    axes[0][1].set_title("Y")
    axes[0][1].plot(t, ee_vel[:, 1])
    axes[1][0].set_title("Z")
    axes[1][0].plot(t, ee_vel[:, 2])
    axes[1][1].set_title("X-Y")
    axes[1][1].plot(t, np.linalg.norm(ee_vel[:, :2], axis=1))

    fig = plt.figure()
    plt.plot(ee_pos[:, 0], ee_pos[:, 1])
    plt.show()
