import torch
import numpy as np
from air_hockey.environments.iiwa.kinematics_torch import KinematicsTorch
from hard_coded_policy.src.hard_coded_policy.bezier_planner import BezierTrajectory

from scipy import optimize as opt
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt

def opt_fun(x):
    vf_mag = x[0]
    return -vf_mag

# x=[vf_mag, psi, alpha_z, alpha_y', ]
def constraints_fun(x, ):
    global bezier_traj, p0, pf, vf_dir, kine, num_constraints
    vf = vf_dir * x[0]
    bezier_traj.fit(p0, pf, vf, tf=10)

    q_dot_total = np.zeros(7 * num_constraints)
    for i, t_i in enumerate(np.linspace(0, bezier_traj.t_min, num_constraints)):
        p, dp = bezier_traj(t_i)
        p_ee, rot_ee = puck2ee(p, x)
        res, q_i = kine.inverse_kinematics(p_ee, rot_ee, torch.tensor(x[1]), gc)
        if res:
            dp_ee = np.concatenate([dp, np.zeros(4)])
            q_dot_total[7*i:7*i+7] = np.linalg.pinv(kine.get_jacobian(q_i).detach().numpy()) @ dp_ee
        else:
            q_dot_total[7 * i:7 * i + 7] = 10
    # print(q_dot_total[-8: -1])
    return q_dot_total

def puck2ee(p_, x):
    cart_rotation = R.from_euler('YZY',[180, x[2], -np.pi/2], degrees=True).as_matrix()
    cart_position = np.array([p_[0], p_[1], 0.25])
    cart_robot_ee = cart_position - 0.16 * cart_rotation[:, 2]
    return torch.tensor(cart_robot_ee).float(), torch.tensor(cart_rotation).float()

def validate(x):
    vf = vf_dir * x[0]
    bezier_traj.fit(p0, pf, vf)
    t = np.linspace(0, bezier_traj.t_min, 100)

    q = []
    dq = []
    for t_i in t:
        p, dp = bezier_traj(t_i)
        p_ee, rot_ee = puck2ee(p, x)
        res, q_i = kine.inverse_kinematics(p_ee, rot_ee, torch.tensor(x[1]), gc)
        if res:
            q.append(q_i.detach().numpy())
            dp_ee = np.concatenate([dp, np.zeros(4)])
            dq.append(np.linalg.pinv(kine.get_jacobian(q_i).detach().numpy()) @ dp_ee)

    q = np.array(q)
    dq = np.array(dq)
    for i in range(7):
        fig, axes = plt.subplots(2, 1)
        axes[0].plot(q[:, i])
        axes[0].set_title("Joint " + str(i+1) + " Position")
        axes[1].plot(dq[:, i])
        axes[1].set_title("Joint " + str(i+1) + " Velocity")
        axes[1].plot([0, q.shape[0]], [q_dot_limits[i], q_dot_limits[i]], 'r--')
        axes[1].plot([0, q.shape[0]], [-q_dot_limits[i], -q_dot_limits[i]], 'r--')
    plt.show()


if __name__ == "__main__":
    kine = KinematicsTorch()
    bezier_traj = BezierTrajectory(dim=2, lb=[0.5, -0.45], ub=[0.9, 0.45])

    p0 = np.array([0.6, 0.0])
    pf = np.array([0.7, 0.2])
    vf_dir = (np.array([2.28, 0.0]) - pf) / np.linalg.norm(np.array([2.28, 0.0]) - pf)
    gc = torch.tensor([1, 1, 1])
    num_constraints = 10
    q_dot_limits = np.repeat(np.deg2rad([[85, 85, 100, 75, 130, 135, 135]]), num_constraints, axis=0).flatten()

    x0 = np.array([0.2, 0.5, 0.0, np.deg2rad(-65)])
    validate(x0)

    constraints = opt.NonlinearConstraint(constraints_fun, lb=-q_dot_limits, ub=q_dot_limits, jac='3-point')
    bounds = opt.Bounds(lb=np.array([0., -np.pi, -np.pi/2, -np.pi/2]), ub=np.array([2, np.pi, np.pi/2, np.pi/2]))
    res = opt.minimize(fun=opt_fun, x0=x0,
                       args=(), method='SLSQP',
                       bounds=bounds, constraints=constraints)
    print(res.success)
    print(res.x)

    validate(res.x)
