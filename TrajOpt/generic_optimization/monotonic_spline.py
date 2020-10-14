import os
import time

import numpy as np
import quaternion
import scipy.optimize as opt
import torch

from TrajOpt.generic_optimization.joint_space_spline import AutogradNumpyWrapper
from TrajOpt.spline.monotonic_cubic_spline import MonotonicCubicSpline
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch


class OptimizerCubicSpline:
    def __init__(self, q_0, dq_0, x_f, dx_f_dir, t_f, n_via_points, kinematics_: KinematicsTorch):
        self._q_0 = q_0
        self._dq_0 = dq_0
        self._x_f = x_f
        self._dx_f_dir = dx_f_dir
        self._n_via_points = n_via_points
        self._n_segment = n_via_points + 1
        self._kine = kinematics_
        self._t_f = t_f

        # [7(N+2)] N via point + final position and velocity
        self._n_dim = self._q_0.shape[0]

        # global configuration of initial point
        self.gc = torch.tensor(np.sign(self._q_0[[1, 3, 5]])).double()

        q_min = self._kine.joint_limits[:, 0].detach().numpy()
        q_max = self._kine.joint_limits[:, 1].detach().numpy()
        dq_min = self._kine.joint_vel_limits[:, 0].detach().numpy()
        dq_max = self._kine.joint_vel_limits[:, 1].detach().numpy()

        # Equality constraint: [position constraint (3), Velocity Direction Constraint (1),
        #                       Intermediate Point stay on the table surface (N)]
        self.constraint_eq_b = np.concatenate([self._x_f[:3], np.array([1., ]),
                                               np.ones(n_via_points) * self._x_f[2]])
        self.constraint_ineq_b = np.concatenate([dq_min] * self._n_segment + [dq_max] * self._n_segment)

        # Boundaries for N via point and find position and velocity
        self.bound_lb = np.concatenate([np.concatenate([q_min] * self._n_segment), dq_min, ])
        self.bound_ub = np.concatenate([np.concatenate([q_max] * self._n_segment), dq_max, ])

        # Cartesian Boundary condition
        self._cartesian_lb = np.array([0.6, -0.45])
        self._cartesian_ub = np.array([1.5, 0.45])

        self._cur_iter = 0

    @property
    def t_f(self):
        return self._t_f

    def _obj_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x_tmp = x.view((-1, 7))
        q_f = x_tmp[-1]
        q_i = x_tmp[:-1]

        spline = MonotonicCubicSpline(self._n_dim, self._q_0, q_i, q_f, self._t_f)
        dq_f = spline.v_f

        # Maximize the hitting velocity
        jac = self._kine.get_jacobian(q_f)
        return -jac[:2] @ dq_f @ (jac[:2] @ dq_f)

    def _eq_constraints_fun(self, x):
        eq_constraints = torch.zeros(3 + 1 + self._n_via_points)
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x_tmp = x.view((-1, 7))
        q_i = x_tmp[:-1]
        q_f = x_tmp[-1]

        spline = MonotonicCubicSpline(self._n_dim, self._q_0, q_i, q_f, self._t_f)
        dq_f = spline.v_f

        eq_constraints[:3] = self._final_position_constraint(q_f)
        eq_constraints[3] = self._hit_velocity_constraint(q_f, dq_f)
        eq_constraints[4:4 + self._n_via_points] = self._via_point_position_constraint(q_i)

        return eq_constraints

    def _ineq_constraints_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x_tmp = x.view((-1, 7))
        q_i = x_tmp[:-1]
        q_f = x_tmp[-1]

        spline = MonotonicCubicSpline(self._n_dim, self._q_0, q_i, q_f, self._t_f)

        max_velocities = spline.get_max_velocities()
        vel_ineq = (self._kine.joint_vel_limits[:, 0].repeat(self._n_segment, 1) ** 2 - max_velocities ** 2).view(-1)

        x_i = torch.zeros((self._n_via_points, 2))
        for i in range(self._n_via_points):
            x_i[i] = self._kine.forward_kinematics(q_i[i])[:2]
        cart_ineq = torch.cat(
            [x_i - torch.from_numpy(self._cartesian_lb), torch.from_numpy(self._cartesian_ub) - x_i]).view(-1)
        return torch.cat([vel_ineq, cart_ineq])

    def optimize(self):
        obj_ = AutogradNumpyWrapper(self._obj_fun)
        eq_constraints = AutogradNumpyWrapper(self._eq_constraints_fun)
        ineq_constraints = AutogradNumpyWrapper(self._ineq_constraints_fun)
        constraints = [{'type': 'eq', 'fun': eq_constraints.fun, 'jac': eq_constraints.jac},
                       {'type': 'ineq', 'fun': ineq_constraints.fun, 'jac': ineq_constraints.jac}]

        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        x0 = self._get_feasible_init()

        self.opt_call_back(x0)
        res = opt.minimize(fun=obj_.fun, x0=x0, method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           jac=obj_.jac,
                           options={'maxiter': 1000, 'disp': True, 'ftol': 1e-5},
                           callback=self.opt_call_back)
        return res

    def _final_position_constraint(self, q_f):
        pose = self._kine.forward_kinematics(q_f)
        return pose[:3] - torch.from_numpy(self._x_f[:3])

    def _hit_velocity_constraint(self, q_f, dq_f):
        jac = self._kine.get_jacobian(q_f)
        dx_f = (jac[:3] @ dq_f)
        return dx_f @ torch.tensor(self._dx_f_dir).double() / torch.norm(dx_f) - 1.0

    def _via_point_position_constraint(self, x_i):
        x_i_z = torch.zeros(self._n_via_points)
        for i in range(self._n_via_points):
            q_i = x_i[i]
            x_i_z[i] = self._kine.forward_kinematics(q_i)[2]
        return x_i_z - self._x_f[2]

    def _get_feasible_init(self):
        x_init = np.zeros((self._n_segment + 1, 7))

        quat_f = np.array([0, 0.8660254, 0, 0.5])

        # get initial and final pose
        pose_0 = self._kine.forward_kinematics(torch.tensor(self._q_0)).detach().numpy()
        pose_f = np.concatenate([self._x_f, quat_f])
        pose_i = self._get_intermediate_pose(pose_0, pose_f)

        # get initial redundancy
        psi_f, psi_i = self._get_redundancy(pose_i, pose_f)

        # get initial joint
        for i in range(self._n_via_points):
            _, x_init[i] = self._kine.inverse_kinematics(torch.tensor(pose_i[i]).double(),
                                                         torch.tensor(psi_i[i]).double(), self.gc)
        _, x_init[-2] = self._kine.inverse_kinematics(torch.tensor(pose_f).double(),
                                                      torch.tensor(psi_f).double(), self.gc)

        dx_f = np.concatenate([self._dx_f_dir * 0.5, np.array([0., 0., 0., 0.])])

        # dx_f = np.array([1.37978352e+00, -1.69298925e-01, 5.62490350e-08, -6.84660594e-01, -6.67649354e-02,
        #                  6.95568517e-01, 2.29986683e-01])
        dq_f = np.linalg.pinv(self._kine.get_jacobian(torch.tensor(x_init[-2]).double()).detach().numpy()) @ dx_f

        x_init[-1] = dq_f
        x_init = x_init.reshape(-1)

        proper_t = False
        while not proper_t:
            if torch.all(torch.abs(self._eq_constraints_fun(x_init)) < 1e-6) and \
                    torch.all(torch.gt(self._ineq_constraints_fun(x_init), -1e-6)):
                proper_t = True
            else:
                self._t_f = self._t_f * 2

        return x_init

    def _get_intermediate_pose(self, pose_0, pose_f):
        pose_i = np.zeros((self._n_via_points, 7))

        # intermediate position
        pose_i[:, :3] = np.linspace(pose_0[:3], pose_f[:3], self._n_segment + 1)[1:-1]

        # intermediate quaternion
        quat_0 = quaternion.from_float_array(pose_0[3:])
        quat_f = quaternion.from_float_array(pose_f[3:])
        quat_i = quaternion.slerp(quat_0, quat_f, 0, 1, np.linspace(0, 1, self._n_segment + 1)[1:-1])
        for i in range(self._n_via_points):
            pose_i[i, 3:] = quat_i[i].components
        return pose_i

    def _get_redundancy(self, pose_i, pose_f):
        psi_0 = self._kine.get_redundancy(torch.tensor(self._q_0).double())
        feasible_psi = self._kine.get_feasible_interval(torch.tensor(pose_f).double(), self.gc)
        if feasible_psi is not None:
            idx = torch.argmax(feasible_psi[:, 1] - feasible_psi[:, 0])
            psi_f = feasible_psi[idx].mean()
        else:
            raise ValueError("No Feasible redundancy")

        psi_i = torch.linspace(psi_0, psi_f, self._n_segment + 1)[1:-1]
        # check if psi_i is feasible
        for i in range(self._n_via_points):
            feasible_psi_i = self._kine.get_feasible_interval(torch.tensor(pose_i[i]).double(), self.gc)
            if feasible_psi_i is not None:
                if torch.any(torch.logical_and(torch.lt(feasible_psi_i[:, 0], psi_i[i]),
                                               torch.gt(feasible_psi_i[:, 1], psi_i[i]))):
                    continue
                else:
                    idx = torch.argmax(feasible_psi_i[:, 1] - feasible_psi_i[:, 0])
                    psi_i[i] = feasible_psi[idx].mean()
                    print("Interpolated redundancy is not in feasible interval, Using: ", psi_i[i])
            else:
                raise ValueError("No Feasible redundancy")
        return psi_f.detach().numpy(), psi_i.detach().numpy()

    def opt_call_back(self, xk):
        xk_tensor = torch.tensor(xk).view((-1, 7))
        q_i = xk_tensor[:-2]
        q_f = xk_tensor[-2]
        dq_f = xk_tensor[-1]

        output_dir = "result/figures/heuristic"

        cubic_spline = MonotonicCubicSpline(self._n_dim, torch.tensor(self._q_0), q_i, q_f, self._t_f)
        output_dir_joint = os.path.join(output_dir, "iter_{}".format(self._cur_iter))
        if not os.path.exists(output_dir_joint):
            os.makedirs(output_dir_joint)
        cubic_spline.plot(output_dir_joint)
        trajectories, t = cubic_spline.get_trajectory(1 / 50.)

        ee_pos = torch.empty(trajectories.shape[0], 3)
        ee_vel = torch.empty(trajectories.shape[0], 3)
        for i in range(trajectories.shape[0]):
            ee_pos[i] = self._kine.forward_kinematics(trajectories[i, 0])[:3]
            ee_vel[i] = (self._kine.get_jacobian(trajectories[i, 0])[:3] @ trajectories[i, 1])
        self._cur_iter += 1

        ee_pos = ee_pos.detach().numpy()
        ee_vel = ee_vel.detach().numpy()

        import matplotlib.pyplot as plt

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
        fig.savefig(os.path.join(output_dir, "cart_pos_{}".format(self._cur_iter)))
        plt.close(fig)

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
        fig.savefig(os.path.join(output_dir, "cart_vel_{}".format(self._cur_iter)))
        plt.close(fig)

        fig = plt.figure()
        plt.plot(ee_pos[:, 0], ee_pos[:, 1])
        index = (np.linspace(0, 1, self._n_segment + 1) * trajectories.shape[0]).astype(np.int)[:-1]
        plt.scatter(ee_pos[index, 0], ee_pos[index, 1], marker='*')
        fig.savefig(os.path.join(output_dir, "xy_pos_{}".format(self._cur_iter)))
        plt.close(fig)

        return False