import time

import numpy as np
import quaternion
import scipy.optimize as opt
import torch

from TrajOpt.spline.cubic_spline import CubicSpline
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch


class AutogradNumpyWrapper:
    def __init__(self, func):
        self._fun = func
        self._cached_x = None
        self._cached_y = None
        self._cached_jac = None

    def is_new(self, x):
        if self._cached_x is None:
            return True
        else:
            # compare x to cached_x to determine if we've been given a new input
            x, self._cached_x = np.array(x), np.array(self._cached_x)
            error = np.abs(x - self._cached_x)
            return error.max() > 1e-8

    def cache(self, x):
        self._cached_x = x
        x_tensor = torch.tensor(x, requires_grad=True)

        y = self._fun(x_tensor)
        self._cached_y = y.detach().numpy()

        if y.shape == torch.Size([]):
            jac = torch.autograd.grad(y, x_tensor, retain_graph=True)[0]
        else:
            jac = torch.zeros(y.shape[0], x.shape[0])
            for i in range(y.shape[0]):
                jac[i] = torch.autograd.grad(y[i], x_tensor, retain_graph=True)[0]
        self._cached_jac = jac.detach().numpy()

    def fun(self, x):
        if self.is_new(x):
            self.cache(x)
        return self._cached_y

    def jac(self, x):
        if self.is_new(x):
            self.cache(x)
        return self._cached_jac


class OptimizerCubicSpline:
    def __init__(self, q_0, dq_0, x_f, dx_f_dir, n_via_points, t_f, kinematics_: KinematicsTorch):
        self._q_0 = q_0
        self._dq_0 = dq_0
        self._x_f = x_f
        self._dx_f_dir = dx_f_dir
        self._n_via_points = n_via_points
        self._n_segment = n_via_points + 1
        self._t_f = t_f
        self._kine = kinematics_

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

    def _obj_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x.requires_grad_(True)
        q_f = x[-14:-7]
        dq_f = x[-7:]
        jac = self._kine.get_jacobian(q_f)
        return (-(jac[:2] @ dq_f) @ (jac[:2] @ dq_f))

    def _eq_constraints_fun(self, x):
        eq_constraints = torch.zeros(3 + 1 + self._n_via_points)
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x = x.view((-1, 7))
        q_f = x[-2]
        dq_f = x[-1]

        eq_constraints[:3] = self._final_position_constraint(q_f)
        eq_constraints[3] = self._hit_velocity_constraint(q_f, dq_f)
        eq_constraints[4:4 + self._n_via_points] = self._via_point_position_constraint(x[:-2])

        return eq_constraints

    def _ineq_constraints_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x = x.view((-1, 7))
        q_i = x[:-2]
        q_f = x[-2]
        dq_f = x[-1]

        cubic_spline = CubicSpline(self._n_dim, torch.tensor(self._q_0), q_i, q_f,
                                   torch.tensor(self._dq_0), dq_f,
                                   self._t_f, bound_type='velocity')

        max_velocities = cubic_spline.get_max_velocities()

        return torch.cat([max_velocities - self._kine.joint_vel_limits[:, 0].repeat(self._n_segment, 1),
                          self._kine.joint_vel_limits[:, 1].repeat(self._n_segment, 1) - max_velocities]).view(-1)

    def optimize(self):
        obj_ = AutogradNumpyWrapper(self._obj_fun)
        eq_constraints = AutogradNumpyWrapper(self._eq_constraints_fun)
        ineq_constraints = AutogradNumpyWrapper(self._ineq_constraints_fun)
        constraints = [{'type': 'eq', 'fun': eq_constraints.fun, 'jac': eq_constraints.jac},
                       {'type': 'ineq', 'fun': ineq_constraints.fun, 'jac': ineq_constraints.jac}]

        # constraints = [{'type': 'eq', 'fun': eq_constraints.fun},
        #                {'type': 'ineq', 'fun': ineq_constraints.fun}]

        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        x0 = self._get_feasible_init()
        res = opt.minimize(fun=obj_.fun, x0=x0, method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           jac=obj_.jac,
                           options={'maxiter': 5000, 'disp': True})
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

        dx_f = np.concatenate([self._dx_f_dir * 0.1, np.array([0., 0., 0., 0.])])
        dq_f = np.linalg.pinv(self._kine.get_jacobian(torch.tensor(x_init[-2]).double()).detach().numpy()) @ dx_f

        x_init[-1] = dq_f
        return x_init.reshape(-1)

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
            id = torch.argmax(feasible_psi[:, 1] - feasible_psi[:, 0])
            psi_f = feasible_psi[id].mean()
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
                    id = torch.argmax(feasible_psi_i[:, 1] - feasible_psi_i[:, 0])
                    psi_i[i] = feasible_psi[id].mean()
                    print("Interpolated redundancy is not in feasible interval, Using: ", psi_i[i])
            else:
                raise ValueError("No Feasible redundancy")
        return psi_f.detach().numpy(), psi_i.detach().numpy()


if __name__ == "__main__":
    table_height = 0.1915
    n_via_points = 2
    t_f = 2.
    goal_position = np.array([0.9, 0.2, table_height])

    hit_direction = (np.array([2.53, 0.0, table_height]) - goal_position)
    hit_direction = hit_direction / np.linalg.norm(hit_direction)

    tcp_pos = np.array([0., 0., 0.3455])
    tcp_quat = np.array([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=torch.tensor(tcp_pos).double(),
                                 tcp_quat=torch.tensor(tcp_quat).double())

    q_0 = np.array([-1.0207648e-07, 5.0181419e-01, 6.8465688e-08, -1.1696992e+00,
                    -3.7518330e-08, 1.4700792e+00, 0])
    dq_0 = np.zeros(7)
    optimizer = OptimizerCubicSpline(q_0=q_0, dq_0=dq_0, x_f=goal_position,
                                     dx_f_dir=hit_direction, n_via_points=n_via_points,
                                     t_f=t_f, kinematics_=kinematics)
    t_start = time.time()
    res = optimizer.optimize()
    t_end = time.time()

    print(res.success)
    print(res.x)
    print("Optimization Time:", t_end - t_start)

    file_name = "result_" + np.array2string(goal_position[:2], separator=',') + ".npy"
    result = {"q_0": q_0,
              "dq_0": dq_0,
              "table_height": table_height,
              "goal_position": goal_position,
              "hit_direction": hit_direction,
              "n_via_points": n_via_points,
              "t_f": t_f,
              "tcp_pos": tcp_pos,
              "tcp_quat": tcp_quat,
              "result": res.x}

    np.save(file_name, result)
