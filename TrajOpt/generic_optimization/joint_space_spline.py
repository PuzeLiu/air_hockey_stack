import numpy as np
import scipy.optimize as opt
import torch

from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch
from TrajOpt.spline.cubic_spline import CubicSpline


class AutogradNumpyWrapper:
    def __init__(self, func):
        self.func = func
        self.cached_x = None
        self.cached_y = None
        self.cached_jac = None

    def is_new(self, x):
        if self.cached_x:
            return True
        else:
            # compare x to cached_x to determine if we've been given a new input
            x, self.cached_x = np.array(x), np.array(self.cached_x)
            error = np.abs(x - self.cached_x)
            return error.max() > 1e-8

    def cache(self, x):
        self.cached_x = x
        x_tensor = torch.tensor(x, requires_grad=True)
        y = self.func(x_tensor)
        jac = torch.zeros(y.shape[0], x.shape[0])
        for i in range(y.shape[0]):
            jac[i] = torch.autograd.grad(y[i], x, retain_graph=True)[0]
        self.cached_y = y.detach().numpy()
        self.cached_jac = jac.detach().numpy()

    def fun(self, x):
        if self.is_new(x):
            self.cache(x)
        return self.cached_y

    def jac(self, x):
        if self.is_new(x):
            self.cache(x)
        return self.cached_jac


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
        self._n_dim = self._q_0.shape[0] * (n_via_points + 2)

        q_min = self._kine.joint_limits[:, 0].detach().numpy()
        q_max = self._kine.joint_limits[:, 1].detach().numpy()
        dq_min = self._kine.joint_vel_limits[:, 0].detach().numpy()
        dq_max = self._kine.joint_vel_limits[:, 1].detach().numpy()

        # Equality constraint: [position constraint (3), Velocity Direction Constraint (1),
        #                       Intermediate Point stay on the table surface (N)]
        self.constraint_eq_b = np.concatenate([self._x_f[:3], np.array([1., ]),
                                               np.ones(n_via_points) * self._x_f[3]])
        self.constraint_ineq_b = np.concatenate([dq_min] * self._n_segment + [dq_max] * self._n_segment)

        # Boundaries for N via point and find position and velocity
        self.bound_lb = np.concatenate([np.concatenate([q_min] * self._n_segment), dq_min])
        self.bound_ub = np.concatenate([np.concatenate([q_max] * self._n_segment), dq_max])

    def _obj_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x.requires_grad_(True)
        q_f = x[-14:-7]
        dq_f = x[-7:]
        jac = self._kine.get_jacobian(q_f)
        return -(jac[:2] @ dq_f) @ (jac[:2] @ dq_f)

    def _eq_constraints_fun(self, x):
        eq_constraints = torch.zeros(3 + 1 + self._n_via_points)
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        q_f = x[-14:-7]
        dq_f = x[-7:]

        eq_constraints[:3] = self._final_position_constraint(q_f)
        eq_constraints[3] = self._hit_velocity_constraint(q_f, dq_f)
        eq_constraints[4:4 + self._n_via_points] = self._via_point_position_constraint(x)

        return eq_constraints

    def _ineq_constraints_fun(self, x):
        ineq_constraints = torch.zeros(2 * 7 * self._n_via_points)
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        for i in range(self._n_via_points):
            q = x[7*i: 7*i+7]




    def optimize(self):
        constraints = [{'type': 'eq', 'fun': self.constraints_fun}]
        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        pose = np.concatenate([self.x_des, np.array([0, 0.8660254, 0, 0.5])])
        gc = torch.tensor([1., -1., 1.]).double()

        feasible_psi = self.kine.get_feasible_interval(torch.tensor(pose).double(), gc)
        if feasible_psi is not None:
            psi_0 = feasible_psi[0].mean()
        else:
            raise ValueError("No Feasible solution")
        res, q0 = self.kine.inverse_kinematics(torch.tensor(pose).double(), psi_0, gc)
        qd0 = np.linalg.pinv(self.kine.get_jacobian(q0).detach().numpy()) @ np.concatenate(
            (self.v_des_dir * 0.1, np.array([0., 0., 0., 0.])))
        self.x0 = np.concatenate((q0.detach().numpy(), qd0))

        res = opt.minimize(fun=self.obj_fun, x0=self.x0,
                           args=('numpy',), method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           # jac=self.obj_jac,
                           options={'maxiter': 5000, 'disp': True})
        return res

    def _final_position_constraint(self, q_f):
        pose = self._kine.forward_kinematics(q_f)
        return pose[:3] - self._x_f[:3]

    def _hit_velocity_constraint(self, q_f, dq_f):
        jac = self._kine.get_jacobian(q_f)
        dx_f = (jac[:3] @ dq_f)
        return dx_f @ self._dx_f_dir / torch.norm(dx_f) - 1.0

    def _via_point_position_constraint(self, x):
        x_i_z = torch.zeros(self._n_via_points)
        for i in range(self._n_via_points):
            q_i = x[7*i: 7*(i+1)]
            x_i_z[i] = self._kine.forward_kinematics(q_i)[2]
        return x_i_z - self._x_f[2]