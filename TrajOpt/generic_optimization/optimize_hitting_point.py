import os
import torch
import numpy as np
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch, Rotation

from scipy import optimize as opt

from TrajOpt.generic_optimization.joint_space_spline import AutogradNumpyWrapper


class OptimizerHitPoint:
    def __init__(self, x_des, v_des_dir, kinematics_):
        self.kine = kinematics_

        q_min = self.kine.joint_limits[:, 0].detach().numpy()
        q_max = self.kine.joint_limits[:, 1].detach().numpy()
        qd_min = self.kine.joint_vel_limits[:, 0].detach().numpy()
        qd_max = self.kine.joint_vel_limits[:, 1].detach().numpy()

        self._x_f = x_des
        self.v_des_dir = v_des_dir
        self.bound_lb = np.concatenate((q_min, qd_min))
        self.bound_ub = np.concatenate((q_max, qd_max))
        self.constr_b = np.concatenate((self._x_f[:3], np.array([1., ])))

    def _obj_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x_tmp = x.view((-1, 7))
        q = x_tmp[0]
        qd = x_tmp[1]
        jac = self.kine.get_jacobian(q)
        return -((jac[:2] @ qd).dot(jac[:2] @ qd))

    def _eq_constraints_fun(self, x):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        x_tmp = x.view((-1, 7))
        q = x_tmp[0]
        qd = x_tmp[1]
        pose = self.kine.forward_kinematics(q)
        xyz = pose[:3]

        jac = self.kine.get_jacobian(q)
        xyz_d = jac[:3] @ qd
        cos_theta = (torch.from_numpy(self.v_des_dir) @ xyz_d) / torch.norm(xyz_d)
        return torch.cat((xyz, cos_theta.unsqueeze(0))) - torch.from_numpy(self.constr_b)


    def optimize(self):
        obj_ = AutogradNumpyWrapper(self._obj_fun)
        eq_constraints = AutogradNumpyWrapper(self._eq_constraints_fun)

        constraints = [{'type': 'eq', 'fun': eq_constraints.fun, 'jac': eq_constraints.jac}]
        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        x0 = self._get_feasible_init()

        obj_.fun(x0)
        eq_constraints.fun(x0)

        res = opt.minimize(fun=obj_.fun, x0=x0, method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           jac=obj_.jac,
                           options={'maxiter': 1000, 'disp': True, 'ftol': 1e-8})
        return res

    def _get_feasible_init(self):
        quat_f = np.array([0, 0.8660254, 0, 0.5])
        pose_f = np.concatenate([self._x_f, quat_f])
        gc = torch.tensor([1., -1., 1.]).double()
        feasible_psi = self.kine.get_feasible_interval(torch.tensor(pose_f).double(), gc)
        if feasible_psi is not None:
            psi_0 = feasible_psi[0].mean()
        else:
            raise ValueError("No Feasible solution")
        res, q0 = self.kine.inverse_kinematics(torch.tensor(pose_f).double(), psi_0, gc)
        qd0 = np.linalg.pinv(self.kine.get_jacobian(q0).detach().numpy()) @ np.concatenate(
            (self.v_des_dir * 0.1, np.array([0., 0., 0., 0.])))
        x0 = np.concatenate((q0.detach().numpy(), qd0))
        return x0


if __name__ == "__main__":
    table_height = 0.1915
    puck_position = np.array([0.9, 0.2, table_height])
    hit_direction = (np.array([2.53, 0.0, table_height]) - puck_position)
    hit_direction = hit_direction / np.linalg.norm(hit_direction)

    goal_position = puck_position - hit_direction * 0.03

    tcp_pos = np.array([0., 0., 0.3455])
    tcp_quat = np.array([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=torch.tensor(tcp_pos).double(),
                                 tcp_quat=torch.tensor(tcp_quat).double())

    optimizer = OptimizerHitPoint(goal_position, hit_direction, kinematics)

    res = optimizer.optimize()
    print(res.success)

    print("After Optimization:")
    print(res.x)
    print("Final Obj: ", -optimizer._obj_fun(res.x))
    print("Pose: ", optimizer.kine.forward_kinematics(torch.tensor(res.x[:7])))
    print("Desired Pose: ", optimizer._x_f)
    print("Velocity: ", optimizer.kine.get_jacobian(torch.tensor(res.x[:7])).detach().numpy() @ res.x[7:])
    print("Desired:", optimizer.v_des_dir)

    output_dir = "result"
    file_name = np.array2string(puck_position[:2], separator=',') + "_optimal_init.npy"
    np.save(os.path.join(output_dir, file_name), res.x)
