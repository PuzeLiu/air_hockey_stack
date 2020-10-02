import torch
import numpy as np
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch, Rotation

from scipy import optimize as opt


class OptimizerHitPoint:
    def __init__(self, x_des, v_des_dir, kinematics_):
        self.kine = kinematics_

        q_min = self.kine.joint_limits[:, 0].detach().numpy()
        q_max = self.kine.joint_limits[:, 1].detach().numpy()
        qd_min = self.kine.joint_vel_limits[:, 0].detach().numpy()
        qd_max = self.kine.joint_vel_limits[:, 1].detach().numpy()

        self.x_des = x_des
        self.v_des_dir = v_des_dir
        self.bound_lb = np.concatenate((q_min, qd_min))
        self.bound_ub = np.concatenate((q_max, qd_max))
        self.constr_b = np.concatenate((self.x_des[:3], np.array([1., ])))

    def obj_fun(self, x, out='torch'):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        q = x[:7]
        qd = x[7:]
        jac = self.kine.get_jacobian(q)
        if out == 'torch':
            return -((jac[:2] @ qd).dot(jac[:2] @ qd))
        elif out == 'numpy':
            return -((jac[:2] @ qd).dot(jac[:2] @ qd)).detach().numpy()

    def obj_jac(self, x, args):
        x = torch.tensor(x)

        def obj_fun_wrapper(x):
            return self.obj_fun(x, 'torch')

        jac = torch.autograd.functional.jacobian(obj_fun_wrapper, x)
        return jac.detach().numpy()

    def constraints_fun(self, x, out='torch'):
        x = x if isinstance(x, torch.Tensor) else torch.tensor(x)
        q = x[:7]
        qd = x[7:]
        pose = self.kine.forward_kinematics(q)
        xyz = pose[:3]

        jac = self.kine.get_jacobian(q)
        xyz_d = (jac[:3] @ qd).detach().numpy()
        quat_d = torch.norm(jac[3:] @ qd).detach().numpy()
        cos_theta = self.v_des_dir.dot(xyz_d) / np.linalg.norm(xyz_d)
        if out == 'torch':
            return torch.cat((xyz, torch.tensor([cos_theta]))) - torch.from_numpy(self.constr_b)
        elif out == "numpy":
            return np.concatenate((xyz.detach().numpy(), [cos_theta])) - self.constr_b

    def constraints_jac(self, x, args):
        x = torch.tensor(x)

        def const_fun_wrapper(x):
            return self.constraints_fun(x, 'torch')

        return torch.autograd.functional.jacobian(const_fun_wrapper, x).detach().numpy()

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
                           jac=self.obj_jac,
                           options={'maxiter': 5000, 'disp': True})
        return res


if __name__ == "__main__":
    table_height = 0.1915
    puck_position = np.array([0.7, 0.2, table_height])
    hit_direction = (np.array([2.53, 0.0, table_height]) - puck_position)
    hit_direction = hit_direction / np.linalg.norm(hit_direction)
    tcp_pos = np.array([0., 0., 0.3455])
    tcp_quat = np.array([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=torch.tensor(tcp_pos).double(),
                                 tcp_quat=torch.tensor(tcp_quat).double())
    optimizer = OptimizerHitPoint(puck_position, hit_direction, kinematics)

    ret = optimizer.optimize()
    print(ret.success)

    print("Before Optimization:")
    print(optimizer.x0)
    print("Obj: ", -optimizer.obj_fun(optimizer.x0, 'numpy'))
    print("After Optimization:")
    print(ret.success)
    print(ret.x)
    print("Final Obj: ", -optimizer.obj_fun(ret.x, 'numpy'))
    print("Pose: ", optimizer.kine.forward_kinematics(torch.tensor(ret.x[:7])))
    print("Desired Pose: ", optimizer.x_des)
    print("Velocity: ", optimizer.kine.get_jacobian(torch.tensor(ret.x[:7])).detach().numpy() @ ret.x[7:])
    print("Desired:", optimizer.v_des_dir)
