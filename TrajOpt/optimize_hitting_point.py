import torch
import numpy as np
from air_hockey.environments.iiwa.kinematics_torch import KinematicsTorch, Rotation

from scipy import optimize as opt
from scipy.spatial.transform import Rotation as R


class Optimizer_Hit():
    def __init__(self, x_des, v_des_dir, table_height):
        self.kine = KinematicsTorch(tcp_pos=torch.tensor([0, 0, 0.3]).double())
        q_min = self.kine.joint_limits[:, 0].detach().numpy()
        q_max = self.kine.joint_limits[:, 1].detach().numpy()
        qd_min = self.kine.joint_vel_limits[:, 0].detach().numpy()
        qd_max = self.kine.joint_vel_limits[:, 1].detach().numpy()

        self.x_des = np.concatenate([x_des, np.array([table_height, np.deg2rad(180.)])])
        self.v_des_dir = np.concatenate([v_des_dir / np.linalg.norm(v_des_dir), np.array([0., 0.])])
        self.bound_lb = np.concatenate((q_min, qd_min))
        self.bound_ub = np.concatenate((q_max, qd_max))
        self.constr_b = np.concatenate((self.x_des[:3], np.array([0, 1.])))

    def obj_fun(self, x):
        q = x[:7]
        qd = x[7:]
        jac = self.kine.get_jacobian(torch.tensor(q)).detach().numpy()
        return -((jac[:2] @ qd).dot(jac[:2] @ qd))

    def obj_jac(self, x):
        x = torch.tensor(x)
        def obj_fun_torch(x):
            q = x[:7]
            qd = x[7:]
            jac = self.kine.get_jacobian(q)
            return (jac[:2] @ qd).dot(jac[:2] @ qd)
        jac = torch.autograd.functional.jacobian(obj_fun_torch, x)
        return jac.detach().numpy()

    def constraints_fun(self, x):
        q = torch.tensor(x[:7])
        qd = torch.tensor(x[7:])
        pose = self.kine.forward_kinematics(q)
        xyz = pose[:3]

        y_axis = Rotation.from_euler_yzy(pose[3:]).as_matrix()[:, 1]
        y_axis_vertical = torch.dot(y_axis, torch.tensor([0., 0., 1.]).double())

        jac = self.kine.get_jacobian(q)
        xyz_d = (jac[:4] @ qd).detach().numpy()
        cos_theta = self.v_des_dir.dot(xyz_d) / np.linalg.norm(xyz_d)

        return np.concatenate((xyz.detach().numpy(), [y_axis_vertical.item(), cos_theta])) - self.constr_b

    def constraints_jac(self, x):
        x = torch.tensor(x)
        def const_fun_torch(x):
            q = x[:7]
            qd = x[7:]
            pose = self.kine.forward_kinematics(q)
            xyz = pose[:3]

            y_axis = Rotation.from_euler_yzy(pose[3:]).as_matrix()[:, 1]
            y_axis_vertical = torch.dot(y_axis, torch.tensor([0., 0., 1.]).double()).unsqueeze(0)

            jac = self.kine.get_jacobian(q)
            xyz_d = (jac[:4] @ qd)
            cos_theta = torch.dot(torch.tensor(self.v_des_dir), xyz_d) / torch.norm(xyz_d).unsqueeze(0)

            return torch.cat([xyz, y_axis_vertical, cos_theta]) - torch.tensor(self.constr_b)
        return torch.autograd.functional.jacobian(const_fun_torch, x).detach().numpy()

    def optimize(self):
        constraints = [{'type': 'eq', 'fun': self.constraints_fun}] #, 'jac': self.constraints_jac}]
        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        pose = np.concatenate([self.x_des, np.array([0., 0.])])
        res, q0 = self.kine.inverse_kinematics(torch.tensor(pose).double(),
                                               torch.tensor([0]).double(),
                                               torch.tensor([1., 1., 1.]).double())
        qd0 = np.linalg.pinv(self.kine.get_jacobian(q0).detach().numpy()) @ np.array([0.1, 0.1, 0., 0., 0., 0.])
        self.x0 = np.concatenate((q0.detach().numpy(), qd0))

        res = opt.minimize(fun=self.obj_fun, x0=self.x0,
                           args=(), method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           jac=self.obj_jac,
                           options={'maxiter': 1000, 'disp': True})
        return res

def puck2ee(p_, x):
    cart_rotation = R.from_euler('YZY',[180, x[2], -np.pi/2], degrees=True).as_matrix()
    cart_position = np.array([p_[0], p_[1], 0.25])
    cart_robot_ee = cart_position - 0.16 * cart_rotation[:, 2]
    return torch.tensor(cart_robot_ee).float(), torch.tensor(cart_rotation).float()

if __name__ == "__main__":
    p_puck = np.array([0.7, 0.2])
    v_h_direct = (np.array([2.28, 0.0]) - p_puck) / np.linalg.norm(np.array([2.28, 0.0]) - p_puck)
    table_height = 0.28
    optimizer = Optimizer_Hit(p_puck, v_h_direct, table_height)

    ret = optimizer.optimize()
    print(ret.success)

    print("Before Optimization:")
    print(optimizer.x0)
    print(optimizer.obj_fun(optimizer.x0))
    print("After Optimization:")
    print(ret.success)
    print(ret.x)
    print("Final Obj: ", optimizer.obj_fun(ret.x))
    print("Pose: ", optimizer.kine.forward_kinematics(torch.tensor(ret.x[:7])))
    print("Desired Pose: ", optimizer.x_des)
    print("Velocity: ", optimizer.kine.get_jacobian(torch.tensor(ret.x[:7])).detach().numpy() @ ret.x[7:])
    print("Desired:", optimizer.v_des_dir)
