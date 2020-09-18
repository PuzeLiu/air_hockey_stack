import torch
import numpy as np
from air_hockey.environments.iiwa.kinematics_torch import KinematicsTorch

from scipy import optimize as opt
from scipy.spatial.transform import Rotation as R


class Optimizer_Hit():
    def __init__(self, x_des, v_des_dir):
        self.kine = KinematicsTorch()
        q_min = self.kine.joint_limits[:, 0].detach().numpy()
        q_max = self.kine.joint_limits[:, 1].detach().numpy()
        qd_min = self.kine.joint_vel_limits[:, 0].detach().numpy()
        qd_max = self.kine.joint_vel_limits[:, 1].detach().numpy()

        self.x_des = x_des
        self.v_des_dir = v_des_dir / np.linalg.norm(v_des_dir)
        self.bound_lb = np.concatenate((q_min, qd_min))
        self.bound_ub = np.concatenate((q_max, qd_max))
        self.constr_lb = np.concatenate((x_des[:4], np.array([1])))
        self.constr_ub = np.concatenate((x_des[:4], np.array([1])))

    def obj_fun(self, x):
        q = x[:7]
        qd = x[7:]
        jac = self.kine.get_jacobian(torch.tensor(q)).detach().numpy()
        return -(jac[:2] @ qd).dot(jac[:2] @ qd)

    def obj_jac(self, x):
        x = torch.tensor(x)
        def obj_fun_torch(x):
            q = x[:7]
            qd = x[7:]
            jac = self.kine.get_jacobian(torch.tensor(q))
            return (jac[:2] @ qd).dot(jac[:2] @ qd)
        jac = torch.autograd.functional.jacobian(obj_fun_torch, x)
        return jac.detach().numpy()

    def constraints_fun(self, x):
        q = x[:7]
        qd = x[7:]
        xyzr = self.kine.forward_kinematics(torch.tensor(q)).detach().numpy()[:4]
        jac = self.kine.get_jacobian(torch.tensor(q)).detach().numpy()
        xyzr_d = jac[:4] @ qd
        cos_theta = self.v_des_dir.dot(xyzr_d) / np.linalg.norm(xyzr_d)
        return np.concatenate((xyzr, [cos_theta]))

    def optimize(self):
        constraints = opt.NonlinearConstraint(self.constraints_fun, lb=self.constr_lb, ub=self.constr_ub,
                                              jac='3-point')
        bounds = opt.Bounds(lb=self.bound_lb, ub=self.bound_ub)

        pose = torch.cat((torch.tensor(self.x_des), torch.tensor([0., 0.]))).double()
        res, q0 = self.kine.inverse_kinematics(pose, torch.tensor([0]).double(), torch.tensor([1., 1., 1.]).double())
        qd0 = np.linalg.pinv(self.kine.get_jacobian(q0).detach().numpy()) @ np.array([0.1, 0.1, 0., 0., 0., 0.])
        self.x0 = np.concatenate((q0.detach().numpy(), qd0))

        res = opt.minimize(fun=self.obj_fun, x0=self.x0,
                           args=(), method='SLSQP',
                           bounds=bounds, constraints=constraints,
                           jac=self.obj_jac,
                           # jac='2-point',
                           options={'maxiter': 1000, 'disp': True})
        return res

def puck2ee(p_, x):
    cart_rotation = R.from_euler('YZY',[180, x[2], -np.pi/2], degrees=True).as_matrix()
    cart_position = np.array([p_[0], p_[1], 0.25])
    cart_robot_ee = cart_position - 0.16 * cart_rotation[:, 2]
    return torch.tensor(cart_robot_ee).float(), torch.tensor(cart_rotation).float()

if __name__ == "__main__":
    p_puck = np.array([0.7, 0.2])
    v_h_direct = (np.array([2.28, 0.0]) - p_puck) / np.linalg.norm(np.array([2.28, 0.0] - p_puck))

    optimizer = Optimizer_Hit(p_puck, v_h_direct)

    ret = optimizer.optimize()
    print(ret.success)

    print("Before Optimization:")
    print(optimizer.x0)
    print(-optimizer.obj_fun(optimizer.x0))
    print("After Optimization:")
    print(ret.success)
    print(ret.x)
    print(-optimizer.obj_fun(ret.x))
    print("Pose:", optimizer.kine.forward_kinematics(torch.tensor(ret.x[:7])), "Desired Pose: ")
