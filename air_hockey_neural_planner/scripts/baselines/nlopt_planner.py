from copy import copy
from time import perf_counter

import numpy as np
from scipy.optimize import minimize
import pinocchio as pino

from manifold_planning.utils.constants import Limits
import sys, os

BASELINES_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.dirname(BASELINES_DIR)
PACKAGE_DIR = os.path.dirname(SCRIPTS_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPTS_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)
sys.path.append(SCRIPTS_DIR)

from manifold_planning.utils.constants import TableConstraint


class NloptPlanner:
    def __init__(self, n_pts, pino_model, joint_id):
        self.N = n_pts
        self.M = self.N - 2
        self.D = 6
        self.pino_model = pino_model
        self.pino_data = self.pino_model.createData()
        self.joint_id = joint_id
        self.diff_diag, self.diag, self.first, self.last, self.zeros = self.prepare_utils()

    def prepare_utils(self):
        diff_diag = np.zeros(((self.N - 1) * self.D, self.M * self.D))
        diag = np.zeros(((self.N - 1) * self.D, self.M * self.D))
        first = np.zeros((self.D, self.N * self.D))
        last = np.zeros((self.D, self.N * self.D))
        zeros = np.zeros(((self.N - 1) * self.D, self.N * self.D))
        for i in range(self.D):
            s = np.arange(self.M)
            diff_diag[i * (self.N - 1) + s + 1, i * self.M + s] = -1.
            diff_diag[i * (self.N - 1) + s, i * self.M + s] = 1.
            diag[i * (self.N - 1) + s + 1, i * self.M + s] = -1.
            first[i, i * self.N] = 1.
            last[i, (i + 1) * self.N - 1] = 1.
        return diff_diag, diag, first, last, zeros

    def objective(self, x):
        return x[-1]

    def jac(self, x):
        grad = np.zeros_like(x)
        grad[-1] = 1.
        return grad

    def dq_constraint(self, x, q0, qk, dq0, dqk):
        dt = x[-1] / self.N
        q = np.reshape(x[:self.M * self.D], (self.D, self.M)).T
        q_dot = np.reshape(x[self.M * self.D:2 * self.M * self.D], (self.D, self.M)).T
        q = np.concatenate([q0[np.newaxis], q, qk[np.newaxis]], axis=0)
        q_dot = np.concatenate([dq0[np.newaxis], q_dot, dqk[np.newaxis]], axis=0)
        q_diff = q[1:] - q[:-1]
        delta_q = q_dot[:-1] * dt
        r = q_diff - delta_q
        r = np.reshape(r.T, -1)
        return r

    def dq_constraint_jac(self, x, q0, qk, dq0, dqk):
        dt = x[-1] / self.N
        q_dot = np.reshape(x[self.M * self.D:2 * self.M * self.D], (self.D, self.M)).T
        q_dot = np.concatenate([dq0[np.newaxis], q_dot, dqk[np.newaxis]], axis=0)
        q_dot_grad = np.reshape(q_dot[:-1].T, -1)
        grad = np.concatenate([self.diff_diag, self.diag * dt, self.zeros, -q_dot_grad[:, np.newaxis] / self.N], axis=-1)
        return grad

    def table_constraint_jac(self, x):
        q = np.reshape(x[:self.M * self.D], (self.D, self.M)).T
        Js = []
        for i in range(self.M):
            q_ = np.concatenate([q[i], np.zeros(3)], axis=-1)
            J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q_, self.joint_id, pino.LOCAL_WORLD_ALIGNED)[:3, :6]
            Js.append(J)
        Js = np.stack(Js)
        grad = np.zeros((self.M * 3, x.shape[0]))
        for i in range(self.M):
            grad[i, np.arange(0, self.D * self.M, self.M) + i] = Js[i, 0]
            grad[self.M + i, np.arange(0, self.D * self.M, self.M) + i] = Js[i, 1]
            grad[2 * self.M + i, np.arange(0, self.D * self.M, self.M) + i] = Js[i, 2]
        x, y, z = self.table_constraint(x)
        x_grad_sign = np.ones_like(x)
        y_grad_sign = np.where(y > 0., -np.ones_like(y), np.ones_like(y))
        z_grad_sign = np.where(z - 0.16 > 0., -np.ones_like(z), np.ones_like(z))
        grad_sign = np.concatenate([x_grad_sign, y_grad_sign, z_grad_sign])
        # dcdf = np.where(c_val > 0., -np.ones_like(c_val), np.ones_like(c_val))
        return grad * grad_sign[:, np.newaxis]

    def table_constraint(self, x):
        q = np.reshape(x[:self.M * self.D], (self.D, self.M)).T
        z = []
        x = []
        y = []
        for i in range(self.M):
            q_ = np.concatenate([q[i], np.zeros(3)], axis=-1)
            pino.forwardKinematics(self.pino_model, self.pino_data, q_)
            xyz_pino = self.pino_data.oMi[-1].translation
            x.append(copy(xyz_pino[0]))
            y.append(copy(xyz_pino[1]))
            z.append(copy(xyz_pino[2]))
        x = np.stack(x)
        y = np.stack(y)
        z = np.stack(z)
        return x, y, z

    def abs_table_constraint(self, x):
        x, y, z = self.table_constraint(x)
        z_loss = 0.01 - np.abs(z - TableConstraint.Z)
        x_loss = x - TableConstraint.XLB
        y_loss = TableConstraint.YRT - np.abs(y)
        loss = np.concatenate([x_loss, y_loss, z_loss])
        return loss

    #def table_constraint_jac(self, x):
    #    q = np.reshape(x[:self.M * self.D], (self.D, self.M)).T
    #    Js = []
    #    for i in range(self.M):
    #        q_ = np.concatenate([q[i], np.zeros(3)], axis=-1)
    #        J_z = pino.computeFrameJacobian(self.pino_model, self.pino_data, q_, self.joint_id, pino.LOCAL_WORLD_ALIGNED)[2, :6]
    #        Js.append(J_z)
    #    Js = np.stack(Js)
    #    grad = np.zeros((self.M, x.shape[0]))
    #    for i in range(self.M):
    #        grad[i, np.arange(0, self.D * self.M, self.M) + i] = Js[i]
    #    c_val = self.table_constraint(x)
    #    dcdf = np.where(c_val > 0., -np.ones_like(c_val), np.ones_like(c_val))
    #    return grad * dcdf[:, np.newaxis]

    #def table_constraint(self, x):
    #    q = np.reshape(x[:self.M * self.D], (self.D, self.M)).T
    #    z = []
    #    for i in range(self.M):
    #        q_ = np.concatenate([q[i], np.zeros(3)], axis=-1)
    #        pino.forwardKinematics(self.pino_model, self.pino_data, q_)
    #        xyz_pino = self.pino_data.oMi[-1].translation
    #        z.append(copy(xyz_pino[-1]))
    #    z = np.stack(z)
    #    z_dist = z - 0.16
    #    return z_dist

    #def abs_table_constraint(self, x):
    #    return 0.01 - np.abs(self.table_constraint(x))

    def ddq_constraint(self, x, q0, qk, dq0, dqk):
        dt = x[-1] / self.N
        q_dot = np.reshape(x[self.M * self.D:2 * self.M * self.D], (self.D, self.M)).T
        q_dot = np.concatenate([dq0[np.newaxis], q_dot, dqk[np.newaxis]], axis=0)
        q_ddot = np.reshape(x[2 * self.M * self.D:2 * self.M * self.D + self.N * self.D], (self.D, self.N)).T
        q_dot_diff = q_dot[1:] - q_dot[:-1]
        delta_q_dot = q_ddot[:-1] * dt
        r = q_dot_diff - delta_q_dot
        r = np.reshape(r.T, -1)
        return r

    def ddq_constraint_jac(self, x, q0, qk, dq0, dqk):
        dt = x[-1] / self.N
        q_ddot = np.reshape(x[2 * self.M * self.D:2 * self.M * self.D + self.N * self.D], (self.D, self.N)).T
        q_ddot_grad = np.reshape(q_ddot[:-1].T, -1)
        grad = np.concatenate([self.zeros, self.diff_diag, self.diag * dt, -q_ddot_grad[:, np.newaxis] / self.N], axis=-1)
        return grad

    def solve(self, q0, dq0, qk, dqk):
        q0 = q0[:self.D]
        dq0 = dq0[:self.D]
        qk = qk[:self.D]
        dqk = dqk[:self.D]

        q_ = np.concatenate([qk, np.zeros(3)], axis=-1)
        J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q_, self.joint_id, pino.LOCAL_WORLD_ALIGNED)[:3, :6]
        vk = J @ dqk
        print(vk)

        a_0 = q0[np.newaxis]
        a_1 = (dq0 + 3 * q0)[np.newaxis]
        a_3 = qk[np.newaxis]
        a_2 = (3 * qk - dqk)[np.newaxis]
        t = np.linspace(0., 1., self.N)[:, np.newaxis]
        q_ = a_3 * t ** 3 + a_2 * t ** 2 * (1 - t) + a_1 * t * (1 - t) ** 2 + a_0 * (1 - t) ** 3
        q_dot_ = 3 * a_3 * t ** 2 + a_2 * (-3 * t ** 2 + 2 * t) + a_1 * (
                3 * t ** 2 - 4 * t + 1) - a_0 * 3 * (1 - t) ** 2
        q_ddot_ = 6 * a_3 * t ** 1 + a_2 * (-6 * t + 2) + \
                  a_1 * (6 * t - 4) + a_0 * 6 * (1 - t)

        init_q = q_  # + 0.01 * (2*np.random.random((self.N, 1)) - 1.)
        init_dq = q_dot_
        init_ddq = q_ddot_

        q_dot_mul = np.max(np.abs(q_dot_) / Limits.q_dot[np.newaxis])
        q_ddot_mul = np.max(np.abs(q_ddot_) / Limits.q_ddot[np.newaxis])
        T0 = np.maximum(q_dot_mul, np.sqrt(q_ddot_mul))


        init_x = np.zeros((2 * (self.M * self.D) + self.N * self.D + 1))
        init_x[:self.M * self.D] = np.reshape(init_q[1:-1].T, -1)
        init_x[self.M * self.D:2 * self.M * self.D] = np.reshape(init_dq[1:-1].T, -1)
        init_x[2 * self.M * self.D:2 * self.M * self.D + self.N * self.D] = np.reshape(init_ddq.T, -1)
        init_x[-1] = T0

        qLimit = np.tile(Limits.q[:self.D, np.newaxis], (1, self.N - 2))
        qLimit = np.reshape(qLimit, -1)
        dqLimit = np.tile(Limits.q_dot[:self.D, np.newaxis], (1, self.N - 2))
        dqLimit = np.reshape(dqLimit, -1)
        ddqLimit = np.tile(Limits.q_ddot[:self.D, np.newaxis], (1, self.N))
        ddqLimit = np.reshape(ddqLimit, -1)
        lb = (-qLimit).tolist() + (-dqLimit).tolist() + (-ddqLimit).tolist() + [0.1]
        ub = qLimit.tolist() + dqLimit.tolist() + ddqLimit.tolist() + [None]
        bounds = list(zip(lb, ub))

        table_constraint_dict = {
            "type": "ineq",
            "fun": self.abs_table_constraint,
            "jac": self.table_constraint_jac,
        }

        dq_constraint_dict = {
            "type": "eq",
            "fun": self.dq_constraint,
            "jac": self.dq_constraint_jac,
            "args": [q0, qk, dq0, dqk],
        }

        ddq_constraint_dict = {
            "type": "eq",
            "fun": self.ddq_constraint,
            "jac": self.ddq_constraint_jac,
            "args": [q0, qk, dq0, dqk],
        }

        init_x_ = copy(init_x)
        tic = perf_counter()
        res = minimize(self.objective, init_x_, method="SLSQP", jac=self.jac, bounds=bounds,
                       constraints=[table_constraint_dict, dq_constraint_dict, ddq_constraint_dict],
                       options={"disp": False, "maxiter": 100})
        planning_time = perf_counter() - tic
        x = res.x

        q = x[:self.D * self.M]
        q = np.reshape(q, (self.D, self.M)).T
        q = np.concatenate([q0[np.newaxis], q, qk[np.newaxis]], axis=0)
        q_dot = np.reshape(x[self.M * self.D:2 * self.M * self.D], (self.D, self.M)).T
        dq = np.concatenate([dq0[np.newaxis], q_dot, dqk[np.newaxis]], axis=0)
        ddq = np.reshape(x[2 * self.M * self.D:2 * self.M * self.D + self.N * self.D], (self.D, self.N)).T
        t = np.linspace(0., x[-1], self.N)
        return q, dq, ddq, t, planning_time
