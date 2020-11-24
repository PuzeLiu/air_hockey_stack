import torch
import matplotlib.pyplot as plt

from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch
from TrajOpt.spline.bezier_spline import BezierTrajectory

import qpsolvers
import numpy as np
import os


def get_nullspace(A, rcond=None):
    ut, st, vht = torch.Tensor.svd(A, some=False, compute_uv=True)
    vht = vht.T
    Mt, Nt = ut.shape[0], vht.shape[1]
    if rcond is None:
        rcondt = torch.finfo(st.dtype).eps * max(Mt, Nt)
    tolt = torch.max(st) * rcondt
    numt = torch.sum(st > tolt, dtype=int)
    nullspace = vht[numt:, :].T.cpu().conj()
    # nullspace.backward(torch.ones_like(nullspace),retain_graph=True)
    return nullspace


class NullSpaceTrajectory:
    def __init__(self, q_0, x_f, v_f, table_height, kinematics: KinematicsTorch):
        self._q_0 = q_0
        self._x_f = x_f
        self._v_f = v_f
        self._table_height = table_height
        self._kine = kinematics

        self._x_0 = self._kine.forward_kinematics(self._q_0)
        lb = torch.tensor([0.62, -0.45, 0.18])
        ub = torch.tensor([2.6, 0.45, 0.20])
        self._bezier = BezierTrajectory(3, self._x_0[:3], self._x_f[:3], self._v_f[:3], lb, ub)

        self.t_f = self._bezier.t_f
        self.t_stop = torch.tensor(0.5)
        self.error_correction_gain = torch.ones(3).double()

    def generate_trajectory(self, step_size=0.01):
        trajectories_desired, t_desired = self._bezier.get_trajectory(step_size)

        trajectories_desired, t_desired = self.append_stop_trajectory(trajectories_desired, t_desired, step_size)

        q_i = self._q_0.clone()

        trajectories_joint = torch.zeros((trajectories_desired.shape[0], 2, 7)).double()
        trajectories_cartesian = torch.zeros_like(trajectories_desired)

        for i, t_i in enumerate(t_desired):
            x_i_desired = trajectories_desired[i, 0]
            x_d_i_desired = trajectories_desired[i, 1]
            x_i = self._kine.forward_kinematics(q_i)[:3]
            jac_i = self._kine.get_jacobian(q_i)[:3].detach()
            q_i = q_i.detach()
            null_jac = get_nullspace(jac_i)

            lamb = 1e-6
            K = torch.eye(3).double() * 10
            # Damped Least Squares
            A = jac_i.T @ torch.inverse(jac_i @ jac_i.T + lamb * torch.eye(3))
            # Position Errors
            b = A @ (K @ (x_i_desired - x_i) + x_d_i_desired)

            # Construct QP Solver
            W = torch.diag(torch.tensor([1., 20., 1., 20., 1., 10., 1.]).double())
            P, q, G, h = self.construct_QP(W, null_jac, b)
            alpha = qpsolvers.solve_qp(P, q, G, h)

            dq_i = b + null_jac @ torch.from_numpy(alpha)
            q_i += dq_i * step_size

            trajectories_joint[i, 0] = q_i
            trajectories_joint[i, 1] = dq_i

            trajectories_cartesian[i, 0] = x_i
            trajectories_cartesian[i, 1] = jac_i @ dq_i
        return trajectories_joint, trajectories_cartesian, t_desired

    def construct_QP(self, W, N_J, b):
        """
        Construc the QP solver:
            minimize 1/2 x^T Q x + q^T x
            s.t.            G x <= h
                            A x  = b
                        lb <= x <= ub
        Args:
            W: [7 * 7] Weight matrix
            N_J: [7 * 4] The nullspace of the Jacobian
            b: [7 * 1] Projection of cartesian error: N_J * error
        Returns:
            Q: Quadratic Term
            p: Linear Term
            G: Inequality constraint LHS
            h: Inequality constraint RHS
        """
        P = N_J.T @ W @ N_J
        q = b.T @ W @ N_J
        G = np.concatenate([N_J, - N_J], axis=0)
        h = np.concatenate([(self._kine.joint_vel_limits[:, 1] - b).detach().numpy(),
                            (b - self._kine.joint_vel_limits[:, 0]).detach().numpy()])
        return P.detach().numpy(), q.detach().numpy(), G, h

    def append_stop_trajectory(self, trajectory_desired, t_desired, step_size):

        v0 = torch.norm(trajectory_desired[-1, 1])
        v_dir = trajectory_desired[-1, 1] / v0

        # quadratic polynomial
        # f(x) = b_0 x^2 + b_1 x + b_2
        b = torch.tensor([ -v0 / (2 * self.t_stop), v0, 0.])
        t_stop = torch.arange(0, self.t_stop, step_size) + step_size

        trajectory_stop = torch.empty((t_stop.shape[0], 3, trajectory_desired.shape[-1])).double()
        for i, t_i in enumerate(t_stop):
            x_t = trajectory_desired[-1, 0] + v_dir * (b[0] * t_i ** 2 + b[1] * t_i + b[2])
            dx_t = v_dir * (b[0] * t_i * 2 + b[1])
            ddx_t = 2 * b[0]
            trajectory_stop[i, 0] = x_t
            trajectory_stop[i, 1] = dx_t
            trajectory_stop[i, 2] = ddx_t
        trajectory_desired = torch.cat([trajectory_desired, trajectory_stop], dim=0)
        t_desired = torch.cat([t_desired, t_stop], dim=0)
        return trajectory_desired, t_desired



    def plot(self):
        trajectories_joint, trajectories_cartesian, t = self.generate_trajectory(0.01)
        trajectories_desired, t = self._bezier.get_trajectory(0.01)
        plt.figure()
        plt.plot(trajectories_cartesian[:, 0, 0], trajectories_cartesian[:, 0, 1])
        plt.plot(trajectories_desired[:, 0, 0], trajectories_desired[:, 0, 1])
        for i in range(7):
            plt.figure()
            plt.plot(trajectories_joint[:, 1, i])
            plt.plot(torch.ones(trajectories_joint.shape[0]) * self._kine.joint_vel_limits[i, 0], 'r-.')
            plt.plot(torch.ones(trajectories_joint.shape[0]) * self._kine.joint_vel_limits[i, 1], 'r-.')
            plt.title("Velocity Joint_{}".format(i + 1))
        plt.show()

if __name__ == "__main__":
    table_height = 0.1915
    q_0_test = torch.tensor([-9.2250914e-08,  5.8574259e-01,  5.2708398e-08, -1.0400484e+00,
                             -3.1588606e-08,  1.5158017e+00,  3.1415927e+00])
    x_f_test = torch.tensor([0.87022331, 0.20365358, 0.1915])
    v_f_test = torch.tensor([1.37978352e+00, -1.69298925e-01, 5.62490350e-08])
    tcp_pos = torch.tensor([0., 0., 0.3455])
    tcp_quat = torch.tensor([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=tcp_pos,
                                 tcp_quat=tcp_quat)
    output_dir = "result"
    file_name = "trajectory"

    null_space_traj = NullSpaceTrajectory(q_0_test, x_f_test, v_f_test, table_height, kinematics)
    null_space_traj.plot()
    trajectories_joint, trajectories_cartesian, t = null_space_traj.generate_trajectory(0.01)


    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    np.save(os.path.join(output_dir, file_name), trajectories_joint.detach().numpy())
