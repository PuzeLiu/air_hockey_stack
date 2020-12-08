import torch
import os
import matplotlib.pyplot as plt


class BezierTrajectory:
    def __init__(self, dim, x0, xf, vf, lb, ub, phase_polynom="quartic"):
        self._lb = lb
        self._ub = ub
        self._dim = dim

        self._phase_polynom = phase_polynom

        self.x0 = x0
        self.xf = xf
        self.vf = vf

        # Find the length to hit the boundary
        alpha_l = (self.xf - self._lb) / self.vf
        alpha_u = (self.xf - self._ub) / self.vf

        alpha = torch.cat([alpha_l, alpha_u], dim=0)
        alpha_min = torch.min(alpha[torch.gt(alpha, 0.)])
        self.xm = self.xf - alpha_min * self.vf

        # Cubic Trajectory on Phase Variable.
        if self._phase_polynom == "cubic":
            self._a = torch.zeros(4).double()
            self.z_f = 1
            self.dz_f = torch.mean(self.vf[0] / (self.xf[0] - self.xm[0]) / 2)
            self.t_f = 3 * (self.z_f - 0) / self.dz_f
            self._a[0] = 0.
            self._a[1] = 0.
            self._a[2] = 0.
            self._a[3] = self.z_f / self.t_f ** 3
        elif self._phase_polynom == "quartic":
            self._a = torch.zeros(5).double()
            self.dz_f = torch.mean(self.vf[0] / (self.xf[0] - self.xm[0]) / 2)
            self.t_f = 2 / self.dz_f
            self._a[0] = 0.
            self._a[1] = 0.
            self._a[2] = 0.
            self._a[3] = self.dz_f ** 3 / 4.
            self._a[4] = - self.dz_f ** 4 / 16.



    def get_point_i(self, t):
        if self._phase_polynom == "cubic":
            z = self._a[3] * t ** 3
            dz_dt = 3 * self._a[3] * t ** 2
            dz_ddt = 6 * self._a[3] * t
        elif self._phase_polynom == "quartic":
            z = self._a[3] * t ** 3 + self._a[4] * t ** 4
            dz_dt = 3 * self._a[3] * t ** 2 + 4 * self._a[4] * t ** 3
            dz_ddt = 6 * self._a[3] * t + 12 * self._a[4] * t ** 2


        x = self.xm + (1 - z) ** 2 * (self.x0 - self.xm) + z ** 2 * (self.xf - self.xm)
        dx_dz = 2 * (1 - z) * (self.xm - self.x0) + 2 * z * (self.xf - self.xm)
        dx_dt = dx_dz * dz_dt
        dx_ddz = 2 * (self.xf - 2 * self.xm + self.x0)
        dx_ddt = dx_ddz * (dz_dt ** 2) + dx_dz * dz_ddt
        return x, dx_dt, dx_ddt

    def get_trajectory(self, step_size=0.01):
        t = torch.arange(0, self.t_f + step_size, step_size)
        trajectory = torch.empty((t.shape[0], 3, self._dim)).double()
        for i, t_i in enumerate(t):
            x_t, dx_t, ddx_t = self.get_point_i(t_i)
            trajectory[i, 0] = x_t
            trajectory[i, 1] = dx_t
            trajectory[i, 2] = ddx_t
        return trajectory, t

    def plot(self, save_dir=None):
        trajectories, t = self.get_trajectory()
        x = trajectories[:, 0].detach().numpy()
        dx = trajectories[:, 1, :].detach().numpy()
        ddx = trajectories[:, 2, :].detach().numpy()

        for i in range(self._dim):
            fig, axes = plt.subplots(3)
            fig.suptitle("Trajectory for dimension {}".format(i))
            axes[0].plot(t, x[:, i])
            axes[0].set_title("Position")
            axes[1].plot(t, dx[:, i])
            axes[1].set_title("Velocity")
            axes[2].plot(t, ddx[:, i])
            axes[2].set_title("Acceleration")
            if not (save_dir is None):
                fig.savefig(os.path.join(save_dir, "joint_{}.png".format(i)))
                plt.close(fig)

        fig = plt.figure()
        fig.suptitle("Trajectory in X-Y plane")
        plt.plot(x[:, 0], x[:, 1])
        plt.plot([self._lb[0], self._lb[0]], [self._lb[1], self._ub[1]])
        plt.plot([self._lb[0], self._ub[0]], [self._lb[1], self._lb[1]])
        plt.plot([self._ub[0], self._ub[0]], [self._lb[1], self._ub[1]])
        plt.plot([self._lb[0], self._ub[0]], [self._ub[1], self._ub[1]])
        plt.scatter(self.xm[0], self.xm[1], marker='x', color='r')
        if not (save_dir is None):
            fig.savefig(os.path.join(save_dir, "joint_{}.png".format(i)))
            plt.close(fig)

        if save_dir is None:
            plt.show()


if __name__ == "__main__":
    dim = 2
    x0_test = torch.tensor([0.6, 0.0])
    xf_test = torch.tensor([0.9, 0.2])
    vf_test = torch.tensor([-1.0, 0.2])
    lb_test = torch.tensor([0.6, -0.45])
    ub_test = torch.tensor([2.5, 0.45])

    bezier = BezierTrajectory(2, x0_test, xf_test, vf_test, lb_test, ub_test)
    bezier.plot()