import torch
import matplotlib.pyplot as plt
import numpy as np
import os


class MonotonicCubicSpline:
    def __init__(self, n_dim, x_0, x_i, x_f, t_f):
        self._n_dim = n_dim
        self._n_points = x_i.shape[0] + 2
        self._n_segments = x_i.shape[0] + 1
        if x_0.dim() == 1:
            x_0 = x_0.unsqueeze(0)
        if x_f.dim() == 1:
            x_f = x_f.unsqueeze(0)
        self._points = torch.cat([x_0, x_i, x_f])
        self._t_f = t_f

        self._t_i = torch.zeros(self._n_points)
        for i in range(self._n_points):
            self._t_i[i] = i * self._t_f / self._n_segments
        self._h_i = (self._t_i[1:] - self._t_i[:-1]).unsqueeze(1)

        self._delta_k = (self._points[1:] - self._points[:-1]) / (self._t_i[1:] - self._t_i[:-1]).unsqueeze(1)
        self._m_k = torch.zeros((self._n_points, self._n_dim))
        self._m_k[0] = self._delta_k[0]
        self._m_k[-1] = self._delta_k[-1]
        for i in range(1, self._n_points - 1):
            self._m_k[i] = (self._delta_k[i] + self._delta_k[i - 1]) / 2
            idx = torch.le(self._delta_k[i] * self._delta_k[i - 1], 0.)
            self._m_k[i, idx] = 0.
            self._m_k[i + 1, idx] = 0.

        for i in range(1, self._delta_k.shape[0]):
            for j in range(self._n_dim):
                if not torch.isclose(self._delta_k[i, j], torch.tensor(0.)):
                    alpha_i = self._m_k[i, j] / self._delta_k[i, j]
                    beta_i = self._m_k[i + 1, j] / self._delta_k[i, j]
                    if alpha_i < 0.:
                        self._m_k[i, j] = 0.
                    if beta_i < 0.:
                        self._m_k[i + 1, j] = 0.
                    if (alpha_i ** 2 + beta_i ** 2) > 9.:
                        tau_i = 3. / torch.sqrt(alpha_i ** 2 + beta_i ** 2)
                        self._m_k[i, j] = tau_i * alpha_i * self._delta_k[i, j]
                        self._m_k[i + 1, j] = tau_i * beta_i * self._delta_k[i, j]

    @property
    def v_f(self):
        return self._m_k[-1]

    @staticmethod
    def _h_00(t):
        return 2 * (t ** 3) - 3 * (t ** 2) + 1

    @staticmethod
    def _h_00_d(t):
        return 6 * (t ** 2) - 6 * t

    @staticmethod
    def _h_00_dd(t):
        return 12 * t - 6

    @staticmethod
    def _h_10(t):
        return t ** 3 - 2 * (t ** 2) + t

    @staticmethod
    def _h_10_d(t):
        return 3 * (t ** 2) - 4 * t + 1

    @staticmethod
    def _h_10_dd(t):
        return 6 * t - 4

    @staticmethod
    def _h_01(t):
        return - 2 * (t ** 3) + 3 * (t ** 2)

    @staticmethod
    def _h_01_d(t):
        return - 6 * (t ** 2) + 6 * t

    @staticmethod
    def _h_01_dd(t):
        return - 12 * t + 6

    @staticmethod
    def _h_11(t):
        return t ** 3 - t ** 2

    @staticmethod
    def _h_11_d(t):
        return 3 * (t ** 2) - 2 * t

    @staticmethod
    def _h_11_dd(t):
        return 6 * t - 2

    def __call__(self, t_i):
        for i in range(self._n_segments):
            if self._t_i[i] <= t_i <= self._t_i[i + 1]:
                t_x = (t_i - self._t_i[i]) / self._h_i[i]
                x = self._points[i] * self._h_00(t_x) + self._h_i[i] * self._m_k[i] * self._h_10(t_x) + \
                    self._points[i + 1] * self._h_01(t_x) + self._h_i[i] * self._m_k[i + 1] * self._h_11(t_x)
                dx = self._points[i] * self._h_00_d(t_x) + self._h_i[i] * self._m_k[i] * self._h_10_d(t_x) + \
                    self._points[i + 1] * self._h_01_d(t_x) + self._h_i[i] * self._m_k[i + 1] * self._h_11_d(t_x)
                ddx = self._points[i] * self._h_00_dd(t_x) + self._h_i[i] * self._m_k[i] * self._h_10_dd(t_x) + \
                    self._points[i + 1] * self._h_01_dd(t_x) + self._h_i[i] * self._m_k[i + 1] * self._h_11_dd(t_x)
                return x, dx, ddx

    def get_trajectory(self, step_size=0.01):
        t = torch.arange(0, self._t_f, step_size)
        trajectory = torch.empty((t.shape[0], 3, self._n_dim))
        for i, t_i in enumerate(t):
            x_t, dx_t, ddx_t = self(t_i)
            trajectory[i, 0] = x_t
            trajectory[i, 1] = dx_t
            trajectory[i, 2] = ddx_t
        return trajectory, t

    def plot(self, save_dir=None):
        trajectories, t = self.get_trajectory()
        x = trajectories[:, 0].detach().numpy()
        dx = trajectories[:, 1, :].detach().numpy()
        ddx = trajectories[:, 2, :].detach().numpy()

        for i in range(self._n_dim):
            fig, axes = plt.subplots(3)
            fig.suptitle("Trajectory for dimension {}".format(i))
            axes[0].plot(t, x[:, i])
            axes[0].scatter(np.linspace(0., self._t_f, self._n_points)[1:-1], self._points[1:-1, i], marker='*')
            axes[0].set_title("Position")
            axes[1].plot(t, dx[:, i])
            axes[1].set_title("Velocity")
            # max_velocities, tau = self.get_max_velocities()
            # axes[1].scatter(tau[:, i], max_velocities[:, i], marker='*')
            axes[2].plot(t, ddx[:, i])
            axes[2].set_title("Acceleration")
            if not (save_dir is None):
                fig.savefig(os.path.join(save_dir, "joint_{}.png".format(i)))
                plt.close(fig)

        if save_dir is None:
            plt.show()

    def get_max_velocities(self):
        max_velocities = torch.zeros((self._n_segments, self._n_dim)).double()
        tau = torch.zeros(self._n_segments, self._n_dim).double()
        for i in range(self._n_segments):
            for j in range(self._n_dim):
                a = (6 * self._points[i, j] - 6 * self._points[i + 1, j] + 3 * self._h_i[i] * self._m_k[i, j] +
                     3 * self._h_i[i] * self._m_k[i + 1, j])
                b = (- 6 * self._points[i, j] + 6 * self._points[i + 1, j] - 4 * self._h_i[i] * self._m_k[i, j]
                     - 2 * self._h_i[i] * self._m_k[i + 1, j])
                c = self._h_i[i] * self._m_k[i, j]
                if a == 0.:
                    if b > 0.:
                        tau[i, j] = 1.
                        max_velocities[i, j] = a + b + c
                    else:
                        tau[i, j] = 0.
                        max_velocities[i, j] = c
                else:
                    tau[i, j] = - b / (2 * a)
                    if tau[i, j] < 0. or tau[i, j] > 1.:
                        max_velocities[i, j] = c if a < 0 else a + b + c
                    else:
                        max_velocities[i, j] = (4 * a * c - b ** 2) / (4 * a)

        # t_at_max = self._h_i * tau + self._t_i[:-1].unsqueeze(1)
        return max_velocities


if __name__ == "__main__":
    n_dim = 2
    x_0 = torch.tensor([0., 0.])
    x_f = torch.tensor([-10., 10.])
    x_i = torch.tensor([[0., 2.],
                        [4., 4.],
                        [4., 4.],
                        [5., 5.],
                        [6., 6.],
                        [2., 2.],
                        [-15., 5.],
                        [8., 8.]])
    t_f = 1

    spline = MonotonicCubicSpline(n_dim, x_0, x_i, x_f, t_f)
    max_velocities = spline.get_max_velocities()
    spline.plot()
