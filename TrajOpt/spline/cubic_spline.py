import matplotlib.pyplot as plt
import numpy as np
import torch


class CubicSpline:
    def __init__(self, n_dim, x_0, x_i, x_f, b_0, b_f, t_f, bound_type='velocity'):
        self.n_dim = n_dim
        self.n_points = x_i.shape[0] + 2
        self.n_segments = x_i.shape[0] + 1
        if x_0.dim() == 1:
            x_0 = x_0.unsqueeze(0)
        if x_f.dim() == 1:
            x_f = x_f.unsqueeze(0)
        self.points = torch.cat([x_0, x_i, x_f])
        self.t_f = t_f

        self.K = torch.zeros((self.n_points, self.n_points))
        self.b = torch.zeros((self.n_points, self.n_dim))
        self.t_i = torch.linspace(0, self.t_f, self.n_points)
        # h_i = [0, h1, ..., hn]
        self.h_i = (self.t_i[1:] - self.t_i[:-1]).unsqueeze(1)

        if bound_type == 'acceleration':
            # set initial and end value
            self.K[0, 0] = 2 / self.h_i[0]
            self.K[0, 1] = 1 / self.h_i[0]
            self.b[0] = 3 * (self.points[1] - self.points[0]) / (self.h_i[0]) ** 2 - b_0 / 2

            self.K[-1, -1] = 2 / self.h_i[-1]
            self.K[-1, -2] = 1 / self.h_i[-1]
            self.b[-1] = 3 * (self.points[-1] - self.points[-2]) / (self.h_i[-1]) ** 2 - b_f / 2
        elif bound_type == 'velocity':
            # set initial and end value
            self.K[0, 0] = 1
            self.K[0, 1] = 0
            self.b[0] = b_0

            self.K[-1, -1] = 1
            self.K[-1, -2] = 0
            self.b[-1] = b_f
        else:
            raise ValueError("Unknown Boundary Type")

        for i in range(1, self.n_points - 1):
            self.K[i, i - 1] = 1 / self.h_i[i - 1]
            self.K[i, i] = 2 * (1 / self.h_i[i - 1] + 1 / self.h_i[i])
            self.K[i, i + 1] = 1 / self.h_i[i]
            self.b[i] = 3 * ((self.points[i] - self.points[i - 1]) / (self.h_i[i - 1]) ** 2 +
                             (self.points[i + 1] - self.points[i]) / (self.h_i[i]) ** 2)

        self.k, _ = torch.solve(self.b, self.K)
        self.a = self.k[:-1] * self.h_i - (self.points[1:] - self.points[:-1])
        self.b = -self.k[1:] * self.h_i + (self.points[1:] - self.points[:-1])

    def get_max_velocities(self):
        max_velocities = torch.zeros((self.n_segments, self.n_dim)).double()
        tau = torch.zeros(self.n_segments, self.n_dim)
        for i in range(self.n_segments):
            tau[i] = (2 * self.a[i] - self.b[i]) / (self.a[i] - self.b[i]) / 3
            max_velocities[i] = ((self.points[i + 1] - self.points[i] + self.a[i]) -
                                 (2 * self.a[i] - self.b[i]) ** 2 / (self.a[i] - self.b[i]) / 3) / self.h_i[i]

        if torch.any(torch.lt(tau, 0.)):
            id_x, id_y = torch.where(tau < 0.)
            max_velocities[id_x, id_y] = (self.points[id_x + 1, id_y] - self.points[id_x, id_y] + self.a[id_x, id_y]) / self.h_i[id_x, 0]
            tau[id_x, id_y] = 0.
        if torch.any(torch.gt(tau, 1.)):
            id_x, id_y = torch.where(tau > 1.)
            max_velocities[id_x, id_y] = (self.points[id_x + 1, id_y] - self.points[id_x, id_y] - self.b[id_x, id_y]) / self.h_i[id_x, 0]
            tau[id_x, id_y] = 1.

        # t_at_max = self.h_i * tau + self.t_i[:-1].unsqueeze(1)
        return max_velocities

    def __call__(self, t_i):
        for i in range(self.n_segments):
            if self.t_i[i] <= t_i <= self.t_i[i + 1]:
                t_x = (t_i - self.t_i[i]) / self.h_i[i]
                x = (1 - t_x) * self.points[i] + t_x * self.points[i + 1] + t_x * (1 - t_x) * (
                        (1 - t_x) * self.a[i] + t_x * self.b[i])
                dx = ((self.points[i + 1] - self.points[i]) + (1 - 2 * t_x) * (self.a[i] * (1 - t_x) + self.b[i] * t_x)
                      + t_x * (1 - t_x) * (self.b[i] - self.a[i])) / self.h_i[i]
                ddx = 2 * (self.b[i] - 2 * self.a[i] + (self.a[i] - self.b[i]) * t_x * 3) / (self.h_i[i] ** 2)
                return x, dx, ddx

    def get_trajectory(self, step_size):
        t = torch.arange(0, self.t_f + step_size, step_size)
        trajectory = torch.empty((t.shape[0], 3, self.n_dim))
        for i, t_i in enumerate(t):
            x_t, dx_t, ddx_t = self(t_i)
            trajectory[i, 0] = x_t
            trajectory[i, 1] = dx_t
            trajectory[i, 2] = ddx_t
        return trajectory, t

    def plot(self):
        step_size = 0.01
        trajectories, t = self.get_trajectory(step_size)
        x = trajectories[:, 0].detach().numpy()
        dx = trajectories[:, 1, :].detach().numpy()
        ddx = trajectories[:, 2, :].detach().numpy()

        for i in range(self.n_dim):
            fig, axes = plt.subplots(3)
            fig.suptitle("Trajectory for dimension {}".format(i))
            axes[0].plot(t, x[:, i])
            axes[0].scatter(np.linspace(0., self.t_f, self.n_points)[1:-1], self.points[1:-1, i], marker='*')
            axes[0].set_title("Position")
            axes[1].plot(t, dx[:, i])
            axes[1].set_title("Velocity")
            axes[2].plot(t, ddx[:, i])
            axes[2].set_title("Acceleration")
        plt.show()


if __name__ == "__main__":
    import numpy as np

    n_dim = 2
    n_via_points = 5
    p0 = torch.tensor([1., 2.])
    pf = torch.tensor([10., 20.])
    p_via = torch.tensor(np.linspace(p0, pf, n_via_points + 2)[1:-1] + 100 * np.random.random((n_via_points, n_dim)))
    bound_0 = torch.tensor([0., 0.])
    bound_f = torch.tensor([10., 10.])
    tf = 2

    cubic_spline = CubicSpline(n_dim, p0, p_via, pf, bound_0, bound_f, tf, bound_type='velocity')
    cubic_spline.plot()
