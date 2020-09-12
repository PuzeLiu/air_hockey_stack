import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt


class CubicSpline(nn.Module):
    def __init__(self, n_dim):
        super(CubicSpline, self).__init__()
        self.n_dim = n_dim
        self.n_points = 2   # 2 points without intermediate points
        self.n_segments = 1
        self.points = torch.zeros((self.n_points, n_dim)).double()
        self.K = torch.zeros((self.n_points, self.n_points)).double()
        self.M = torch.zeros((self.n_points, self.n_dim)).double()

    def set_boudaries(self, x0, xf, v0, vf, tf):
        if x0.shape[0] != self.n_dim or xf.shape[0] != self.n_dim:
            raise ValueError("x0 and xf should be dimension of ", self.n_dim)
        else:
            self.points[0] = x0 if isinstance(x0, torch.Tensor) else torch.tensor(x0)
            self.points[-1] = xf if isinstance(xf, torch.Tensor) else torch.tensor(xf)

        self.tf = tf

    def set_via_points(self, x_i):
        if x_i.shape[1] is not self.n_dim:
            raise ValueError("xi should be dimension of ", self.n_dim)
        self.n_points = x_i.shape[0] + 2
        self.n_segments = n_via_points + 1
        self.x_i = torch.nn.Parameter(x_i if isinstance(x_i, torch.Tensor) else torch.tensor(x_i))
        self.points = torch.cat((self.points[[0]], self.x_i, self.points[[1]]), dim=0)
        self.K = torch.zeros((self.n_points, self.n_points)).double()

        self.t_i = torch.linspace(0, self.tf, self.n_points)

        # h_i = [0, h1, ..., hn]
        self.h_i = torch.cat((torch.tensor([0.]), self.t_i[1:] - self.t_i[:-1]), dim=0)

        # set K[1:n-1, 1:n-1] matrix from 1 to n-1
        for i in range(1, self.n_points - 1):
            self.K[i, i - 1] = 1 / self.h_i[i]
            self.K[i, i] = 2 * (1 / self.h_i[i] + 1/ self.h_i[i + 1])
            self.K[i, i + 1] = 1 / self.h_i[i + 1]
            self.b[i] = 3 * ((self.points[i] - self.points[i - 1]) / (self.h_i[i])**2 +
                             (self.points[i + 1] - self.points[i]) / (self.h_i[i + 1])**2)

    def fit(self):
        v0 = torch.zeros(2)
        vf = torch.zeros(2)
        d_i = 6 * self._divide_difference(v0, vf)
        self.M = torch.mm(torch.inverse(self.K), d_i)

    def _divide_difference(self, v0, vf):
        # d_i = [d_1, ..., d_{n-1}]
        d_i = 6 * ((self.points[2:] - self.points[1:-1]) / self.h_i[1:].unsqueeze(-1) -
                   (self.points[1:-1] - self.points[:-2]) / self.h_i[:-1].unsqueeze(-1)) / \
              (self.h_i[1:] + self.h_i[:-1]).unsqueeze(-1)
        d_0 = 6 * ((self.points[1] - self.points[0]) / self.h_i[0] - v0) / self.h_i[0]
        d_n = 6 * (vf - (self.points[-1] - self.points[-2]) / self.h_i[-1]) / self.h_i[-1]
        return torch.cat((d_0.unsqueeze(0), d_i, d_n.unsqueeze(0)), dim=0)

    def __call__(self, t_i):
        for i in range(1, self.n_points):
            if self.t_i[i - 1] <= t_i <= self.t_i[i]:
                return

if __name__ == "__main__":
    n_dim = 2
    n_via_points = 4
    x0 = np.array([1, 2])
    xf = np.array([10, 20])
    tf = 5

    cubic_spline = CubicSpline(n_dim)
    cubic_spline.set_boudaries(x0, xf, tf)
    via_points = np.linspace(x0, xf, n_via_points + 2)[1:-1]    # + np.random.random((n_via_points, n_dim))
    cubic_spline.set_via_points(via_points)
    cubic_spline.fit()
    x = []
    dx = []
    ddx = []
    for t in np.linspace(0, tf, 1000):
        x_t, dx_t, ddx_t = cubic_spline(t)
        x.append(x_t.detach().numpy())
        dx.append(dx_t.detach().numpy())
        ddx.append(ddx_t.detach().numpy())
    x = np.array(x)
    dx = np.array(dx)
    ddx = np.array(ddx)

    plt.figure()
    plt.plot(np.linspace(0, tf, 1000), x)
    plt.scatter(np.linspace(0, tf, n_via_points + 2)[1:-1], via_points[:, 0], marker='o')
    plt.scatter(np.linspace(0, tf, n_via_points + 2)[1:-1], via_points[:, 1], marker='*')
    plt.figure()
    plt.plot(np.linspace(0, tf, 1000), dx)
    plt.figure()
    plt.plot(np.linspace(0, tf, 1000), ddx)
    plt.scatter(np.linspace(0, tf, n_via_points + 2), cubic_spline.M[:, 0].detach().numpy(), marker='o')
    plt.scatter(np.linspace(0, tf, n_via_points + 2), cubic_spline.M[:, 1].detach().numpy(), marker='*')
    plt.show()
