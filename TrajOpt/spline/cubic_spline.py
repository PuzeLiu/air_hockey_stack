import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt


class CubicSpline(nn.Module):
    def __init__(self, n_dim, n_via_points):
        super(CubicSpline, self).__init__()
        self.n_dim = n_dim
        self.n_points = n_via_points + 2
        self.n_segments = n_via_points + 1
        self.points = torch.zeros((n_via_points + 2, n_dim))
        self.register_parameter("x_i", torch.nn.Parameter(torch.zeros(n_via_points, n_dim)))
        self.K = torch.zeros((self.n_points, self.n_points))
        self.M = torch.zeros((self.n_points, self.n_dim))

    def set_boudaries(self, x0, xf, tf):
        if x0.shape[0] != self.n_dim or xf.shape[0] != self.n_dim:
            raise ValueError("x0 and xf should be dimension of ", self.n_dim)
        else:
            self.points[0] = x0 if isinstance(x0, torch.Tensor) else torch.tensor(x0)
            self.points[-1] = xf if isinstance(xf, torch.Tensor) else torch.tensor(xf)

        self.t_i = torch.linspace(0, tf, self.n_segments + 1)

        # h_i = [h1, ..., hn]
        self.h_i = self.t_i[1:] - self.t_i[:-1]
        # mu_i = [mu_1, mu_2, ..., mu_{n-1}]
        mu_i = self.h_i[:-1] / (self.h_i[:-1] + self.h_i[1:])
        mu_n = torch.ones(1)
        # lambda_i = [lambda_1, ..., lambda_{n-1}]
        lambda_i = self.h_i[1] / (self.h_i[:-1] + self.h_i[1])
        lambda_0 = torch.ones(1)

        self.K = torch.diag(torch.ones(self.points.shape[0]) * 2) + \
                 torch.diag(torch.cat((lambda_0, lambda_i), dim=0), diagonal=1) + \
                 torch.diag(torch.cat((mu_i, mu_n), dim=0), diagonal=-1)

    def set_via_points(self, x_i):
        if x_i.shape[0] is not self.n_segments - 1 or x_i.shape[1] is not self.n_dim:
            raise ValueError("xi should be dimension of ", self.n_dim)
        self.x_i = torch.nn.Parameter(x_i if isinstance(x_i, torch.Tensor) else torch.tensor(x_i))
        self.points[1:-1] = self.x_i

    def fit(self):
        v0 = torch.zeros(2)
        vf = torch.zeros(2)
        d_i = 6 * self._divide_difference(v0, vf)
        self.M = torch.mm(torch.inverse(self.K), d_i)

    def _divide_difference(self, v0, vf):
        # d_i = [d_1, ..., d_{n-1}]
        d_i = ((self.points[2:] - self.points[1:-1]) / self.h_i[1:].unsqueeze(-1)
                - (self.points[1:-1] - self.points[:-2]) / self.h_i[:-1].unsqueeze(-1)) / \
              (self.h_i[1:] + self.h_i[:-1]).unsqueeze(-1)
        d_0 = 6 / self.h_i[0] * ((self.points[1] - self.points[0]) / self.h_i[0] - v0)
        d_n = 6 / self.h_i[-1] * (vf - (self.points[-1] - self.points[-2]) / self.h_i[-1])
        return torch.cat((d_0.unsqueeze(0), d_i, d_n.unsqueeze(0)), dim=0)

    def __call__(self, t):
        for i in range(1, self.n_points):
            if t >= self.t_i[i-1] and t<=self.t_i[i]:
                return self.M[i - 1] * (self.t_i[i] - t) ** 3 / (6 * self.h_i[i - 1]) + \
                       self.M[i] * (t - self.t_i[i - 1]) ** 3 / (6 * self.h_i[i - 1]) + \
                       (self.points[i - 1] - self.M[i - 1] * self.h_i[i - 1]**2 / 6) * (self.t_i[i] - t) / self.h_i[i - 1] + \
                       (self.points[i] - self.M[i] * self.h_i[i - 1]**2 / 6) * (t - self.t_i[i - 1]) / self.h_i[i - 1]

if __name__ == "__main__":
    cubic_spline = CubicSpline(2, 5)
    cubic_spline.set_boudaries(np.array([1, 2]), np.array([10, 20]), 5)
    via_points = np.random.random((5, 2)) * 10
    cubic_spline.set_via_points(via_points)
    cubic_spline.fit()
    x = []
    for t in np.linspace(0, 5, 1000):
        x.append(cubic_spline(t).detach().numpy())
    x = np.array(x)

    plt.figure()
    plt.plot(np.linspace(0, 5, 1000), x)
    plt.scatter(np.linspace(0, 5, 7)[1:-1], via_points[:, 0], marker='o')
    plt.scatter(np.linspace(0, 5, 7)[1:-1], via_points[:, 1], marker='*')
    plt.show()