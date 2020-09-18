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

    def set_boudaries(self, x0, xf, b0, bf, tf):
        if x0.shape[0] != self.n_dim or xf.shape[0] != self.n_dim:
            raise ValueError("x0 and xf should be dimension of ", self.n_dim)
        else:
            self.points[0] = (x0 if isinstance(x0, torch.Tensor) else torch.tensor(x0)).double()
            self.points[-1] = (xf if isinstance(xf, torch.Tensor) else torch.tensor(xf)).double()
        self.b0 = (b0 if isinstance(b0, torch.Tensor) else torch.tensor(b0)).double()
        self.bf = (bf if isinstance(bf, torch.Tensor) else torch.tensor(bf)).double()
        self.tf = tf

    def set_via_points(self, x_i):
        if x_i.shape[1] is not self.n_dim:
            raise ValueError("xi should be dimension of ", self.n_dim)
        self.n_points = x_i.shape[0] + 2
        self.n_segments = n_via_points + 1
        self.x_i = torch.nn.Parameter(x_i if isinstance(x_i, torch.Tensor) else torch.tensor(x_i))
        self.points = torch.cat((self.points[[0]], self.x_i, self.points[[1]]), dim=0)
        self.K = torch.zeros((self.n_points, self.n_points)).double()
        self.b = torch.zeros((self.n_points, self.n_dim)).double()

        self.t_i = torch.linspace(0, self.tf, self.n_points)

        # h_i = [0, h1, ..., hn]
        self.h_i = self.t_i[1:] - self.t_i[:-1]

        # set K[1:n-1, 1:n-1] matrix from 1 to n-1
        for i in range(1, self.n_points - 1):
            self.K[i, i - 1] = 1 / self.h_i[i - 1]
            self.K[i, i] = 2 * (1 / self.h_i[i - 1] + 1/ self.h_i[i])
            self.K[i, i + 1] = 1 / self.h_i[i]
            self.b[i] = 3 * ((self.points[i] - self.points[i - 1]) / (self.h_i[i - 1]) ** 2 +
                             (self.points[i + 1] - self.points[i]) / (self.h_i[i]) ** 2)


    def fit(self, type='acceleration'):
        if type is 'acceleration':
            # set initial and end value
            self.K[0, 0] = 2 / self.h_i[0]
            self.K[0, 1] = 1 / self.h_i[0]
            self.b[0] = 3 * (self.points[1] - self.points[0]) / (self.h_i[0]) ** 2 - self.b0 / 2

            self.K[-1, -1] = 2 / self.h_i[-1]
            self.K[-1, -2] = 1 / self.h_i[-1]
            self.b[-1] = 3 * (self.points[-1] - self.points[-2]) / (self.h_i[-1]) ** 2 - self.bf / 2
        elif type is 'velocity':
            # set initial and end value
            self.K[0, 0] = 1
            self.K[0, 1] = 0
            self.b[0] = self.b0

            self.K[-1, -1] = 1
            self.K[-1, -2] = 0
            self.b[-1] = self.bf

        self.k = torch.mm(torch.inverse(self.K), self.b)
        self.a = self.k[:-1] * self.h_i.unsqueeze(1) - (self.points[1:] - self.points[:-1])
        self.b = -self.k[1:] * (self.h_i).unsqueeze(1) + (self.points[1:] - self.points[:-1])

    def __call__(self, t_i):
        for i in range(self.n_points - 1):
            if self.t_i[i] <= t_i <= self.t_i[i + 1]:
                t_x = (t_i - self.t_i[i]) / self.h_i[i]
                x = (1 - t_x) * self.points[i] + t_x * self.points[i + 1] + t_x * (1 - t_x) * ((1 - t_x) * self.a[i] + t_x *self.b[i])
                dx = ((self.points[i + 1] - self.points[i]) + (1 - 2 * t_x) * (self.a[i] * (1 - t_x) + self.b[i] * t_x)
                      + t_x * (1 - t_x) * (self.b[i] - self.a[i])) / self.h_i[i]
                ddx = 2 * (self.b[i] - 2 * self.a[i] + (self.a[i] - self.b[i]) * t_x * 3) / (self.h_i[i] ** 2)
                return x, dx, ddx

if __name__ == "__main__":
    n_dim = 2
    n_via_points = 4
    p0 = np.array([1., 2.])
    pf = np.array([10., 20.])
    bound_0 = np.array([0., 0.])
    bound_f = np.array([0., 0.])
    tf = 5

    cubic_spline = CubicSpline(n_dim)
    cubic_spline.set_boudaries(p0, pf, bound_0, bound_f, tf)
    via_points = np.linspace(p0, pf, n_via_points + 2)[1:-1] + np.random.random((n_via_points, n_dim))
    cubic_spline.set_via_points(via_points)
    cubic_spline.fit(type='velocity')
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
    plt.show()
