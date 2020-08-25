import numpy as np

class BezierTrajectory:
    def __init__(self, dim, lb, ub):
        self.lb = lb
        self.ub = ub
        self.dim = dim
        self.a = np.zeros(4)

    def fit(self, x0, xf, vf, tf=np.inf):
        self.x0 = x0
        self.xf = xf
        self.vf = vf

        # Calculate middle point
        if self.vf[0] > 0:
            y = self.xf[1] - self.vf[1] / self.vf[0] * (self.xf[0] - self.lb[0])
            if self.lb[1] <= y <= self.ub[1]:
                self.xm = np.array([self.lb[0], y])
        elif self.vf[0] < 0:
            y = self.xf[1] - self.vf[1] / self.vf[0] * (self.xf[0] - self.ub[0])
            if self.lb[1] <= y <= self.ub[1]:
                self.xm = np.array([self.ub[0], y])

        if self.vf[1] > 0:
            x = self.xf[0] - self.vf[0] / self.vf[1] * (self.xf[1] - self.lb[1])
            if self.lb[0] <= x <= self.ub[0]:
                self.xm = np.array([x, self.lb[1]])
        elif self.vf[1] < 0:
            x = self.xf[0] - self.vf[0] / self.vf[1] * (self.xf[1] - self.ub[1])
            if self.lb[0] <= x <= self.ub[0]:
                self.xm = np.array([x, self.ub[1]])

        # Cubic Trajectory on Phase Variable.
        self.zf = 1
        self.d_zf = self.vf[0] / (self.xf[0] - self.xm[0]) / 2
        self.t_min = 3 * (self.zf - 0) / self.d_zf

        self.a[0] = 0.
        self.a[1] = 0.
        self.a[2] = 0.
        self.a[3] = self.zf / self.t_min ** 3

        if tf < self.t_min:
            return False
        else:
            return True

    def generate_trajectory(self, t):
        # z = self._trapezoid_trajectory(tf)
        z, dz_dt = self._cubic_trajectory(t)

        x = self.xm + (1 - z) ** 2 * (self.x0 - self.xm) + z ** 2 * (self.xf - self.xm)
        dx_dz = 2 * (1 - z) * (self.xm - self.x0) + 2 * z * (self.xf - self.xm)
        dx_dt = dx_dz * dz_dt
        return x, dx_dt

    def _cubic_trajectory(self, t):
        if t <= self.t_min:
            z = self.a[3] * t ** 3
            dz = 3 * self.a[3] * t ** 2
            return z, dz
        else:
            print(t)