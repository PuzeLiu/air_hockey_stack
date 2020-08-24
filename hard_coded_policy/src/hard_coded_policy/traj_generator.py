import numpy as np
# import matplotlib.pyplot as plt

class Poly3():
    def __init__(self, q_0, q_f, v_0, v_f, t_f):
        self.t_f = t_f
        self.d = q_0
        self.c = v_0
        self.b = (3 * (q_f - self.c * self.t_f -self.d) - (v_f - self.c)*self.t_f) / (self.t_f* self.t_f)
        self.a = (v_f - 2* self.b * self.t_f - self.c) / (3*self.t_f*self.t_f)

    def __call__(self, t):
        if t<=self.t_f:
            q = self.a * t**3 + self.b * t**2+ self.c * t + self.d
            qd = 3 * self.a * t**2 + 2 * self.b * t + self.c
            qdd = 6 * self.a * t + 2 * self.b
        else:
            q = self.a * self.t_f ** 3 + self.b * self.t_f **2+ self.c * self.t_f + self.d
            qd = 3 * self.a * self.t_f**2 + 2 * self.b * self.t_f + self.c
            qdd =6 * self.a * self.t_f + 2 * self.b
        return q, qd, qdd

class TrapezoidVel:
    def __init__(self, q_0, q_f, max_vel, acc):
        self.q_0 = np.atleast_1d(q_0).astype(np.float)
        self.q_f = np.atleast_1d(q_f).astype(np.float)
        self.max_vel = np.atleast_1d(max_vel).astype(np.float)
        self.acc = np.atleast_1d(acc).astype(np.float)

        if self.max_vel.shape != self.q_0.shape:
            self.max_vel = np.tile(self.max_vel.astype(np.float), self.q_0.shape)
        else:
            self.max_vel = np.abs(self.max_vel.astype(np.float))

        if self.acc.shape != self.q_0.shape:
            self.acc = np.tile(self.acc, self.q_0.shape)
        else:
            self.acc = np.abs(self.acc.astype(np.float))

        self.sign = np.sign(self.q_f - self.q_0)
        q_diff = np.abs(self.q_f - self.q_0)
        q_max_acc = self.max_vel ** 2 / (self.acc * 2)

        self.vel = np.zeros_like(self.q_0)
        self.t_acc = np.zeros_like(self.q_0)
        self.t_dec = np.zeros_like(self.q_0)
        self.t_f_idx = np.zeros_like(self.q_0)
        self.q_1 = np.zeros_like(self.q_0)
        self.q_2 = np.zeros_like(self.q_0)
        for i in range(self.q_0.shape[0]):
            if q_diff[i] <= 2 * q_max_acc[i]:
                self.t_acc[i] = np.sqrt(q_diff[i] / self.acc[i])
                self.t_dec[i] = self.t_acc[i]
                self.vel[i] = self.t_acc[i] * self.acc[i] * self.sign[i]
                self.t_f_idx[i] = 2 * self.t_acc[i]
                self.q_1[i] = self.q_0[i] + self.sign[i] * 0.5*self.acc[i] * self.t_acc[i]**2
                self.q_2[i] = self.q_1[i]
            else:
                self.t_acc[i] = self.max_vel[i] / self.acc[i]
                self.t_dec[i] = self.t_acc[i] + (q_diff[i] - 2*q_max_acc[i])/self.max_vel[i]
                self.vel[i] = self.sign[i] * self.max_vel[i]
                self.t_f_idx[i] = self.t_dec[i] + self.t_acc[i]
                self.q_1[i] = self.q_0[i] + self.sign[i] * 0.5 * self.acc[i] * self.t_acc[i] ** 2
                self.q_2[i] = self.q_1[i] + self.sign[i] * self.max_vel[i] * (self.t_dec[i] - self.t_acc[i])

        self.t_f = np.max(self.t_f_idx)

    def __call__(self, t):
        q = np.zeros_like(self.q_0)
        qd = np.zeros_like(self.q_0)
        qdd = np.zeros_like(self.q_0)
        idx1 = t<=self.t_acc
        idx2 = np.logical_and(t<=self.t_dec, np.logical_not(idx1))
        idx3 = np.logical_not(np.logical_or(idx1, idx2))
        idx4 = t>=self.t_f_idx

        q[idx1] = self.q_0[idx1] + self.sign[idx1] * 0.5*self.acc[idx1] * t**2
        qd[idx1] = self.sign[idx1] * self.acc[idx1] * t
        qdd[idx1] = self.sign[idx1] * self.acc[idx1]

        qd[idx2] = self.sign[idx2] * self.max_vel[idx2]
        q[idx2] = self.q_1[idx2] + qd[idx2] * (t - self.t_acc[idx2])
        qdd[idx2] = 0

        q[idx3] = self.q_2[idx3] + self.vel[idx3] * (t - self.t_dec[idx3]) - 0.5 * self.sign[idx3] * self.acc[idx3] * (t-self.t_dec[idx3])**2
        qd[idx3] = self.vel[idx3] - self.sign[idx3]*self.acc[idx3] * (t- self.t_dec[idx3])
        qdd[idx3] = -self.sign[idx3] * self.acc[idx3]

        q[idx4] = self.q_f[idx4]
        qd[idx4] = 0
        qdd[idx4] = 0

        return q,qd, qdd

    def generate_trajectory(self, step_size):
        t = np.arange(0, self.t_f, step=step_size)
        q = np.zeros((t.shape[0], self.q_0.shape[0]))
        qd = np.zeros((t.shape[0], self.q_0.shape[0]))
        qdd = np.zeros((t.shape[0], self.q_0.shape[0]))
        for i, t_step in enumerate(t):
            q_i, qd_i, qdd_i = self.__call__(t_step)
            q[i] = q_i
            qd[i] = qd_i
            qdd[i] = qdd_i

        return q, qd, qdd


class TrapezoidAcc:
    def __init__(self, q_0, q_f, max_vel, max_acc, jerk):
        self.q_0 = q_0
        self.q_f = q_f
        self.max_vel = np.abs(max_vel)
        self.max_acc = np.abs(max_acc)
        self.jerk = np.abs(jerk)


        q_diff = np.abs(self.q_f - self.q_0)

        self.t_key_point = np.zeros(8)
        self.q_key_point = np.zeros(8)
        self.qd_key_point = np.zeros(8)
        self.qdd_key_point = np.zeros(8)
        self.jerk_key_point = np.zeros(8)

        self.sign = np.sign(self.q_f - self.q_0)
        self.jerk_key_point[1] = self.sign * self.jerk
        self.jerk_key_point[3] = -self.sign * self.jerk
        self.jerk_key_point[5] = -self.sign * self.jerk
        self.jerk_key_point[7] = self.sign * self.jerk

        # velocity when reach the maximum acceleration
        v_max_acc = self.max_acc ** 2 / self.jerk / 2
        # check whether to meet acceleration limit or velocity limit first
        if self.max_vel <= 2 * v_max_acc:
            t_max_acc = np.sqrt(self.max_vel / self.jerk)
            self.t_key_point[1] = np.sqrt(self.max_vel / self.jerk)
            self.t_key_point[2] = 0.
            self.t_key_point[3] = self.t_key_point[1]
        else:
            t_max_acc = self.max_acc / self.jerk
            self.t_key_point[1] = self.max_acc / self.jerk
            self.t_key_point[2] = (self.max_vel - 2 * v_max_acc) / self.max_acc
            self.t_key_point[3] = self.t_key_point[1]

        # The moving distance without constant acceleration and constant velocity phase
        q_max_acc = self.jerk * t_max_acc ** 3
        for i in range(1, 4):
            self.qdd_key_point[i] = self.qdd_key_point[i - 1] + self.jerk_key_point[i] * self.t_key_point[i]
            self.qd_key_point[i] = self.qd_key_point[i - 1] + self.qdd_key_point[i - 1] * self.t_key_point[i] + \
                                   self.jerk_key_point[i] * self.t_key_point[i] ** 2 / 2
            self.q_key_point[i] = self.q_key_point[i - 1] + self.qd_key_point[i - 1] * self.t_key_point[i] + \
                                  self.qdd_key_point[i - 1] * self.t_key_point[i] ** 2 / 2 + \
                                  self.jerk_key_point[i] * self.t_key_point[i] ** 3 / 6
                                  
        if q_diff < np.abs(q_max_acc) * 2:
            # The case cannot reach maximum acceleration
            t_max_acc = np.cbrt(q_diff / 2 / self.jerk)
            print(q_diff, t_max_acc)
            self.t_key_point[1] = t_max_acc
            self.t_key_point[2] = 0.
            self.t_key_point[3] = self.t_key_point[1]
        elif q_diff < np.abs(self.q_key_point[3]) * 2:
            # The case does not reach maximum velocity
            a = 1
            b = 3 * t_max_acc
            c = 2 * t_max_acc ** 2 - q_diff / self.jerk / t_max_acc
            tb = (-b + np.sqrt(b**2 - 4 * a *c)) / (2 * a)

            self.t_key_point[2] = tb
        else:
            # The case reach maximum velocity and hold
            self.t_key_point[4] = (q_diff - 2 * np.abs(self.q_key_point[3])) / self.max_vel

        self.t_key_point[5] = self.t_key_point[1]
        self.t_key_point[6] = self.t_key_point[2]
        self.t_key_point[7] = self.t_key_point[3]

        self.t_f = np.sum(self.t_key_point)

        # Generate key point consider the direction
        self.q_key_point[0] = self.q_0

        # regenerate every key point.
        for i in range(1, 8):
            self.qdd_key_point[i] = self.qdd_key_point[i - 1] + self.jerk_key_point[i] * self.t_key_point[i]
            self.qd_key_point[i] = self.qd_key_point[i - 1] + self.qdd_key_point[i - 1] * self.t_key_point[i] + \
                                   self.jerk_key_point[i] * self.t_key_point[i] ** 2 / 2
            self.q_key_point[i] = self.q_key_point[i - 1] + self.qd_key_point[i - 1] * self.t_key_point[i] + \
                                  self.qdd_key_point[i - 1] * self.t_key_point[i] ** 2 / 2 + \
                                  self.jerk_key_point[i] * self.t_key_point[i] ** 3 / 6

    def __call__(self, t):
        idx = np.nan
        t_phase = np.nan
        for i in range(1, 8):
            if np.sum(self.t_key_point[:i]) <= t < np.sum(self.t_key_point[:i + 1]):
                idx = i
                t_phase = t - np.sum(self.t_key_point[:i])
                break
        qdd = self.qdd_key_point[idx - 1] + self.jerk_key_point[idx] * t_phase
        qd = self.qd_key_point[idx - 1] + self.qdd_key_point[idx - 1] * t_phase + \
             self.jerk_key_point[idx] * t_phase ** 2 / 2
        q = self.q_key_point[idx - 1] + self.qd_key_point[idx - 1] * t_phase + \
                                      self.qdd_key_point[idx - 1] * t_phase ** 2 / 2 + \
                                      self.jerk_key_point[idx] * t_phase ** 3 / 6

        return q, qd, qdd

    def generate_trajectory(self, step_size):
        t = np.arange(0, self.t_f, step=step_size)
        q = np.zeros((t.shape[0]))
        qd = np.zeros((t.shape[0]))
        qdd = np.zeros((t.shape[0]))
        for i, t_step in enumerate(t):
            q_i, qd_i, qdd_i = self.__call__(t_step)
            q[i] = q_i
            qd[i] = qd_i
            qdd[i] = qdd_i

        return q, qd, qdd
