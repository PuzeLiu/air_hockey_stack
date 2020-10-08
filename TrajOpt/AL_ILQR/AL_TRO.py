import numpy as np


class ALTRO:
    def __init__(self, dynamics, cost, hessian, constraint_list):
        self.dynamics = dynamics
        self.cost = cost
        self._use_hessian = hessian and dynamics.has_hessian
        self.constraint_list = constraint_list

        self.mu = 1e-4
        self.max_out_iterations = 500

    def solve(self, x0, u_init, callback):
        xs = self._forward_rollout(x0, u_init)
        converged = False
        iter_out = 0
        while not converged and iter_out < self.max_out_iterations:

            iter_out += 1

    def _forward_rollout(self, x0, us):
        N = us.shape[0]
        xs = np.zeros((N + 1, self.dynamics.state_size))
        xs[0] = x0

        for i in range(N):
            xs[i + 1] = self.dynamics.f(xs[i], us[i])

        return xs
