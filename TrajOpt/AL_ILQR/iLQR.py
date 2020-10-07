import warnings

import numpy as np


class iLQR:
    def __init__(self, dynamics, cost, hessian=False):
        """ Constructs an iLQR solver

        :param dynamics: System Dynamics
        :param cost: Cost Function
        :param max_reg: Maximum regularization term to break early due to divergence.
        :param hessian:
        """
        self.iter_cur = 0
        self.dynamics = dynamics
        self.cost = cost
        self._use_hessian = hessian and dynamics.has_hessian

        # Parameters for iLQR
        self.max_iterations = 500
        self.tolerance = 1e-6

        # Parameters for regularization of Hessian Matrix
        self.reg_init = 0.
        self.reg_min = 1e-8
        self.reg_max = 1e10
        self.reg_scale = 2.0

        # Parameters of line search
        self.line_scale = 0.5
        self.line_max_iterations = 10
        self.line_search_lb = 1e-8
        self.line_search_ub = 10

        self.iteration_count = 0

    def solve(self, x0, u_init, callback=None):
        """

        :param x0: Initial state.
        :param u_init: Initial control path.
        :param n_iterations: Maximum number of iterations. Default: 100.
        :param tolerance: Termination Tolerance. Default: 1e-6.
        :return:
            xs: optimal state path.
            us: optimal control path.
            J: optimal cost
        """
        us = u_init.copy()

        # Reset regularization term
        self.reg = self.reg_init
        self.delta_reg = 0.

        changed = True
        converged = False
        self.iter_cur = 0

        params = None
        xs = None
        J_prev = None

        while not converged and self.iter_cur < self.max_iterations:
            if changed:
                # initial roll out
                xs, L, params = self._foward_rollout(x0, us)
                J_prev = L.sum()

            K, d, delta_V_sum = self.backward_pass(params)
            if K is None or d is None or delta_V_sum is None:
                self.iter_cur += 1
                break

            xs_new, us_new, J_new, changed = self.forward_pass(xs, us, K, d, delta_V_sum, J_prev)
            if not changed:
                print("Not accepted")
                self._regularization_update("increase")
                if self.reg > self.reg_max:
                    warnings.warn("exceeded max regularization term")
                    break
                self.iter_cur += 1
                continue

            self._regularization_update("decrease")

            if np.abs(J_new - J_prev) < self.tolerance:
                print("Convergend")
                break

            xs = xs_new
            us = us_new
            J = J_new

            if callback:
                callback(self.iter_cur, xs, us, J_prev)

            self.iter_cur += 1
        return xs, us, J

    def forward_pass(self, xs, us, K, d, delta_V, J_prev):
        xs_new = np.zeros_like(xs)
        us_new = np.zeros_like(us)
        xs_new[0] = xs[0].copy()
        L = np.zeros(xs.shape[0])

        J_new = np.inf
        alpha = 1.0
        iteration = 0
        accepted = False
        while not accepted and iteration < self.line_max_iterations:
            for i in range(us.shape[0]):
                us_new[i] = us[i] + K[i].dot(xs_new[i] - xs[i]) + alpha * d[i]
                xs_new[i + 1] = self.dynamics.f(xs_new[i], us_new[i])
                L[i] = self.cost.l(xs_new[i], us_new[i], terminal=False)
            L[-1] = self.cost.l(xs_new[-1], None, terminal=True)
            J_new = L.sum()

            delta_V_alpha = -alpha * (delta_V[0] + alpha * delta_V[1])
            if delta_V_alpha > 0.0:
                z = (J_prev - J_new) / delta_V_alpha
            else:
                z = -1.0

            iteration += 1
            alpha = self.line_scale * alpha

            if (self.line_search_lb <= z <= self.line_search_ub) and J_new < J_prev:
                accepted = True

        return xs_new, us_new, J_new, accepted

    def backward_pass(self, params):
        """ Compute the backward pass of value function

        :param params: [dict]
                N: Number of steps.
                F_x: Jacobian of state path w.r.t. x    [N, state_size, state_size].
                F_u: Jacobian of state path w.r.t. u    [N, state_size, action_size].
                L_x: Jacobian of cost path w.r.t. x     [N+1, state_size]
                L_u: Jacobian of cost path w.r.t. u     [N, action_size]
                L_xx: Hessian of the cost path w.r.t. x, x [N+1, state_size, state_size]
                L_ux: Hessian of the cost path w.r.t. u, x [N+1, action_size, state_size]
                L_uu: Hessian of the cost path w.r.t. u, u [N+1, action_size, action_size]
                F_xx: Hessian of the state path w.r.t. x, x if Hessian are used DDP
                                                        [N+1, state_size, state_size]
                F_ux: Hessian of the state path w.r.t. u, x if Hessian are used DDP
                                                        [N+1, action_size, state_size]
                F_uu: Hessian of the state path w.r.t. u, u if Hessian are used DDP
                                                        [N+1, action_size, action_size]
                I_mu: Indicator of the active constraints if constraints exist ALTRO
                                                        [N+1, num_constraints]
                C_x: Jacobian of the constraints w.r.t x if constraints exist ALTRO
                                                        [N+1, num_constraints, state_size]
                C_u: Jacobian of the constraints w.r.t u if constraints exist ALTRO
                                                        [N+1, num_constraints, action_size]
        :return:
                K: feedback gains [N, action_size, state_size].
                d: feedforward gains [N, action_size].
                delta_V: expected cost change [N, 2], storing two terms: d_k.dot(Q_uu), 1/2 d_k.dot(Q_uu).dot(d_k)
        """
        d = np.empty((params['N'], self.dynamics.action_size))
        K = np.empty((params['N'], self.dynamics.action_size, self.dynamics.state_size))
        delta_V = np.zeros((params['N'], 2))

        V_x = params['L_x'][-1]
        V_xx = params['L_xx'][-1]

        i = params['N'] - 1
        while i > 0:
            Q_x = params['L_x'][i] + params['F_x'][i].T.dot(V_x)
            Q_u = params['L_u'][i] + params['F_u'][i].T.dot(V_x)

            Q_xx = params['L_xx'][i] + params['F_x'][i].T.dot(V_xx).dot(params['F_x'][i])

            # Regularization from Tassa, Erez and Todorov IROS 2012
            reg = self.reg * np.eye(self.dynamics.state_size)
            Q_ux = params['L_ux'][i] + params['F_u'][i].T.dot(V_xx + reg).dot(params['F_x'][i])
            Q_uu = params['L_uu'][i] + params['F_u'][i].T.dot(V_xx + reg).dot(params['F_u'][i])

            if self._use_hessian:
                Q_xx += np.tensordot(V_x, params['F_xx'][i], axes=1)
                Q_ux += np.tensordot(V_x, params['F_ux'][i], axes=1)
                Q_uu += np.tensordot(V_x, params['F_uu'][i], axes=1)

            # Check if Q_uu is all positive definite
            if np.any(np.linalg.eigvals(Q_uu) <= 0):
                self._regularization_update("increase")
                if self.reg > self.reg_max:
                    warnings.warn("exceeded max regularization term")
                    return None, None, None

                delta_V = np.zeros((params['N'], 2))
                # self.iter_cur += 1

                i = params['N'] - 1
                continue

            # Calculate gains
            d[i] = -np.linalg.solve(Q_uu, Q_u)
            K[i] = -np.linalg.solve(Q_uu, Q_ux)
            delta_V[i] = [d[i].T.dot(Q_u), 0.5 * d[i].T.dot(Q_uu).dot(d[i])]

            V_x = Q_x + K[i].T.dot(Q_uu).dot(d[i]) + K[i].T.dot(Q_u) + Q_ux.T.dot(d[i])
            V_xx = Q_xx + K[i].T.dot(Q_uu).dot(K[i]) + K[i].T.dot(Q_ux) + Q_ux.T.dot(K[i])
            V_xx = 0.5 * (V_xx + V_xx.T)  # For symmetry

            i -= 1

        return K, d, delta_V.sum(axis=0)

    def _foward_rollout(self, x0, us):
        """
        Apply the forward dynamics to have a trajectory and system paramters
        :param x0 [state_size]: Initial State
        :param us [N, action_size]: Control path
        :return:
            xs [N+1, state_size]: State path
            L [N+1]: Cost path
            params [dict]:
                N: Number of steps.
                F_x: Jacobian of state path w.r.t. x    [N, state_size, state_size].
                F_u: Jacobian of state path w.r.t. u    [N, state_size, action_size].
                L_x: Jacobian of cost path w.r.t. x     [N+1, state_size]
                L_u: Jacobian of cost path w.r.t. u     [N, action_size]
                L_xx: Hessian of the cost path w.r.t. x, x [N+1, state_size, state_size]
                L_ux: Hessian of the cost path w.r.t. u, x [N+1, action_size, state_size]
                L_uu: Hessian of the cost path w.r.t. u, u [N+1, action_size, action_size]
                F_xx: Hessian of the state path w.r.t. x, x if Hessian are used DDP
                                                        [N, state_size, state_size, state_size]
                F_ux: Hessian of the state path w.r.t. u, x if Hessian are used DDP
                                                        [N, state_size, action_size, state_size]
                F_uu: Hessian of the state path w.r.t. u, u if Hessian are used DDP
                                                        [N, state_size, action_size, action_size]
        """
        state_size = self.dynamics.state_size
        action_size = self.dynamics.action_size
        N = us.shape[0]

        xs = np.empty((N + 1, state_size))
        F_x = np.empty((N, state_size, state_size))
        F_u = np.empty((N, state_size, action_size))

        if self._use_hessian:
            F_xx = np.empty((N, state_size, state_size, state_size))
            F_ux = np.empty((N, state_size, action_size, state_size))
            F_uu = np.empty((N, state_size, action_size, action_size))
        else:
            F_xx = None
            F_ux = None
            F_uu = None

        L = np.empty(N + 1)
        L_x = np.empty((N + 1, state_size))
        L_u = np.empty((N + 1, action_size))

        L_xx = np.empty((N + 1, state_size, state_size))
        L_ux = np.empty((N, action_size, state_size))
        L_uu = np.empty((N, action_size, action_size))

        xs[0] = x0
        for i in range(N):
            x_i = xs[i]
            u_i = us[i]

            xs[i + 1] = self.dynamics.f(x_i, u_i)
            F_x[i] = self.dynamics.f_x(x_i, u_i)
            F_u[i] = self.dynamics.f_u(x_i, u_i)

            if self._use_hessian:
                F_xx[i] = self.dynamics.f_xx(x_i, u_i)
                F_ux[i] = self.dynamics.f_ux(x_i, u_i)
                F_uu[i] = self.dynamics.f_uu(x_i, u_i)

            L[i] = self.cost.l(x_i, u_i, terminal=False)
            L_x[i] = self.cost.l_x(x_i, u_i, terminal=False)
            L_u[i] = self.cost.l_u(x_i, u_i, terminal=False)
            L_xx[i] = self.cost.l_xx(x_i, u_i, terminal=False)
            L_ux[i] = self.cost.l_ux(x_i, u_i, terminal=False)
            L_uu[i] = self.cost.l_uu(x_i, u_i, terminal=False)

        x_f = xs[-1]
        L[-1] = self.cost.l(x_f, None, terminal=True)
        L_x[-1] = self.cost.l_x(x_f, None, terminal=True)
        L_xx[-1] = self.cost.l_xx(x_f, None, terminal=True)

        params = {'L_x': L_x,
                  'L_u': L_u,
                  'L_xx': L_xx,
                  'L_ux': L_ux,
                  'L_uu': L_uu,
                  'F_x': F_x,
                  'F_u': F_u,
                  'F_xx': F_xx,
                  'F_ux': F_ux,
                  'F_uu': F_uu,
                  'N': N}
        return xs, L, params

    def _regularization_update(self, status):
        if status == "increase":
            self.delta_reg = np.maximum(1., self.delta_reg) * self.reg_scale
            self.reg = np.maximum(self.reg_min, self.reg * self.delta_reg)

        elif status == "decrease":
            self.delta_reg = np.minimum(1., self.delta_reg) / self.reg_scale
            self.reg = self.reg * self.delta_reg
            if self.reg < self.reg_min:
                self.reg = 0.
