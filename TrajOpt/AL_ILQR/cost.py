import abc

from jax import jacfwd, jacrev
from jax import numpy as np


class Cost:
    """Instantaneous Cost.
    NOTE: The terminal cost needs to at most be a function of x and i, whereas
          the non-terminal cost can be a function of x, u and i.
    """

    @abc.abstractmethod
    def l(self, x, u, terminal=False):
        """Instantaneous cost function.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            Instantaneous cost (scalar).
        """
        raise NotImplementedError

    @abc.abstractmethod
    def l_x(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/dx [state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def l_u(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/du [action_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def l_xx(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dx^2 [state_size, state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def l_ux(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u and x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dudx [action_size, state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def l_uu(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/du^2 [action_size, action_size].
        """
        raise NotImplementedError


class AutoDiffCost(Cost):
    """Auto-differentiated Instantaneous Cost.
    NOTE: The terminal cost needs to at most be a function of x and i, whereas
          the non-terminal cost can be a function of x, u and i.
    """

    def __init__(self, l_instant, l_terminal, state_size, action_size):
        """Constructs an AutoDiffCost.
        Args:
            l_instant: instantaneous cost.
                This needs to be a function of x and u and must return a scalar.
            l_terminal: Vector Theano tensor expression for terminal cost.
                This needs to be a function of x only and must return a scalar.
            state_size:
            action_size: Theano action input variables [action_size].
        """
        self._l = l_instant
        self._l_terminal = l_terminal

        self.state_size = state_size
        self.action_size = action_size

        super(AutoDiffCost, self).__init__()

    def l(self, x, u, terminal=False):
        """Instantaneous cost function.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            Instantaneous cost (scalar).
        """
        if terminal:
            return self._l_terminal(x)
        else:
            return self._l(x, u)

    def l_x(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/dx [state_size].
        """
        if terminal:
            return jacrev(self._l_terminal, 0)(x)
        else:
            return jacrev(self._l, 0)(x, u)

    def l_u(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/du [action_size].
        """
        if terminal:
            return jacrev(self._l_terminal, 1)(x)
        else:
            return jacrev(self._l, 1)(x, u)

    def l_xx(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dx^2 [state_size, state_size].
        """
        if terminal:
            return jacfwd(jacrev(self._l_terminal, 0), 0)(x)
        else:
            return jacfwd(jacrev(self._l, 0), 0)(x, u)

    def l_ux(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u and x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dudx [action_size, state_size].
        """
        if terminal:
            return np.zeros((self.action_size, self.state_size))
        else:
            return jacfwd(jacrev(self._l, 1), 0)(x, u)

    def l_uu(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/du^2 [action_size, action_size].
        """
        if terminal:
            return np.zeros((self.action_size, self.action_size))
        else:
            return jacfwd(jacrev(self._l, 1), 1)(x, u)


class QRCost(Cost):

    """Quadratic Regulator Instantaneous Cost."""

    def __init__(self, Q, R, Q_terminal=None, x_goal=None, u_goal=None):
        """Constructs a QRCost.
        Args:
            Q: Quadratic state cost matrix [state_size, state_size].
            R: Quadratic control cost matrix [action_size, action_size].
            Q_terminal: Terminal quadratic state cost matrix
                [state_size, state_size].
            x_goal: Goal state [state_size].
            u_goal: Goal control [action_size].
        """
        self.Q = np.array(Q)
        self.R = np.array(R)

        if Q_terminal is None:
            self.Q_terminal = self.Q
        else:
            self.Q_terminal = np.array(Q_terminal)

        if x_goal is None:
            self.x_goal = np.zeros(Q.shape[0])
        else:
            self.x_goal = np.array(x_goal)

        if u_goal is None:
            self.u_goal = np.zeros(R.shape[0])
        else:
            self.u_goal = np.array(u_goal)

        # Precompute some common constants.
        self._Q_plus_Q_T = self.Q + self.Q.T
        self._R_plus_R_T = self.R + self.R.T
        self._Q_plus_Q_T_terminal = self.Q_terminal + self.Q_terminal.T

        super(QRCost, self).__init__()

    def l(self, x, u, terminal=False):
        """Instantaneous cost function.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            Instantaneous cost (scalar).
        """
        Q = self.Q_terminal if terminal else self.Q
        R = self.R
        x_diff = x - self.x_goal
        squared_x_cost = x_diff.T.dot(Q).dot(x_diff)

        if terminal:
            return squared_x_cost

        u_diff = u - self.u_goal
        return squared_x_cost + u_diff.T.dot(R).dot(u_diff)

    def l_x(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/dx [state_size].
        """
        Q_plus_Q_T = self._Q_plus_Q_T_terminal if terminal else self._Q_plus_Q_T
        x_diff = x - self.x_goal
        return x_diff.T.dot(Q_plus_Q_T)

    def l_u(self, x, u, terminal=False):
        """Partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            dl/du [action_size].
        """
        if terminal:
            return np.zeros_like(self.u_goal)

        u_diff = u - self.u_goal
        return u_diff.T.dot(self._R_plus_R_T)

    def l_xx(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dx^2 [state_size, state_size].
        """
        return self._Q_plus_Q_T_terminal if terminal else self._Q_plus_Q_T

    def l_ux(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u and x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/dudx [action_size, state_size].
        """
        return np.zeros((self.R.shape[0], self.Q.shape[0]))

    def l_uu(self, x, u, terminal=False):
        """Second partial derivative of cost function with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size]. None if terminal.
            terminal: Compute terminal cost. Default: False.
        Returns:
            d^2l/du^2 [action_size, action_size].
        """
        if terminal:
            return np.zeros_like(self.R)

        return self._R_plus_R_T
