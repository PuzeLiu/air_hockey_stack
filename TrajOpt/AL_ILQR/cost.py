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
