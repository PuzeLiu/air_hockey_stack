import abc

from jax import jacfwd
from jax import numpy as np


class Dynamics:
    """Dynamics Model."""

    @property
    @abc.abstractmethod
    def state_size(self):
        """State size."""
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def action_size(self):
        """Action size."""
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def has_hessian(self):
        """Whether the second order derivatives are available."""
        raise NotImplementedError

    @abc.abstractmethod
    def f(self, x, u):
        """Dynamics model.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            Next state [state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def f_x(self, x, u):
        """Partial derivative of dynamics model with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            df/dx [state_size, state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def f_u(self, x, u):
        """Partial derivative of dynamics model with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            df/du [state_size, action_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def f_xx(self, x, u):
        """Second partial derivative of dynamics model with respect to x.
        Note:
            This is not necessary to implement if you're planning on skipping
            Hessian evaluation as the iLQR implementation does by default.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/dx^2 [state_size, state_size, state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def f_ux(self, x, u):
        """Second partial derivative of dynamics model with respect to u and x.
        Note:
            This is not necessary to implement if you're planning on skipping
            Hessian evaluation as the iLQR implementation does by default.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/dudx [state_size, action_size, state_size].
        """
        raise NotImplementedError

    @abc.abstractmethod
    def f_uu(self, x, u):
        """Second partial derivative of dynamics model with respect to u.
        Note:
            This is not necessary to implement if you're planning on skipping
            Hessian evaluation as the iLQR implementation does by default.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/du^2 [state_size, action_size, action_size].
        """
        raise NotImplementedError


class AutoDiffDynamics(Dynamics):
    """Auto-differentiated Dynamics Model."""

    def __init__(self, f, x_dim, u_dim, hessian=False):
        """Constructs an AutoDiffDynamics model.
        Args:
            f: Dynamics function.
            x_dim: Theano state input variables.
            u_dim: Theano action input variables.
            hessian: Evaluate the dynamic model's second order derivatives.
                Default: only use first order derivatives. (i.e. iLQR instead
                of DDP).
        """
        self._f = f
        self._state_size = x_dim
        self._action_size = u_dim

        self._has_hessian = hessian

        super(AutoDiffDynamics, self).__init__()

    @property
    def state_size(self):
        """State size."""
        return self._state_size

    @property
    def action_size(self):
        """Action size."""
        return self._action_size

    @property
    def has_hessian(self):
        """Whether the second order derivatives are available."""
        return self._has_hessian

    def f(self, x, u):
        """Dynamics model.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            Next state [state_size].
        """
        return self._f(x, u)

    def f_x(self, x, u):
        """Partial derivative of dynamics model with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            df/dx [state_size, state_size].
        """
        return jacfwd(self._f, 0)(x, u)

    def f_u(self, x, u):
        """Partial derivative of dynamics model with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            df/du [state_size, action_size].
        """
        return jacfwd(self._f, 1)(x, u)

    def f_xx(self, x, u):
        """Second partial derivative of dynamics model with respect to x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/dx^2 [state_size, state_size, state_size].
        """
        if self._has_hessian:
            return jacfwd(jacfwd(self._f, 0), 0)(x, u)
        else:
            return np.zeros((self._state_size, self._state_size, self._state_size))

    def f_ux(self, x, u):
        """Second partial derivative of dynamics model with respect to u and x.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/dudx [state_size, action_size, state_size].
        """
        if self._has_hessian:
            return jacfwd(jacfwd(self._f, 1), 0)(x, u)
        else:
            return np.zeros((self._state_size, self._action_size, self._state_size))

    def f_uu(self, x, u):
        """Second partial derivative of dynamics model with respect to u.
        Args:
            x: Current state [state_size].
            u: Current control [action_size].
        Returns:
            d^2f/du^2 [state_size, action_size, action_size].
        """
        if self._has_hessian:
            return jacfwd(jacfwd(self._f, 1), 1)(x, u)
        else:
            return np.zeros((self._state_size, self._action_size, self._action_size))
