import numpy as np
from jax import jacfwd
import abc


class AbstractConstraint:
    @property
    @abc.abstractmethod
    def type(self):
        raise NotImplementedError

    @abc.abstractmethod
    def c(self, x, u, terminal=False):
        raise NotImplementedError

    @abc.abstractmethod
    def c_x(self, x, u, terminal=False):
        raise NotImplementedError

    @abc.abstractmethod
    def c_u(self, x, u, terminal=False):
        raise NotImplementedError


class AutoDiffConstraint(AbstractConstraint):
    def __init__(self, c_i, c_terminal, state_size, action_size, type):
        self._c = c_i
        self._c_terminal = c_terminal
        if type is not 'eq' or type is not 'ineq':
            raise AttributeError("type should be 'eq' for equality constraint "
                                 "or 'ineq' for inequality constraint")
        self._type = type

        self.state_size = state_size
        self.action_size = action_size

        super(AutoDiffConstraint, self).__init__()

    def type(self):
        return self._type

    def c(self, x, u, terminal=False):
        if terminal:
            return self._c_terminal(x)
        return self._c(x, u)

    def c_x(self, x, u, terminal=False):
        if terminal:
            return jacfwd(self._c_terminal, 0)(x)
        return jacfwd(self._c, 0)(x, u)

    def c_u(self, x, u, terminal=False):
        if terminal:
            return np.zeros(self.action_size)
        return jacfwd(self._c, 1)(x, u)
