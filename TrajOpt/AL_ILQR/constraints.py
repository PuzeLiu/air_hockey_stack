import numpy as np
from jax import numpy as jnp
from jax import jacfwd, ops
import abc


class AbstractConstraint:
    @property
    @abc.abstractmethod
    def type(self):
        raise NotImplementedError

    @abc.abstractmethod
    def c(self, x, u, i, terminal=False):
        raise NotImplementedError

    @abc.abstractmethod
    def c_x(self, x, u, i, terminal=False):
        raise NotImplementedError

    @abc.abstractmethod
    def c_u(self, x, u, i, terminal=False):
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

    def c(self, x, u, i, terminal=False):
        if terminal:
            return self._c_terminal(x, i)
        return self._c(x, u, i)

    def c_x(self, x, u, i, terminal=False):
        if terminal:
            return jacfwd(self._c_terminal, 0)(x, i)
        return jacfwd(self._c, 0)(x, u, i)

    def c_u(self, x, u, i, terminal=False):
        if terminal:
            return np.zeros(self.action_size)
        return jacfwd(self._c, 1)(x, u, i)


class ConstraintList:
    def __init__(self, state_size, action_size):
        self._state_size = state_size
        self._action_size = action_size
        self._constraint_list = []

    @property
    def state_size(self):
        return self._state_size

    @property
    def action_size(self):
        return self._action_size

    @property
    def constraint_size(self):
        return len(self._constraint_list)

    @property
    def constraint_list(self):
        return self._constraint_list

    def add(self, constraint):
        self._constraint_list.append(constraint)

    def c(self, x, u, i, terminal):
        c = jnp.zeros(self.constraint_size)
        for index, constraint in enumerate(self._constraint_list):
            ops.index_update(c, index, constraint.c(x, u, i, terminal))
        return c

    def c_x(self, x, u, i, terminal):
        c_x = jnp.zeros(self.constraint_size, self.state_size)
        for index, constraint in enumerate(self._constraint_list):
            ops.index_update(c_x, index, constraint.c_x(x, u, i, terminal))
        return c_x

    def c_u(self, x, u, i, terminal):
        c_u = jnp.zeros(self.constraint_size, self.action_size)
        for index, constraint in enumerate(self._constraint_list):
            ops.index_update(c_u, index, constraint.c_u(x, u, i, terminal))
        return c_u
