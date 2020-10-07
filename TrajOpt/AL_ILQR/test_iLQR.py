import matplotlib.pyplot as plt
import numpy as np
from jax import numpy as jnp

from TrajOpt.AL_ILQR.cost import AutoDiffCost
from TrajOpt.AL_ILQR.dynamics import AutoDiffDynamics
from TrajOpt.AL_ILQR.iLQR import iLQR

mc = 1.0
mp = 0.1
l = 1.0
g = 9.81
dt = 0.05


def cart_pole_dynamics_fun(x, u):
    x_ = x[0]
    x_d = x[1]
    theta = x[2]
    theta_d = x[3]
    f = u[0]

    temp = (f + mp * l * theta_d ** 2 * jnp.sin(theta)) / (mc + mp)
    numerator = g * jnp.sin(theta) - jnp.cos(theta) * temp
    denominator = l * (4.0 / 3. - mp * jnp.cos(theta) ** 2 / (mc + mp))
    theta_dd = numerator / denominator

    x_dd = temp - mp * l * theta_dd * jnp.cos(theta) / mc + mp

    return jnp.array([x_ + x_d * dt,
                      x_d + x_dd * dt,
                      theta + theta_d * dt,
                      theta_d + theta_dd * dt])


def l_instant(x, u):
    return jnp.dot(jnp.dot(x, Q), x) + jnp.dot(jnp.dot(u, R), u)


def l_terminal(x):
    return (x - x_goal).dot(Q_terminal).dot(x - x_goal)


def callback(x_iter, xs, us, J, **kwargs):
    print("Iteration: {}, Optimal Cost: {}".format(x_iter, J))
    if x_iter % 10 == 0:
        steps = us.shape[0]
        t = np.arange(0, dt * (steps + 1), dt)
        fig, axes = plt.subplots(2)
        fig.suptitle("Iteration: {}, Optimal Cost: {}".format(x_iter, J))
        axes[0].set_title("state")
        axes[0].legend(['x', 'theta', 'x_dot', 'theta_legend'])
        axes[0].plot(t, xs)

        axes[1].set_title("control action")
        axes[1].plot(t[:-1], us)
        plt.show()


if __name__ == "__main__":
    hessian = False
    cart_pole_dynamics = AutoDiffDynamics(cart_pole_dynamics_fun, 4, 1, hessian)

    x_goal = jnp.zeros(4)

    Q = jnp.array([[1.0, 0.0, 0.0, 0.0],
                   [0.0, 10.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0, 10.0]])
    R = jnp.eye(1) * 0.01

    Q_terminal = jnp.eye(4) * 1000

    cost = AutoDiffCost(l_instant, l_terminal, 4, 4)

    ilqr = iLQR(cart_pole_dynamics, cost, hessian)

    x0 = jnp.array([0., 0., np.pi, 0.])
    u_init = np.random.uniform(-1, 1, (50, 1))
    ilqr.solve(x0, u_init, callback=callback)
