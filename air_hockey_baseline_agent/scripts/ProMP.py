import matplotlib.pyplot as plt
import numpy as np
import torch
import torch as to


class ProMP:
    def __init__(self, dim, n_basis, center, width, t_total=1., step=0.01, basis='rbf'):
        self.dim = dim
        self.n_basis = n_basis
        self.center = center 
        self.width = width
        self.t_total = t_total
        self.t_step = step

        self.mu_w = to.zeros(self.dim * self.n_basis).double()
        self.sigma_w = to.eye(self.dim * self.n_basis).double()

        if basis == 'rbf':
            self.phi = self.phi_rbf_1d

    def sample_weight(self):
        weight = to.distributions.MultivariateNormal(self.mu_w, self.sigma_w).sample().double()
        return weight

    def get_trajectory(self, weight):
        t_traj = to.linspace(0, self.t_total, int(self.t_total / self.t_step))
        traj = []
        for t_i in t_traj:
            traj.append(self.get_point(t_i, weight, order=1).numpy())
        return np.array(traj)

    def get_point(self, t, w, order):
        if not to.is_tensor(t):
            t = to.tensor(t)
        psi_t = self.psi(t, order)
        return psi_t @ w

    def psi(self, t, order):
        phi_t = self.phi(t, order)
        return to.block_diag(*[phi_t for _ in range(self.dim)])

    def phi_rbf_1d(self, t, order):
        phi = self._phi_rbf(t)
        if order == 0:
            return phi.unsqueeze(0)

        def jac(x):
            return to.autograd.functional.jacobian(self._phi_rbf, x, create_graph=True)

        dphi = jac(t).detach()
        if order == 1:
            return to.vstack([phi, dphi])

        ddphi = to.autograd.functional.jacobian(jac, t)
        if order == 2:
            return to.vstack([phi, dphi, ddphi])
        else:
            raise NotImplementedError

    def _phi_rbf(self, t):
        z = self._phase(t)
        b_rbf = to.exp(- (z - self.center) ** 2 / 2 / self.width)
        return b_rbf / to.sum(b_rbf)

    def _phase(self, t):
        return t / self.t_total

    def plot_phase(self):
        t_traj = to.linspace(0, 1, 100) * self.t_total
        phase_traj = []
        for t_i in t_traj:
            phi = self._phi_rbf(t_i)
            dphi = to.autograd.functional.jacobian(self._phi_rbf, t_i, create_graph=True).detach()
            phase_traj.append(to.cat([phi, dphi]).numpy())
        phase_traj = np.array(phase_traj)

        fig, axes = plt.subplots(2)
        for i in range(self.n_basis):
            axes[0].plot(t_traj, phase_traj[:, i])
            axes[1].plot(t_traj, phase_traj[:, self.n_basis + i])

    def conditioning(self, t, y_t, sigma_y=1e-6, order=1):
        if not to.is_tensor(t):
            t = to.tensor(t)

        psi = self.psi(t, order)

        if np.isscalar(sigma_y):
            sigma_y = to.eye(psi.shape[0]) * sigma_y

        A = self.sigma_w @ psi.t() @ torch.inverse(sigma_y + psi @ self.sigma_w @ psi.t())
        self.mu_w = self.mu_w + A @ (y_t - psi @ self.mu_w)
        self.sigma_w = self.sigma_w - A @ psi @ self.sigma_w

    def plot_mean_var(self):
        t_traj = to.linspace(0, t_total, int(t_total / step))
        mean_traj = []
        std_traj = []
        for t_i in t_traj:
            psi_i = self.psi(t_i, order=1)
            mean_traj.append((psi_i @ self.mu_w).numpy())
            std_traj.append((psi_i @ self.sigma_w @ psi_i.t()).diag().sqrt().numpy())
        mean_traj = np.array(mean_traj)
        std_traj = np.array(std_traj)

        fig, ax = plt.subplots(dims, 2)
        ax[0, 0].set_title("Position")
        ax[0, 1].set_title("Velocity")
        ax[0, 0].set_ylabel("Joint_1")
        ax[1, 0].set_ylabel("Joint_2")
        for i in range(dims):
            ax[i, 0].plot(t_traj, mean_traj[:, 2 * i + 0], ls='--', color='tab:blue')
            ax[i, 1].plot(t_traj, mean_traj[:, 2 * i + 1], ls='--', color='tab:orange')
            ax[i, 0].fill_between(t_traj, (mean_traj - 1.96 * std_traj)[:, 2 * i + 0],
                                  (mean_traj + 1.96 * std_traj)[:, 2 * i + 0], color='tab:blue', alpha=0.2)
            ax[i, 1].fill_between(t_traj, (mean_traj - 1.96 * std_traj)[:, 2 * i + 1],
                                  (mean_traj + 1.96 * std_traj)[:, 2 * i + 1], color='tab:orange', alpha=0.2)
            ax[i, 0].grid()
            ax[i, 1].grid()
        return ax


if __name__ == '__main__':
    dims = 2
    n_b = 20
    width = 0.03
    step = 0.01

    to.manual_seed(16663538334000523044)
    # print(to.random.seed())
    n_samples = 5

    # goal = to.rand(4) * to.tensor([0.6, 1.5, 1.0, 1.0]) + to.tensor([0.2, 0.5, -0.5, -0.5])
    goal = to.rand(6) * to.tensor([0.6, 1.5, 0.0, 1.0, 1.0, 0.0]) + to.tensor([0.2, 0.5, 0.0, -0.5, -0.5, 0.0])
    t_goal = to.norm(goal[1::2]) / 2
    t_total = 2 * t_goal

    centers = to.linspace(0 - np.sqrt(width), 1 + np.sqrt(width), n_b).double()

    promp = ProMP(dim=dims, n_basis=n_b, center=centers, width=width, t_total=t_total, step=step)

    promp.plot_phase()

    promp.conditioning(t_goal, goal, sigma_y=to.diag(to.tensor([1e-4, 1e-3, 1e-2, 1e-6, 1e-3, 1e-2])), order=2)
    promp.conditioning(to.tensor(0.0), to.tensor([0.2, 0.0, 0.0, 0.0]), sigma_y=1e-6, order=1)
    promp.conditioning(t_total, to.tensor([0.2, 0.0, 0.0, 0.0]), sigma_y=1e-6, order=1)

    print(promp.mu_w)

    axes = promp.plot_mean_var()

    # Plot samples
    fig2, ax2 = plt.subplots(1)
    ax2.set_aspect(1.0)
    ax2.set_xlim(-0.6, 0.6)
    ax2.set_ylim(-0.1, 1.0)
    ax2.plot([-0.52, -0.52], [0.8, 0.0], lw=5, color='k')
    ax2.plot([0.52, 0.52], [0.8, 0.0], lw=5, color='k')
    ax2.plot([0.52, -0.52], [0.0, 0.0], lw=5, color='k')

    t_traj = to.linspace(0, t_total, int(t_total / step))
    for j in range(n_samples):
        weight = promp.sample_weight()
        traj = promp.get_trajectory(weight)
        for i in range(dims):
            axes[i, 0].plot(t_traj, traj[:, 2 * i + 0])
            axes[i, 1].plot(t_traj, traj[:, 2 * i + 1])
        ax2.plot(-traj[:, 2], traj[:, 0])

    stride = int(goal.shape[0] / 2)
    v_norm = to.norm(goal[1::stride]) / 0.1
    axes[0, 0].scatter(t_goal, goal[0], s=30, color='b')
    axes[0, 1].scatter(t_goal, goal[1], s=30, color='b')
    axes[1, 0].scatter(t_goal, goal[stride], s=30, color='b')
    axes[1, 1].scatter(t_goal, goal[stride + 1], s=30, color='b')
    ax2.arrow(-goal[stride], goal[0], -goal[stride + 1] / v_norm, goal[1] / v_norm, width=0.01, color='b')

    plt.show()
