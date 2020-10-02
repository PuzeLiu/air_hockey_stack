import numpy as np
import quaternion
import torch
from TrajOpt.optimize_hitting_point import OptimizerHitPoint
from air_hockey.policies.bezier import BezierTrajectory
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch


class TrajGenerator:
    def __init__(self, goal_: np.ndarray, ee_height_offset_: float, kinematics_: KinematicsTorch):
        self.goal = goal_
        self.kinematics = kinematics_
        self.ee_height_offset = np.array([ee_height_offset_])

        self.trajectory_generator = BezierTrajectory(dim=2, lb=np.array([0.6, -0.45]), ub=np.array([1.1, 0.45]))

        self.parameters = dict()

    def plan(self, q_0, puck_p_):

        # res_hit = self._optimize_hit_point(puck_p_)
        # TODO delete debug value
        res_hit = (np.array([0.57602171, 1.26136975, -2.42489628, -0.15190681, 1.96321625, 2.05665823, -0.83832055]),
                   np.array([-1.27760855, -0.01640784, -1.2001923, -0.81845422, 2.2688658, -2.34649657, -1.3276784]))
        if res_hit is None:
            return
        q_f = res_hit[0]
        qd_f = res_hit[1]

        gc_f = np.sign(q_f[1:7:2])
        gc_0 = np.sign(q_0[1:7:2])
        if not np.array_equal(gc_0, gc_f):
            print("Global Configuration are not equal")
            return

        # Cartesian Position
        x_0 = self.kinematics.forward_kinematics(torch.tensor(q_0).double()).detach().numpy()
        x_f = self.kinematics.forward_kinematics(torch.tensor(q_f).double()).detach().numpy()
        v_f = self.kinematics.get_jacobian(torch.tensor(q_f).double()).detach().numpy() @ qd_f
        self.trajectory_generator.fit(x_0[:2], x_f[:2], v_f[:2])
        t_f = self.trajectory_generator.t_min
        t_m = t_f * 99 / 100

        # Redundancy Parameter
        psi_0 = self.kinematics.get_redundancy(torch.tensor(q_0).double()).detach().numpy()
        psi_f = self.kinematics.get_redundancy(torch.tensor(q_f).double()).detach().numpy()

        # Rotation
        quat_0 = np.quaternion(*x_0[3:])
        quat_f = np.quaternion(*x_f[3:])
        quatd_f = np.quaternion(*v_f[3:])

        angular_vel = 2 * quatd_f * quat_f.conj()
        rot_vec = angular_vel.components[1:] * (1 - t_m)
        quat_rot = quaternion.from_rotation_vector(rot_vec)
        quat_rot_inv = quat_rot.conj()

        quat_m = quat_rot_inv * quat_f * quat_rot_inv.conj()

        self.parameters['t_m'] = t_m
        self.parameters['t_f'] = t_f
        self.parameters['psi_0'] = psi_0
        self.parameters['psi_f'] = psi_f
        self.parameters['quat_0'] = quat_0
        self.parameters['quat_m'] = quat_m
        self.parameters['quat_f'] = quat_f
        self.parameters['gc'] = gc_0

    def get_joint_position(self, t):
        if t < self.parameters['t_f']:
            x_t, dx_t = self.trajectory_generator.generate_trajectory(t)
            psi_t = (1 - t / self.parameters['t_f']) * self.parameters['psi_0'] + \
                    t / self.parameters['t_f'] * self.parameters['psi_f']

            if t <= self.parameters['t_m']:
                quat_t = quaternion.slerp(self.parameters['quat_0'], self.parameters['quat_m'],
                                          0., self.parameters['t_m'], t)
            else:
                quat_t = quaternion.slerp(self.parameters['quat_m'], self.parameters['quat_f'],
                                          self.parameters['t_m'], self.parameters['t_f'], t)
            quat_t = quaternion.slerp(self.parameters['quat_0'], self.parameters['quat_f'],
                                      0., self.parameters['t_f'], t)

            pose = np.concatenate([x_t, self.ee_height_offset, quat_t.components])

            res, q = self.kinematics.inverse_kinematics(torch.from_numpy(pose), torch.tensor([psi_t]), self.parameters['gc'])
            if res:
                return q.detach().numpy()
            else:
                raise ValueError("Unable to solve inverse kinematics")

    def _optimize_hit_point(self, puck_p):
        hit_point = np.concatenate([puck_p[:2], self.ee_height_offset])
        hit_direction = np.concatenate([self.goal - puck_p[:2], np.array([0.])])
        hit_direction = hit_direction / np.linalg.norm(hit_direction)

        optimizer = OptimizerHitPoint(hit_point, hit_direction, self.kinematics)
        ret = optimizer.optimize()
        if ret.success:
            return ret.x[:7], ret.x[7:]
        else:
            print("Unable to find a hitting point")
            return None


if __name__ == "__main__":
    tcp_pos = torch.tensor([0., 0., 0.3455]).double()
    tcp_quat = torch.tensor([1., 0., 0., 0., ]).double()
    kinematics = KinematicsTorch(tcp_pos, tcp_quat)

    q_0 = np.array([-1.0207648e-07,  5.0181419e-01,  6.8465688e-08, -1.1696992e+00,
                    -3.7518330e-08,  1.4700792e+00, 0.0])
    # Table height + mallet height
    ee_height_offset = 0.1915
    puck_p = np.array([0.7, 0.2])
    goal = np.array([2.53, 0.0])

    traj_generator = TrajGenerator(goal, ee_height_offset, kinematics)
    traj_generator.plan(q_0, puck_p)

    q_last = q_0

    q_total = list()
    qd_total = list()
    t_total = np.arange(0, traj_generator.parameters['t_f'], 0.001)
    for t in t_total:
        q_t = traj_generator.get_joint_position(t)
        q_total.append(q_t)

    q_total = np.array(q_total)
    qd_total = (q_total[1:, :] - q_total[:-1, :]) / 0.001

    np.save('q.npy', q_total)
    np.save('qd.npy', qd_total)

    import matplotlib.pyplot as plt
    for i in range(7):
        fig, axes = plt.subplots(2, 1)
        axes[0].set_title('Pos Joint ' + str(i + 1))
        axes[0].plot(q_total[:, i])
        axes[1].set_title('Vel Joint ' + str(i + 1 ))
        axes[1].plot(qd_total[:, i])
    plt.show()
    print("123")