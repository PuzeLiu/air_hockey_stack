import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

plt.switch_backend('agg')

import pandas as pd
from scipy import signal
from scipy import optimize as opt
from torch.utils.tensorboard import SummaryWriter

from utils import read_data_from_dirs

"""
Get initial velocity from data
@:param method: gust, sos, vanilla 
"""


def get_init_value(data, method='gust'):
    v0 = []
    for d in data:
        velocity = (d[1:, :2] - d[:-1, :2]) / (d[1:, -1:] - d[:-1, -1:])
        # vel_mag = np.linalg.norm(velocity, axis=1)

        if velocity.shape[0] < 15:
            # print("length is smaller than 15, use the first value as initial")
            v0.append(velocity[0])
        else:
            if method == "gust":
                b, a = signal.ellip(4, 0.1, 120, 0.125)
                vel_gust = signal.filtfilt(b, a, velocity, method="gust")
                v0.append(vel_gust[0])
            elif method == "sos":
                sos = signal.butter(4, 0.125, output='sos')
                vel_sos = signal.sosfiltfilt(sos, velocity)
                v0.append(vel_sos[0])
            elif method == "vanilla":
                v0.append(velocity[0])
            else:
                print("Unknown method")
    return np.array(v0)


def optimize(data):
    logdir = os.path.abspath(__file__ + "../../logs/regression") + "/" + datetime.now().strftime(("%Y-%m-%d-%H-%M-%S"))
    writer = SummaryWriter(logdir)

    def obj_param(x, v0):
        obj = []
        for i, d in enumerate(data):
            if d.shape[0] >= 15:
                velocity = (d[1:, :2] - d[:-1, :2]) / (d[1:, -1:] - d[:-1, -1:])
                # vel = np.linalg.norm(velocity, axis=1)
                t = (d[1:, -1] - d[0, -1])[:, np.newaxis]
                vel_hat = np.exp(- x[0] * t) * v0[i] + x[1] / x[0] * np.exp(- x[0] * t) - x[1] / x[0]
                obj.extend((velocity - vel_hat) ** 2)
        return np.array(obj).mean()

    def obj_v(v0, x, axis):
        obj = []
        for i, d in enumerate(data):
            velocity = ((d[1:, axis] - d[:-1, axis]) / (d[1:, -1] - d[:-1, -1]))
            # vel = np.linalg.norm(velocity, axis=1)
            t = (d[1:, -1] - d[0, -1])
            vel_hat = np.exp(- x[0] * t) * v0[i] + x[1] / x[0] * np.exp(- x[0] * t) - x[1] / x[0]
            obj.extend((velocity - vel_hat) ** 2)
        return np.array(obj).mean()

    # initial guess [air_drag, lateral_friction]
    x0 = np.array([0.01, 0.05])
    bound = opt.Bounds(np.array([1e-6, 1e-6]), np.array([1., 1.]))

    v_init = get_init_value(data)
    v = v_init

    res_param = None
    res_v_x = None
    res_v_y = None
    for i in range(20):
        res_param = opt.minimize(obj_param, x0, args=(v,), method="Powell", bounds=bound, tol=1e-6)
        if not res_param.success:
            print("optimization failed")
        x0 = res_param.x

        v_x = v[:, 0]
        v_y = v[:, 1]
        res_v_x = opt.minimize(obj_v, v_x, args=(res_param.x, 0), method="BFGS", tol=1e-6)
        res_v_y = opt.minimize(obj_v, v_y, args=(res_param.x, 1), method="BFGS", tol=1e-6)
        if not res_v_x.success:
            print("optimization velocity x failed")
        if not res_v_y.success:
            print("optimization velocity y failed")
        v = np.vstack((res_v_x.x, res_v_y.x)).T

        fig_total, ax_total = plt.subplots(3)
        t_prev = 0
        for j, d in enumerate(data):
            if d.shape[0] >= 15:
                fig, ax = plt.subplots(3)
                velocity = (d[1:, :2] - d[:-1, :2]) / (d[1:, -1:] - d[:-1, -1:])
                t = (d[1:, -1] - d[0, -1])[:, np.newaxis]
                vel_model = np.exp(- res_param.x[0] * t) * v[j] + \
                            res_param.x[1] / res_param.x[0] * np.exp(- res_param.x[0] * t) - res_param.x[1] / \
                            res_param.x[0]

                ax[0].plot(t, velocity[:, 0], label="origin")
                ax[0].plot(t, vel_model[:, 0], label="model")
                ax[0].set_title("Velocity X")

                ax[1].plot(t, velocity[:, 1], label="origin")
                ax[1].plot(t, vel_model[:, 1], label="model")
                ax[1].set_title("Velocity Y")

                ax[2].plot(t, np.linalg.norm(velocity, axis=1), label="origin")
                ax[2].plot(t, np.linalg.norm(vel_model, axis=1), label="model")
                ax[2].set_title("Velocity Magnitude")

                writer.add_figure("test/figure/" + str(j), fig, i)

                ax_total[0].plot(t + t_prev, velocity[:, 0], label="origin")
                ax_total[0].plot(t + t_prev, vel_model[:, 0], label="model")
                ax_total[1].plot(t + t_prev, velocity[:, 1], label="origin")
                ax_total[1].plot(t + t_prev, vel_model[:, 1], label="model")
                ax_total[2].plot(t + t_prev, np.linalg.norm(velocity, axis=1), label="origin")
                ax_total[2].plot(t + t_prev, np.linalg.norm(vel_model, axis=1), label="model")
                t_prev = d[-1, -1]

        writer.add_scalar("loss/param", res_param.fun, i)
        writer.add_scalar("loss/vx", res_v_x.fun, i)
        writer.add_scalar("loss/vy", res_v_y.fun, i)
        writer.add_figure("test/total_view", fig_total, i)

    writer.close()

    pd.DataFrame([res_param.x, res_v_x.x, res_v_y.x]).to_csv(os.path.join(logdir, "result.csv"), index=False)
    return res_param.x, [res_v_x.x, res_v_y.x]


def main():
    dirs = ["2020-11-03-14-52-31",
            "2020-11-03-14-53-29",
            "2020-11-03-14-54-16",
            "2020-11-03-14-54-29"]

    data = read_data_from_dirs(dirs)

    param, v0 = optimize(data)
    print(param)
    print(v0)


if __name__ == "__main__":
    main()
